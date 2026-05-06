/***************************************************************************************
* Copyright (c) 2020-2021 Institute of Computing Technology, Chinese Academy of Sciences
* Copyright (c) 2020-2021 Peng Cheng Laboratory
*
* XiangShan is licensed under Mulan PSL v2.
* You can use this software according to the terms and conditions of the Mulan PSL v2.
* You may obtain a copy of Mulan PSL v2 at:
*          http://license.coscl.org.cn/MulanPSL2
*
* THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY KIND,
* EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO NON-INFRINGEMENT,
* MERCHANTABILITY OR FIT FOR A PARTICULAR PURPOSE.
*
* See the Mulan PSL v2 for more details.
***************************************************************************************/

package xiangshan.backend.rename

import chipsalliance.rocketchip.config.Parameters
import chisel3._
import chisel3.util._
import utils.ParallelPriorityMux
import xiangshan._

class InitBitReadPort(implicit p: Parameters) extends XSBundle {
  val hold = Input(Bool())
  val addr = Input(UInt(5.W))
  val isFp = Input(Bool())
  val data = Output(Bool())
}

class InitBitWritePort(implicit p: Parameters) extends XSBundle {
  val wen = Bool()
  val addr = UInt(5.W)
  val isFp = Bool()
}

// Internal write format after merging ROB walk restore and rename writes.
class InitBitUpdatePort(implicit p: Parameters) extends XSBundle {
  val wen = Bool()
  val addr = UInt(5.W)
  val isFp = Bool()
  val data = Bool()
}

/**
  * InitBitTable tracks whether each architectural integer / floating-point
  * logical register has been initialized since the latest DASICS clear.
  *
  * The read side intentionally mirrors RenameTable timing: Decode sends the
  * logical register address at T0, and Rename consumes the registered data at
  * T1. Writes are also delayed to T1, with explicit bypass for a T0 write that
  * targets a T0 read address.
  *
  *   T0: Decode / write request                  T1: Rename / table update
  *   -------------------------------------       ----------------------------------
  *   readPorts.addr,isFp --------------------->  readPorts.data is consumed
  *   renameWrite or ROB walk ----+               t1SpecWritePorts update spec bits
  *                               |
  *                               +------------>  bypass wins over registered read
  *
  *   if hold = 1:
  *     - keep the previously latched read address and domain;
  *     - keep the previously visible read data unless a DASICS clear changes
  *       the held logical register, or a matching T0 write bypasses to the
  *       held address.
  *
  * This table is logical-state metadata only. It does not clear PRF contents.
  * Rename.scala uses the read bits to rewrite untrusted source operands to the
  * reserved zero PRF when a source logical register is marked uninitialized.
  */
class InitBitTable(implicit p: Parameters) extends XSModule {
  val io = IO(new Bundle {
    val robCommits = Flipped(new RobCommitIO)
    val renameWrite = Vec(RenameWidth, Input(new InitBitWritePort))
    val readPorts = Vec(RenameWidth, Vec(4, new InitBitReadPort))
    val dasicsEn = Input(Bool())
  })

  val intInitBitSpec = RegInit("hFFFFFFFF".U(32.W))
  val intInitBitArch = RegInit("hFFFFFFFF".U(32.W))
  val fpInitBitSpec = RegInit("hFFFFFFFF".U(32.W))
  val fpInitBitArch = RegInit("hFFFFFFFF".U(32.W))

  private def bitMask(wen: Bool, addr: UInt): UInt = Mux(wen, UIntToOH(addr, 32), 0.U(32.W))
  private def readSpec(addr: UInt, isFp: Bool, intBits: UInt, fpBits: UInt): Bool = Mux(isFp, fpBits(addr), intBits(addr))

  // Apply delayed speculative writes with the same reverse-priority policy as
  // RenameTable: the later port in program order wins when ports hit one lreg.
  private def applySpecWrites(oldBits: UInt, writes: Vec[InitBitUpdatePort], fp: Boolean): UInt = {
    val nextBits = Wire(Vec(32, Bool()))
    for (i <- 0 until 32) {
      val hitVec = writes.map(w => w.wen && (w.isFp === fp.B) && (w.addr === i.U))
      val hit = VecInit(hitVec).asUInt.orR
      val data = ParallelPriorityMux(hitVec.reverse, writes.map(_.data).reverse)
      nextBits(i) := Mux(hit, data, oldBits(i))
    }
    nextBits.asUInt
  }

  private val commitIntMask = (0 until CommitWidth).map { i =>
    val valid = io.robCommits.isCommit && io.robCommits.commitValid(i)
    bitMask(valid && io.robCommits.info(i).rfWen, io.robCommits.info(i).ldest)
  }.reduce(_ | _)
  private val commitFpMask = (0 until CommitWidth).map { i =>
    val valid = io.robCommits.isCommit && io.robCommits.commitValid(i)
    bitMask(valid && io.robCommits.info(i).fpWen, io.robCommits.info(i).ldest)
  }.reduce(_ | _)
  private val postClearCommitIntMask = (1 until CommitWidth).map { i =>
    val valid = io.robCommits.isCommit && io.robCommits.commitValid(i)
    bitMask(valid && io.robCommits.info(i).rfWen, io.robCommits.info(i).ldest)
  }.reduceOption(_ | _).getOrElse(0.U(32.W))
  private val postClearCommitFpMask = (1 until CommitWidth).map { i =>
    val valid = io.robCommits.isCommit && io.robCommits.commitValid(i)
    bitMask(valid && io.robCommits.info(i).fpWen, io.robCommits.info(i).ldest)
  }.reduceOption(_ | _).getOrElse(0.U(32.W))

  val doClear = io.dasicsEn && io.robCommits.dasicsCallCommit

  // Start from ROB walk restore writes. A younger in-flight write stores the
  // old init bit in the ROB; walking restores that old value.
  val specWritePorts = Wire(Vec(CommitWidth, new InitBitUpdatePort))
  for (i <- 0 until CommitWidth) {
    val info = io.robCommits.info(i)
    specWritePorts(i).wen := io.robCommits.isWalk && io.robCommits.walkValid(i) && (info.rfWen || info.fpWen)
    specWritePorts(i).addr := info.ldest
    specWritePorts(i).isFp := info.fpWen
    specWritePorts(i).data := info.old_init_bit_value
  }
  // Rename writes mark the destination logical register initialized. These
  // ports override walk writes in the normal RenameTable style.
  for ((spec, rename) <- specWritePorts.zip(io.renameWrite)) {
    when (rename.wen) {
      spec.wen := true.B
      spec.addr := rename.addr
      spec.isFp := rename.isFp
      spec.data := true.B
    }
  }

  val t1SpecWritePorts = RegNext(specWritePorts)

  // T1 table update follows program order around DASICSCALL:
  // older state and current commit writes are cleared first, then younger
  // same-cycle commit slots and delayed rename writes are applied after clear.
  val intSpecNoClear = applySpecWrites(intInitBitSpec, t1SpecWritePorts, fp = false)
  val fpSpecNoClear = applySpecWrites(fpInitBitSpec, t1SpecWritePorts, fp = true)
  val intSpecAfterClear = ((intInitBitSpec | commitIntMask) & ~DasicsClearIntMask) | postClearCommitIntMask
  val fpSpecAfterClear = ((fpInitBitSpec | commitFpMask) & ~DasicsClearFpMask) | postClearCommitFpMask
  val intArchWriteNext = intInitBitArch | commitIntMask
  val fpArchWriteNext = fpInitBitArch | commitFpMask

  val intSpecNext = Mux(doClear, applySpecWrites(intSpecAfterClear, t1SpecWritePorts, fp = false), intSpecNoClear)
  val fpSpecNext = Mux(doClear, applySpecWrites(fpSpecAfterClear, t1SpecWritePorts, fp = true), fpSpecNoClear)
  val intArchNext = Mux(doClear, (intArchWriteNext & ~DasicsClearIntMask) | postClearCommitIntMask, intArchWriteNext)
  val fpArchNext = Mux(doClear, (fpArchWriteNext & ~DasicsClearFpMask) | postClearCommitFpMask, fpArchWriteNext)

  intInitBitArch := intArchNext
  fpInitBitArch := fpArchNext
  intInitBitSpec := intSpecNext
  fpInitBitSpec := fpSpecNext

  for ((port, i) <- io.readPorts.flatten.zipWithIndex) {
    // READ path, aligned with RenameTable:
    // - t1RData is the registered table read result;
    // - t1RAddr/t1RIsFp remember the T0 read request while Decode is held;
    // - doClear refreshes held read data from the post-clear table state;
    // - t1Bypass forwards a T0 write to the T1 read result.
    val t1RAddr = RegEnable(port.addr, !port.hold)
    val t1RIsFp = RegEnable(port.isFp, !port.hold)
    val readAddr = Mux(port.hold, t1RAddr, port.addr)
    val readIsFp = Mux(port.hold, t1RIsFp, port.isFp)
    val heldReadData = readSpec(readAddr, readIsFp, intSpecNext, fpSpecNext)
    val freshReadData = readSpec(port.addr, port.isFp, intSpecNext, fpSpecNext)
    val t1RData = RegNext(Mux(port.hold, Mux(doClear, heldReadData, port.data), freshReadData))
    val t0Bypass = specWritePorts.map(w => w.wen && (w.addr === readAddr) && (w.isFp === readIsFp))
    val t1Bypass = RegNext(VecInit(t0Bypass))
    val bypassData = ParallelPriorityMux(t1Bypass.reverse, t1SpecWritePorts.map(_.data).reverse)
    port.data := Mux(t1Bypass.asUInt.orR, bypassData, t1RData)
  }
}
