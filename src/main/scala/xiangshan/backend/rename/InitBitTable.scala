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

  private val renameIntMask = io.renameWrite.map(w => bitMask(w.wen && !w.isFp, w.addr)).reduce(_ | _)
  private val renameFpMask = io.renameWrite.map(w => bitMask(w.wen && w.isFp, w.addr)).reduce(_ | _)

  private val commitIntMask = (0 until CommitWidth).map { i =>
    val valid = io.robCommits.isCommit && io.robCommits.commitValid(i)
    bitMask(valid && io.robCommits.info(i).rfWen, io.robCommits.info(i).ldest)
  }.reduce(_ | _)
  private val commitFpMask = (0 until CommitWidth).map { i =>
    val valid = io.robCommits.isCommit && io.robCommits.commitValid(i)
    bitMask(valid && io.robCommits.info(i).fpWen, io.robCommits.info(i).ldest)
  }.reduce(_ | _)

  private def walkNext(oldBits: UInt, fp: Boolean): UInt = {
    val nextBits = Wire(Vec(32, Bool()))
    for (i <- 0 until 32) {
      val hitVec = (0 until CommitWidth).map { j =>
        val info = io.robCommits.info(j)
        val domainWen = if (fp) info.fpWen else info.rfWen
        io.robCommits.isWalk && io.robCommits.walkValid(j) && domainWen && info.ldest === i.U
      }
      val hit = VecInit(hitVec).asUInt.orR
      val data = ParallelPriorityMux(hitVec.reverse, io.robCommits.info.map(_.old_init_bit_value).reverse)
      nextBits(i) := Mux(hit, data, oldBits(i))
    }
    nextBits.asUInt
  }

  val doClear = io.dasicsEn && io.robCommits.dasicsCallJrCommit
  val doWalk = io.robCommits.isWalk && io.robCommits.walkValid.asUInt.orR

  val intSpecNext = Wire(UInt(32.W))
  val fpSpecNext = Wire(UInt(32.W))
  val intArchNext = Wire(UInt(32.W))
  val fpArchNext = Wire(UInt(32.W))

  intArchNext := intInitBitArch | commitIntMask
  fpArchNext := fpInitBitArch | commitFpMask
  intSpecNext := Mux(doWalk, walkNext(intInitBitSpec, fp = false), intInitBitSpec | renameIntMask)
  fpSpecNext := Mux(doWalk, walkNext(fpInitBitSpec, fp = true), fpInitBitSpec | renameFpMask)

  when (doClear) {
    intInitBitArch := intArchNext & ~DasicsClearIntMask
    intInitBitSpec := (intSpecNext | commitIntMask) & ~DasicsClearIntMask
    fpInitBitArch := fpArchNext & ~DasicsClearFpMask
    fpInitBitSpec := (fpSpecNext | commitFpMask) & ~DasicsClearFpMask
  }.otherwise {
    intInitBitArch := intArchNext
    fpInitBitArch := fpArchNext
    intInitBitSpec := intSpecNext
    fpInitBitSpec := fpSpecNext
  }

  for ((port, i) <- io.readPorts.flatten.zipWithIndex) {
    val addr = RegEnable(port.addr, !port.hold)
    val isFp = RegEnable(port.isFp, !port.hold)
    val readAddr = Mux(port.hold, addr, port.addr)
    val readIsFp = Mux(port.hold, isFp, port.isFp)
    val readData = Mux(readIsFp, fpSpecNext(readAddr), intSpecNext(readAddr))
    port.data := RegNext(readData)
  }
}
