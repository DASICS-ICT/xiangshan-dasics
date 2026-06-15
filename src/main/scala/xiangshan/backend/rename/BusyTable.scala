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
import xiangshan._
import utils._

class BusyTableReadIO(val dataWidth: Int = -1)(implicit p: Parameters) extends XSBundle {
  private val phyRegIdxWidth = if (dataWidth < 0) PhyRegIdxWidth else dataWidth

  val req = Input(UInt(phyRegIdxWidth.W))
  val resp = Output(Bool())
}

class BusyTable(
  numReadPorts: Int,
  numWritePorts: Int,
  numPhyRegsParam: Int = -1,
  phyRegIdxWidthParam: Int = -1,
  perfPrefix: String = "std_freelist"
)(implicit p: Parameters) extends XSModule with HasPerfEvents {
  private val numPhyRegs = if (numPhyRegsParam < 0) NRPhyRegs else numPhyRegsParam
  private val phyRegIdxWidth = if (phyRegIdxWidthParam < 0) PhyRegIdxWidth else phyRegIdxWidthParam

  require(numPhyRegs > 0, "busy table needs at least one physical register")
  require(phyRegIdxWidth > 0, "physical register id width must be positive")

  val io = IO(new Bundle() {
    // set preg state to busy
    val allocPregs = Vec(RenameWidth, Flipped(ValidIO(UInt(phyRegIdxWidth.W))))
    // set preg state to ready (write back regfile + rob walk)
    val wbPregs = Vec(numWritePorts, Flipped(ValidIO(UInt(phyRegIdxWidth.W))))
    // read preg state
    val read = Vec(numReadPorts, new BusyTableReadIO(phyRegIdxWidth))
  })

  val table = RegInit(0.U(numPhyRegs.W))

  def reqVecToMask(rVec: Vec[Valid[UInt]]): UInt = {
    ParallelOR(rVec.map(v => Mux(v.valid, UIntToOH(v.bits, numPhyRegs), 0.U(numPhyRegs.W))))
  }

  val wbMask = reqVecToMask(io.wbPregs)
  val allocMask = reqVecToMask(io.allocPregs)

  val tableAfterWb = table & (~wbMask).asUInt
  val tableAfterAlloc = tableAfterWb | allocMask

  io.read.foreach(r => r.resp := !table(r.req))

  table := tableAfterAlloc

  val oddTable = table.asBools.zipWithIndex.filter(_._2 % 2 == 1).map(_._1)
  val evenTable = table.asBools.zipWithIndex.filter(_._2 % 2 == 0).map(_._1)
  val busyCount = RegNext(RegNext(PopCount(oddTable)) + RegNext(PopCount(evenTable)))

  XSDebug(p"table    : ${Binary(table)}\n")
  XSDebug(p"tableNext: ${Binary(tableAfterAlloc)}\n")
  XSDebug(p"allocMask: ${Binary(allocMask)}\n")
  XSDebug(p"wbMask   : ${Binary(wbMask)}\n")
  for (i <- 0 until numPhyRegs) {
    XSDebug(table(i), "%d is busy\n", i.U)
  }

  XSPerfAccumulate("busy_count", PopCount(table))

  val perfEvents = Seq(
    (s"${perfPrefix}_1_4_valid", busyCount < (numPhyRegs / 4).U                                      ),
    (s"${perfPrefix}_2_4_valid", busyCount > (numPhyRegs / 4).U && busyCount <= (numPhyRegs / 2).U    ),
    (s"${perfPrefix}_3_4_valid", busyCount > (numPhyRegs / 2).U && busyCount <= (numPhyRegs * 3 / 4).U),
    (s"${perfPrefix}_4_4_valid", busyCount > (numPhyRegs * 3 / 4).U                                  )
  )
  generatePerfEvent()
}
