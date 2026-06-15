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

package xiangshan.backend.regfile

import chipsalliance.rocketchip.config.Parameters
import chisel3._
import utils._
import xiangshan._

class VectorRegfile(implicit p: Parameters) extends XSModule {
  require(VecRfReadLatency >= 1, s"VecRfReadLatency must be at least 1 cycle, got $VecRfReadLatency")

  private val baseReadLatency = 1
  private val extraReadLatency = VecRfReadLatency - baseReadLatency

  private def delayValid(in: Bool, n: Int): Bool = {
    var out = in
    for (_ <- 0 until n) {
      out = RegNext(out, false.B)
    }
    out
  }

  val io = IO(new Bundle {
    val read = new VectorRfReadPort
    val write = new VectorRfWritePort
  })

  val baseReadData = Regfile(
    NRVecPhyRegs,
    Seq(io.read.addr),
    Seq(io.write.wen),
    Seq(io.write.addr),
    Seq(io.write.data),
    hasZero = false,
    fastSim = !env.FPGAPlatform
  ).head

  // Regfile has one-cycle read data, so response valid follows the same base latency.
  val baseRespValid = RegNext(io.read.valid, false.B)
  io.read.respValid := delayValid(baseRespValid, extraReadLatency)
  io.read.data := DelayN(baseReadData, extraReadLatency)
}
