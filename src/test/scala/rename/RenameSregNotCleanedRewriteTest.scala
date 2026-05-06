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

package renametest

import chipsalliance.rocketchip.config.Parameters
import chisel3._
import chiseltest._
import chiseltest.ChiselScalatestTester
import org.scalatest.flatspec.AnyFlatSpec
import org.scalatest.matchers.must.Matchers
import top.DefaultConfig
import xiangshan._
import xiangshan.backend.rename.RenameZeroRewrite

class RenameRewriteProbe(implicit p: Parameters) extends XSModule {
  val io = IO(new Bundle {
    val dasicsEn = Input(Bool())
    val dasicsUntrusted = Input(Bool())
    val sregNotCleaned = Input(Bool())
    val initBit = Input(Bool())
    val srcType = Input(SrcType())
    val psrc = Input(UInt(PhyRegIdxWidth.W))
    val rewrittenPsrc = Output(UInt(PhyRegIdxWidth.W))
  })

  val shouldRewrite = RenameZeroRewrite.shouldRewrite(
    io.dasicsEn,
    io.dasicsUntrusted,
    io.sregNotCleaned,
    io.initBit,
    io.srcType
  )
  io.rewrittenPsrc := Mux(shouldRewrite, RenameZeroRewrite.zeroPReg(io.srcType, 0, 0), io.psrc)
}

class RenameSregNotCleanedRewriteTest extends AnyFlatSpec with ChiselScalatestTester with Matchers {
  behavior of "Rename sreg_not_cleaned rewrite"

  private val baseConfig: Parameters = new DefaultConfig
  private implicit val p: Parameters = baseConfig.alterPartial({
    case XSCoreParamsKey => baseConfig(XSTileKey).head
  })

  private def setupCase(
    c: RenameRewriteProbe,
    dasicsEn: Boolean,
    sregNotCleaned: Boolean,
    dasicsUntrusted: Boolean,
    initBit: Boolean,
    srcType: UInt = SrcType.reg
  ): Unit = {
    c.io.dasicsEn.poke(dasicsEn.B)
    c.io.sregNotCleaned.poke(sregNotCleaned.B)
    c.io.dasicsUntrusted.poke(dasicsUntrusted.B)
    c.io.initBit.poke(initBit.B)
    c.io.srcType.poke(srcType)
    c.io.psrc.poke(23.U)
  }

  it should "extend DASICS source rewrite to the trap cleanup window" in {
    test(new RenameRewriteProbe) { c =>
      setupCase(c, dasicsEn = true, sregNotCleaned = false, dasicsUntrusted = false, initBit = false)
      c.io.rewrittenPsrc.expect(23.U)

      setupCase(c, dasicsEn = true, sregNotCleaned = false, dasicsUntrusted = true, initBit = false)
      c.io.rewrittenPsrc.expect(0.U)

      setupCase(c, dasicsEn = true, sregNotCleaned = true, dasicsUntrusted = false, initBit = false)
      c.io.rewrittenPsrc.expect(0.U)

      setupCase(c, dasicsEn = true, sregNotCleaned = true, dasicsUntrusted = false, initBit = true)
      c.io.rewrittenPsrc.expect(23.U)

      setupCase(c, dasicsEn = false, sregNotCleaned = true, dasicsUntrusted = true, initBit = false)
      c.io.rewrittenPsrc.expect(23.U)
    }
  }
}
