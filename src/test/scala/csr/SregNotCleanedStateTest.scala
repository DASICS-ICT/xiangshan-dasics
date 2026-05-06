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

package csrtest

import chipsalliance.rocketchip.config.Parameters
import chisel3._
import chiseltest._
import chiseltest.ChiselScalatestTester
import org.scalatest.flatspec.AnyFlatSpec
import org.scalatest.matchers.must.Matchers
import top.DefaultConfig
import xiangshan._
import xiangshan.backend.fu.SregNotCleanedState
import xiangshan.backend.fu.util.HasCSRConst

class SregNotCleanedStateProbe(implicit p: Parameters) extends XSModule with HasCSRConst {
  val io = IO(new Bundle {
    val trapValid = Input(Bool())
    val trapPrivMode = Input(UInt(2.W))
    val trapDasicsUntrusted = Input(Bool())
    val dasicsUEnable = Input(Bool())
    val xretValid = Input(Bool())
    val xretLegal = Input(Bool())
    val xretReturnMode = Input(UInt(2.W))
    val xretReturnDasicsUntrusted = Input(Bool())
    val state = Output(Bool())
  })

  val state = Module(new SregNotCleanedState)
  state.io.trapValid := io.trapValid
  state.io.trapPrivMode := io.trapPrivMode
  state.io.trapDasicsUntrusted := io.trapDasicsUntrusted
  state.io.dasicsUEnable := io.dasicsUEnable
  state.io.xretValid := io.xretValid
  state.io.xretLegal := io.xretLegal
  state.io.xretReturnMode := io.xretReturnMode
  state.io.xretReturnDasicsUntrusted := io.xretReturnDasicsUntrusted
  io.state := state.io.sregNotCleaned
}

class SregNotCleanedStateTest extends AnyFlatSpec with ChiselScalatestTester with Matchers with HasCSRConst {
  behavior of "sreg_not_cleaned state"

  private val baseConfig: Parameters = new DefaultConfig
  private implicit val p: Parameters = baseConfig.alterPartial({
    case XSCoreParamsKey => baseConfig(XSTileKey).head
  })

  private def idle(c: SregNotCleanedStateProbe): Unit = {
    c.io.trapValid.poke(false.B)
    c.io.trapPrivMode.poke(ModeM)
    c.io.trapDasicsUntrusted.poke(false.B)
    c.io.dasicsUEnable.poke(true.B)
    c.io.xretValid.poke(false.B)
    c.io.xretLegal.poke(false.B)
    c.io.xretReturnMode.poke(ModeM)
    c.io.xretReturnDasicsUntrusted.poke(false.B)
  }

  private def trap(c: SregNotCleanedStateProbe, mode: UInt, untrusted: Boolean, dasicsUEnable: Boolean = true): Unit = {
    idle(c)
    c.io.trapValid.poke(true.B)
    c.io.trapPrivMode.poke(mode)
    c.io.trapDasicsUntrusted.poke(untrusted.B)
    c.io.dasicsUEnable.poke(dasicsUEnable.B)
    c.clock.step()
    idle(c)
  }

  private def xret(
    c: SregNotCleanedStateProbe,
    legal: Boolean,
    returnMode: UInt,
    returnDasicsUntrusted: Boolean = true
  ): Unit = {
    idle(c)
    c.io.xretValid.poke(true.B)
    c.io.xretLegal.poke(legal.B)
    c.io.xretReturnMode.poke(returnMode)
    c.io.xretReturnDasicsUntrusted.poke(returnDasicsUntrusted.B)
    c.clock.step()
    idle(c)
  }

  it should "set only on enabled U untrusted traps" in {
    test(new SregNotCleanedStateProbe) { c =>
      idle(c)
      trap(c, ModeU, untrusted = false)
      c.io.state.expect(false.B)

      trap(c, ModeS, untrusted = true)
      c.io.state.expect(false.B)

      trap(c, ModeM, untrusted = true)
      c.io.state.expect(false.B)

      trap(c, ModeU, untrusted = true, dasicsUEnable = false)
      c.io.state.expect(false.B)

      trap(c, ModeU, untrusted = true)
      c.io.state.expect(true.B)
    }
  }

  it should "keep state across nested returns to S and illegal xRET, then clear on final xRET to U untrusted" in {
    test(new SregNotCleanedStateProbe) { c =>
      idle(c)
      trap(c, ModeU, untrusted = true)
      c.io.state.expect(true.B)

      xret(c, legal = true, returnMode = ModeS)
      c.io.state.expect(true.B)

      trap(c, ModeS, untrusted = false)
      c.io.state.expect(true.B)

      xret(c, legal = true, returnMode = ModeS)
      c.io.state.expect(true.B)

      xret(c, legal = false, returnMode = ModeU)
      c.io.state.expect(true.B)

      xret(c, legal = true, returnMode = ModeU, returnDasicsUntrusted = false)
      c.io.state.expect(true.B)

      xret(c, legal = true, returnMode = ModeU, returnDasicsUntrusted = true)
      c.io.state.expect(false.B)
    }
  }
}
