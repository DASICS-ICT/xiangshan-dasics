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
import xiangshan.backend.rename._

class InitBitTableTest extends AnyFlatSpec with ChiselScalatestTester with Matchers {
  behavior of "InitBitTable"

  private val baseConfig: Parameters = new DefaultConfig
  private implicit val p: Parameters = baseConfig.alterPartial({
    case XSCoreParamsKey => baseConfig(XSTileKey).head
  })

  private def idle(c: InitBitTable): Unit = {
    c.io.robCommits.isCommit.poke(false.B)
    c.io.robCommits.isWalk.poke(false.B)
    c.io.robCommits.dasicsCallJrCommit.poke(false.B)
    c.io.dasicsEn.poke(false.B)
    for (i <- 0 until c.io.robCommits.commitValid.length) {
      c.io.robCommits.commitValid(i).poke(false.B)
      c.io.robCommits.walkValid(i).poke(false.B)
    }
    for (i <- 0 until c.io.renameWrite.length) {
      c.io.renameWrite(i).wen.poke(false.B)
      c.io.renameWrite(i).addr.poke(0.U)
      c.io.renameWrite(i).isFp.poke(false.B)
    }
    for (i <- 0 until c.io.readPorts.length) {
      for (j <- 0 until c.io.readPorts(i).length) {
        c.io.readPorts(i)(j).hold.poke(false.B)
        c.io.readPorts(i)(j).addr.poke(0.U)
        c.io.readPorts(i)(j).isFp.poke(false.B)
      }
    }
  }

  it should "clear only DASICS caller-saved integer and floating-point init bits when enabled" in {
    test(new InitBitTable) { c =>
      idle(c)
      c.io.dasicsEn.poke(true.B)
      c.io.robCommits.dasicsCallJrCommit.poke(true.B)
      c.clock.step()

      c.io.robCommits.dasicsCallJrCommit.poke(false.B)
      c.io.readPorts(0)(0).addr.poke(5.U)
      c.io.readPorts(0)(0).isFp.poke(false.B)
      c.io.readPorts(0)(1).addr.poke(10.U)
      c.io.readPorts(0)(1).isFp.poke(false.B)
      c.io.readPorts(0)(2).addr.poke(0.U)
      c.io.readPorts(0)(2).isFp.poke(true.B)
      c.io.readPorts(0)(3).addr.poke(12.U)
      c.io.readPorts(0)(3).isFp.poke(true.B)
      c.clock.step()

      c.io.readPorts(0)(0).data.expect(false.B)
      c.io.readPorts(0)(1).data.expect(true.B)
      c.io.readPorts(0)(2).data.expect(false.B)
      c.io.readPorts(0)(3).data.expect(true.B)
    }
  }

  it should "restore walked speculative state with oldest-walk priority" in {
    test(new InitBitTable) { c =>
      idle(c)
      c.io.dasicsEn.poke(true.B)
      c.io.robCommits.dasicsCallJrCommit.poke(true.B)
      c.clock.step()

      c.io.robCommits.dasicsCallJrCommit.poke(false.B)
      c.io.robCommits.isWalk.poke(true.B)
      c.io.robCommits.walkValid(0).poke(true.B)
      c.io.robCommits.info(0).rfWen.poke(true.B)
      c.io.robCommits.info(0).fpWen.poke(false.B)
      c.io.robCommits.info(0).ldest.poke(5.U)
      c.io.robCommits.info(0).old_init_bit_value.poke(true.B)
      c.io.robCommits.walkValid(1).poke(true.B)
      c.io.robCommits.info(1).rfWen.poke(true.B)
      c.io.robCommits.info(1).fpWen.poke(false.B)
      c.io.robCommits.info(1).ldest.poke(5.U)
      c.io.robCommits.info(1).old_init_bit_value.poke(false.B)
      c.clock.step()

      c.io.robCommits.isWalk.poke(false.B)
      c.io.robCommits.walkValid(0).poke(false.B)
      c.io.robCommits.walkValid(1).poke(false.B)
      c.io.readPorts(0)(0).addr.poke(5.U)
      c.io.readPorts(0)(0).isFp.poke(false.B)
      c.clock.step()

      c.io.readPorts(0)(0).data.expect(false.B)
    }
  }

  it should "set speculative init bits on rename and architectural init bits on commit" in {
    test(new InitBitTable) { c =>
      idle(c)
      c.io.dasicsEn.poke(true.B)
      c.io.robCommits.dasicsCallJrCommit.poke(true.B)
      c.clock.step()

      c.io.robCommits.dasicsCallJrCommit.poke(false.B)
      c.io.renameWrite(0).wen.poke(true.B)
      c.io.renameWrite(0).addr.poke(5.U)
      c.io.renameWrite(0).isFp.poke(false.B)
      c.clock.step()

      c.io.renameWrite(0).wen.poke(false.B)
      c.io.readPorts(0)(0).addr.poke(5.U)
      c.io.readPorts(0)(0).isFp.poke(false.B)
      c.clock.step()
      c.io.readPorts(0)(0).data.expect(true.B)

      c.io.dasicsEn.poke(true.B)
      c.io.robCommits.dasicsCallJrCommit.poke(true.B)
      c.clock.step()
      c.io.robCommits.dasicsCallJrCommit.poke(false.B)
      c.io.robCommits.isCommit.poke(true.B)
      c.io.robCommits.commitValid(0).poke(true.B)
      c.io.robCommits.info(0).rfWen.poke(true.B)
      c.io.robCommits.info(0).fpWen.poke(false.B)
      c.io.robCommits.info(0).ldest.poke(5.U)
      c.clock.step()
      c.io.robCommits.isCommit.poke(false.B)
      c.io.robCommits.commitValid(0).poke(false.B)

      c.io.dasicsEn.poke(true.B)
      c.io.robCommits.dasicsCallJrCommit.poke(true.B)
      c.clock.step()
      c.io.robCommits.dasicsCallJrCommit.poke(false.B)
      c.io.readPorts(0)(0).addr.poke(5.U)
      c.io.readPorts(0)(0).isFp.poke(false.B)
      c.clock.step()
      c.io.readPorts(0)(0).data.expect(false.B)
    }
  }
}
