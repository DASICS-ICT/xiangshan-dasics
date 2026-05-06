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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the Mulan PSL v2 for more details.
 ***************************************************************************************/

package robtest

import chisel3._
import chiseltest._
import chiseltest.ChiselScalatestTester
import org.scalatest.flatspec.AnyFlatSpec
import org.scalatest.matchers.must.Matchers
import xiangshan._
import xiangshan.backend.rob.RobCommitDasicsCallDetector

class DasicsCallCommitDetectorProbe extends Module {
  val io = IO(new Bundle {
    val isCommit = Input(Bool())
    val commitValid = Input(Bool())
    val fuType = Input(UInt(4.W))
    val fuOpType = Input(UInt(3.W))
    val clear = Output(Bool())
  })

  io.clear := RobCommitDasicsCallDetector(
    io.isCommit,
    io.commitValid,
    io.fuType,
    io.fuOpType
  )
}

class DasicsCallCommitDetectorTest extends AnyFlatSpec with ChiselScalatestTester with Matchers {
  behavior of "RobCommitDasicsCallDetector"

  private def expectClear(c: DasicsCallCommitDetectorProbe, fuType: UInt, fuOpType: UInt, expected: Boolean): Unit = {
    c.io.isCommit.poke(true.B)
    c.io.commitValid.poke(true.B)
    c.io.fuType.poke(fuType)
    c.io.fuOpType.poke(fuOpType)
    c.io.clear.expect(expected.B)
  }

  it should "treat both dasicscall.j and dasicscall.jr commits as init-bit clear triggers" in {
    test(new DasicsCallCommitDetectorProbe) { c =>
      expectClear(c, FuType.jmp, JumpOpType.dasicscall_j, expected = true)
      expectClear(c, FuType.jmp, JumpOpType.dasicscall_jr, expected = true)
      expectClear(c, FuType.jmp, JumpOpType.jalr, expected = false)
      expectClear(c, FuType.alu, JumpOpType.dasicscall_j, expected = false)

      c.io.isCommit.poke(false.B)
      c.io.commitValid.poke(true.B)
      c.io.fuType.poke(FuType.jmp)
      c.io.fuOpType.poke(JumpOpType.dasicscall_j)
      c.io.clear.expect(false.B)

      c.io.isCommit.poke(true.B)
      c.io.commitValid.poke(false.B)
      c.io.clear.expect(false.B)
    }
  }
}
