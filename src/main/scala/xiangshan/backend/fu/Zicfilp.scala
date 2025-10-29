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

package xiangshan.backend.fu

import chisel3._
import chisel3.util._
import chipsalliance.rocketchip.config.Parameters
import utils._
import xiangshan._

// Zicfilp (Control Flow Integrity - Landing Pad) 常量定义
trait ZicfilpConst {
  val ElpLabelWidth = 20  // Landing pad label位宽 (bits [31:12])
}

// ELP (Expected Landing Pad) 操作类型
// 用于标记指令对ELP状态的影响
object ElpOpType {
  def none: UInt = "b00".U   // 无ELP操作
  def set: UInt = "b01".U    // 设置ELP（JALR with rs1 not in {x1, x5, x7}）
  def clear: UInt = "b10".U  // 清除ELP（LPAD指令）

  def apply() = UInt(2.W)

  def isSet(op: UInt): Bool = op === set
  def isClear(op: UInt): Bool = op === clear
  def isNone(op: UInt): Bool = op === none
}
