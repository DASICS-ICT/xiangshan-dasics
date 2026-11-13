package xiangshan.backend.fu

import chisel3._
import chisel3.util._

/** Checker-local event. The controller adds the bank without merging ports. */
class BoundUseEvent(slots: Int = 16) extends Bundle {
  val slot = UInt(log2Ceil(slots).W)
}
class BoundPolicyUse extends Bundle {
  val isJump = Bool()
  val slot = UInt(4.W)
}
class BoundSlotMasks extends Bundle {
  val mem = UInt(16.W)
  val jump = UInt(4.W)
}
class BoundQueryRequest extends Bundle {
  val isJump = Bool()
  val mask = UInt(16.W)
  val validSlots = UInt(16.W)
}
class BoundQueryResponse extends Bundle {
  val found = Bool()
  val slot = UInt(4.W)
}
/** Policy-neutral protocol. cancel aborts an outstanding request before commit.
  * Policies commit query effects exactly on response.fire, never on a stall.
  * The controller serializes management and keeps the selected bank stable.
  */
class BoundReplacementIO(ports: Int) extends Bundle {
  val use = Input(Vec(ports, Valid(new BoundPolicyUse)))
  val install = Input(new BoundSlotMasks)
  val invalidate = Input(new BoundSlotMasks)
  val query = Flipped(Decoupled(new BoundQueryRequest))
  val result = Decoupled(new BoundQueryResponse)
  val cancel = Input(Bool())
}

/** One shared combinational ring selector, also used for the four-slot bank. */
class BoundSCSelect extends Module {
  val io = IO(new Bundle {
    val mask = Input(UInt(16.W))
    val validSlots = Input(UInt(16.W))
    val referenced = Input(UInt(16.W))
    val hand = Input(UInt(4.W))
    val result = Output(new BoundQueryResponse)
    val clear = Output(UInt(16.W))
  })
  def rotate(x: UInt): UInt = (Cat(x, x) >> io.hand)(15, 0)
  val allowed = rotate(io.mask)
  val empty = rotate(io.mask & ~io.validSlots)
  val cold = rotate(io.mask & io.validSlots & ~io.referenced)
  val chosen = Mux(empty.orR, empty, Mux(cold.orR, cold, allowed))
  val offset = PriorityEncoder(chosen)
  io.result.found := allowed.orR
  io.result.slot := (io.hand + offset)(3, 0)
  val before = ((1.U(17.W) << offset) - 1.U)(15, 0)
  val clearRotated = Mux(empty.orR, 0.U, Mux(cold.orR, allowed & before, allowed))
  io.clear := (Cat(clearRotated, clearRotated) << io.hand)(31, 16)
}

/** SC owns both banks' R and hand. Ordinary CSR writes never write this state. */
class BoundSecondChance(ports: Int) extends Module {
  val io = IO(new BoundReplacementIO(ports))
  // Policy-specific observation, deliberately separate from the query ABI.
  val referenced = IO(Output(new BoundSlotMasks))
  val memR = RegInit(0.U(16.W))
  val jumpR = RegInit(0.U(4.W))
  val memHand = RegInit(0.U(4.W))
  val jumpHand = RegInit(0.U(2.W))
  referenced.mem := memR
  referenced.jump := jumpR

  val select = Module(new BoundSCSelect)
  val jump = io.query.bits.isJump
  select.io.mask := Mux(jump, io.query.bits.mask & 15.U, io.query.bits.mask)
  select.io.validSlots := io.query.bits.validSlots
  select.io.referenced := Mux(jump, jumpR, memR)
  select.io.hand := Mux(jump, jumpHand, memHand)
  io.query.ready := io.result.ready && !io.cancel
  io.result.valid := io.query.valid && !io.cancel
  io.result.bits := select.io.result
  val commit = io.result.fire && io.result.bits.found
  when (commit) {
    when (jump) { jumpHand := io.result.bits.slot(1, 0) + 1.U }
      .otherwise { memHand := io.result.bits.slot + 1.U }
  }
  def hits(isJump: Boolean, slots: Int): UInt = VecInit((0 until slots).map { i =>
    io.use.map(e => e.valid && e.bits.isJump === isJump.B && e.bits.slot === i.U).reduce(_ || _)
  }).asUInt
  val memClear = Mux(commit && !jump, select.io.clear, 0.U)
  val jumpClear = Mux(commit && jump, select.io.clear(3, 0), 0.U)
  // Management cannot overlap use; invalidate wins defensively, then install/use.
  memR := ((memR & ~memClear) | io.install.mem | hits(false, 16)) & ~io.invalidate.mem
  jumpR := ((jumpR & ~jumpClear) | io.install.jump | hits(true, 4)) & ~io.invalidate.jump
}
