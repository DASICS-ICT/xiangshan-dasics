package xiangshan.frontend

import chipsalliance.rocketchip.config.Parameters
import chisel3._
import chisel3.util._
import utils._
import xiangshan._

/** The incoming jump source belongs to a fetch packet, not to its predicted successor.
  * A late BPU correction replays that packet with its original incoming source.
  */
class FtqBranchTracker(implicit p: Parameters) extends XSModule with HasCircularQueuePtrHelper {
  val io = IO(new Bundle {
    val allocate = Input(Valid(new FtqPtr))
    val fetch = Input(Bool()) // only a surviving FTQ -> IFU handshake
    val fetchPtr = Input(new FtqPtr)
    val branch = Input(Valid(UInt(VAddrBits.W)))
    val rewind = Input(Valid(new FtqPtr)) // the actual BPU ifuPtr recovery
    val redirect = Input(Valid(Valid(UInt(VAddrBits.W))))
    val source = Output(Valid(UInt(VAddrBits.W)))
  })
  val pending = RegInit(0.U.asTypeOf(Valid(UInt(VAddrBits.W))))
  val incoming = Mem(FtqSize, Valid(UInt(VAddrBits.W)))
  // These tags check snapshot lifetime; an absent source is still a valid snapshot.
  val saved = RegInit(VecInit(Seq.fill(FtqSize)(false.B)))
  val generation = Reg(Vec(FtqSize, Bool()))
  io.source := pending

  when (io.allocate.valid) {
    saved(io.allocate.bits.value) := false.B
  }
  when (io.fetch && !io.rewind.valid && !io.redirect.valid) {
    incoming.write(io.fetchPtr.value, pending)
    saved(io.fetchPtr.value) := true.B
    generation(io.fetchPtr.value) := io.fetchPtr.flag
    pending := io.branch
  }
  // Cancellation must be handled even when IFU is stalled. If this entry has
  // not been sent yet, pending is already its incoming source: retain it.
  when (io.rewind.valid && !io.redirect.valid) {
    when (isAfter(io.fetchPtr, io.rewind.bits)) {
      assert(saved(io.rewind.bits.value) &&
        generation(io.rewind.bits.value) === io.rewind.bits.flag,
        "FTQ replay must have an incoming-source snapshot")
      pending := incoming(io.rewind.bits.value)
    }
  }
  // Match FTQ pointer priority: backend/IFU redirect > BPU rewind > fetch.
  when (io.redirect.valid) {
    pending := io.redirect.bits
  }
}
