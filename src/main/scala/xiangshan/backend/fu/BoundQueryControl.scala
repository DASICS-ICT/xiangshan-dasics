package xiangshan.backend.fu

import chisel3._
import chisel3.util._
import chipsalliance.rocketchip.config.Parameters
import xiangshan._
import utils.ZeroExt

class BoundQueryInput(implicit p: Parameters) extends XSBundle {
  val request = new BoundQueryRequest
  val tag = new FuOutput(XLEN)
}

/** One policy-neutral transaction, with fall-through for combinational policies.
  * CSR checks permission before presenting a request. The policy owns all state
  * effects and commits at result.fire; cancel must abort an uncommitted request.
  */
class BoundQueryControl(implicit p: Parameters) extends XSModule {
  val io = IO(new Bundle {
    val in = Flipped(Decoupled(new BoundQueryInput))
    val out = Decoupled(new FuOutput(XLEN))
    val request = Decoupled(new BoundQueryRequest)
    val result = Flipped(Decoupled(new BoundQueryResponse))
    val cancel = Output(Bool())
    val redirect = Flipped(Valid(new Redirect))
    val busy = Output(Bool())
  })
  val pending = RegInit(false.B)
  val saved = Reg(new BoundQueryInput)
  val active = Mux(pending, saved, io.in.bits)
  val killed = active.tag.uop.robIdx.needFlush(io.redirect)
  val activeValid = pending || io.in.valid
  io.busy := pending
  io.cancel := activeValid && killed
  io.request.valid := !pending && io.in.valid && !killed
  io.request.bits := active.request
  io.in.ready := !pending && io.request.ready && !killed
  io.result.ready := activeValid && io.out.ready && !killed
  io.out.valid := activeValid && io.result.valid && !killed
  io.out.bits := active.tag
  io.out.bits.data := Mux(io.result.bits.found, ZeroExt(io.result.bits.slot, XLEN), Fill(XLEN, 1.U(1.W)))
  when (io.request.fire && !io.result.fire) { pending := true.B; saved := io.in.bits }
  when (pending && (io.result.fire || killed)) { pending := false.B }
}
