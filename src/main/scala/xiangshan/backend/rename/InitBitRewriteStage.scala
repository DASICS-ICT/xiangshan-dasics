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
import utils.{ParallelPriorityMux, XSError}
import xiangshan._

// ────────────────────────────────────────────────────────────────────────────
// InitBitRewriteStage — Scheme B (ADR 0003) Stage N+1 of split rename.
//
// Inserted between the existing 1-cycle rename stage (Stage N) and the
// dispatch stage. For each in-flight uop it:
//   (1) Latches the uop from Stage N (T → T+1 via internal RegNext).
//   (2) Reads init_bit_spec[lsrc(0..2)] and init_bit_spec[ldest] from
//       InitBitTable. addr/isFp are issued at T from io.fromRename.bits
//       (un-latched) so InitBitTable's internal RegNext lands data at
//       T+1 in lockstep with our latched uop.
//   (3) Forwards Stage N (T-cycle) rename writes via the t0/t1 bypass
//       three-piece pattern (字面同构 RenameTable.scala:74-77), with
//       explicit j<i cuts to enforce program order.
//   (4) Rewrites psrc to PRF[0] when (dasicsUntrusted && !init_bit &&
//       dasicsEn). Invariant I4 short-circuits this when
//       dasicsUntrusted=0 (trusted-domain bypass with no overhead).
//   (5) Captures effective init_bit at ldest as old_init_bit_value for
//       later ROB walk-back replay.
//
// Pipeline placement (CtrlBlock, commit 6):
//   rename.io.out  ───(direct, no PipelineConnect)──▶ this.fromRename
//   InitBitTable.io.readPort.{addr,isFp} ◀──(driven by this from
//                                              io.fromRename.bits)
//   this.toDispatch ──(PipelineConnect)──▶ dispatch.io.fromRename
//
// Total rename → dispatch latency: T (rename) → T+1 (this stage) →
// T+2 (dispatch enq) — one cycle longer than baseline.
//
// ─── ADR 0003 cross-reference ──────────────────────────────────────────
//   §Decision §3 (X3 split rename)        — Stage N+1 timing & placement
//   §Decision §4 (forwarding mux-tree)    — t0/t1 bypass + j<i cut
//   §Decision §5 (old_init_bit_value)     — ldest position capture
//   §Decision §6 (mux short-circuit)      — dasicsUntrusted gating
//   Invariants  I1, I3, I4                 — write value 1, PRF[0]=0,
//                                              trusted bypass
// ────────────────────────────────────────────────────────────────────────────

class InitBitRewriteStage(implicit p: Parameters) extends XSModule {
  val io = IO(new Bundle {
    // Input from Stage N (rename) -- direct connection (no PipelineConnect).
    // Arrives at T-cycle; this module's internal RegNext latches to T+1.
    val fromRename       = Vec(RenameWidth, Flipped(DecoupledIO(new MicroOp)))

    // Output to dispatch (consumer side will use PipelineConnect).
    // Combinational at T+1; PipelineConnect latches to T+2 for dispatch.
    val toDispatch       = Vec(RenameWidth, DecoupledIO(new MicroOp))

    // Stage N rename writes (same signal that drives InitBitTable.renameWrite).
    // Direct connection; this module RegNext-latches internally to align
    // with fromRename's T+1 visibility.
    val initBitWritePorts = Vec(RenameWidth, Input(new InitBitWritePort))

    // InitBitTable read ports (4 per uop = lsrc 0/1/2 + ldest, total 24).
    // This module drives addr/isFp at T (from io.fromRename.bits, NOT
    // from the latched copy) so InitBitTable's T+1 data lines up with
    // this module's T+1 internal latch.
    val initBitTableRead = Vec(RenameWidth * 4, Flipped(new InitBitReadPort))

    // dasicsEn master gate (= dasicsCfg.uEnable).
    val dasicsEn         = Input(Bool())

    // Pipeline flush (redirect / exception / fence.i).
    val flush            = Input(Bool())
  })

  // ──────────────── Latch Stage N output to T+1 ─────────────────────────
  // Internal RegNext aligns three things to T+1:
  //   (1) latched      <- T-cycle io.fromRename.bits   (uop data)
  //   (2) t1_wInitBit  <- T-cycle io.initBitWritePorts (rename writes)
  //   (3) (no latch on initBitTableRead.data — InitBitTable does its own
  //        RegNext, so .data lands at T+1 already)
  // ────────────────────────────────────────────────────────────────────────

  val latched     = RegNext(VecInit(io.fromRename.map(_.bits)))
  val t1_wInitBit = RegNext(io.initBitWritePorts)

  // ──────────────── Drive InitBitTable read addr at T (un-latched) ───────
  // Must use io.fromRename (T) NOT latched (T+1), so InitBitTable's
  // internal RegNext lands data at T+1 in lockstep with `latched`.
  // Port layout: i*4 + 0/1/2 = lsrc(0/1/2),  i*4 + 3 = ldest.
  // ────────────────────────────────────────────────────────────────────────

  for (i <- 0 until RenameWidth) {
    val uop = io.fromRename(i).bits
    for (k <- 0 until 3) {
      io.initBitTableRead(i * 4 + k).addr := uop.ctrl.lsrc(k)
      io.initBitTableRead(i * 4 + k).isFp := uop.ctrl.srcType(k) === SrcType.fp
    }
    io.initBitTableRead(i * 4 + 3).addr := uop.ctrl.ldest
    io.initBitTableRead(i * 4 + 3).isFp := uop.ctrl.fpWen
  }

  // ──────────────── Forwarding (cross-stage + intra-stage) ───────────────
  // InitBitTable.data lands at T+1 reflecting writes up to T-1; we forward
  // T-cycle Stage N rename writes here. Pattern mirrors
  // RenameTable.scala:74-77 verbatim with two adjustments:
  //   (1) Explicit `if (j < i)` cut forbids self-reference (j==i) and
  //       future-write (j>i) forwarding — enforces program order. The
  //       baseline RAT gray zone (no cut) is bypassed by our explicit
  //       boundary; Scheme B correctness is independent of that gray
  //       zone (ADR 0003 §Decision §4).
  //   (2) bypass_data = constant true.B (invariant I1: rename writes
  //       are always 1). ParallelPriorityMux degrades to OR-tree under
  //       synth constant propagation; RTL form mirrors RAT for review.
  // ────────────────────────────────────────────────────────────────────────

  val effectiveLsrcInitBit = Wire(Vec(RenameWidth, Vec(3, Bool())))

  for (i <- 0 until RenameWidth) {
    val tUop = io.fromRename(i).bits
    for (k <- 0 until 3) {
      val readAddr = tUop.ctrl.lsrc(k)
      val readIsFp = tUop.ctrl.srcType(k) === SrcType.fp

      val t0_bypass = io.initBitWritePorts.zipWithIndex.map { case (w, j) =>
        if (j < i) w.wen && (w.ldest === readAddr) && (w.isFp === readIsFp)
        else false.B
      }
      val t1_bypass = RegNext(VecInit(t0_bypass))

      val bypass_data = ParallelPriorityMux(
        t1_bypass.reverse,
        t1_wInitBit.map(_ => true.B).reverse
      )

      val baseInitBit = io.initBitTableRead(i * 4 + k).data
      effectiveLsrcInitBit(i)(k) := Mux(t1_bypass.asUInt.orR,
                                        bypass_data, baseInitBit)
    }
  }

  // ──────────────── Output: psrc rewrite + old_init_bit_value ────────────
  // For each lsrc(k):
  //   shouldRewrite = dasicsUntrusted && !effectiveInitBit && dasicsEn
  //                && (srcType is reg or fp)
  //   psrc(k) := if (shouldRewrite) ZeroPRegIdx(isFp) else latched.psrc(k)
  //
  // For each uop's old_init_bit_value (used by ROB walk-back):
  //   sample effective init_bit at ldest position, same forwarding network
  //   as lsrc (RAT three-piece + j<i cut).
  //
  // Invariant I4 (trusted-domain bypass): dasicsUntrusted=0 forces
  // shouldRewrite=0 → psrc unchanged → identical to baseline. No
  // mispredict-path overhead in trusted code.
  // ────────────────────────────────────────────────────────────────────────

  for (i <- 0 until RenameWidth) {
    // Default: pass-through latched uop (pdest, ctrl, dasicsUntrusted,
    // all unchanged). Only psrc(k) and old_init_bit_value are overridden.
    io.toDispatch(i).bits := latched(i)

    // psrc rewrite per lsrc(k)
    for (k <- 0 until 3) {
      val srcType   = latched(i).ctrl.srcType(k)
      val isFp_ik   = srcType === SrcType.fp
      val isReg_ik  = srcType === SrcType.reg || srcType === SrcType.fp
      val zeroPReg  = Mux(isFp_ik, FpZeroPRegIdx.U, IntZeroPRegIdx.U)
      val shouldRewrite = latched(i).dasicsUntrusted   &&
                         !effectiveLsrcInitBit(i)(k) &&
                          io.dasicsEn                 &&
                          isReg_ik
      io.toDispatch(i).bits.psrc(k) := Mux(shouldRewrite,
                                           zeroPReg,
                                           latched(i).psrc(k))
    }

    // old_init_bit_value: effective init_bit at ldest with same RAT
    // three-piece forwarding pattern, but on ldest instead of lsrc.
    val ldestAddr = io.fromRename(i).bits.ctrl.ldest
    val ldestIsFp = io.fromRename(i).bits.ctrl.fpWen

    val ldestT0Bypass = io.initBitWritePorts.zipWithIndex.map { case (w, j) =>
      if (j < i) w.wen && (w.ldest === ldestAddr) && (w.isFp === ldestIsFp)
      else false.B
    }
    val ldestT1Bypass   = RegNext(VecInit(ldestT0Bypass))
    val ldestBypassData = ParallelPriorityMux(
      ldestT1Bypass.reverse,
      t1_wInitBit.map(_ => true.B).reverse
    )
    val ldestBaseInitBit = io.initBitTableRead(i * 4 + 3).data
    val ldestEffective   = Mux(ldestT1Bypass.asUInt.orR,
                               ldestBypassData,
                               ldestBaseInitBit)

    io.toDispatch(i).bits.old_init_bit_value := ldestEffective
  }

  // ──────────────── Pipeline handshake + flush ───────────────────────────
  // 1-cycle stage state machine for each lane i:
  //   T:    fromRename.fire arrives → latched_valid goes 1 at T+1
  //   T+1:  toDispatch.valid=1, downstream may consume
  //   T+2+: if toDispatch.fire, latched_valid clears (slot empty)
  //         if fromRename.fire same cycle, slot refilled (back-to-back)
  //
  // Flush priority (highest → lowest):
  //   flush               clears latched_valid (drop in-flight uop)
  //   fromRename.fire     sets   latched_valid (accept new uop)
  //   toDispatch.fire     clears latched_valid (consumed)
  //
  // Data path (latched, t1_wInitBit) is RegNext — updates every cycle
  // regardless of fire; "garbage data" in cycles where latched_valid=0
  // is filtered by toDispatch.valid=0 so downstream never observes it.
  // ────────────────────────────────────────────────────────────────────────

  val latched_valid = RegInit(VecInit(Seq.fill(RenameWidth)(false.B)))

  for (i <- 0 until RenameWidth) {
    when (io.flush) {
      latched_valid(i) := false.B
    } .elsewhen (io.fromRename(i).fire) {
      latched_valid(i) := true.B
    } .elsewhen (io.toDispatch(i).fire) {
      latched_valid(i) := false.B
    }

    // Upstream ready: slot empty OR downstream will consume this cycle.
    io.fromRename(i).ready := !latched_valid(i) || io.toDispatch(i).ready
    // Downstream valid: slot has meaningful data and not flushing.
    io.toDispatch(i).valid := latched_valid(i) && !io.flush
  }
}
