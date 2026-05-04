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
// InitBitTable — Scheme B (ADR 0003) liveness table for caller-saved regs.
//
// State (4 × 32-bit packed registers):
//   int_init_bit_spec / arch  -- speculative + architectural copies,
//                                 indexed by integer logical-register number.
//   fp_init_bit_spec  / arch  -- same for fp logical-register number.
//
//   Bit i = 1 ==> logical reg i is "live" (read returns its real PRF value).
//   Bit i = 0 ==> logical reg i was cleared at the most recent
//                 dasicscall.jr commit and has not been re-written;
//                 untrusted-domain reads are rewritten to PRF[0] by
//                 Stage N+1 (InitBitRewriteStage).
//
// Boot = 0xFFFFFFFF on both copies (invariant I5: firmware/Linux boots
// in "all live" semantics, Scheme B path bypassed).
//
// Synthesis hint: 13 bits per copy are write-constants under invariants
// I1+I5+I6 and will fold to constant-1 wires. Effective storage trends
// to 19 × 4 = 76 FFs (verification target in ADR 0003).
//
// ─── ADR 0003 cross-reference ──────────────────────────────────────────
//   §Decision §1 (data structure)         — 4 × 32-bit RegInit, 76 FF target
//   §Decision §2 (clear mask + dasicsEn)  — clearActive gates DasicsClearIntMask
//                                            / DasicsClearFpMask (Parameters.scala)
//   §Decision §5 (write port priority)    — 4-level Mux chain in spec block
//                                            (P1 flush > P2 clear > P3 walk
//                                             > P4 rename > hold)
//   §Decision §3 (split rename / X3)      — consumed by InitBitRewriteStage
//                                            (commit 4/7); this module
//                                            provides raw spec-bit reads,
//                                            forwarding done in Stage N+1
//   Invariants  I1 (rename writes 1)       — by construction, no runtime check
//               I2 (dasicscall.jr exclusive)— XSError catches any collision
//               I5 (boot all-1)             — RegInit("hFFFFFFFF")
//               I6 (dasicsEn full bypass)   — clearActive gates clear
// ────────────────────────────────────────────────────────────────────────────

class InitBitWritePort(implicit p: Parameters) extends XSBundle {
  val wen   = Bool()
  val ldest = UInt(5.W)
  val isFp  = Bool()
}

class InitBitWalkPort(implicit p: Parameters) extends XSBundle {
  val wen      = Bool()
  val ldest    = UInt(5.W)
  val isFp     = Bool()
  val oldValue = Bool()
}

class InitBitReadPort(implicit p: Parameters) extends XSBundle {
  val addr = Input(UInt(5.W))
  val isFp = Input(Bool())
  val data = Output(Bool())
}

class InitBitTable(implicit p: Parameters) extends XSModule {
  val io = IO(new Bundle {
    // Stage N rename writes: ldest.init_bit |= 1 for each rfWen/fpWen.
    val renameWrite = Vec(RenameWidth, Input(new InitBitWritePort))

    // Stage N+1 reads: lsrc(0/1/2) + ldest = 4 ports per uop, 6 uops.
    val readPort    = Vec(RenameWidth * 4, new InitBitReadPort)

    // ROB commit-time signals (atomically driven from Rob.scala).
    val commit      = Input(new Bundle {
      // Per-port architectural retire writes.
      val archWrite          = Vec(CommitWidth, new InitBitWritePort)
      // dasicscall.jr commit clear pulse (ROB head is dasicscall.jr).
      val dasicsCallJrCommit = Bool()
    })

    // ROB walk-back: replay old_init_bit_value into spec, reverse order.
    val walkWrite   = Vec(CommitWidth, Input(new InitBitWalkPort))

    // Global flush (exception / fence.i / cross-stage redirect).
    val flush       = Input(Bool())

    // Master gate (= dasicsCfg.uEnable; ADR 0003 §Decision §2).
    val dasicsEn    = Input(Bool())
  })

  // ──────────────────── State ────────────────────
  val int_init_bit_spec = RegInit("hFFFFFFFF".U(32.W))
  val int_init_bit_arch = RegInit("hFFFFFFFF".U(32.W))
  val fp_init_bit_spec  = RegInit("hFFFFFFFF".U(32.W))
  val fp_init_bit_arch  = RegInit("hFFFFFFFF".U(32.W))

  // ──────────── Spec write ports — 4-level priority chain ────────────────
  // Order (highest to lowest priority, ADR 0003 §Decision §5):
  //   P1  flush                                    spec := arch
  //   P2  dasicscall.jr commit clear (dasicsEn)    spec &= ~mask
  //   P3  walk-back (per-bit)                      spec[ldest] := oldValue
  //   P4  rename (per-bit, write value = 1; I1)    spec[ldest] := 1
  //   hold                                          spec unchanged
  //
  // P2 and P3 are mutually exclusive (dasicscall.jr ROB-exclusive at
  // commit; ADR 0002 + I2). The priority chain still expresses correct
  // intent if the invariant later relaxes.
  // ────────────────────────────────────────────────────────────────────────

  // Rename per-side OH (write value is constant 1; simple OR aggregation).
  val intRenameOH = io.renameWrite.map(w =>
    Mux(w.wen && !w.isFp, UIntToOH(w.ldest, 32), 0.U(32.W))).reduce(_ | _)
  val fpRenameOH  = io.renameWrite.map(w =>
    Mux(w.wen &&  w.isFp, UIntToOH(w.ldest, 32), 0.U(32.W))).reduce(_ | _)

  // Walk-back per-side: (anyHitMask 32-bit, per-bit value 32-bit).
  // Highest port index wins per bit = oldest walked uop; identical
  // convention to RenameTable.scala:62-68.
  private def walkPerBit(matchFp: Boolean): (UInt, UInt) = {
    val perPort = io.walkWrite.map { w =>
      val typeMatch = if (matchFp) w.isFp else !w.isFp
      val oh = Mux(w.wen && typeMatch, UIntToOH(w.ldest, 32), 0.U(32.W))
      (oh, w.oldValue)
    }
    val anyMask  = perPort.map(_._1).reduce(_ | _)
    val valueVec = VecInit((0 until 32).map(k =>
      ParallelPriorityMux(perPort.map(_._1(k)).reverse,
                          perPort.map(_._2).reverse)))
    (anyMask, valueVec.asUInt)
  }
  val (intWalkMask, intWalkVal) = walkPerBit(matchFp = false)
  val (fpWalkMask , fpWalkVal ) = walkPerBit(matchFp = true)

  // dasicscall.jr clear: gate the constant mask by dasicsEn + commit pulse.
  val clearActive  = io.commit.dasicsCallJrCommit && io.dasicsEn
  val intClearMask = Mux(clearActive, DasicsClearIntMask.U(32.W), 0.U(32.W))
  val fpClearMask  = Mux(clearActive, DasicsClearFpMask.U(32.W),  0.U(32.W))

  // Apply 4-level priority per bit; re-pack to 32-bit register write.
  int_init_bit_spec := VecInit((0 until 32).map { k =>
    Mux(io.flush,         int_init_bit_arch(k),     // P1
    Mux(intClearMask(k),  false.B,                  // P2
    Mux(intWalkMask(k),   intWalkVal(k),            // P3
    Mux(intRenameOH(k),   true.B,                   // P4
                          int_init_bit_spec(k)))))  // hold
  }).asUInt

  fp_init_bit_spec := VecInit((0 until 32).map { k =>
    Mux(io.flush,         fp_init_bit_arch(k),
    Mux(fpClearMask(k),   false.B,
    Mux(fpWalkMask(k),    fpWalkVal(k),
    Mux(fpRenameOH(k),    true.B,
                          fp_init_bit_spec(k)))))
  }).asUInt

  // ──────────────── Arch write ports — commit OR + clear ─────────────────
  // The arch copy is the architectural retired state, updated only by
  // committed uops. No flush write (flush merely reseeds spec from arch).
  // No walk write (walk only revises speculative state).
  //
  // Per-cycle update:
  //   arch_next = (arch | retireOR) & ~clearMask
  //
  // OR term and clear mask are bit-disjoint by construction:
  //   * dasicscall.jr commit cycle puts ra (bit 1) in retireOR.
  //   * Clear masks 0xF000_00E0 / 0xF000_00FF intentionally exclude bit 1.
  // So OR-then-AND yields the same result as AND-then-OR: order-free.
  // ────────────────────────────────────────────────────────────────────────

  val intCommitOR = io.commit.archWrite.map(w =>
    Mux(w.wen && !w.isFp, UIntToOH(w.ldest, 32), 0.U(32.W))).reduce(_ | _)
  val fpCommitOR  = io.commit.archWrite.map(w =>
    Mux(w.wen &&  w.isFp, UIntToOH(w.ldest, 32), 0.U(32.W))).reduce(_ | _)

  int_init_bit_arch := (int_init_bit_arch | intCommitOR) & ~intClearMask
  fp_init_bit_arch  := (fp_init_bit_arch  | fpCommitOR)  & ~fpClearMask

  // ──────────────── Read ports — T0/T+1 synchronous read ─────────────────
  // At cycle T: addr/isFp present; at cycle T+1: data delivered (RegNext).
  //
  // The latched value reflects writes up to and including cycle T-1.
  // Cross-stage forwarding for T-cycle Stage N rename writes is performed
  // inside InitBitRewriteStage (commit 4/7), NOT here.
  //
  // No internal t0/t1 bypass like RenameTable.scala:74-77 — walk and
  // dasicscall.jr clear are mutually exclusive with valid Stage N+1
  // traffic (ADR 0002 + I2); rename forwarding requires per-i (j<i)
  // program-order cuts that only Stage N+1 can express.
  // ────────────────────────────────────────────────────────────────────────

  for (r <- io.readPort) {
    val table = Mux(r.isFp, fp_init_bit_spec, int_init_bit_spec)
    r.data := RegNext(table(r.addr))
  }

  // ──────────────── Invariant assertions ─────────────────────────────────
  // I2 (ADR 0002): dasicscall.jr is ROB-exclusive at commit; walk-back
  // cannot coexist with dasicscall.jr commit clear in the same cycle.
  // Catches any future regression in Rob.scala that violates this.

  private val walkActive = io.walkWrite.map(_.wen).reduce(_ || _)
  XSError(clearActive && walkActive,
    "I2 violated: dasicscall.jr commit clear and walk-back collide " +
    "(should be mutually exclusive per ADR 0002 + ROB hasBlockBackward)\n")
}
