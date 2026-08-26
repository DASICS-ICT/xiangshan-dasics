package futest

import chisel3._
import chiseltest._
import chiseltest.ChiselScalatestTester
import chipsalliance.rocketchip.config.Parameters
import org.scalatest.flatspec.AnyFlatSpec
import org.scalatest.matchers.must.Matchers
import top.MinimalConfig
import xiangshan.{XSCoreParamsKey, XSTileKey}
import xiangshan.backend.fu._

class DasicsBoundGrainHarness(implicit p: Parameters) extends Module {
  private val core = p(XSCoreParamsKey)
  val io = IO(new Bundle {
    val memLo = Input(UInt(core.XLEN.W))
    val memHi = Input(UInt(core.XLEN.W))
    val jumpLo = Input(UInt(core.XLEN.W))
    val jumpHi = Input(UInt(core.XLEN.W))
    val jumpAddr = Input(UInt(core.VAddrBits.W))
    val memLoEffective = Output(UInt(core.XLEN.W))
    val memHiEffective = Output(UInt(core.XLEN.W))
    val jumpLoEffective = Output(UInt(core.XLEN.W))
    val jumpHiEffective = Output(UInt(core.XLEN.W))
    val jumpMatch = Output(Bool())
  })

  val memCfg = WireInit(0.U.asTypeOf(new DasicsMemConfig))
  val memEntry = Wire(new DasicsEntry)
  memEntry.gen(memCfg, io.memLo, io.memHi)

  val jumpCfg = WireInit(0.U.asTypeOf(new DasicsJumpConfig))
  val jumpEntry = Wire(new DasicsJumpEntry)
  jumpEntry.gen(jumpCfg, io.jumpLo, io.jumpHi)

  io.memLoEffective := memEntry.boundLo
  io.memHiEffective := memEntry.boundHi
  io.jumpLoEffective := jumpEntry.boundLo
  io.jumpHiEffective := jumpEntry.boundHi
  io.jumpMatch := jumpEntry.boundMatch(io.jumpAddr)
}

class DasicsMemCheckerTest extends AnyFlatSpec with ChiselScalatestTester with Matchers {
  behavior of "DASICS memory checker"

  private def testParameters: Parameters = {
    val base = new MinimalConfig(1)
    base.alterPartial { case XSCoreParamsKey => base(XSTileKey).head }
  }

  it should "preserve byte endpoints for memory bounds and mask only jump bit zero" in {
    implicit val p: Parameters = testParameters
    test(new DasicsBoundGrainHarness) { dut =>
      dut.io.memLo.poke(0x1003.U)
      dut.io.memHi.poke(0x100b.U)
      dut.io.jumpLo.poke(0x1003.U)
      dut.io.jumpHi.poke(0x1007.U)
      dut.io.jumpAddr.poke(0x1002.U)

      dut.io.memLoEffective.expect(0x1003.U)
      dut.io.memHiEffective.expect(0x100b.U)
      dut.io.jumpLoEffective.expect(0x1002.U)
      dut.io.jumpHiEffective.expect(0x1006.U)
      dut.io.jumpMatch.expect(true.B)
      dut.io.jumpAddr.poke(0x1004.U)
      dut.io.jumpMatch.expect(true.B)
      dut.io.jumpAddr.poke(0x1006.U)
      dut.io.jumpMatch.expect(false.B)
    }
  }

  it should "check complete byte-granular intervals and permissions in one entry" in {
    implicit val p: Parameters = testParameters
    test(new DasicsMemChecker) { dut =>
      dut.io.mode.poke(0.U)
      dut.io.mainCfg.uEnable.poke(true.B)
      dut.io.mainCfg.sEnable.poke(false.B)
      dut.io.mainCfg.closeUJumpFault.poke(false.B)
      dut.io.mainCfg.closeULoadFault.poke(false.B)
      dut.io.mainCfg.closeUStoreFault.poke(false.B)
      dut.io.mainCfg.closeUEcallFault.poke(false.B)
      dut.io.mainCfg.closeSJumpFault.poke(false.B)
      dut.io.mainCfg.closeSLoadFault.poke(false.B)
      dut.io.mainCfg.closeSStoreFault.poke(false.B)
      dut.io.mainCfg.closeSEcallFault.poke(false.B)

      for (entry <- dut.io.resource) {
        entry.cfg.v.poke(false.B)
        entry.cfg.u.poke(false.B)
        entry.cfg.r.poke(false.B)
        entry.cfg.w.poke(false.B)
        entry.boundLo.poke(0.U)
        entry.boundHi.poke(0.U)
      }

      def setEntry(index: Int, lo: BigInt, hi: BigInt, read: Boolean, write: Boolean): Unit = {
        val entry = dut.io.resource(index)
        entry.cfg.v.poke(true.B)
        entry.cfg.r.poke(read.B)
        entry.cfg.w.poke(write.B)
        entry.boundLo.poke(lo.U)
        entry.boundHi.poke(hi.U)
      }

      def clearEntries(): Unit = {
        for (entry <- dut.io.resource) {
          entry.cfg.v.poke(false.B)
        }
      }

      def check(addr: BigInt, lgSize: Int, op: UInt, expectedFault: Int, untrusted: Boolean = true): Unit = {
        dut.io.req.bits.addr.poke(addr.U)
        dut.io.req.bits.lgSize.poke(lgSize.U)
        dut.io.req.bits.operation.poke(op)
        dut.io.req.bits.inUntrustedZone.poke(untrusted.B)
        dut.io.req.valid.poke(true.B)
        dut.clock.step()
        dut.io.req.valid.poke(false.B)
        dut.io.resp.dasics_fault.expect(expectedFault.U)
      }

      setEntry(0, 0x1003, 0x100b, read = true, write = true)
      check(0x100a, 0, DasicsOp.read, 0)
      check(0x1009, 1, DasicsOp.read, 0)
      check(0x100a, 1, DasicsOp.read, 2)
      check(0x1007, 2, DasicsOp.write, 0)
      check(0x1008, 2, DasicsOp.write, 3)
      check(0x1003, 3, DasicsOp.read, 0)
      check(0x1004, 3, DasicsOp.read, 2)
      check(0x1002, 0, DasicsOp.read, 2)

      clearEntries()
      setEntry(0, 0x2000, 0x2004, read = true, write = false)
      setEntry(1, 0x2004, 0x2008, read = true, write = false)
      check(0x2000, 3, DasicsOp.read, 2)

      clearEntries()
      setEntry(0, 0x3000, 0x3008, read = true, write = false)
      setEntry(1, 0x3000, 0x3008, read = false, write = true)
      check(0x3000, 3, DasicsOp.readWrite, 3)
      dut.io.resource(1).cfg.r.poke(true.B)
      check(0x3000, 3, DasicsOp.readWrite, 0)

      clearEntries()
      setEntry(0, 0x7ffffffff0L, 0x7fffffffffL, read = true, write = true)
      check(0x7ffffffffcL, 3, DasicsOp.read, 2)
      check(0x7ffffffffcL, 3, DasicsOp.read, 0, untrusted = false)

      clearEntries()
      setEntry(0, 0x4000, 0x4000, read = true, write = true)
      check(0x4000, 0, DasicsOp.read, 2)
      clearEntries()
      setEntry(0, 0x5008, 0x5000, read = true, write = true)
      check(0x5008, 0, DasicsOp.write, 3)
    }
  }
}
