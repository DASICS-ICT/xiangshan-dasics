// See README.md for license details.

package xiangshan.backend.fu

import chisel3._
import chisel3.util._
import chipsalliance.rocketchip.config.Parameters
import utils._
import xiangshan._
import xiangshan.backend.fu.util.HasCSRConst

trait DasicsConst {
  val NumDasicsBounds = 16  // For load/store
  val DasicsGrain = 3 // 8 bytes of granularity
}

// For load/store only
class DasicsConfig extends Bundle {
  val v: Bool = Bool()  // valid
  val x: Bool = Bool()  // unused
  val w: Bool = Bool()  // write
  val r: Bool = Bool()  // read

  def valid: Bool = v
  def write: Bool = w
  def read: Bool = r
}

object DasicsConfig extends DasicsConfig

// For load/store only
class DasicsEntry(implicit p: Parameters) extends XSBundle with DasicsConst {
  val cfg = new DasicsConfig
  val boundHi, boundLo = UInt((XLEN - DasicsGrain).W) // bounds are 8-byte aligned

  // Lowest bits read/write as 0
  def boundRegMask: UInt = (~((1 << DasicsGrain) - 1).U(XLEN.W)).asUInt

  // Only check bounds, not checking permission
  def boundMatch(addr: UInt): Bool = {
    val addrForComp = addr(XLEN - 1, DasicsGrain)
    (addrForComp >= boundLo) && (addrForComp < boundHi)
  }

  // assign values (bounds parameter are XLEN-length)
  def gen(cfg: DasicsConfig, boundLo: UInt, boundHi: UInt): Unit = {
    this.cfg := cfg
    this.boundLo := boundLo(XLEN - 1, DasicsGrain)
    this.boundHi := boundHi(XLEN - 1, DasicsGrain)
  }
}

trait DasicsMethod extends DasicsConst { this: HasXSParameter =>
  def dasicsInit(): (Vec[UInt], Vec[UInt]) = {
    val dasicsCfgPerCSR = XLEN / DasicsConfig.getWidth
    val cfgs = WireInit(0.U.asTypeOf(Vec(NumDasicsBounds / dasicsCfgPerCSR, UInt(XLEN.W))))
    val bounds = WireInit(0.U.asTypeOf(Vec(NumDasicsBounds * 2, UInt(XLEN.W))))
    (cfgs, bounds)
  }

  def dasicsGenMapping(
    init: () => (Vec[UInt], Vec[UInt]),
    num: Int = NumDasicsBounds,
    cfgBase: Int, boundBase: Int,
    entries: Vec[DasicsEntry]
  ): Map[Int, (UInt, UInt, UInt => UInt, UInt, UInt => UInt)] = {
    val dasicsCfgPerCSR = XLEN / DasicsConfig.getWidth
    def dasicsCfgIndex(i: Int) = i / dasicsCfgPerCSR
    // init_value: (cfgs, bounds)
    val init_value = init()
    // DasicsConfigs merged into CSR
    val cfgMerged = RegInit(init_value._1)
    val cfgs = WireInit(cfgMerged).asTypeOf(Vec(num, new DasicsConfig))
    val bounds = RegInit(init_value._2)

    // Wire entries to the registers
    for (i <- entries.indices) {
      entries(i).gen(cfgs(i), boundLo = bounds(i * 2), boundHi = bounds(i * 2 + 1))
    }

    val cfg_mapping = Map(
      (0 until num by dasicsCfgPerCSR).map(i =>
        MaskedRegMap(addr = cfgBase + dasicsCfgIndex(i), reg = cfgMerged(i / dasicsCfgPerCSR))
      ) : _*
    )

    val bound_mapping = Map(
      (0 until num * 2).map(i => MaskedRegMap(
        addr = boundBase + i, reg = bounds(i),
        wmask = DasicsEntry.boundRegMask, rmask = DasicsEntry.boundRegMask
      )) : _*
    )

    cfg_mapping ++ bound_mapping
  }

  // Singleton companion object for DasicsEntry, with implicit parameters set
  private object DasicsEntry extends DasicsEntry
}

class DasicsIO(implicit p: Parameters) extends XSBundle with DasicsConst {
  val distribute_csr: DistributedCSRIO = Flipped(new DistributedCSRIO())
  val entries: Vec[DasicsEntry] = Output(Vec(NumDasicsBounds, new DasicsEntry()))
}

class Dasics(implicit p: Parameters) extends XSModule with DasicsMethod with HasCSRConst {
  val io: DasicsIO = IO(new DasicsIO())

  val w = io.distribute_csr.w

  private val dasics = Wire(Vec(NumDasicsBounds, new DasicsEntry()))
  val mapping: Map[Int, (UInt, UInt, UInt => UInt, UInt, UInt => UInt)] =
    dasicsGenMapping(init = dasicsInit, cfgBase = DasicsLibCfgBase, boundBase = DasicsLibBoundBase, entries = dasics)

  val rdata: UInt = Wire(UInt(XLEN.W))
  MaskedRegMap.generate(mapping, w.bits.addr, rdata, w.valid, w.bits.data)

  io.entries := dasics
}
