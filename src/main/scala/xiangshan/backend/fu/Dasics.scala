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

object DasicsOp{
  def read   = "b00".U
  def write  = "b01".U
  def jump   = "b10".U

  def apply() = UInt(2.W)
  def isWrite(op:UInt) = op === write
  def isRead(op:UInt)  = op === read
  def isJump(op:UInt)  = op === jump
}

object DasicsCheckFault {
    def noDasicsFault     = "b000".U


    def SReadDascisFault  = "b001".U
    def SWriteDasicsFault = "b010".U
    def SJumpDasicsFault  = "b011".U

    def UReadDascisFault  = "b100".U
    def UWriteDasicsFault = "b101".U
    def UJumpDasicsFault  = "b110".U
    
    def apply() = UInt(3.W)
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
  val boundHi, boundLo = UInt(XLEN.W)//UInt((VAddrBits - DasicsGrain).W) // bounds are 8-byte aligned

  // Lowest bits read/write as 0
  def boundRegMask: UInt = (~((1 << DasicsGrain) - 1).U(XLEN.W)).asUInt

  // Only check bounds, not checking permission
  def boundMatch(addr: UInt): Bool = {
    val addrForComp = addr(VAddrBits - 1, DasicsGrain)
    (addrForComp >= boundLo(VAddrBits - 1, DasicsGrain)) && (addrForComp < boundHi(VAddrBits - 1, DasicsGrain))
  }

  // assign values (bounds parameter are XLEN-length)
  def gen(cfg: DasicsConfig, boundLo: UInt, boundHi: UInt): Unit = {
    this.cfg := cfg
    this.boundLo := boundLo(VAddrBits - 1, DasicsGrain)
    this.boundHi := boundHi(VAddrBits - 1, DasicsGrain)
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
  val mapping = dasicsGenMapping(init = dasicsInit, cfgBase = DasicsLibCfgBase, boundBase = DasicsLibBoundBase, entries = dasics)

  val rdata: UInt = Wire(UInt(XLEN.W))
  MaskedRegMap.generate(mapping, w.bits.addr, rdata, w.valid, w.bits.data)


  io.entries := dasics
}

class DasicsReqBundle(implicit p: Parameters) extends XSBundle with DasicsConst {
  val addr = Output(UInt(VAddrBits.W))
  val inUntrustedZone = Output(Bool())
  val operation = Output(DasicsOp())
}

class DasicsRespBundle(implicit p: Parameters) extends XSBundle with DasicsConst{
  val dasics_fault = Output(DasicsCheckFault())
}

class DasicsCheckerIO(implicit p: Parameters) extends XSBundle with DasicsConst{
  val mode = Input(UInt(4.W))
  val resource = Flipped(Output(Vec(NumDasicsBounds, new DasicsEntry())))
  val req = Flipped(Valid(new DasicsReqBundle()))
  val resp = new DasicsRespBundle()

  //connect for every dasics request
  def connect(addr:UInt, inUntrustedZone:Bool, operation: UInt, entries: Vec[DasicsEntry]): Unit = {
    this.req.bits.addr := addr
    this.req.bits.inUntrustedZone := inUntrustedZone
    this.req.bits.operation := operation
    this.resource := entries
  }
}

trait DasicsCheckerMethod {
  //def dasics_check(addr:UInt, isUntrustedZone: Bool, op: UInt, dasics: Vec[DasicsEntry]): Bool

  def dasics_addr_bound(addr:UInt, dasics:Vec[DasicsEntry]) = {
    VecInit(dasics.map(entry => entry.cfg.valid && entry.boundMatch(addr)))
  }

  def dasics_mem_check(req: Valid[DasicsReqBundle], dasics: Vec[DasicsEntry]): Bool = {
    val inBoundVec = dasics_addr_bound(req.bits.addr, dasics)
    val boundMatchVec = dasics.zipWithIndex.map{case(entry, index)=>
      inBoundVec(index) && ( DasicsOp.isRead(req.bits.operation) &&  entry.cfg.r || DasicsOp.isWrite(req.bits.operation) && entry.cfg.w )
    }
    !boundMatchVec.reduce(_ || _) && req.bits.inUntrustedZone && req.valid
  }

  def dasics_jump_check(addr: UInt, inUntrustedZone: Bool, op: UInt, dasics: Vec[DasicsEntry]): Bool = {
    val inBoundVec = dasics_addr_bound(addr, dasics)
    val boundMatchVec = dasics.zipWithIndex.map { case (entry, index) =>
      inBoundVec(index) &&  DasicsOp.isJump(op) && entry.cfg.x
    }
    !boundMatchVec.reduce(_ || _) && inUntrustedZone
  }
}

class DasicsChecker(checkerConfig: Int)(implicit p: Parameters) extends XSModule
  with DasicsCheckerMethod
  with DasicsConst
  with HasCSRConst
{
  val io = IO(new DasicsCheckerIO)

  val req = io.req
  val dasics_entries = io.resource

  val dasics_mem_fault = RegNext(dasics_mem_check(req, dasics_entries), init = false.B)
  val dasics_jump_fault = false.B//dasics_jump_check(req.addr, req.inUntrustedZone, req.operation, dasics_entries)

  
  io.resp.dasics_fault := DasicsCheckFault.noDasicsFault 
  when(io.mode === ModeS){
    when(DasicsOp.isRead(req.bits.operation) && dasics_mem_fault){
      io.resp.dasics_fault := DasicsCheckFault.SReadDascisFault
    }.elsewhen(DasicsOp.isWrite(req.bits.operation) && dasics_mem_fault){
      io.resp.dasics_fault := DasicsCheckFault.SWriteDasicsFault
    }
  }.elsewhen(io.mode === ModeU){
    when(DasicsOp.isRead(req.bits.operation) && dasics_mem_fault){
      io.resp.dasics_fault := DasicsCheckFault.UReadDascisFault
    }.elsewhen(DasicsOp.isWrite(req.bits.operation) && dasics_mem_fault){
      io.resp.dasics_fault := DasicsCheckFault.UWriteDasicsFault
    }    
  }
    
}

class DasicsMainCfg(implicit p: Parameters) extends XSBundle {
  val uEnable, sEnable = Bool()

  private val UENA = 0x1
  private val SENA = 0x0

  def gen(reg: UInt): Unit = {
    this.uEnable := reg(UENA)
    this.sEnable := reg(SENA)
  }
}

class DasicsMainBound(implicit p: Parameters) extends XSBundle with DasicsConst {
  val boundHi, boundLo = UInt((VAddrBits - DasicsGrain).W)

  def getPcTags(startAddr: UInt): Vec[Bool] = {
    val startBlock = startAddr(VAddrBits - 1, DasicsGrain)
    val startOffset = startAddr(DasicsGrain - 1, 1) // instructions are 2-byte aligned

    // diff{Lo,Hi}: (VAddrBits, DasicsGrain) (with a sign bit)
    val diffLo = boundLo -& startBlock
    val diffHi = boundHi -& startBlock

    val fetchBlock = FetchWidth * 4
    val fetchGrain = log2Ceil(fetchBlock) // MinimalConfig: 4

    // detect edge cases
    val loClose = diffLo(VAddrBits - DasicsGrain, fetchGrain - DasicsGrain + 1) === 0.U
    val hiClose = diffHi(VAddrBits - DasicsGrain, fetchGrain - DasicsGrain + 1) === 0.U

    // get the low bits (fetchGrain, 0)
    // TODO: the following only works for MinimalConfig (FetchWidth = 4)
    val diffLoLSB = diffLo(fetchGrain - DasicsGrain, 0)
    val diffHiLSB = diffHi(fetchGrain - DasicsGrain, 0)
    val loBlockMask = (~0.U(3.W) << diffLoLSB)(2, 0).asBools
    val loCloseMask = (Cat(loBlockMask.map(Fill(4, _)).reverse) >> startOffset)(7, 0)
    val hiBlockMask = (Cat(0.U(3.W), ~0.U(3.W)) << diffHiLSB)(5, 3).asBools
    val hiCloseMask = (Cat(hiBlockMask.map(Fill(4, _)).reverse) >> startOffset)(7, 0)

    val loMask = Mux(diffLo(VAddrBits - DasicsGrain), Fill(8, 1.U(1.W)), Mux(loClose, loCloseMask, 0.U(8.W)))
    val hiMask = Mux(diffHi(VAddrBits - DasicsGrain), 0.U(8.W), Mux(hiClose, hiCloseMask, Fill(8, 1.U(1.W))))

    VecInit((loMask & hiMask).asBools)
  }

  // assign values (parameters are XLEN-length)
  def gen(boundLo: UInt, boundHi: UInt): Unit = {
    this.boundLo := boundLo(VAddrBits - 1, DasicsGrain)
    this.boundHi := boundHi(VAddrBits - 1, DasicsGrain)
  }
}

class DasicsTaggerIO(implicit p: Parameters) extends XSBundle {
  val distribute_csr: DistributedCSRIO = Flipped(new DistributedCSRIO())
  val privMode: UInt = Input(UInt(2.W))
  val addr: UInt = Input(UInt(VAddrBits.W))
  // TODO: change FetchWidth * 2 to PredictWidth, by accounting for non-C extension
  val notTrusted: Vec[Bool] = Output(Vec(FetchWidth * 2, Bool()))
}

// Tag every instruction as trusted/untrusted in frontend
class DasicsTagger(implicit p: Parameters) extends XSModule with HasCSRConst {
  val io: DasicsTaggerIO = IO(new DasicsTaggerIO())

  private val mainCfgReg = RegInit(UInt(XLEN.W), 0.U)
  private val sMainBoundHi = RegInit(UInt(XLEN.W), 0.U)
  private val sMainBoundLo = RegInit(UInt(XLEN.W), 0.U)
  private val uMainBoundHi = RegInit(UInt(XLEN.W), 0.U)
  private val uMainBoundLo = RegInit(UInt(XLEN.W), 0.U)

  private val mainCfg = Wire(new DasicsMainCfg())
  mainCfg.gen(mainCfgReg)
  private val mainBound = Wire(new DasicsMainBound())
  private val boundLo = Mux(io.privMode === ModeU, uMainBoundLo, sMainBoundLo)
  private val boundHi = Mux(io.privMode === ModeU, uMainBoundHi, sMainBoundHi)
  mainBound.gen(boundLo, boundHi)
  private val cmpTags = mainBound.getPcTags(io.addr)
  io.notTrusted := Mux(
    io.privMode === ModeU && mainCfg.uEnable || io.privMode === ModeS && mainCfg.sEnable,
    cmpTags,
    VecInit(Seq.fill(FetchWidth * 2)(false.B))
  )

  val w = io.distribute_csr.w
  val mapping: Map[Int, (UInt, UInt, UInt => UInt, UInt, UInt => UInt)] = Map(
    MaskedRegMap(DasicsSMainCfg, mainCfgReg, "hf".U(XLEN.W)),
    MaskedRegMap(DasicsSMainBoundLo, sMainBoundLo),
    MaskedRegMap(DasicsSMainBoundHi, sMainBoundHi),
    MaskedRegMap(DasicsUMainCfg, mainCfgReg, "h2".U(XLEN.W)),
    MaskedRegMap(DasicsUMainBoundLo, uMainBoundLo),
    MaskedRegMap(DasicsUMainBoundHi, uMainBoundHi)
  )
  val rdata: UInt = Wire(UInt(XLEN.W))
  MaskedRegMap.generate(mapping, w.bits.addr, rdata, w.valid, w.bits.data)
}
