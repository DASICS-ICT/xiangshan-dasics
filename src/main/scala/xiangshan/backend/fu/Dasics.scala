// See README.md for license details.

package xiangshan.backend.fu

import chisel3._
import chisel3.util._
import chipsalliance.rocketchip.config.Parameters
import utils._
import xiangshan._
import xiangshan.backend.fu.util.HasCSRConst

trait DasicsConst {
  val NumDasicsMemBounds  = 16  // For load/store
  val NumDasicsJumpBounds = 4   // For jal/jalr
  val DasicsFaultWidth    = 3 
  // 8 bytes of granularity
  val DasicsGrain         = 8
  val DasicsGrainBit      = log2Ceil(DasicsGrain)   
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

// priorty: S > U, Jump > Store > Load > ECall
object DasicsFaultReason {
    def noDasicsFault     = "b000".U

    def EcallDasicsFault = "b001".U
    def LoadDasicsFault  = "b010".U
    def StoreDasicsFault = "b011".U
    def JumpDasicsFault  = "b100".U

    // MPK > Dasics
    def LoadMPKFault   = "b101".U
    def StoreMPKFault  = "b110".U
}

// Dasics Config
abstract class DasicsConfig extends Bundle

class DasicsMemConfig extends DasicsConfig {
  val v: Bool = Bool()  // valid
  val u: Bool = Bool()  // unused
  val r: Bool = Bool()  // read
  val w: Bool = Bool()  // write
  
  def valid: Bool = v
  def write: Bool = w
  def read: Bool = r
}

class DasicsJumpConfig extends DasicsConfig {
  val v = Bool()

  def valid: Bool = v
}

object DasicsMemConfig  extends DasicsMemConfig
object DasicsJumpConfig extends DasicsJumpConfig

class DasicsEntry(implicit p: Parameters) extends XSBundle with DasicsConst {

  val cfg = new DasicsMemConfig
  val boundHi, boundLo = UInt(XLEN.W)

  // Lowest bits read/write as 0
  def boundRegMask: UInt = (~(DasicsGrain - 1).U(XLEN.W)).asUInt

  // Only check bounds, not checking permission
  // bounds are 8-byte aligned
  def boundMatch(addr: UInt): Bool = {
    val addrForComp = addr(VAddrBits - 1, DasicsGrainBit)
    (addrForComp >= boundLo(VAddrBits - 1, DasicsGrainBit)) && (addrForComp < boundHi(VAddrBits - 1, DasicsGrainBit))
  }

  // assign values (bounds parameter are XLEN-length)
  def gen(cfg: DasicsConfig, boundLo: UInt, boundHi: UInt): Unit = {
    this.cfg := cfg
    this.boundLo := Cat(boundLo(VAddrBits - 1, DasicsGrainBit),0.U(DasicsGrainBit.W))
    this.boundHi := Cat(boundHi(VAddrBits - 1, DasicsGrainBit),0.U(DasicsGrainBit.W))
  }
}

class DasicsJumpEntry(implicit p: Parameters) extends XSBundle with DasicsConst {

  val cfg = new DasicsJumpConfig
  val boundHi, boundLo = UInt(XLEN.W)

  // Lowest bits read/write as 0
  def boundRegMask: UInt = (~(DasicsGrain - 1).U(XLEN.W)).asUInt

  // Only check bounds, not checking permission
  // bounds are 8-byte aligned
  def boundMatch(addr: UInt): Bool = {
    val addrForComp = addr(VAddrBits - 1, DasicsGrainBit)
    (addrForComp >= boundLo(VAddrBits - 1, DasicsGrainBit)) && (addrForComp < boundHi(VAddrBits - 1, DasicsGrainBit))
  }

  // assign values (bounds parameter are XLEN-length)
  def gen(cfg: DasicsConfig, boundLo: UInt, boundHi: UInt): Unit = {
    this.cfg := cfg
    this.boundLo := Cat(boundLo(VAddrBits - 1, DasicsGrainBit),0.U(DasicsGrainBit.W))
    this.boundHi := Cat(boundHi(VAddrBits - 1, DasicsGrainBit),0.U(DasicsGrainBit.W))
  }
}

trait DasicsMethod extends DasicsConst { this: HasXSParameter =>
  def DasicsMemInit(): (Vec[UInt], Vec[UInt]) = {
    val DasicsMemCfgPerCSR = XLEN / DasicsMemConfig.getWidth //64/4=16
    val cfgs = WireInit(0.U.asTypeOf(Vec(NumDasicsMemBounds / DasicsMemCfgPerCSR, UInt(XLEN.W))))
    val bounds = WireInit(0.U.asTypeOf(Vec(NumDasicsMemBounds * 2, UInt(XLEN.W))))
    (cfgs, bounds)
  }

  def DasicsJumpInit(): (Vec[UInt], Vec[UInt]) = {
    val DasicsJumpCfgPerCSR = 4 
    val cfgs = WireInit(0.U.asTypeOf(Vec(NumDasicsJumpBounds / DasicsJumpCfgPerCSR, UInt(XLEN.W))))
    val bounds = WireInit(0.U.asTypeOf(Vec(NumDasicsJumpBounds * 2, UInt(XLEN.W))))
    (cfgs, bounds)
  }
  
  /* Dasics Memory Bound Register Mapping Generate */ 
  def DasicsGenMemMapping(
    mem_init: () => (Vec[UInt], Vec[UInt]),
    memNum: Int = NumDasicsMemBounds,
    memCfgBase: Int, memBoundBase: Int,
    memEntries: Vec[DasicsEntry],
  ): Map[Int, (UInt, UInt, UInt => UInt, UInt, UInt => UInt)] = {
    val DasicsMemCfgPerCSR = XLEN / DasicsMemConfig.getWidth
    def DasicsMemCfgIndex(i: Int) = i / DasicsMemCfgPerCSR
    // init_value: (cfgs, bounds)
    val mem_init_value = mem_init()

    // DasicsConfigs merged into CSR
    val mem_cfg_merged = RegInit(mem_init_value._1)
    val mem_cfgs = WireInit(mem_cfg_merged).asTypeOf(Vec(NumDasicsMemBounds, new DasicsMemConfig))
    val mem_bounds = RegInit(mem_init_value._2)

    // Wire entries to the registers
    for (i <- memEntries.indices) {
      memEntries(i).gen(mem_cfgs(i), boundLo = mem_bounds(i * 2), boundHi = mem_bounds(i * 2 + 1))
    }

    val mem_cfg_mapping = Map(
      (0 until memNum by DasicsMemCfgPerCSR).map(i =>
        MaskedRegMap(addr = memCfgBase + DasicsMemCfgIndex(i), reg = mem_cfg_merged(i / DasicsMemCfgPerCSR))
      ) : _*
    )

    val mem_bound_mapping = Map(
      (0 until memNum * 2).map(i => MaskedRegMap(
        addr = memBoundBase + i, reg = mem_bounds(i),
        wmask = DasicsEntry.boundRegMask, rmask = DasicsEntry.boundRegMask
      )) : _*
    )

    mem_cfg_mapping ++ mem_bound_mapping
  }

  /* Dasics Jump Bound Register Mapping Generate */
  def DasicsGenJumpMapping(
    jump_init: () => (Vec[UInt], Vec[UInt]), 
    jumpNum: Int = NumDasicsJumpBounds,
    jumpCfgBase: Int, jumpBoundBase: Int,
    jumpEntries: Vec[DasicsJumpEntry]
  ): Map[Int, (UInt, UInt, UInt => UInt, UInt, UInt => UInt)] = {
    val DasicsJumpCfgPerCSR = 4
    def DasicsJumpCfgIndex(i: Int) = i / DasicsJumpCfgPerCSR
    // init_value: (cfgs, bounds)
    val jump_init_value = jump_init()

    class DasicsJumpConfigExt extends DasicsConfig{
      val reserve = UInt((16 - DasicsJumpConfig.getWidth).W)
      val data = new DasicsJumpConfig
    } 

    // DasicsConfigs merged into CSR
    val jump_cfg_merged = RegInit(jump_init_value._1)
    val jump_cfgs = WireInit(jump_cfg_merged).asTypeOf(Vec(NumDasicsJumpBounds, new DasicsJumpConfigExt))
    val jump_bounds = RegInit(jump_init_value._2)

    // Wire entries to the registers
    for (i <- jumpEntries.indices) {
      jumpEntries(i).gen(jump_cfgs(i).data, boundLo = jump_bounds(i * 2), boundHi = jump_bounds(i * 2 + 1))
    }

    val jump_cfg_mapping = Map(
      (0 until jumpNum by DasicsJumpCfgPerCSR).map(i =>
        MaskedRegMap(addr = jumpCfgBase + DasicsJumpCfgIndex(i), reg = jump_cfg_merged(i / DasicsJumpCfgPerCSR))
      ) : _*
    )

    val jump_bound_mapping = Map(
      (0 until jumpNum * 2).map(i => MaskedRegMap(
        addr = jumpBoundBase + i, reg = jump_bounds(i),
        wmask = DasicsEntry.boundRegMask, rmask = DasicsEntry.boundRegMask
      )) : _*
    )

    jump_cfg_mapping ++ jump_bound_mapping
  }


  // Singleton companion object for DasicsEntry, with implicit parameters set
  private object DasicsEntry extends DasicsEntry
}

class DasicsMemIO(implicit p: Parameters) extends XSBundle with DasicsConst {
  val distribute_csr: DistributedCSRIO = Flipped(new DistributedCSRIO())
  val entries: Vec[DasicsEntry] = Output(Vec(NumDasicsMemBounds, new DasicsEntry))
  val mainCfg = Output(new DasicsMainCfg)
}
class DasicsReqBundle(implicit p: Parameters) extends XSBundle with DasicsConst {
  val addr = Output(UInt(VAddrBits.W))
  val inUntrustedZone = Output(Bool())
  val operation = Output(DasicsOp())
}

class DasicsRespBundle(implicit p: Parameters) extends XSBundle with DasicsConst{
  val dasics_fault = Output(UInt(DasicsFaultWidth.W))
  val mode = Output(UInt(2.W))
}

class DasicsRespDataBundle(implicit p: Parameters) extends XSBundle with DasicsConst{
  val dasics_fault = UInt(DasicsFaultWidth.W)
  val mode = UInt(2.W)
}

class DasicsMemCheckerIO(implicit p: Parameters) extends XSBundle with DasicsConst{
  val mode      = Input(UInt(2.W))
  val resource  = Flipped(Output(Vec(NumDasicsMemBounds, new DasicsEntry)))
  val mainCfg   = Input(new DasicsMainCfg)
  val req       = Flipped(Valid(new DasicsReqBundle()))
  val resp      = new DasicsRespBundle()

  //connect for every Dasics request
  def connect(addr:UInt, inUntrustedZone:Bool, operation: UInt, entries: Vec[DasicsEntry], mainCfg: DasicsMainCfg): Unit = {
    this.req.bits.addr := addr
    this.req.bits.inUntrustedZone := inUntrustedZone
    this.req.bits.operation := operation
    this.resource := entries
    this.mainCfg := mainCfg
  }
}

class MemDasics(implicit p: Parameters) extends XSModule with DasicsMethod with HasCSRConst {
  val io: DasicsMemIO = IO(new DasicsMemIO())

  val w = io.distribute_csr.w

  private val dasics = Wire(Vec(NumDasicsMemBounds, new DasicsEntry))
  val mapping = DasicsGenMemMapping(mem_init = DasicsMemInit, memCfgBase = DasicsLibCfgBase, memBoundBase = DasicsLibBoundBase, memEntries = dasics)

  private val dasics_main_cfg = RegInit(0.U(XLEN.W))
  private val mainCfg = Wire(new DasicsMainCfg())
  mainCfg.gen(dasics_main_cfg)

  val dasics_config_mapping = Map(
    MaskedRegMap(DasicsSMainCfg, dasics_main_cfg, "h3ff".U(XLEN.W)),
    MaskedRegMap(DasicsUMainCfg, dasics_main_cfg, "h3e".U(XLEN.W))
  )

  val rdata: UInt = Wire(UInt(XLEN.W))
  MaskedRegMap.generate(mapping ++ dasics_config_mapping, w.bits.addr, rdata, w.valid, w.bits.data)


  io.entries   := dasics
  io.mainCfg   := mainCfg
 }

trait DasicsCheckerMethod extends DasicsConst{
  //def dasics_check(addr:UInt, isUntrustedZone: Bool, op: UInt, Dasics: Vec[DasicsEntry]): Bool
  def dasics_mem_check(req: Valid[DasicsReqBundle], dasics: Vec[DasicsEntry]): Bool = {
    val inBoundVec = VecInit(dasics.map(entry => entry.cfg.valid && entry.boundMatch(req.bits.addr)))
    val boundMatchVec = dasics.zipWithIndex.map{case(entry, index)=>
      inBoundVec(index) && ( DasicsOp.isRead(req.bits.operation) &&  entry.cfg.r || DasicsOp.isWrite(req.bits.operation) && entry.cfg.w )
    }
    !boundMatchVec.reduce(_ || _) && req.bits.inUntrustedZone && req.valid
  }

  def dasics_jump_check(req: Valid[DasicsReqBundle], dasics: Vec[DasicsJumpEntry]): Bool = {
    val inBoundVec = VecInit(dasics.map(entry => entry.cfg.valid && entry.boundMatch(req.bits.addr)))
    val boundMatchVec = dasics.zipWithIndex.map{case(entry, index)=>
      inBoundVec(index) &&  DasicsOp.isJump(req.bits.operation)
    }
    !boundMatchVec.reduce(_ || _) && req.bits.inUntrustedZone && req.valid
  }
  def dasics_jump_check(addr: UInt, dasics: Vec[DasicsJumpEntry]): Bool = {
    val inBoundVec = VecInit(dasics.map(entry => entry.cfg.valid && entry.boundMatch(addr)))
    val boundMatchVec = dasics.zipWithIndex.map{case(entry, index)=>
      inBoundVec(index)
    }
    !boundMatchVec.reduce(_ || _)
  }
  def dasics_jump_in_bound(addr: UInt, boundHi:UInt, boundLo:UInt): Bool ={
    //warning VAddrBits may cause bug?
    val addrForComp = addr(addr.getWidth - 1, DasicsGrainBit)
    (addrForComp >= boundLo(boundLo.getWidth - 1, DasicsGrainBit)) && (addrForComp < boundHi(boundHi.getWidth - 1, DasicsGrainBit))
  }
}


class DasicsMemChecker(implicit p: Parameters) extends XSModule
  with DasicsCheckerMethod
  with DasicsConst
  with HasCSRConst
{
  val io = IO(new DasicsMemCheckerIO)

  val req = io.req
  val dasics_entries = io.resource

  val dasics_mem_fault = RegInit(false.B)
  val dasics_req_read  = RegInit(false.B)
  val dasics_req_write = RegInit(false.B)

  when(io.req.valid){
    dasics_mem_fault := dasics_mem_check(req, dasics_entries)
    dasics_req_read  := DasicsOp.isRead(req.bits.operation)
    dasics_req_write := DasicsOp.isWrite(req.bits.operation)
  }

  when(dasics_req_write && dasics_mem_fault && 
        ((io.mode === ModeU && io.mainCfg.uEnable && !io.mainCfg.closeUStoreFault) ||
        (io.mode === ModeS && io.mainCfg.sEnable && !io.mainCfg.closeSStoreFault))){
    io.resp.dasics_fault := DasicsFaultReason.StoreDasicsFault
  } 
  .elsewhen (dasics_req_read && dasics_mem_fault && 
        ((io.mode === ModeU && io.mainCfg.uEnable && !io.mainCfg.closeULoadFault) ||
        (io.mode === ModeS && io.mainCfg.sEnable && !io.mainCfg.closeSLoadFault))){
    io.resp.dasics_fault := DasicsFaultReason.LoadDasicsFault
  }.otherwise{
    io.resp.dasics_fault := DasicsFaultReason.noDasicsFault
  }
  io.resp.mode := io.mode
}

class DasicsBranchChecker(implicit p: Parameters) extends XSModule
  with DasicsMethod 
  with DasicsCheckerMethod 
  with HasCSRConst {
  
  val io: DasicsBranchIO = IO(new DasicsBranchIO())

  val w = io.distribute_csr.w

  private val dasics = Wire(Vec(NumDasicsJumpBounds, new DasicsJumpEntry))
  val mapping = DasicsGenJumpMapping(jump_init = DasicsJumpInit, jumpCfgBase = DasicsJmpCfgBase, jumpBoundBase = DasicsJmpBoundBase, jumpEntries = dasics)

  private val dasics_main_call = RegInit(0.U(XLEN.W))
  private val dasics_return_pc = RegInit(0.U(XLEN.W))
  private val dasics_azone_return_pc = RegInit(0.U(XLEN.W))
  private val dasics_main_cfg = RegInit(0.U(XLEN.W))
  private val dasics_smain_bound_hi = RegInit(0.U(XLEN.W))
  private val dasics_smain_bound_lo = RegInit(0.U(XLEN.W))
  private val dasics_umain_bound_hi = RegInit(0.U(XLEN.W))
  private val dasics_umain_bound_lo = RegInit(0.U(XLEN.W))

  val control_flow_mapping = Map(
    MaskedRegMap(DasicsMainCall, dasics_main_call),
    MaskedRegMap(DasicsReturnPc, dasics_return_pc),
    MaskedRegMap(DasicsActiveZoneReturnPc,dasics_azone_return_pc),
    MaskedRegMap(DasicsSMainCfg, dasics_main_cfg, "h3ff".U(XLEN.W)),
    MaskedRegMap(DasicsSMainBoundLo, dasics_smain_bound_lo),
    MaskedRegMap(DasicsSMainBoundHi, dasics_smain_bound_hi),
    MaskedRegMap(DasicsUMainCfg, dasics_main_cfg, "h3e".U(XLEN.W)),
    MaskedRegMap(DasicsUMainBoundLo, dasics_umain_bound_lo),
    MaskedRegMap(DasicsUMainBoundHi, dasics_umain_bound_hi)
  )

  val rdata: UInt = Wire(UInt(XLEN.W))
  MaskedRegMap.generate(mapping ++ control_flow_mapping, w.bits.addr, rdata, w.valid, w.bits.data)

  private val mainCfg = Wire(new DasicsMainCfg())
  mainCfg.gen(dasics_main_cfg)
  private val boundLo = Mux(io.mode === ModeS, dasics_smain_bound_lo, dasics_umain_bound_lo)
  private val boundHi = Mux(io.mode === ModeS, dasics_smain_bound_hi, dasics_umain_bound_hi)

  private val branchUntrusted = ((io.mode === ModeU && mainCfg.uEnable && !mainCfg.closeUJumpFault) || 
                                 (io.mode === ModeS && mainCfg.sEnable && !mainCfg.closeSJumpFault)) &&
                                !dasics_jump_in_bound(
                                addr = io.lastBranch, boundHi = boundHi(VAddrBits - 1, 0), boundLo = boundLo(VAddrBits - 1, 0)
                                )
  private val targetOutOfActive = dasics_jump_check(io.target, dasics)
  private val illegalBranch = io.valid && branchUntrusted && targetOutOfActive &&
    (io.target =/= dasics_return_pc) && (io.target =/= dasics_main_call) && (io.target =/= dasics_azone_return_pc)

  io.resp.dasics_fault := Mux(
    illegalBranch,
    DasicsFaultReason.JumpDasicsFault,
    DasicsFaultReason.noDasicsFault
  )
  io.resp.mode := io.mode
}
class DasicsMainCfg(implicit p: Parameters) extends XSBundle {
  val closeSJumpFault  = Bool()
  val closeSLoadFault   = Bool()
  val closeSStoreFault  = Bool()
  val closeSEcallFault  = Bool()
  
  val closeUJumpFault  = Bool()
  val closeULoadFault   = Bool()
  val closeUStoreFault  = Bool()
  val closeUEcallFault  = Bool()

  val uEnable = Bool()
  val sEnable = Bool()

  private val CSFT = 0x9
  private val CSLT = 0x8
  private val CSST = 0x7
  private val CSET = 0x6

  private val CUFT = 0x5
  private val CULT = 0x4
  private val CUST = 0x3
  private val CUET = 0x2

  private val UENA = 0x1
  private val SENA = 0x0

  def gen(reg: UInt): Unit = {
    this.closeSJumpFault  := reg(CSFT)
    this.closeSLoadFault  := reg(CSLT)
    this.closeSStoreFault := reg(CSST)
    this.closeSEcallFault := reg(CSET)

    this.closeUJumpFault  := reg(CUFT)
    this.closeULoadFault  := reg(CULT)
    this.closeUStoreFault := reg(CUST)
    this.closeUEcallFault := reg(CUET)

    this.uEnable := reg(UENA)
    this.sEnable := reg(SENA)
  }
}
class DasicsMainBound(implicit p: Parameters) extends XSBundle with DasicsConst {
  val boundHi, boundLo = UInt((VAddrBits - DasicsGrainBit).W)

  def getPcTags(startAddr: UInt): Vec[Bool] = {
    val startBlock = startAddr(VAddrBits - 1, DasicsGrainBit)
    val startOffset = startAddr(DasicsGrainBit - 1, 1) // instructions are 2-byte aligned

    // diff{Lo,Hi}: (VAddrBits, DasicsGrainBit) (with a sign bit)
    val diffLo = boundLo -& startBlock
    val diffHi = boundHi -& startBlock

    val fetchBlock = FetchWidth * 4
    val fetchGrain = log2Ceil(fetchBlock) // MinimalConfig: 4; DefConfig: 8
    val numDasicsBlocks = fetchBlock / DasicsGrain // MinConf: 2; DefConf: 4
    val instPerDasicsBlock = DasicsGrain / 2 // 4 compressed instructions per Dasics block

    // detect edge cases
    val loClose = diffLo(VAddrBits - DasicsGrainBit, fetchGrain - DasicsGrainBit + 1) === 0.U
    val hiClose = diffHi(VAddrBits - DasicsGrainBit, fetchGrain - DasicsGrainBit + 1) === 0.U

    // get the low bits (fetchGrain, 0)
    val diffLoLSB = diffLo(fetchGrain - DasicsGrainBit, 0)
    val diffHiLSB = diffHi(fetchGrain - DasicsGrainBit, 0)

    val maskGen = 0.U((numDasicsBlocks + 1).W) // MinConf: 000; DefConf: 00000
    val loBlockMask = (~maskGen << diffLoLSB)(numDasicsBlocks, 0).asBools
    val loCloseMask =
      (VecInit(loBlockMask.map(Fill(instPerDasicsBlock, _))).asUInt >> startOffset)(FetchWidth * 2 - 1, 0)
    val hiBlockMask = (~(Cat(~maskGen, maskGen) << diffHiLSB))(2 * numDasicsBlocks + 1, numDasicsBlocks + 1).asBools
    val hiCloseMask =
      (VecInit(hiBlockMask.map(Fill(instPerDasicsBlock, _))).asUInt >> startOffset)(FetchWidth * 2 - 1, 0)

    val loMask = Mux(
      diffLo(VAddrBits - DasicsGrainBit),
      Fill(FetchWidth * 2, 1.U(1.W)), // boundLo < startAddr
      Mux(loClose, loCloseMask, 0.U((FetchWidth * 2).W))
    )
    val hiMask = Mux(
      diffHi(VAddrBits - DasicsGrainBit),
      0.U((FetchWidth * 2).W),  // boundHi < startAddr
      Mux(hiClose, hiCloseMask, Fill(FetchWidth * 2, 1.U(1.W)))
    )

    VecInit((~(loMask & hiMask)).asBools) // tags mean untrusted, so revert them
  }

  // assign values (parameters are XLEN-length)
  def gen(boundLo: UInt, boundHi: UInt): Unit = {
    this.boundLo := boundLo(VAddrBits - 1, DasicsGrainBit)
    this.boundHi := boundHi(VAddrBits - 1, DasicsGrainBit)
  }
}

class DasicsBranchIO(implicit p: Parameters) extends XSBundle with DasicsConst {
  val distribute_csr: DistributedCSRIO = Flipped(new DistributedCSRIO())
  val mode: UInt = Input(UInt(2.W))
  val valid: Bool = Input(Bool())
  val lastBranch: UInt = Input(UInt(VAddrBits.W))
  val target: UInt = Input(UInt(VAddrBits.W))
  val resp = new DasicsRespBundle()
}

class DasicsTaggerIO(implicit p: Parameters) extends XSBundle {
  val distribute_csr: DistributedCSRIO = Flipped(new DistributedCSRIO())
  val mode: UInt = Input(UInt(2.W))
  val addr: UInt = Input(UInt(VAddrBits.W))
  // TODO: change FetchWidth * 2 to PredictWidth, by accounting for non-C extension
  val notTrusted: Vec[Bool] = Output(Vec(FetchWidth * 2, Bool()))
}

// Tag every instruction as trusted/untrusted in frontend
class DasicsTagger(implicit p: Parameters) extends XSModule with HasCSRConst {
  val io: DasicsTaggerIO = IO(new DasicsTaggerIO())

  private val dasics_main_cfg = RegInit(UInt(XLEN.W), 0.U)
  private val dasics_smain_bound_hi = RegInit(UInt(XLEN.W), 0.U)
  private val dasics_smain_bound_lo = RegInit(UInt(XLEN.W), 0.U)
  private val dasics_umain_bound_hi = RegInit(UInt(XLEN.W), 0.U)
  private val dasics_umain_bound_lo = RegInit(UInt(XLEN.W), 0.U)

  private val mainCfg = Wire(new DasicsMainCfg())
  mainCfg.gen(dasics_main_cfg)
  private val mainBound = Wire(new DasicsMainBound())
  private val boundLo = Mux(io.mode === ModeS,dasics_smain_bound_lo,dasics_umain_bound_lo)
  private val boundHi = Mux(io.mode === ModeS,dasics_smain_bound_hi,dasics_umain_bound_hi)
  mainBound.gen(boundLo, boundHi)
  private val cmpTags = mainBound.getPcTags(io.addr)
  io.notTrusted := Mux(
    io.mode === ModeU && mainCfg.uEnable || io.mode === ModeS && mainCfg.sEnable,
    cmpTags,
    VecInit(Seq.fill(FetchWidth * 2)(false.B))
  )

  val w = io.distribute_csr.w
  val mapping: Map[Int, (UInt, UInt, UInt => UInt, UInt, UInt => UInt)] = Map(
    MaskedRegMap(DasicsSMainCfg, dasics_main_cfg, "h3ff".U(XLEN.W)),
    MaskedRegMap(DasicsSMainBoundLo, dasics_smain_bound_lo),
    MaskedRegMap(DasicsSMainBoundHi, dasics_smain_bound_hi),
    MaskedRegMap(DasicsUMainCfg, dasics_main_cfg, "h3e".U(XLEN.W)),
    MaskedRegMap(DasicsUMainBoundLo, dasics_umain_bound_lo),
    MaskedRegMap(DasicsUMainBoundHi, dasics_umain_bound_hi)
  )
  val rdata: UInt = Wire(UInt(XLEN.W))
  MaskedRegMap.generate(mapping, w.bits.addr, rdata, w.valid, w.bits.data)
}
