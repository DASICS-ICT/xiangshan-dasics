// See README.md for license details.

package xiangshan.backend.fu

import chisel3._
import chisel3.util._
import chipsalliance.rocketchip.config.Parameters
import utils._
import xiangshan._
import xiangshan.backend.fu.util.HasCSRConst

trait DasicsConst {
  val NumDasicsMemBounds  = 32  // For load/store
  val NumDasicsJmpBounds  = 8   // For jal/jalr
  val DasicsFaultWidth    = 3 
  val DasicsCfgWidth      = 4 
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

class DasicsBound (implicit p: Parameters) extends XSBundle with DasicsConst{

  val cfg = UInt(DasicsCfgWidth.W)
  val offset = UInt((XLEN - DasicsCfgWidth - VAddrBits).W) // 2MB
  val base = UInt(VAddrBits.W)

  def gen(bound: UInt): Unit = {
    this.cfg := bound(XLEN - 1, XLEN - DasicsCfgWidth)
    this.offset := bound(XLEN - DasicsCfgWidth - 1, VAddrBits)
    this.base := bound(VAddrBits - 1, 0)
  }

  def toUInt(): UInt = {
    Cat(this.cfg,this.offset,this.base)
  }

  def RegMask: UInt = (~(DasicsGrain - 1).U(XLEN.W)).asUInt
}

class DasicsMemCfg extends Bundle {
  // v | u | r | w
  val v = Bool()
  val u = Bool()
  val r = Bool()
  val w = Bool()
}

class DasicsJmpCfg extends Bundle {
  // v | x | x | x
  val v = Bool()
  val padding = UInt(3.W)
}

class DasicsMainCfg extends Bundle {
  // e | j | l | s
  val e = Bool()
  val j = Bool()
  val l = Bool()
  val s = Bool()
  def v: Bool = e | j | l | s
}

class DasicsEntry(implicit p: Parameters) extends XSBundle with DasicsConst {

  val bound = new DasicsBound

  // Lowest bits read/write as 0


  // Only check bounds, not checking permission
  // bounds are 8-byte aligned
  def boundMatch(addr: UInt): Bool = {
    val addrForComp = addr(VAddrBits - 1, DasicsGrainBit)
    (addrForComp >= (bound.base)(VAddrBits - 1, DasicsGrainBit)) && (addrForComp < (bound.base + bound.offset)(VAddrBits - 1, DasicsGrainBit))
  }

  // assign values (bounds parameter are XLEN-length)
  def gen(bound: UInt): Unit = {
    this.bound.gen(Cat(bound(XLEN - 1, DasicsGrainBit),0.U(DasicsGrainBit.W)))
  }
}


trait DasicsMethod extends DasicsConst { this: HasXSParameter =>
  /* Dasics Bound Register Mapping Generate */ 
  def DasicsGenMapping(
    BoundNum: Int,
    BoundBase: Int,
    Entries: Vec[DasicsEntry],
  ): Map[Int, (UInt, UInt, UInt => UInt, UInt, UInt => UInt)] = {

    val init_value = WireInit(0.U.asTypeOf(Vec(BoundNum, UInt(XLEN.W))))
    val bounds = RegInit(init_value)

    // Wire entries to the registers
    for (i <- Entries.indices) {
      Entries(i).gen(bound = bounds(i))
    }

    val bound_mapping = Map(
      (0 until BoundNum).map(i => MaskedRegMap(
        addr = BoundBase + i, reg = bounds(i),
        wmask = DasicsBound.RegMask, rmask = DasicsBound.RegMask
      )) : _*
    )
    bound_mapping
  }
  
  private object DasicsBound extends DasicsBound
}

class DasicsMemConverterIO(implicit p: Parameters) extends XSBundle with DasicsConst {
  val distribute_csr: DistributedCSRIO = Flipped(new DistributedCSRIO())
  val entries: Vec[DasicsEntry] = Output(Vec(NumDasicsMemBounds, new DasicsEntry))
  val mode = Input(UInt(2.W))
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
  val mainCfg  = Flipped(Output(new DasicsMainCfg))
  val req       = Flipped(Valid(new DasicsReqBundle()))
  val resp      = new DasicsRespBundle()
}

class DasicsMemConverter(implicit p: Parameters) extends XSModule with DasicsMethod with HasCSRConst {
  val io: DasicsMemConverterIO = IO(new DasicsMemConverterIO())

  val w = io.distribute_csr.w

  private val dasics = Wire(Vec(NumDasicsMemBounds, new DasicsEntry))
  private val dasics_smain_bound_val = RegInit(0.U(XLEN.W))
  private val dasics_umain_bound_val = RegInit(0.U(XLEN.W))
  private val dasics_main_bound = Wire(new DasicsBound)

  val mapping = DasicsGenMapping(BoundNum = NumDasicsMemBounds, BoundBase = DasicsMemBoundBase, Entries = dasics)
  val control_flow_mapping = Map(
    MaskedRegMap(DasicsSMainBound, dasics_smain_bound_val, wmask=dasics_main_bound.RegMask, rmask=dasics_main_bound.RegMask),
    MaskedRegMap(DasicsUMainBound, dasics_umain_bound_val, wmask=dasics_main_bound.RegMask, rmask=dasics_main_bound.RegMask)
  )
  val rdata: UInt = Wire(UInt(XLEN.W)) // dummy
  MaskedRegMap.generate(mapping ++ control_flow_mapping, w.bits.addr, rdata, w.valid, w.bits.data)

  dasics_main_bound.gen(Mux(io.mode === ModeS, dasics_smain_bound_val, dasics_umain_bound_val))

  io.entries   := dasics
  io.mainCfg   := dasics_main_bound.cfg.asTypeOf(new DasicsMainCfg)
 }


trait DasicsCheckerMethod extends DasicsConst with HasXSParameter{
  //def dasics_check(addr:UInt, isUntrustedZone: Bool, op: UInt, Dasics: Vec[DasicsEntry]): Bool
  def dasics_mem_check(req: Valid[DasicsReqBundle], dasics: Vec[DasicsEntry]): Bool = {
    val inBoundVec = VecInit(dasics.map(entry => entry.bound.cfg.asTypeOf(new DasicsMemCfg).v && entry.boundMatch(req.bits.addr)))
    val boundMatchVec = dasics.zipWithIndex.map{case(entry, index)=>
      inBoundVec(index) && (DasicsOp.isRead(req.bits.operation) && entry.bound.cfg.asTypeOf(new DasicsMemCfg).r || DasicsOp.isWrite(req.bits.operation) && entry.bound.cfg.asTypeOf(new DasicsMemCfg).w)
    }
    !boundMatchVec.reduce(_ || _) && req.bits.inUntrustedZone && req.valid
  }
  def dasics_jump_check(addr: UInt, dasics: Vec[DasicsEntry]): Bool = {
    val inBoundVec = VecInit(dasics.map(entry => entry.bound.cfg.asTypeOf(new DasicsJmpCfg).v && entry.boundMatch(addr)))
    val boundMatchVec = dasics.zipWithIndex.map{case(entry, index)=>
      inBoundVec(index)
    }
    !boundMatchVec.reduce(_ || _)
  }
  def dasics_in_bound(addr: UInt, bound: DasicsBound): Bool = {
    val addrForComp = addr(VAddrBits - 1, DasicsGrainBit)
    (addrForComp >= (bound.base)(VAddrBits - 1, DasicsGrainBit)) && (addrForComp < (bound.base + bound.offset)(VAddrBits - 1, DasicsGrainBit))
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
        ((io.mode === ModeU || io.mode === ModeS) && io.mainCfg.s)){
    io.resp.dasics_fault := DasicsFaultReason.StoreDasicsFault
  } 
  .elsewhen (dasics_req_read && dasics_mem_fault && 
        ((io.mode === ModeU || io.mode === ModeS) && io.mainCfg.l)){
    io.resp.dasics_fault := DasicsFaultReason.LoadDasicsFault
  }.otherwise{
    io.resp.dasics_fault := DasicsFaultReason.noDasicsFault
  }
  io.resp.mode := io.mode
}

class DasicsJumpIO(implicit p: Parameters) extends XSBundle with DasicsConst {
  val distribute_csr: DistributedCSRIO = Flipped(new DistributedCSRIO())
  val mode: UInt = Input(UInt(2.W))
  val valid: Bool = Input(Bool())
  val lastJump: UInt = Input(UInt(VAddrBits.W))
  val target: UInt = Input(UInt(VAddrBits.W))
  val resp = new DasicsRespBundle()
}

class DasicsJumpChecker(implicit p: Parameters) extends XSModule
  with DasicsMethod 
  with DasicsCheckerMethod 
  with HasCSRConst {
  
  val io: DasicsJumpIO = IO(new DasicsJumpIO())

  val w = io.distribute_csr.w

  private val dasics = Wire(Vec(NumDasicsJmpBounds, new DasicsEntry))
  val mapping = DasicsGenMapping(BoundNum = NumDasicsJmpBounds, BoundBase = DasicsJmpBoundBase, Entries = dasics)

  private val dasics_main_call = RegInit(0.U(XLEN.W))
  private val dasics_return_pc = RegInit(0.U(XLEN.W))
  private val dasics_azone_return_pc = RegInit(0.U(XLEN.W))
  private val dasics_smain_bound_val = RegInit(0.U(XLEN.W))
  private val dasics_umain_bound_val = RegInit(0.U(XLEN.W))

  private val dasics_main_bound = Wire(new DasicsBound)

  val control_flow_mapping = Map(
    MaskedRegMap(DasicsMainCall, dasics_main_call),
    MaskedRegMap(DasicsReturnPc, dasics_return_pc),
    MaskedRegMap(DasicsActiveZoneReturnPc,dasics_azone_return_pc),
    MaskedRegMap(DasicsSMainBound, dasics_smain_bound_val, wmask=dasics_main_bound.RegMask, rmask=dasics_main_bound.RegMask),
    MaskedRegMap(DasicsUMainBound, dasics_umain_bound_val, wmask=dasics_main_bound.RegMask, rmask=dasics_main_bound.RegMask)
  )

  val rdata: UInt = Wire(UInt(XLEN.W))
  MaskedRegMap.generate(mapping ++ control_flow_mapping, w.bits.addr, rdata, w.valid, w.bits.data)


  dasics_main_bound.gen(Mux(io.mode === ModeS, dasics_smain_bound_val, dasics_umain_bound_val))

  private val dasics_main_cfg = dasics_main_bound.cfg.asTypeOf(new DasicsMainCfg)

  private val jumpUntrusted = ((io.mode === ModeU || io.mode === ModeS) && dasics_main_cfg.j) &&
                                  !dasics_in_bound(addr = io.lastJump, bound = dasics_main_bound)
                                
  private val targetOutOfActive = dasics_jump_check(io.target, dasics)
  private val illegalJump = io.valid && jumpUntrusted && targetOutOfActive &&
    (io.target =/= dasics_return_pc) && (io.target =/= dasics_main_call) && (io.target =/= dasics_azone_return_pc)

  io.resp.dasics_fault := Mux(
    illegalJump,
    DasicsFaultReason.JumpDasicsFault,
    DasicsFaultReason.noDasicsFault
  )
  io.resp.mode := io.mode
}

class DasicsTaggerIO(implicit p: Parameters) extends XSBundle {
  val distribute_csr: DistributedCSRIO = Flipped(new DistributedCSRIO())
  val mode: UInt = Input(UInt(2.W))
  val addr: UInt = Input(UInt(VAddrBits.W))
  // TODO: change FetchWidth * 2 to PredictWidth, by accounting for non-C extension
  val notTrusted: Vec[Bool] = Output(Vec(FetchWidth * 2, Bool()))
}

// Tag every instruction as trusted/untrusted in frontend
class DasicsTagger(implicit p: Parameters) extends XSModule with DasicsConst with HasCSRConst{
  val io: DasicsTaggerIO = IO(new DasicsTaggerIO())

  def getPcTags(startAddr: UInt, bound: DasicsBound): Vec[Bool] = {
    val startBlock = startAddr(VAddrBits - 1, DasicsGrainBit)
    val startOffset = startAddr(DasicsGrainBit - 1, 1) // instructions are 2-byte aligned

    val boundLo = bound.base(VAddrBits - 1, DasicsGrainBit)
    val boundHi = (bound.base + bound.offset)(VAddrBits - 1, DasicsGrainBit)

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

  private val dasics_smain_bound_val = RegInit(0.U(XLEN.W))
  private val dasics_umain_bound_val = RegInit(0.U(XLEN.W))
  private val dasics_main_bound = Wire(new DasicsBound)

  val w = io.distribute_csr.w
  val mapping: Map[Int, (UInt, UInt, UInt => UInt, UInt, UInt => UInt)] = Map(
    MaskedRegMap(DasicsSMainBound, dasics_smain_bound_val, wmask=dasics_main_bound.RegMask, rmask=dasics_main_bound.RegMask),
    MaskedRegMap(DasicsUMainBound, dasics_umain_bound_val, wmask=dasics_main_bound.RegMask, rmask=dasics_main_bound.RegMask)
  )
  val rdata: UInt = Wire(UInt(XLEN.W))
  MaskedRegMap.generate(mapping, w.bits.addr, rdata, w.valid, w.bits.data)

  dasics_main_bound.gen(Mux(io.mode === ModeS, dasics_smain_bound_val, dasics_umain_bound_val))

  private val dasics_main_cfg = dasics_main_bound.cfg.asTypeOf(new DasicsMainCfg)

  private val cmpTags = getPcTags(io.addr, dasics_main_bound)
  io.notTrusted := Mux(
    (io.mode === ModeU || io.mode === ModeS) && dasics_main_cfg.v,
    cmpTags,
    VecInit(Seq.fill(FetchWidth * 2)(false.B))
  )

}
