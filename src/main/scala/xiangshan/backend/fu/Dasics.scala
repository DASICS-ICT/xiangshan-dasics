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
  // 8 bytes of granularity
  val DasicsGrain         = 8
  val DasicsGrainBit      = log2Ceil(DasicsGrain)
  // 4 levels
  val DasicsMaxLevel = 4
  val DasicsLevelBit: Int = log2Ceil(DasicsMaxLevel)
  // scratchpad: a DasicsJumpEntry for editing
  val DasicsScratchpadIndex = 31
}

object DasicsConst extends DasicsConst

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

class DasicsControlFlow(implicit p: Parameters) extends XSBundle {
  val under_check = Flipped(ValidIO(new Bundle () {
    val mode = UInt(4.W)
    val pc = UInt(XLEN.W)
    val target = UInt(XLEN.W)
    val pc_in_trust_zone = Bool() 
  }))
  
  val check_result = Output(new Bundle () {
    val control_flow_legal = Bool()
  })
}

class DasicsEntry(implicit p: Parameters) extends XSBundle with DasicsConst {

  val cfg = new DasicsMemConfig
  val boundHi, boundLo = UInt(XLEN.W)
  val level: UInt = UInt(DasicsLevelBit.W)

  // Lowest bits read/write as 0
  def boundRegMask: UInt = (~(DasicsGrain - 1).U(XLEN.W)).asUInt

  // Only check bounds, not checking permission
  // bounds are 8-byte aligned
  def boundMatch(addr: UInt): Bool = {
    val addrForComp = addr(VAddrBits - 1, DasicsGrainBit)
    (addrForComp >= boundLo(VAddrBits - 1, DasicsGrainBit)) && (addrForComp < boundHi(VAddrBits - 1, DasicsGrainBit))
  }

  // assign values (bounds parameter are XLEN-length)
  def gen(cfg: DasicsConfig, boundLo: UInt, boundHi: UInt, level: UInt): Unit = {
    this.cfg := cfg
    this.boundLo := Cat(boundLo(VAddrBits - 1, DasicsGrainBit),0.U(DasicsGrainBit.W))
    this.boundHi := Cat(boundHi(VAddrBits - 1, DasicsGrainBit),0.U(DasicsGrainBit.W))
    this.level := level
  }
}

class DasicsJumpEntry(implicit p: Parameters) extends XSBundle with DasicsConst {

  val cfg = new DasicsJumpConfig
  val boundHi, boundLo = UInt(XLEN.W)
  val level: UInt = UInt(DasicsLevelBit.W)

  // Lowest bits read/write as 0
  def boundRegMask: UInt = (~(DasicsGrain - 1).U(XLEN.W)).asUInt

  // Only check bounds, not checking permission
  // bounds are 8-byte aligned
  def boundMatch(addr: UInt): Bool = {
    val addrForComp = addr(VAddrBits - 1, DasicsGrainBit)
    (addrForComp >= boundLo(VAddrBits - 1, DasicsGrainBit)) && (addrForComp < boundHi(VAddrBits - 1, DasicsGrainBit))
  }

  // match bound and output level one-hot code
  def boundMatchLevel(addr: UInt): Vec[Bool] = {
    VecInit((0 until DasicsMaxLevel).map(lv => Mux(lv.U === this.level, boundMatch(addr), false.B)))
  }

  // assign values (bounds parameter are XLEN-length)
  def gen(cfg: DasicsConfig, boundLo: UInt, boundHi: UInt, level: UInt): Unit = {
    this.cfg := cfg
    this.boundLo := Cat(boundLo(VAddrBits - 1, DasicsGrainBit),0.U(DasicsGrainBit.W))
    this.boundHi := Cat(boundHi(VAddrBits - 1, DasicsGrainBit),0.U(DasicsGrainBit.W))
    this.level := level
  }

  def gen(cfgReg: UInt, boundLo: UInt, boundHi: UInt, level: UInt): Unit = {
    val cfgWire = Wire(new DasicsJumpConfig)
    cfgWire.v := cfgReg(0)
    this.cfg := cfgWire
    this.boundLo := Cat(boundLo(VAddrBits - 1, DasicsGrainBit),0.U(DasicsGrainBit.W))
    this.boundHi := Cat(boundHi(VAddrBits - 1, DasicsGrainBit),0.U(DasicsGrainBit.W))
    this.level := level
  }

  // each level has a hit vec
  def getPcTags(startAddr: UInt): Vec[UInt] = {
    // use MainBound facility
    val mb = Wire(new DasicsMainBound)
    mb.gen(this.boundLo, this.boundHi)
    val pcHitVec = VecInit(mb.getPcTags(startAddr).map(!_)).asUInt  // main bound tag means miss, so revert them
    VecInit((0 until DasicsMaxLevel).map { level =>
      Mux(level.U === this.level, pcHitVec, 0.U((FetchWidth * 2).W))
    })
  }
}

class DasicsUntrustedRwStatus extends Bundle {
  val status: UInt = UInt(2.W)

  def isDeny: Bool = status === DasicsUntrustedRwStatus.deny
  def isRO: Bool = status === DasicsUntrustedRwStatus.ro
  def isRW: Bool = status === DasicsUntrustedRwStatus.rw
  def isEmpty: Bool = status === DasicsUntrustedRwStatus.empty
}

private object DasicsUntrustedRwStatus {
  val deny: UInt = 0.U(2.W)
  val ro: UInt = 1.U(2.W)
  val rw: UInt = 2.U(2.W)
  val empty: UInt = 3.U(2.W)
}

trait DasicsMethod extends DasicsConst { this: HasXSParameter =>
  def dasicsMemInit(): (Vec[UInt], Vec[UInt], Vec[UInt]) = {
    val dasicsMemCfgPerCSR = XLEN / DasicsMemConfig.getWidth
    val cfgs = WireInit(0.U.asTypeOf(Vec(NumDasicsMemBounds / dasicsMemCfgPerCSR, UInt(XLEN.W))))
    val bounds = WireInit(0.U.asTypeOf(Vec(NumDasicsMemBounds * 2, UInt(XLEN.W))))
    val levels = WireInit(0.U.asTypeOf(Vec(NumDasicsMemBounds, UInt(DasicsLevelBit.W))))
    (cfgs, bounds, levels)
  }

  def dasicsJumpInit(): (Vec[UInt], Vec[UInt], Vec[UInt]) = {
    val dasicsJumpCfgPerCSR = 4
    val cfgs = WireInit(0.U.asTypeOf(Vec(NumDasicsJumpBounds / dasicsJumpCfgPerCSR, UInt(XLEN.W))))
    val bounds = WireInit(0.U.asTypeOf(Vec(NumDasicsJumpBounds * 2, UInt(XLEN.W))))
    val levels = WireInit(0.U.asTypeOf(Vec(NumDasicsMemBounds, UInt(DasicsLevelBit.W))))
    (cfgs, bounds, levels)
  }

  /* Dasics Memory Bound Register Mapping Generate */
  def dasicsGenMemMapping(
    mem_init: () => (Vec[UInt], Vec[UInt], Vec[UInt]),
    memNum: Int = NumDasicsMemBounds,
    memCfgBase: Int, memBoundBase: Int,
    memEntries: Vec[DasicsEntry]
  ): (
    Map[Int, (UInt, UInt, UInt => UInt, UInt, UInt => UInt)], Map[Int, (UInt, UInt, UInt => UInt, UInt, UInt => UInt)]
  ) = {
    val dasicsMemCfgPerCSR = XLEN / DasicsMemConfig.getWidth
    def dasicsMemCfgIndex(i: Int) = i / dasicsMemCfgPerCSR
    // init_value: (cfgs, bounds, levels)
    val mem_init_value = mem_init()

    // DasicsConfigs merged into CSR
    val mem_cfg_merged = RegInit(mem_init_value._1)
    val mem_cfgs = WireInit(mem_cfg_merged).asTypeOf(Vec(memNum, new DasicsMemConfig))
    val mem_bounds = RegInit(mem_init_value._2)
    val mem_levels = RegInit(mem_init_value._3)

    // Wire entries to the registers
    for (i <- memEntries.indices) {
      memEntries(i).gen(mem_cfgs(i), boundLo = mem_bounds(i * 2), boundHi = mem_bounds(i * 2 + 1), mem_levels(i))
    }

    val mem_cfg_mapping = Map(
      (0 until memNum by dasicsMemCfgPerCSR).map(i =>
        MaskedRegMap(addr = memCfgBase + dasicsMemCfgIndex(i), reg = mem_cfg_merged(i / dasicsMemCfgPerCSR))
      ) : _*
    )

    val mem_bound_mapping = Map(
      (0 until memNum * 2).map(i => MaskedRegMap(
        addr = memBoundBase + i, reg = mem_bounds(i),
        wmask = DasicsEntry.boundRegMask, rmask = DasicsEntry.boundRegMask
      )) : _*
    )

    val level_private_mapping = Map((0 until memNum).map(i => MaskedRegMap(addr = i, reg = mem_levels(i))) : _*)

    (mem_cfg_mapping ++ mem_bound_mapping, level_private_mapping)
  }

  /* Dasics Jump Bound Register Mapping Generate */
  def dasicsGenJumpMapping(
    jump_init: () => (Vec[UInt], Vec[UInt], Vec[UInt]),
    jumpNum: Int = NumDasicsJumpBounds,
    jumpCfgBase: Int, jumpBoundBase: Int,
    jumpEntries: Vec[DasicsJumpEntry]
  ): (
    Map[Int, (UInt, UInt, UInt => UInt, UInt, UInt => UInt)], Map[Int, (UInt, UInt, UInt => UInt, UInt, UInt => UInt)]
  ) = {
    val dasicsJumpCfgPerCSR = 4
    def dasicsJumpCfgIndex(i: Int) = i / dasicsJumpCfgPerCSR
    // init_value: (cfgs, bounds, level)
    val jump_init_value = jump_init()

    class DasicsJumpConfigExt extends DasicsConfig{
      val reserve = UInt((16 - DasicsJumpConfig.getWidth).W)
      val data = new DasicsJumpConfig
    } 

    // DasicsConfigs merged into CSR
    val jump_cfg_merged = RegInit(jump_init_value._1)
    val jump_cfgs = WireInit(jump_cfg_merged).asTypeOf(Vec(jumpNum, new DasicsJumpConfigExt))
    val jump_bounds = RegInit(jump_init_value._2)
    val jump_levels = RegInit(jump_init_value._3)

    // Wire entries to the registers
    for (i <- jumpEntries.indices) {
      jumpEntries(i).gen(
        jump_cfgs(i).data, boundLo = jump_bounds(i * 2), boundHi = jump_bounds(i * 2 + 1), jump_levels(i)
      )
    }

    val jump_cfg_mapping = Map(
      (0 until jumpNum by dasicsJumpCfgPerCSR).map(i =>
        MaskedRegMap(addr = jumpCfgBase + dasicsJumpCfgIndex(i), reg = jump_cfg_merged(i / dasicsJumpCfgPerCSR))
      ) : _*
    )

    val jump_bound_mapping = Map(
      (0 until jumpNum * 2).map(i => MaskedRegMap(
        addr = jumpBoundBase + i, reg = jump_bounds(i),
        wmask = DasicsJumpEntry.boundRegMask, rmask = DasicsJumpEntry.boundRegMask
      )) : _*
    )

    val level_private_mapping = Map((0 until jumpNum).map(i => MaskedRegMap(addr = i, reg = jump_levels(i))) :_*)

    (jump_cfg_mapping ++ jump_bound_mapping, level_private_mapping)
  }

  def csrAddrInDasicsMemCfg(addr: UInt, memCfgBase: Int): Bool = {
    val dasicsMemCfgPerCSR = XLEN / DasicsMemConfig.getWidth
    val numCfgRegs = NumDasicsMemBounds / dasicsMemCfgPerCSR
    (addr >= memCfgBase.U) && (addr < (memCfgBase + numCfgRegs).U)
  }

  def csrAddrInDasicsMemBound(addr: UInt, memBoundBase: Int): Bool =
    (addr >= memBoundBase.U) && (addr < (memBoundBase + NumDasicsMemBounds * 2).U)

  def csrAddrInDasicsJmpCfg(addr: UInt, jmpCfgBase: Int): Bool = {
    val dasicsJumpCfgPerCSR = 4
    val numCfgRegs = NumDasicsJumpBounds / dasicsJumpCfgPerCSR
    (addr >= jmpCfgBase.U) && (addr < (jmpCfgBase + numCfgRegs).U)
  }

  def csrAddrInDasicsJmpBound(addr: UInt, jmpBoundBase: Int): Bool =
    (addr >= jmpBoundBase.U) && (addr < (jmpBoundBase + NumDasicsJumpBounds * 2).U)

  def csrAddrInDasicsScratchCfg(addr: UInt, scratchCfg: Int): Bool = addr === scratchCfg.U
  def csrAddrInDasicsScratchBound(addr: UInt, scratchBoundBase: Int): Bool =
    (addr >> 1).asUInt === (scratchBoundBase / 2).asUInt

  def csrAddrInUntrustedSpace(addr: UInt, memCfgBase: Int, memBoundBase: Int, jmpCfgBase: Int, jmpBoundBase: Int,
                              scratchCfg: Int, scratchBoundBase: Int
                             ): Bool = {
    csrAddrInDasicsMemCfg(addr, memCfgBase) || csrAddrInDasicsMemBound(addr, memBoundBase) ||
      csrAddrInDasicsJmpCfg(addr, jmpCfgBase) || csrAddrInDasicsJmpBound(addr, jmpBoundBase) ||
      csrAddrInDasicsScratchCfg(addr, scratchCfg) || csrAddrInDasicsScratchBound(addr, scratchBoundBase)
  }

  def csrAddrInDasicsRetPc(addr: UInt, retPcBase: Int): Bool =
    (addr >> DasicsLevelBit).asUInt === (retPcBase >> DasicsLevelBit).U

  def getDasicsUntrustedMemRwStatus(level: UInt, memEntries: Vec[DasicsEntry]): Vec[DasicsUntrustedRwStatus] =
    VecInit(memEntries.map { entry =>
      val durs = Wire(new DasicsUntrustedRwStatus)
      durs.status := Mux(entry.cfg.valid,
        Mux(level > entry.level,
          DasicsUntrustedRwStatus.deny,
          Mux(level === entry.level, DasicsUntrustedRwStatus.ro, DasicsUntrustedRwStatus.rw)
        ),
        DasicsUntrustedRwStatus.empty
      )
      durs
    })

  def getDasicsUntrustedJmpRwStatus(level: UInt, jmpEntries: Vec[DasicsJumpEntry]): Vec[DasicsUntrustedRwStatus] =
    VecInit(jmpEntries.map { entry =>
      val durs = Wire(new DasicsUntrustedRwStatus)
      durs.status := Mux(entry.cfg.valid,
        Mux(level > entry.level,
          DasicsUntrustedRwStatus.deny,
          Mux(level === entry.level, DasicsUntrustedRwStatus.ro, DasicsUntrustedRwStatus.rw)
        ),
        DasicsUntrustedRwStatus.empty
      )
      durs
    })

  def getDasicsUntrustedScratchRwStatus(level: UInt, scratch: DasicsJumpEntry): DasicsUntrustedRwStatus = {
    val durs = Wire(new DasicsUntrustedRwStatus)
    durs.status := Mux(scratch.cfg.valid,
      // only discriminate empty and rw
      Mux(level >= scratch.level, DasicsUntrustedRwStatus.empty, DasicsUntrustedRwStatus.rw),
      DasicsUntrustedRwStatus.empty
    )
    durs
  }

  object DasicsBndMvType {
    val mem = "b0".U
    val jmp = "b1".U
  }

  // Singleton companion object for DasicsEntry, with implicit parameters set
  private object DasicsEntry extends DasicsEntry
  private object DasicsJumpEntry extends DasicsJumpEntry
}

class DasicsMemIO(implicit p: Parameters) extends XSBundle with DasicsConst {
  val distribute_csr: DistributedCSRIO = Flipped(new DistributedCSRIO())
  val entries: Vec[DasicsEntry] = Output(Vec(NumDasicsMemBounds, new DasicsEntry))
}

class DasicsJumpIO(implicit p: Parameters) extends XSBundle with DasicsConst {
  val distribute_csr: DistributedCSRIO = Flipped(new DistributedCSRIO())
  val entries: Vec[DasicsJumpEntry] = Output(Vec(NumDasicsJumpBounds, new DasicsJumpEntry))
  val control_flow = new DasicsControlFlow
}

class DasicsReqBundle(implicit p: Parameters) extends XSBundle with DasicsConst {
  val addr = Output(UInt(VAddrBits.W))
  val inUntrustedZone = Output(Bool())
  val dasicsLevel = Output(UInt(DasicsLevelBit.W))
  val operation = Output(DasicsOp())
}

class DasicsRespBundle(implicit p: Parameters) extends XSBundle with DasicsConst{
  val dasics_fault = Output(DasicsCheckFault())
}

class DasicsMemCheckerIO(implicit p: Parameters) extends XSBundle with DasicsConst{
  val mode = Input(UInt(4.W))
  val resource = Flipped(Output(Vec(NumDasicsMemBounds, new DasicsEntry)))
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

class DasicsJumpCheckerIO(implicit p: Parameters) extends XSBundle with DasicsConst{
  val pc   = Input(UInt(VAddrBits.W))
  val mode = Input(UInt(4.W))
  val contro_flow = Flipped(new DasicsControlFlow)
  val req = Flipped(Valid(new DasicsReqBundle()))
  val resp = new DasicsRespBundle()

  //connect for every dasics request
  def connect(mode:UInt, pc: UInt, addr:UInt, inUntrustedZone:Bool, operation: UInt, contro_flow: DasicsControlFlow): Unit = {
    this.pc  := pc
    this.mode := mode
    this.req.bits.addr := addr
    this.req.bits.inUntrustedZone := inUntrustedZone
    this.req.bits.operation := operation
    this.contro_flow <> contro_flow
  }
}

class DasicsUntrustedRwCorrectorIO(implicit p: Parameters) extends XSBundle with DasicsMethod {
  val addr: UInt = Input(UInt(12.W))  // CSR register address
  val wdata: UInt = Input(UInt(XLEN.W)) // CSR write data
  val level: UInt = Input(UInt(DasicsLevelBit.W))
  val memEntries: Vec[DasicsEntry] = Input(Vec(NumDasicsMemBounds, new DasicsEntry()))
  val jmpEntries: Vec[DasicsJumpEntry] = Input(Vec(NumDasicsJumpBounds, new DasicsJumpEntry()))
  val scratch: DasicsJumpEntry = Input(new DasicsJumpEntry())
  val memRwStatus: Vec[DasicsUntrustedRwStatus] = Output(Vec(NumDasicsMemBounds, new DasicsUntrustedRwStatus))
  val jmpRwStatus: Vec[DasicsUntrustedRwStatus] = Output(Vec(NumDasicsJumpBounds, new DasicsUntrustedRwStatus))
  val scratchRwStatus: DasicsUntrustedRwStatus = Output(new DasicsUntrustedRwStatus)
  val rMask: UInt = Output(UInt(XLEN.W))
  val wdataFinal: UInt = Output(UInt(XLEN.W))
  val rAllowed: Bool = Output(Bool())
  val wAllowed: Bool = Output(Bool())

  def connectIn(addr: UInt, wdata: UInt, level: UInt, memEntries: Vec[DasicsEntry], jmpEntries: Vec[DasicsJumpEntry],
                scratch: DasicsJumpEntry): Unit = {
    this.addr := addr
    this.wdata := wdata
    this.level := level
    this.memEntries := memEntries
    this.jmpEntries := jmpEntries
    this.scratch := scratch
  }
}

// Instantiated in CSR
class DasicsUntrustedRwCorrector(implicit p: Parameters) extends XSModule with DasicsMethod with HasCSRConst {
  val io: DasicsUntrustedRwCorrectorIO = IO(new DasicsUntrustedRwCorrectorIO())

  val memRwStatus: Vec[DasicsUntrustedRwStatus] = getDasicsUntrustedMemRwStatus(io.level, io.memEntries)
  val jmpRwStatus: Vec[DasicsUntrustedRwStatus] = getDasicsUntrustedJmpRwStatus(io.level, io.jmpEntries)
  val scratchRwStatus = getDasicsUntrustedScratchRwStatus(io.level, io.scratch)

  val isMemCfg: Bool = csrAddrInDasicsMemCfg(io.addr, DasicsLibCfgBase)
  val isMemBound: Bool = csrAddrInDasicsMemBound(io.addr, DasicsLibBoundBase)
  val isJmpCfg: Bool = csrAddrInDasicsJmpCfg(io.addr, DasicsJmpCfgBase)
  val isJmpBound: Bool = csrAddrInDasicsJmpBound(io.addr, DasicsJmpBoundBase)
  val isScratchCfg = csrAddrInDasicsScratchCfg(io.addr, DasicsScratchCfg)
  val isScratchBound = csrAddrInDasicsScratchBound(io.addr, DasicsScratchBase)
  val memBoundOffset: UInt = (io.addr - DasicsLibBoundBase.U)(4, 0)
  val jmpBoundOffset: UInt = (io.addr - DasicsJmpBoundBase.U)(4, 0)
  val memEntry: DasicsEntry = io.memEntries(memBoundOffset(4, 1))

  val memCfgRMask: UInt = VecInit(
    memRwStatus.map(status => status.isRO || status.isRW).map(Fill(DasicsMemConfig.getWidth, _))
  ).asUInt
  val memCfgWBlockMask: UInt = VecInit(memRwStatus.map(_.isRW).map(Fill(DasicsMemConfig.getWidth, _))).asUInt
  val memCfgMerged: UInt = VecInit(io.memEntries.map(_.cfg)).asUInt
  val memCfgCanWrite: Bool = !(io.wdata & (~memCfgMerged).asUInt).orR
  val memCfgWdata = MaskData(memCfgMerged, io.wdata, memCfgWBlockMask)
  val memBoundPairStatus: DasicsUntrustedRwStatus = memRwStatus(memBoundOffset(4, 1))
  val memBoundCanRead: Bool = memBoundPairStatus.isRW || memBoundPairStatus.isRO
  val memBoundShrink: Bool = Mux(memBoundOffset(0), io.wdata <= memEntry.boundHi, io.wdata >= memEntry.boundLo)
  val memBoundCanWrite: Bool = memBoundPairStatus.isRW && memBoundShrink

  val jmpCfgRMask: UInt = VecInit(jmpRwStatus.map(status => status.isRO || status.isRW).map(Fill(16, _))).asUInt
  val jmpCfgWBlockMask: UInt = VecInit(jmpRwStatus.map(_.isRW).map(Fill(16, _))).asUInt
  val jmpCfgMerged: UInt = VecInit(io.jmpEntries.map(_.cfg.asUInt).map(ZeroExt(_, 16))).asUInt
  val jmpCfgCanWrite: Bool = !(io.wdata & (~jmpCfgMerged).asUInt).orR
  val jmpCfgWdata = MaskData(jmpCfgMerged, io.wdata, jmpCfgWBlockMask)
  val jmpBoundPairStatus: DasicsUntrustedRwStatus = jmpRwStatus(jmpBoundOffset(2, 1))
  val jmpBoundCanRead: Bool = jmpBoundPairStatus.isRO || jmpBoundPairStatus.isRW

  val scratchCfgCanRead = scratchRwStatus.isRW
  val scratchBoundCanRead = scratchRwStatus.isRW
  val scratchBoundShrink = Mux(io.addr(0), io.wdata <= io.scratch.boundHi, io.wdata >= io.scratch.boundLo)
  val scratchBoundCanWrite = scratchRwStatus.isRW && scratchBoundShrink

  io.memRwStatus := memRwStatus
  io.jmpRwStatus := jmpRwStatus
  io.scratchRwStatus := scratchRwStatus

  io.rMask := Mux(isMemCfg, memCfgRMask, Mux(isJmpCfg, jmpCfgRMask, Fill(XLEN, 1.U(1.W))))
  io.wdataFinal := Mux(isMemCfg, memCfgWdata, Mux(isJmpCfg, jmpCfgWdata, io.wdata))
  io.rAllowed := Mux(isMemBound,
    memBoundCanRead,
    Mux(isJmpBound, jmpBoundCanRead, Mux(isScratchCfg, scratchCfgCanRead, !isScratchBound || scratchCfgCanRead))
  )
  io.wAllowed := Mux(isMemCfg,
    memCfgCanWrite,
    Mux(isMemBound, memBoundCanWrite, Mux(isJmpCfg, jmpCfgCanWrite, isScratchBound && scratchBoundCanWrite))
  )
}

class DasicsBndMvCheckerIO(implicit p: Parameters) extends XSBundle with DasicsMethod {
  val src, dest = Input(UInt(XLEN.W))
  val bndType: UInt = Input(UInt(12.W))
  val memRwStatus: Vec[DasicsUntrustedRwStatus] = Input(Vec(NumDasicsMemBounds, new DasicsUntrustedRwStatus))
  val jmpRwStatus: Vec[DasicsUntrustedRwStatus] = Input(Vec(NumDasicsJumpBounds, new DasicsUntrustedRwStatus))
  val scratchRwStatus: DasicsUntrustedRwStatus = Input(new DasicsUntrustedRwStatus)
  val allowed: Bool = Output(Bool())
  val destIsMem, destIsJmp, destIsScratch, srcIsScratch = Output(Bool())

  def connectIn(src: UInt, dest: UInt, bndType: UInt, memRwStatus: Vec[DasicsUntrustedRwStatus],
                jmpRwStatus: Vec[DasicsUntrustedRwStatus], scratchRwStatus: DasicsUntrustedRwStatus): Unit = {
    this.src := src
    this.dest := dest
    this.bndType := bndType
    this.memRwStatus := memRwStatus
    this.jmpRwStatus := jmpRwStatus
    this.scratchRwStatus := scratchRwStatus
  }
}

// Instantiated in CSR
class DasicsBndMvChecker(implicit p: Parameters) extends XSModule with DasicsMethod {
  val io: DasicsBndMvCheckerIO = IO(new DasicsBndMvCheckerIO)

  val isMem: Bool = io.bndType === DasicsBndMvType.mem
  val isJmp: Bool = io.bndType === DasicsBndMvType.jmp
  val unknownType: Bool = !(isMem || isJmp)
  val memOutOfBounds: Bool = (io.src >= NumDasicsMemBounds.U) || (io.dest >= NumDasicsMemBounds.U)
  val jmpOutOfBounds: Bool = ((io.src >= NumDasicsJumpBounds.U) && (io.src =/= DasicsScratchpadIndex.U)) ||
    ((io.dest >= NumDasicsJumpBounds.U) && (io.dest =/= DasicsScratchpadIndex.U))
  val memSrcStatus = io.memRwStatus(io.src(3,0))
  val memDestStatus = io.memRwStatus(io.dest(3,0))
  val jmpSrcStatus = io.jmpRwStatus(io.src(1,0))
  val jmpDestStatus = io.jmpRwStatus(io.dest(1,0))
  val memCanAccess = (memSrcStatus.isRO || memSrcStatus.isRW) && (memDestStatus.isRW || memDestStatus.isEmpty)
  val srcIsScratch = isJmp && (io.src === DasicsScratchpadIndex.U)
  val jmpSrcCanRead = Mux(srcIsScratch, io.scratchRwStatus.isRW, jmpSrcStatus.isRO || jmpSrcStatus.isRW)
  val destIsScratch = isJmp && (io.dest === DasicsScratchpadIndex.U)
  val jmpDestCanAccess = (jmpDestStatus.isRW || jmpDestStatus.isEmpty) || destIsScratch
  val jmpCanAccess = jmpSrcCanRead && jmpDestCanAccess

  io.allowed := !unknownType && (
    (isMem && !memOutOfBounds && memCanAccess) || (isJmp && !jmpOutOfBounds && jmpCanAccess)
    )
  io.destIsMem := isMem
  io.destIsJmp := isJmp && !destIsScratch
  io.destIsScratch := destIsScratch
  io.srcIsScratch := srcIsScratch
}

class DasicsBndQueryIO(implicit p: Parameters) extends XSBundle with DasicsConst {
  val memStatus: Vec[DasicsUntrustedRwStatus] = Input(Vec(NumDasicsMemBounds, new DasicsUntrustedRwStatus))
  val jmpStatus: Vec[DasicsUntrustedRwStatus] = Input(Vec(NumDasicsJumpBounds, new DasicsUntrustedRwStatus))
  val bndType: UInt = Input(UInt(12.W))
  val out: UInt = Output(UInt(XLEN.W))
}

class DasicsBndQuery(implicit p: Parameters) extends XSModule with DasicsMethod {
  val io = IO(new DasicsBndQueryIO())

  val memResult = ZeroExt(io.memStatus.asUInt, XLEN)
  val jmpResult = ZeroExt(io.jmpStatus.asUInt, XLEN)

  io.out := LookupTreeDefault(io.bndType, 0.U, List(
    DasicsBndMvType.mem -> memResult,
    DasicsBndMvType.jmp -> jmpResult
  ))
}

class MemDasics(implicit p: Parameters) extends XSModule with DasicsMethod with HasCSRConst {
  val io: DasicsMemIO = IO(new DasicsMemIO())

  val w = io.distribute_csr.w
  private val levelW = io.distribute_csr.dasicsMemLevel
  private val levelWG = io.distribute_csr.dasicsMemLevelGlobal
  private val bmw = io.distribute_csr.dasicsMemBounds // DIBndMv

  private val dasics = Wire(Vec(NumDasicsMemBounds, new DasicsEntry))
  val (mapping, levelMapping) = dasicsGenMemMapping(
    mem_init = dasicsMemInit, memCfgBase = DasicsLibCfgBase, memBoundBase = DasicsLibBoundBase, memEntries = dasics
  )

  val rdata: UInt = Wire(UInt(XLEN.W))
  MaskedRegMap.generate(mapping, w.bits.addr, rdata, w.valid, w.bits.data)
  DasicsRegMap.levelGenerate(
    levelMapping, levelW.bits.addr, levelW.valid, levelW.bits.data, levelWG.valid, levelWG.bits
  )
  DasicsRegMap.memBoundsGenerate(
    mapping, bmw.bits.cfgAddr, bmw.bits.boundLoAddr, bmw.valid, bmw.bits.entry, bmw.bits.cfgData, bmw.bits.cfgMask
  )

  io.entries := dasics
}

/*
class JumpDasics(implicit p: Parameters) extends XSModule 
  with DasicsMethod 
  with DasicsCheckerMethod
  with HasCSRConst 
{
  val io: DasicsJumpIO = IO(new DasicsJumpIO())

  val w = io.distribute_csr.w

  private val dasics = Wire(Vec(NumDasicsJumpBounds, new DasicsJumpEntry))
  val mapping = dasicsGenJumpMapping(jump_init = dasicsJumpInit, jumpCfgBase = DasicsJmpCfgBase, jumpBoundBase = DasicsJmpBoundBase, jumpEntries = dasics)

  private val dasics_main_call = RegInit(0.U(XLEN.W))
  private val dasics_return_pc = RegInit(0.U(XLEN.W))
  private val dasics_azone_return_pc = RegInit(0.U(XLEN.W))
  private val dasics_main_cfg = RegInit(0.U(XLEN.W))
  private val sMainBoundHi = RegInit(0.U(XLEN.W))
  private val sMainBoundLo = RegInit(0.U(XLEN.W))
  private val uMainBoundHi = RegInit(0.U(XLEN.W))
  private val uMainBoundLo = RegInit(0.U(XLEN.W))


  val control_flow_mapping = Map(
    MaskedRegMap(DasicsMainCall, dasics_main_call),
    MaskedRegMap(DasicsReturnPc, dasics_return_pc),
    MaskedRegMap(DasicsActiveZoneReturnPC, dasics_azone_return_pc),
    MaskedRegMap(DasicsSMainCfg, dasics_main_cfg, "h3".U(XLEN.W)),
    MaskedRegMap(DasicsSMainBoundLo, sMainBoundLo),
    MaskedRegMap(DasicsSMainBoundHi, sMainBoundHi),
    MaskedRegMap(DasicsUMainCfg, dasics_main_cfg, "h2".U(XLEN.W)),
    MaskedRegMap(DasicsUMainBoundLo, uMainBoundLo),
    MaskedRegMap(DasicsUMainBoundHi, uMainBoundHi)
  )

  val rdata: UInt = Wire(UInt(XLEN.W))
  MaskedRegMap.generate(mapping ++ control_flow_mapping, w.bits.addr, rdata, w.valid, w.bits.data)

  io.entries := dasics

  //dasics jump checker control flow checking
  val (target, pc, mode) = (io.control_flow.under_check.bits.target, io.control_flow.under_check.bits.pc,io.control_flow.under_check.bits.mode)

  private val mainCfg = Wire(new DasicsMainCfg())
  mainCfg.gen(dasics_main_cfg)
  private val boundLo = Mux(mode === ModeU, uMainBoundLo, sMainBoundLo)
  private val boundHi = Mux(mode === ModeU, uMainBoundHi, sMainBoundHi)

  val isDasicsRet   = false.B   //TODO: add dasicst return instruction
  val isTrustedZone = io.control_flow.under_check.valid && io.control_flow.under_check.bits.pc_in_trust_zone
  val targetInTrustedZone = io.control_flow.under_check.valid && (mode === ModeU && mainCfg.uEnable || mode === ModeS && mainCfg.sEnable) &&
    dasics_jump_in_bound(addr = target(VAddrBits -1, 0), boundHi = boundHi(VAddrBits -1, 0), boundLo = boundLo(VAddrBits -1, 0))
  
  val targetInActiveZone  = io.control_flow.under_check.valid && !dasics_jump_check(target, dasics)
  val isActiveZone        = io.control_flow.under_check.valid && !dasics_jump_check(pc, dasics)

  val legalJumpTarget = isTrustedZone  || 
                        (!isTrustedZone &&  targetInTrustedZone && (target === dasics_return_pc || target === dasics_main_call)) ||
                        targetInActiveZone || 
                        ( isActiveZone && !targetInTrustedZone && !targetInActiveZone && target === dasics_azone_return_pc)

  io.control_flow.check_result.control_flow_legal := legalJumpTarget

}
*/

trait DasicsCheckerMethod extends DasicsConst{
  //def dasics_check(addr:UInt, isUntrustedZone: Bool, op: UInt, dasics: Vec[DasicsEntry]): Bool
  def dasics_mem_check(req: Valid[DasicsReqBundle], dasics: Vec[DasicsEntry]): Bool = {
    val inBoundVec = VecInit(dasics.map(entry => entry.cfg.valid && entry.boundMatch(req.bits.addr)))
    val boundMatchVec = dasics.zipWithIndex.map{case(entry, index)=>
      inBoundVec(index) && (req.bits.dasicsLevel === entry.level) && (
        DasicsOp.isRead(req.bits.operation) &&  entry.cfg.r || DasicsOp.isWrite(req.bits.operation) && entry.cfg.w
        )
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
  def dasics_jump_check(addr: UInt, level: UInt, dasics: Vec[DasicsJumpEntry]): Bool = {
    val inBoundVec = VecInit(dasics.map(entry => entry.cfg.valid && entry.boundMatch(addr) && (level === entry.level)))
    !ParallelOR(inBoundVec)
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

  val dasics_mem_fault = RegNext(dasics_mem_check(req, dasics_entries), init = false.B)
  
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

class DasicsJumpChecker(implicit p: Parameters) extends XSModule
  with DasicsCheckerMethod
  with DasicsConst
  with HasCSRConst
{
  val io = IO(new DasicsJumpCheckerIO)

  val req = io.req
  val dasics_contro_flow = io.contro_flow

  val dasics_jump_fault = req.valid && !dasics_contro_flow.check_result.control_flow_legal

  dasics_contro_flow.under_check.valid := req.valid
  dasics_contro_flow.under_check.bits.mode := io.mode
  dasics_contro_flow.under_check.bits.pc   := io.pc
  dasics_contro_flow.under_check.bits.pc_in_trust_zone := !io.req.bits.inUntrustedZone
  dasics_contro_flow.under_check.bits.target := req.bits.addr

  //dasics jump bound checking
  io.resp.dasics_fault := DasicsCheckFault.noDasicsFault 
  when(io.mode === ModeS){
    when(DasicsOp.isJump(req.bits.operation) && dasics_jump_fault){
      io.resp.dasics_fault := DasicsCheckFault.SJumpDasicsFault
    }
  }.elsewhen(io.mode === ModeU){
    when(DasicsOp.isJump(req.bits.operation) && dasics_jump_fault){
      io.resp.dasics_fault := DasicsCheckFault.UJumpDasicsFault
    }
  }    
}

class DasicsMainBoundBundle(implicit p: Parameters) extends XSBundle {
  val mainCfg: DasicsMainCfg = new DasicsMainCfg()
  val sMainBound: DasicsMainBound = new DasicsMainBound()
  val uMainBound: DasicsMainBound = new DasicsMainBound()
}

class DasicsControlFlowBundle(implicit p: Parameters) extends XSBundle with DasicsConst {
  val dasics_main_call: UInt = UInt(XLEN.W)
  val dasics_return_pc: Vec[UInt] = Vec(DasicsMaxLevel, UInt(XLEN.W))
  val dasics_azone_return_pc: UInt = UInt(XLEN.W)
}

class DasicsFrontendIO(implicit p: Parameters) extends XSBundle with DasicsConst {
  val distributed_csr: DistributedCSRIO = Flipped(new DistributedCSRIO())
  val jump_entries: Vec[DasicsJumpEntry] = Output(Vec(NumDasicsJumpBounds, new DasicsJumpEntry()))
  val main_bounds: DasicsMainBoundBundle = Output(new DasicsMainBoundBundle())
  val control_flow_regs: DasicsControlFlowBundle = Output(new DasicsControlFlowBundle)
}

// registers instantiated in frontend
class DasicsFrontend(implicit p: Parameters) extends XSModule with DasicsMethod with HasCSRConst {
  val io: DasicsFrontendIO = IO(new DasicsFrontendIO())
  val w = io.distributed_csr.w
  private val levelW = io.distributed_csr.dasicsJmpLevel
  private val levelWG = io.distributed_csr.dasicsJmpLevelGlobal
  private val bmw = io.distributed_csr.dasicsJmpBounds

  val dasics: Vec[DasicsJumpEntry] = Wire(Vec(NumDasicsJumpBounds, new DasicsJumpEntry()))
  val (mapping, levelMapping) = dasicsGenJumpMapping(
    jump_init = dasicsJumpInit, jumpCfgBase = DasicsJmpCfgBase, jumpBoundBase = DasicsJmpBoundBase, jumpEntries = dasics
  )
  private val dasics_main_cfg = RegInit(0.U(XLEN.W))
  private val sMainBoundHi = RegInit(0.U(XLEN.W))
  private val sMainBoundLo = RegInit(0.U(XLEN.W))
  private val uMainBoundHi = RegInit(0.U(XLEN.W))
  private val uMainBoundLo = RegInit(0.U(XLEN.W))
  private val dasics_main_call = RegInit(0.U(XLEN.W))
  private val dasics_return_pc = RegInit(0.U.asTypeOf(Vec(DasicsMaxLevel, UInt(XLEN.W))))
  private val dasics_azone_return_pc = RegInit(0.U(XLEN.W))
  val returnPcMapping = Map(
    (0 until DasicsMaxLevel).map(i => MaskedRegMap(DasicsReturnPcBase + i, dasics_return_pc(i))) : _*
  )
  private val control_flow_mapping = returnPcMapping ++ Map(
    MaskedRegMap(DasicsMainCall, dasics_main_call),
    MaskedRegMap(DasicsActiveZoneReturnPC, dasics_azone_return_pc),
    MaskedRegMap(DasicsSMainCfg, dasics_main_cfg, "h3".U(XLEN.W)),
    MaskedRegMap(DasicsSMainBoundLo, sMainBoundLo),
    MaskedRegMap(DasicsSMainBoundHi, sMainBoundHi),
    MaskedRegMap(DasicsUMainCfg, dasics_main_cfg, "h2".U(XLEN.W)),
    MaskedRegMap(DasicsUMainBoundLo, uMainBoundLo),
    MaskedRegMap(DasicsUMainBoundHi, uMainBoundHi)
  )
  private val rdata = Wire(UInt(XLEN.W))
  MaskedRegMap.generate(mapping ++ control_flow_mapping, w.bits.addr, rdata, w.valid, w.bits.data)
  DasicsRegMap.levelGenerate(
    levelMapping, levelW.bits.addr, levelW.valid, levelW.bits.data, levelWG.valid, levelWG.bits
  )
  DasicsRegMap.jmpBoundsGenerate(
    mapping, bmw.bits.cfgAddr, bmw.bits.boundLoAddr, bmw.valid, bmw.bits.entry, bmw.bits.cfgData, bmw.bits.cfgMask
  )
  private val mainCfg = Wire(new DasicsMainCfg())
  mainCfg.gen(dasics_main_cfg)
  private val sMainBound, uMainBound = Wire(new DasicsMainBound())
  sMainBound.gen(sMainBoundLo, sMainBoundHi)
  uMainBound.gen(uMainBoundLo, uMainBoundHi)

  io.jump_entries := dasics
  io.main_bounds.mainCfg := mainCfg
  io.main_bounds.sMainBound := sMainBound
  io.main_bounds.uMainBound := uMainBound
  io.control_flow_regs.dasics_return_pc := dasics_return_pc
  io.control_flow_regs.dasics_main_call := dasics_main_call
  io.control_flow_regs.dasics_azone_return_pc := dasics_azone_return_pc
}

class DasicsBranchIO(implicit p: Parameters) extends XSBundle with DasicsConst {
  val jump_entries: Vec[DasicsJumpEntry] = Input(Vec(NumDasicsJumpBounds, new DasicsJumpEntry()))
  val main_bounds: DasicsMainBoundBundle = Input(new DasicsMainBoundBundle())
  val control_flow_regs: DasicsControlFlowBundle = Input(new DasicsControlFlowBundle())
  val mode: UInt = Input(UInt(2.W))
  val valid: Bool = Input(Bool())
  val lastBranch: UInt = Input(UInt(VAddrBits.W))
  val target: UInt = Input(UInt(VAddrBits.W))
  // synchronize pipeline with IF1 -> IF2
  val f1_fire: Bool = Input(Bool())
  // return resp after a clock cycle
  val s2_resp = new DasicsRespBundle()
}

class DasicsBranchChecker(implicit p: Parameters) extends XSModule
  with DasicsMethod with DasicsCheckerMethod with HasCSRConst {
  val io: DasicsBranchIO = IO(new DasicsBranchIO())
  val dasics: Vec[DasicsJumpEntry] = io.jump_entries
  val (dasics_main_call, dasics_return_pc, dasics_azone_return_pc) = (
    io.control_flow_regs.dasics_main_call,
    io.control_flow_regs.dasics_return_pc,
    io.control_flow_regs.dasics_azone_return_pc
  )

  private val mainCfg = io.main_bounds.mainCfg
  private val mainBound = Mux(io.mode === ModeU, io.main_bounds.uMainBound, io.main_bounds.sMainBound)
  private val boundLo = Cat(mainBound.boundLo, 0.U(DasicsGrainBit.W))
  private val boundHi = Cat(mainBound.boundHi, 0.U(DasicsGrainBit.W))

  private val branchUntrusted = (io.mode === ModeU && mainCfg.uEnable || io.mode === ModeS && mainCfg.sEnable) &&
    !dasics_jump_in_bound(
      addr = io.lastBranch, boundHi = boundHi(VAddrBits - 1, 0), boundLo = boundLo(VAddrBits - 1, 0)
    )
  val brEntryLevelHits = dasics.map(_.boundMatchLevel(io.lastBranch))
  val brLevelHits = (0 until DasicsMaxLevel).map { level =>
    ParallelOR(brEntryLevelHits.map(oh => oh(level)))
  }.zipWithIndex.reverse
  val (brLevel, brHit) = PriorityMuxWithFlag(brLevelHits.map { case (hit, level) => (hit, level.U) })

  // pipeline: check in IF2
  private val f1_fire = io.f1_fire
  val s2_valid: Bool = RegEnable(next = io.valid, init = false.B, enable = f1_fire)
  val s2_target: UInt = RegEnable(next = io.target, enable = f1_fire)
  val s2_brUntrusted: Bool = RegEnable(next = branchUntrusted, enable = f1_fire)
  val s2_brLevel: UInt = RegEnable(next = brLevel, enable = f1_fire)

  private val s2_targetOutOfActive = dasics_jump_check(s2_target, s2_brLevel, dasics)
  private val s2_illegalBranch = s2_valid && s2_brUntrusted && s2_targetOutOfActive &&
    (s2_target =/= dasics_return_pc(s2_brLevel)) && (s2_target =/= dasics_main_call) &&
    (s2_target =/= dasics_azone_return_pc)
  io.s2_resp.dasics_fault := Mux(
    s2_illegalBranch,
    Mux(io.mode === ModeU, DasicsCheckFault.UJumpDasicsFault, DasicsCheckFault.SJumpDasicsFault),
    DasicsCheckFault.noDasicsFault
  )
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
    val hiBlockMask = (Cat(maskGen, ~maskGen) << diffHiLSB)(2 * numDasicsBlocks + 1, numDasicsBlocks + 1).asBools
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

class DasicsTaggerIO(implicit p: Parameters) extends XSBundle with DasicsConst {
  val jmp_entries: Vec[DasicsJumpEntry] = Input(Vec(NumDasicsJumpBounds, new DasicsJumpEntry()))
  val main_bounds: DasicsMainBoundBundle = Input(new DasicsMainBoundBundle())
  val privMode: UInt = Input(UInt(2.W))
  val addr: UInt = Input(UInt(VAddrBits.W))
  val notTrusted: Vec[Bool] = Output(Vec(FetchWidth * 2, Bool()))
  val levelTags: Vec[UInt] = Output(Vec(FetchWidth * 2, UInt(DasicsLevelBit.W)))
  val levelMiss: Vec[Bool] = Output(Vec(FetchWidth * 2, Bool()))
}

// Tag every instruction as trusted/untrusted in frontend
class DasicsTagger(implicit p: Parameters) extends XSModule with DasicsConst with HasCSRConst {
  val io: DasicsTaggerIO = IO(new DasicsTaggerIO())

  private val mainCfg = io.main_bounds.mainCfg
  private val mainBound = Mux(io.privMode === ModeU, io.main_bounds.uMainBound, io.main_bounds.sMainBound)
  private val cmpTags = mainBound.getPcTags(io.addr)

  // for determining levels
  private val entriesLevelsHitVec = io.jmp_entries.map(_.getPcTags(io.addr))
  // transpose and get OR'ed tags for each level
  private val levelsHitVec = (0 until DasicsMaxLevel).map { level =>
    ParallelOR(entriesLevelsHitVec.map(hitVecs => hitVecs(level))).asBools
  }.zipWithIndex.reverse
  private val levelTagsWithFlag = (0 until FetchWidth * 2).map { index =>
    PriorityMuxWithFlag(levelsHitVec.map { case (hitVec, level) => (hitVec(index), level.U(DasicsLevelBit.W)) })
  }

  io.notTrusted := Mux(
    io.privMode === ModeU && mainCfg.uEnable || io.privMode === ModeS && mainCfg.sEnable,
    cmpTags,
    VecInit(Seq.fill(FetchWidth * 2)(false.B))
  )
  io.levelTags := VecInit(levelTagsWithFlag.map(_._1))
  io.levelMiss := VecInit(levelTagsWithFlag.map(!_._2))
}

object DasicsRegMap extends DasicsConst {
  def levelGenerate(mapping: Map[Int, (UInt, UInt, UInt => UInt, UInt, UInt => UInt)],
                    waddr: UInt, wen: Bool, wdata: UInt, globalWen: Bool, globalWdata: UInt): Unit = {
    val chiselMapping = mapping.map { case (a, (r, _, _, _, _)) => (a, a.U, r) }
    val wdata_reg = RegEnable(wdata, wen)
    val globalWdata_reg = RegEnable(globalWdata, globalWen)
    val globalWen_reg = RegNext(globalWen)
    chiselMapping.foreach { case (litA, a, r) =>
      val wen_reg = RegNext(wen && waddr === a)
      when (wen_reg) { r := wdata_reg }
      when (globalWen_reg) { r := globalWdata_reg((litA + 1) * DasicsLevelBit - 1, litA * DasicsLevelBit) }
    }
  }

  def memBoundsGenerate(mapping: Map[Int, (UInt, UInt, UInt => UInt, UInt, UInt => UInt)], cfgAddr: UInt,
                        bndLoAddr: UInt, wen: Bool, wdata: DasicsEntry, cfgData: UInt, cfgMask: UInt): Unit = {
    val chiselMapping = mapping.map { case (a, (r, wm, _, _, _)) => (a.U, r, wm) }
    val wdata_reg = RegEnable(wdata, wen)
    val cfgData_reg = RegEnable(cfgData, wen)
    val cfgMask_reg = RegEnable(cfgMask, wen)
    chiselMapping.foreach { case (a, r, wm) =>
      if (wm != MaskedRegMap.UnwritableMask) {
        val cfgWen_reg = RegNext(wen && cfgAddr === a)
        val bndLoWen_reg = RegNext(wen && bndLoAddr === a)
        val bndHiWen_reg = RegNext(wen && ((bndLoAddr | 1.U) === a))
        when (cfgWen_reg) { r := MaskData(r, cfgData_reg, cfgMask_reg) }
        when (bndLoWen_reg) { r := MaskData(r, wdata_reg.boundLo, wm) }
        when (bndHiWen_reg) { r := MaskData(r, wdata_reg.boundHi, wm) }
      }
    }
  }

  def jmpBoundsGenerate(mapping: Map[Int, (UInt, UInt, UInt => UInt, UInt, UInt => UInt)], cfgAddr: UInt,
                        bndLoAddr: UInt, wen: Bool, wdata: DasicsJumpEntry, cfgData: UInt, cfgMask: UInt): Unit = {
    val chiselMapping = mapping.map { case (a, (r, wm, _, _, _)) => (a.U, r, wm) }
    val wdata_reg = RegEnable(wdata, wen)
    val cfgData_reg = RegEnable(cfgData, wen)
    val cfgMask_reg = RegEnable(cfgMask, wen)
    chiselMapping.foreach { case (a, r, wm) =>
      if (wm != MaskedRegMap.UnwritableMask) {
        val cfgWen_reg = RegNext(wen && cfgAddr === a)
        val bndLoWen_reg = RegNext(wen && bndLoAddr === a)
        val bndHiWen_reg = RegNext(wen && ((bndLoAddr | 1.U) === a))
        when (cfgWen_reg) { r := MaskData(r, cfgData_reg, cfgMask_reg) }
        when (bndLoWen_reg) { r := MaskData(r, wdata_reg.boundLo, wm) }
        when (bndHiWen_reg) { r := MaskData(r, wdata_reg.boundHi, wm) }
      }
    }
  }

  def scratchMvGenerate(cfgReg: UInt, bndLoReg: UInt, bndHiReg: UInt, levelReg: UInt,
                        wen: Bool, wdata: DasicsJumpEntry, wlevel: UInt): Unit = {
    val wdata_reg = RegEnable(wdata, wen)
    val wlevel_reg = RegEnable(wlevel, wen)
    val wen_reg = RegNext(wen)
    when (wen_reg) {
      cfgReg := wdata_reg.cfg.asUInt
      bndLoReg := wdata_reg.boundLo
      bndHiReg := wdata_reg.boundHi
      levelReg := wlevel_reg
    }
  }
}