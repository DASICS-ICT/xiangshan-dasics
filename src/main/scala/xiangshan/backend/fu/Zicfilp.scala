package xiangshan.backend.fu

import chisel3._
import chisel3.util._
import chipsalliance.rocketchip.config.Parameters
import utils._
import xiangshan._
import xiangshan.backend.fu.util.HasCSRConst
import freechips.rocketchip.rocket.PRV.U


object ZicfilpTvalReason {
    def SoftwareCheckCode = "h2".U
}

class ZicfilpPreDecodeInfo extends Bundle{
    // jalr
    val isJalrForELP = Bool()
    // lpad
    val isLpad    = Bool()
    val pcAligned = Bool()
    val label     = UInt(20.W) // label for lpad
    
    def labelZero = label === 0.U // label is zero if the upper 20 bits are all zero
    def isValidLpad    = isLpad && pcAligned
    def needCheckLabel = isValidLpad && !labelZero
}


class CSRInfo(implicit p: Parameters) extends XSBundle with HasCSRConst {
    val distribut_csr = Flipped(new DistributedCSRIO)
    val cpu_mode      = Input(UInt(2.W))
}

class InstInfo(implicit p: Parameters) extends XSBundle {
    val inst_valid    = Vec(PredictWidth, Bool())
    val predecodeInfo = Vec(PredictWidth, new ZicfilpPreDecodeInfo)
}

class SpecELPResp(implicit p: Parameters) extends XSBundle {
    val hasException     = Vec(PredictWidth, Bool())
    val shouldRaiseElp   = Vec(PredictWidth, Bool()) // should raise arch_elp
    val shouldClearElp   = Vec(PredictWidth, Bool()) // should clear elp state (for lpad)
    val needCheckLabel   = Vec(PredictWidth, Bool())
}

class SpecELPIO(implicit p: Parameters) extends XSBundle with HasCSRConst {
    val flush         = Input(Bool())
    val csrInfo       = new CSRInfo
    val instInfo      = Flipped(ValidIO(new InstInfo))
    val resp          = ValidIO(new SpecELPResp)

    val arch_elp_sync = Input(Valid(Bool())) // sync elp state from Backend
}
class SpecELP(implicit p: Parameters) extends XSModule with HasCSRConst {
    val io = IO(new SpecELPIO)

    private val spec_elp = RegInit(false.B)

    //io.spec_elp := spec_elp

    private val mseccfg = RegInit(UInt(XLEN.W), 0.U)
    private val menvcfg = RegInit(UInt(XLEN.W), 0.U)
    private val senvcfg = RegInit(UInt(XLEN.W), 0.U)

    // CSR mapping
    val w = io.csrInfo.distribut_csr.w
    val envcfg_mapping: Map[Int, (UInt, UInt, UInt => UInt, UInt, UInt => UInt)] = Map(
      MaskedRegMap(Mseccfg, mseccfg, "h400".U(XLEN.W)),
      MaskedRegMap(Menvcfg, menvcfg, "h4".U(XLEN.W)),
      MaskedRegMap(Senvcfg, senvcfg, "h4".U(XLEN.W))
    )
    val envcfg_rdata: UInt = Wire(UInt(XLEN.W))
    MaskedRegMap.generate(envcfg_mapping, w.bits.addr, envcfg_rdata, w.valid, w.bits.data)

    val cpu_mode = io.csrInfo.cpu_mode

    val zicfilp_menable = cpu_mode === ModeM && mseccfg(10)
    val zicfilp_senable = cpu_mode === ModeS && menvcfg(2)
    val zicfilp_uenable = cpu_mode === ModeU && senvcfg(2)

    val zicfilp_enable = zicfilp_menable || zicfilp_senable || zicfilp_uenable

    // elp handler
    val instValidVec = io.instInfo.bits.inst_valid
    val instPdVec    = io.instInfo.bits.predecodeInfo
    val shouldRaiseElpVec = WireInit(VecInit((0 until PredictWidth).map(i => false.B)))
    val shouldClearElpVec = WireInit(VecInit((0 until PredictWidth).map(i => false.B)))
    val exceptionVec = WireInit(VecInit((0 until PredictWidth).map(i => false.B)))
    val needCheckLabelVec = WireInit(VecInit((0 until PredictWidth).map(i => false.B)))
    val elpInitState = spec_elp
    val finalElpState = (0 until PredictWidth).foldLeft(elpInitState) { case (elpState, i) =>
        val nextElpState = Mux(!instValidVec(i), elpState, 
                           Mux(!elpState, Mux(instPdVec(i).isJalrForELP, true.B, false.B), 
                           Mux(instPdVec(i).isValidLpad, false.B, true.B)))
        shouldRaiseElpVec(i) := instValidVec(i) && !elpState && instPdVec(i).isJalrForELP
        shouldClearElpVec(i) := instValidVec(i) && elpState && instPdVec(i).isValidLpad
        exceptionVec(i) := instValidVec(i) && elpState && !instPdVec(i).isValidLpad 
        needCheckLabelVec(i) := instValidVec(i) && elpState && instPdVec(i).needCheckLabel 
        nextElpState
    }

    when (io.arch_elp_sync.valid){
        spec_elp := io.arch_elp_sync.bits
    }
    .elsewhen (io.instInfo.valid && zicfilp_enable && !io.flush){
        spec_elp := finalElpState
    }

    io.resp.valid := io.instInfo.valid && zicfilp_enable
    io.resp.bits.hasException := exceptionVec
    io.resp.bits.shouldRaiseElp := shouldRaiseElpVec
    io.resp.bits.shouldClearElp := shouldClearElpVec
    io.resp.bits.needCheckLabel := needCheckLabelVec
}
class ZicfilpRespDataBundle(implicit p: Parameters) extends XSBundle{
    val shouldRaiseElp = Bool() // should raise arch_elp
    val shouldClearElp = Bool() // should clear arch_elp
    val needCheckLabel = Bool()
    val label          = UInt(20.W) // label for lpad
}

class ZicfilpLabelCheckIO(implicit p: Parameters) extends XSBundle {
    val needCheckLabel = Input(Bool())
    val label          = Input(UInt(20.W))
    val x7Label        = Input(UInt(20.W))
    val labelMatch     = Output(Bool())
}

class ZicfilpROBToCSRIO(implicit p: Parameters) extends XSBundle with HasCSRConst {
    val commitShouldRaiseElp = Vec(CommitWidth, Bool())
    val commitShouldClearElp = Vec(CommitWidth, Bool())
}

class ArchElpIO(implicit p: Parameters) extends XSBundle with HasCSRConst {
    val robToCsrZicfilpData    = Flipped(ValidIO(new ZicfilpROBToCSRIO))
    val xretRestore            = Input(Valid(Bool())) // restore elp state when xret
    val arch_elp_value         = Output(Bool())
}

class ArchElp(implicit p: Parameters) extends XSModule with HasCSRConst {
    val io = IO(new ArchElpIO)

    private val arch_elp = RegInit(false.B)

    val updateValid = io.robToCsrZicfilpData.valid
    val raiseVec = io.robToCsrZicfilpData.bits.commitShouldRaiseElp
    val clearVec = io.robToCsrZicfilpData.bits.commitShouldClearElp

    val finalElpState = (0 until CommitWidth).foldLeft(arch_elp) { case (currentElpState, i) =>
        Mux(raiseVec(i), true.B,
        Mux(clearVec(i), false.B, currentElpState))
    }

    when (io.xretRestore.valid){
        arch_elp := io.xretRestore.bits
    }.elsewhen (updateValid){
        arch_elp := finalElpState
    }
    io.arch_elp_value := arch_elp
}