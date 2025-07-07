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
    val labelZero = Bool() // label is zero if the upper 20 bits are all zero

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
    val needCheckLabel   = Vec(PredictWidth, Bool())
}

class SpecELPIO(implicit p: Parameters) extends XSBundle with HasCSRConst {
    val csrInfo       = new CSRInfo
    val instInfo      = Flipped(ValidIO(new InstInfo))
    val resp          = Output(new SpecELPResp)
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
    val pdInfoValid = io.instInfo.valid
    val inst_valid = io.instInfo.bits.inst_valid
    val hasValidInstr = inst_valid.reduce(_ || _)
    // jalr raising elp
    val shouldRaiseElpVec =(inst_valid zip io.instInfo.bits.predecodeInfo).map{
        case (valid, predecode) => 
            valid && predecode.isJalrForELP
    }
    val hasElpRaisingJalr = shouldRaiseElpVec.reduce(_ || _)
    val shouldRaiseElp  = zicfilp_enable && pdInfoValid && !spec_elp && hasElpRaisingJalr

    // lpad raising elp
    val firstValidInstrIdx = PriorityEncoder(inst_valid)
    val firstValidInstrInfo = io.instInfo.bits.predecodeInfo(firstValidInstrIdx) 
    val firstValidInstrIsValidLpad = hasValidInstr && firstValidInstrInfo.isValidLpad
    //By default, a currently valid lpad instruction is assumed to match its label, and a further label-matching check will be performed again at the execution stage.
    val shouldClearElp = pdInfoValid && hasValidInstr && spec_elp 

    // Elp state maintenance
    when(shouldClearElp) {
        spec_elp := false.B
    }.elsewhen(shouldRaiseElp) {
        spec_elp := true.B
    }

    io.resp.shouldRaiseElp := shouldRaiseElpVec
    io.resp.hasException   := VecInit.tabulate(PredictWidth) { i =>
        pdInfoValid && hasValidInstr && spec_elp && i.U === firstValidInstrIdx && !firstValidInstrIsValidLpad
    }
    io.resp.needCheckLabel := VecInit.tabulate(PredictWidth) { i =>
        shouldClearElp && i.U === firstValidInstrIdx && firstValidInstrInfo.needCheckLabel
    } 
}
class ZicfilpRespDataBundle(implicit p: Parameters) extends XSBundle{
    val shouldRaiseElp = Bool() // should raise arch_elp
    val needCheckLabel = Bool()
}