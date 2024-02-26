// See README.md for license details.

package xiangshan.backend.fu

import chisel3._
import chisel3.util._
import chipsalliance.rocketchip.config.Parameters
import utils._
import xiangshan._
import xiangshan.backend.fu.util.HasCSRConst

trait DasicsConst {
  val NumDasicsMemBounds  = 4  // For load/store
  val NumDasicsJumpBounds = 1   // For jal/jalr
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
  def dasicsMemInit(): (Vec[UInt], Vec[UInt]) = {
    val dasicsMemCfgPerCSR = XLEN / DasicsMemConfig.getWidth
    val cfgs = WireInit(0.U.asTypeOf(Vec(if (NumDasicsMemBounds < dasicsMemCfgPerCSR) 1 else NumDasicsMemBounds / dasicsMemCfgPerCSR, UInt(XLEN.W))))
    val bounds = WireInit(0.U.asTypeOf(Vec(NumDasicsMemBounds * 2, UInt(XLEN.W))))
    (cfgs, bounds)
  }

  def dasicsJumpInit(): (Vec[UInt], Vec[UInt]) = {
    val dasicsJumpCfgPerCSR = 4 
    val cfgs = WireInit(0.U.asTypeOf(Vec(if (NumDasicsJumpBounds < dasicsJumpCfgPerCSR) 1 else NumDasicsJumpBounds / dasicsJumpCfgPerCSR, UInt(XLEN.W))))
    val bounds = WireInit(0.U.asTypeOf(Vec(NumDasicsJumpBounds * 2, UInt(XLEN.W))))
    (cfgs, bounds)
  }
  
  /* Dasics Memory Bound Register Mapping Generate */ 
  def dasicsGenMemMapping(
    mem_init: () => (Vec[UInt], Vec[UInt]),
    memNum: Int = NumDasicsMemBounds,
    memCfgBase: Int, memBoundBase: Int,
    memEntries: Vec[DasicsEntry],
  ): Map[Int, (UInt, UInt, UInt => UInt, UInt, UInt => UInt)] = {
    val dasicsMemCfgPerCSR = XLEN / DasicsMemConfig.getWidth
    def dasicsMemCfgIndex(i: Int) = i / dasicsMemCfgPerCSR
    // init_value: (cfgs, bounds)
    val mem_init_value = mem_init()

    // DasicsConfigs merged into CSR
    val mem_cfg_merged = RegInit(mem_init_value._1)
    val mem_cfgs = WireInit(mem_cfg_merged).asTypeOf(Vec(memNum, new DasicsMemConfig))
    val mem_bounds = RegInit(mem_init_value._2)

    // Wire entries to the registers
    for (i <- memEntries.indices) {
      memEntries(i).gen(mem_cfgs(i), boundLo = mem_bounds(i * 2), boundHi = mem_bounds(i * 2 + 1))
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

    mem_cfg_mapping ++ mem_bound_mapping
  }

  /* Dasics Jump Bound Register Mapping Generate */
  def dasicsGenJumpMapping(
    jump_init: () => (Vec[UInt], Vec[UInt]), 
    jumpNum: Int = NumDasicsJumpBounds,
    jumpCfgBase: Int, jumpBoundBase: Int,
    jumpEntries: Vec[DasicsJumpEntry]
  ): Map[Int, (UInt, UInt, UInt => UInt, UInt, UInt => UInt)] = {
    val dasicsJumpCfgPerCSR = 4
    def dasicsJumpCfgIndex(i: Int) = i / dasicsJumpCfgPerCSR
    // init_value: (cfgs, bounds)
    val jump_init_value = jump_init()

    class DasicsJumpConfigExt extends DasicsConfig{
      val reserve = UInt((16 - DasicsJumpConfig.getWidth).W)
      val data = new DasicsJumpConfig
    } 

    // DasicsConfigs merged into CSR
    val jump_cfg_merged = RegInit(jump_init_value._1)
    val jump_cfgs = WireInit(jump_cfg_merged).asTypeOf(Vec(jumpNum, new DasicsJumpConfigExt))
    val jump_bounds = RegInit(jump_init_value._2)

    // Wire entries to the registers
    for (i <- jumpEntries.indices) {
      jumpEntries(i).gen(jump_cfgs(i).data, boundLo = jump_bounds(i * 2), boundHi = jump_bounds(i * 2 + 1))
    }

    val jump_cfg_mapping = Map(
      (0 until jumpNum by dasicsJumpCfgPerCSR).map(i =>
        MaskedRegMap(addr = jumpCfgBase + dasicsJumpCfgIndex(i), reg = jump_cfg_merged(i / dasicsJumpCfgPerCSR))
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
}

class DasicsJumpIO(implicit p: Parameters) extends XSBundle with DasicsConst {
  val distribute_csr: DistributedCSRIO = Flipped(new DistributedCSRIO())
  val entries: Vec[DasicsJumpEntry] = Output(Vec(NumDasicsJumpBounds, new DasicsJumpEntry))
  val control_flow = new DasicsControlFlow
}

class DasicsReqBundle(implicit p: Parameters) extends XSBundle with DasicsConst {
  val addr = Output(UInt(VAddrBits.W))
  val inUntrustedZone = Output(Bool())
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

class MemDasics(implicit p: Parameters) extends XSModule with DasicsMethod with HasCSRConst {
  val io: DasicsMemIO = IO(new DasicsMemIO())

  val w = io.distribute_csr.w

  private val dasics = Wire(Vec(NumDasicsMemBounds, new DasicsEntry))
  val mapping = dasicsGenMemMapping(mem_init = dasicsMemInit, memCfgBase = DasicsLibCfgBase, memBoundBase = DasicsLibBoundBase, memEntries = dasics)

  val rdata: UInt = Wire(UInt(XLEN.W))
  MaskedRegMap.generate(mapping, w.bits.addr, rdata, w.valid, w.bits.data)


  io.entries := dasics
}

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

trait DasicsCheckerMethod extends DasicsConst{
  //def dasics_check(addr:UInt, isUntrustedZone: Bool, op: UInt, dasics: Vec[DasicsEntry]): Bool
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

class DasicsBranchIO(implicit p: Parameters) extends XSBundle with DasicsConst {
  val distribute_csr: DistributedCSRIO = Flipped(new DistributedCSRIO())
  val mode: UInt = Input(UInt(2.W))
  val valid: Bool = Input(Bool())
  val lastBranch: UInt = Input(UInt(VAddrBits.W))
  val target: UInt = Input(UInt(VAddrBits.W))
  val resp = new DasicsRespBundle()
}

class DasicsBranchChecker(implicit p: Parameters) extends XSModule
  with DasicsMethod with DasicsCheckerMethod with HasCSRConst {
  val io: DasicsBranchIO = IO(new DasicsBranchIO())
  val w = io.distribute_csr.w
  val dasics: Vec[DasicsJumpEntry] = Wire(Vec(NumDasicsJumpBounds, new DasicsJumpEntry))
  val mapping: Map[Int, (UInt, UInt, UInt => UInt, UInt, UInt => UInt)] = dasicsGenJumpMapping(
    jump_init = dasicsJumpInit, jumpCfgBase = DasicsJmpCfgBase, jumpBoundBase = DasicsJmpBoundBase, jumpEntries = dasics
  )
  private val dasics_main_call = RegInit(0.U(XLEN.W))
  private val dasics_return_pc = RegInit(0.U(XLEN.W))
  private val dasics_azone_return_pc = RegInit(0.U(XLEN.W))
  private val dasics_main_cfg = RegInit(0.U(XLEN.W))
  private val sMainBoundHi = RegInit(0.U(XLEN.W))
  private val sMainBoundLo = RegInit(0.U(XLEN.W))
  private val uMainBoundHi = RegInit(0.U(XLEN.W))
  private val uMainBoundLo = RegInit(0.U(XLEN.W))
  private val control_flow_mapping = Map(
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
  private val rdata = Wire(UInt(XLEN.W))
  MaskedRegMap.generate(mapping ++ control_flow_mapping, w.bits.addr, rdata, w.valid, w.bits.data)

  private val mainCfg = Wire(new DasicsMainCfg())
  mainCfg.gen(dasics_main_cfg)
  private val boundLo = Mux(io.mode === ModeU, uMainBoundLo, sMainBoundLo)
  private val boundHi = Mux(io.mode === ModeU, uMainBoundHi, sMainBoundHi)

  private val branchUntrusted = (io.mode === ModeU && mainCfg.uEnable || io.mode === ModeS && mainCfg.sEnable) &&
    !dasics_jump_in_bound(
      addr = io.lastBranch, boundHi = boundHi(VAddrBits - 1, 0), boundLo = boundLo(VAddrBits - 1, 0)
    )
  private val targetOutOfActive = dasics_jump_check(io.target, dasics)
  private val illegalBranch = io.valid && branchUntrusted && targetOutOfActive &&
    (io.target =/= dasics_return_pc) && (io.target =/= dasics_main_call) && (io.target =/= dasics_azone_return_pc)
  io.resp.dasics_fault := Mux(
    illegalBranch,
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
    MaskedRegMap(DasicsSMainCfg, mainCfgReg, "h3".U(XLEN.W)),
    MaskedRegMap(DasicsSMainBoundLo, sMainBoundLo),
    MaskedRegMap(DasicsSMainBoundHi, sMainBoundHi),
    MaskedRegMap(DasicsUMainCfg, mainCfgReg, "h2".U(XLEN.W)),
    MaskedRegMap(DasicsUMainBoundLo, uMainBoundLo),
    MaskedRegMap(DasicsUMainBoundHi, uMainBoundHi)
  )
  val rdata: UInt = Wire(UInt(XLEN.W))
  MaskedRegMap.generate(mapping, w.bits.addr, rdata, w.valid, w.bits.data)
}
