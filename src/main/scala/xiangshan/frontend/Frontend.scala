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

package xiangshan.frontend
import org.chipsalliance.cde.config.Parameters
import chisel3._
import chisel3.util._
import freechips.rocketchip.diplomacy.{LazyModule, LazyModuleImp}
import utils._
import xs.utils._
import xiangshan._
import xiangshan.backend.execute.fu.csr.PFEvent
import xiangshan.backend.execute.fu.fence.{FenceIBundle, SfenceBundle}
import xiangshan.backend.execute.fu.{PMP, PMPChecker, PMPReqBundle, FDITagger}
import xiangshan.cache.mmu._
import xiangshan.frontend.icache._
import xs.utils.perf.HasPerfLogging


class Frontend(val parentName:String = "Unknown")(implicit p: Parameters) extends LazyModule with HasXSParameter{

  val instrUncache  = LazyModule(new InstrUncache())
  val icache        = LazyModule(new ICache(parentName = parentName + "icache_"))

  lazy val module = new FrontendImp(this)
}


class FrontendImp (outer: Frontend) extends LazyModuleImp(outer)
  with HasXSParameter
  with HasPerfEvents
  with HasPerfLogging
{
  val io = IO(new Bundle() {
    val hartId = Input(UInt(8.W))
    val reset_vector = Input(UInt(PAddrBits.W))
    val fencei = Flipped(new FenceIBundle)
    val prefetchI = Input(Valid(UInt(XLEN.W)))
    val ptw = new TlbPtwIO(6)
    val backend = new FrontendToCtrlIO
    val sfence = Input(new SfenceBundle)
    val tlbCsr = Input(new TlbCsrBundle)
    val csrCtrl = Input(new CustomCSRCtrlIO)
    val csrUpdate = new DistributedCSRUpdateReq
    val mmioFetchPending = Output(Bool())
    val error  = new L1CacheErrorInfo
    val frontendInfo = new Bundle {
      val ibufFull  = Output(Bool())
      val bpuInfo = new Bundle {
        val bpRight = Output(UInt(XLEN.W))
        val bpWrong = Output(UInt(XLEN.W))
      }
    }
  })
  //fence.i signals bundle not used, tie to default value
  //io.fencei.done := true.B
  //decouped-frontend modules
  val instrUncache = outer.instrUncache.module
  val icache       = outer.icache.module
  val bpu     = Module(new Predictor(parentName = outer.parentName + s"bpu_"))
  val ifu     = Module(new NewIFU)
  val ibuffer =  Module(new Ibuffer)
  val ftq = Module(new Ftq(parentName = outer.parentName + s"ftq_"))

  val tlbCsr = DelayN(io.tlbCsr, 2)
  val csrCtrl = DelayN(io.csrCtrl, 2)
  val sfence = RegNext(RegNext(io.sfence))

  // trigger
  ifu.io.frontendTrigger := csrCtrl.frontend_trigger

  io.mmioFetchPending := ifu.io.mmioFetchPending

  // bpu ctrl
  bpu.io.ctrl := csrCtrl.bp_ctrl
  bpu.io.reset_vector := io.reset_vector

// pmp
  val pmp = Module(new PMP())
  val pmp_check = VecInit(Seq.fill(4)(Module(new PMPChecker(3, sameCycle = true)).io))
  pmp.io.distribute_csr := csrCtrl.distribute_csr
  val pmp_req_vec     = Wire(Vec(4, Valid(new PMPReqBundle())))
  pmp_req_vec(0) <> icache.io.pmp(0).req
  pmp_req_vec(1) <> icache.io.pmp(1).req
  pmp_req_vec(2) <> icache.io.pmp(2).req
  pmp_req_vec(3) <> ifu.io.pmp.req

  for (i <- pmp_check.indices) {
    pmp_check(i).apply(tlbCsr.priv.imode, tlbCsr.satp.mode, pmp.io.pmp, pmp.io.pma, pmp_req_vec(i))
  }
  icache.io.pmp(0).resp <> pmp_check(0).resp
  icache.io.pmp(1).resp <> pmp_check(1).resp
  icache.io.pmp(2).resp <> pmp_check(2).resp
  ifu.io.pmp.resp <> pmp_check(3).resp

  // FDITagger
  val fdiTagger: FDITagger = Module(new FDITagger())
  fdiTagger.io.distribute_csr := csrCtrl.distribute_csr
  fdiTagger.io.privMode := tlbCsr.priv.imode
  fdiTagger.io.addr := ifu.io.fdi.startAddr
  ifu.io.fdi.notTrusted := fdiTagger.io.notTrusted

  // val tlb_req_arb     = Module(new Arbiter(new TlbReq, 2))
  // tlb_req_arb.io.in(0) <> ifu.io.iTLBInter.req
  // tlb_req_arb.io.in(1) <> icache.io.itlb(1).req

  val itlb_requestors = Wire(Vec(6, new BlockTlbRequestIO))
  itlb_requestors(0) <> icache.io.itlb(0)
  itlb_requestors(1) <> icache.io.itlb(1)
  itlb_requestors(2) <> icache.io.itlb(2)
  itlb_requestors(3) <> icache.io.itlb(3)
  itlb_requestors(4) <> icache.io.itlb(4)
  itlb_requestors(5) <> ifu.io.iTLBInter

  // itlb_requestors(1).req <>  tlb_req_arb.io.out

  // ifu.io.iTLBInter.resp  <> itlb_requestors(1).resp
  // icache.io.itlb(1).resp <> itlb_requestors(1).resp

  io.ptw <> TLB(
    //in = Seq(icache.io.itlb(0), icache.io.itlb(1)),
    in = Seq(itlb_requestors(0),itlb_requestors(1),itlb_requestors(2),itlb_requestors(3),itlb_requestors(4),itlb_requestors(5)),
    sfence = DelayN(io.sfence, 1),
    csr = DelayN(io.tlbCsr, 1),
    width = 6,
    nRespDups = 1,
    shouldBlock = true,
    itlbParams
  )

  icache.io.prefetch <> ftq.io.toPrefetch

  val needFlush = RegNext(io.backend.toFtq.redirect.valid)

  //IFU-Ftq
  ifu.io.ftqInter.fromFtq <> ftq.io.toIfu
  ftq.io.toIfu.req.ready :=  ifu.io.ftqInter.fromFtq.req.ready && icache.io.fetch.req.ready

  ftq.io.fromIfu          <> ifu.io.ftqInter.toFtq
  bpu.io.ftq_to_bpu       <> ftq.io.toBpu
  ftq.io.fromBpu          <> bpu.io.bpu_to_ftq

  ftq.io.mmioCommitRead   <> ifu.io.mmioCommitRead
  //IFU-ICache

  icache.io.fetch.req <> ftq.io.toICache.req
  ftq.io.toICache.req.ready :=  ifu.io.ftqInter.fromFtq.req.ready && icache.io.fetch.req.ready

  ifu.io.icacheInter.resp <>    icache.io.fetch.resp
  ifu.io.icacheInter.icacheReady :=  icache.io.toIFU
  icache.io.stop := ifu.io.icacheStop

  ifu.io.icachePerfInfo := icache.io.perfInfo

  icache.io.csr.distribute_csr <> csrCtrl.distribute_csr
  io.csrUpdate := RegNext(icache.io.csr.update)

  icache.io.csr_pf_enable     := RegNext(csrCtrl.l1I_pf_enable)
  icache.io.csr_parity_enable := RegNext(csrCtrl.icache_parity_enable)
  icache.io.backend_redirect  := io.backend.toFtq.redirect.valid

  //IFU-Ibuffer
  ibuffer.io.in <> ifu.io.toIbuffer

  ftq.io.fromBackend <> io.backend.toFtq
  io.backend.fromFtq <> ftq.io.toBackend
  io.frontendInfo.bpuInfo <> ftq.io.bpuInfo

  ifu.io.rob_commits <> io.backend.toFtq.rob_commits

  ibuffer.io.flush := needFlush
  io.backend.cfVec <> ibuffer.io.out

  instrUncache.io.req   <> ifu.io.uncacheInter.toUncache
  ifu.io.uncacheInter.fromUncache <> instrUncache.io.resp
  instrUncache.io.flush := false.B
  io.error <> RegNext(RegNext(icache.io.error))

  icache.io.hartId := io.hartId
  icache.io.fencei <> io.fencei

  val frontendBubble = PopCount((0 until DecodeWidth).map(i => io.backend.cfVec(i).ready && !ibuffer.io.out(i).valid))
  XSPerfAccumulate("FrontendBubble", frontendBubble)
  io.frontendInfo.ibufFull := RegNext(ibuffer.io.full)

  // PFEvent
  val pfevent = Module(new PFEvent)
  pfevent.io.distribute_csr := io.csrCtrl.distribute_csr
  val csrevents = pfevent.io.hpmevent.take(8)

  val perfFromUnits = Seq(ifu, ibuffer, icache, ftq, bpu).flatMap(_.getPerfEvents)
  val perfFromIO    = Seq()
  val perfBlock     = Seq()
  // let index = 0 be no event
  val allPerfEvents = Seq(("noEvent", 0.U)) ++ perfFromUnits ++ perfFromIO ++ perfBlock

  if (printEventCoding) {
    for (((name, inc), i) <- allPerfEvents.zipWithIndex) {
      println("Frontend perfEvents Set", name, inc, i)
    }
  }

  val allPerfInc = allPerfEvents.map(_._2.asTypeOf(new PerfEvent))
  override val perfEvents = HPerfMonitor(csrevents, allPerfInc).getPerfEvents
  generatePerfEvent()
}
