// See LICENSE.Berkeley for license details.
// See LICENSE.SiFive for license details.

package freechips.rocketchip.tile

import Chisel._

import freechips.rocketchip.config._
import freechips.rocketchip.subsystem._
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.rocket._
import freechips.rocketchip.tilelink._
import freechips.rocketchip.util.InOrderArbiter

case object BuildRoCC extends Field[Seq[Parameters => LazyRoCC]](Nil)

class RoCCInstruction extends Bundle {
  val funct = Bits(width = 7)
  val rs2 = Bits(width = 5)
  val rs1 = Bits(width = 5)
  val xd = Bool()
  val xs1 = Bool()
  val xs2 = Bool()
  val rd = Bits(width = 5)
  val opcode = Bits(width = 7)
}

class RoCCCommand(implicit p: Parameters) extends CoreBundle()(p) {
  val inst = new RoCCInstruction
  val rs1 = Bits(width = xLen)
  val rs2 = Bits(width = xLen)
  val status = new MStatus
}

class RoCCResponse(implicit p: Parameters) extends CoreBundle()(p) {
  val rd = Bits(width = 5)
  val data = Bits(width = xLen)
}

class RoCCCoreIO(implicit p: Parameters) extends CoreBundle()(p) {
  val cmd = Decoupled(new RoCCCommand).flip
  val resp = Decoupled(new RoCCResponse)
  val mem = new HellaCacheIO
  val busy = Bool(OUTPUT)
  val interrupt = Bool(OUTPUT)
  val exception = Bool(INPUT)
}

class RoCCIO(val nPTWPorts: Int)(implicit p: Parameters) extends RoCCCoreIO()(p) {
  val ptw = Vec(nPTWPorts, new TLBPTWIO)
  val fpu_req = Decoupled(new FPInput)
  val fpu_resp = Decoupled(new FPResult).flip
}

/** Base classes for Diplomatic TL2 RoCC units **/
abstract class LazyRoCC(
      val opcodes: OpcodeSet,
      val nPTWPorts: Int = 0,
      val usesFPU: Boolean = false
    )(implicit p: Parameters) extends LazyModule {
  val module: LazyRoCCModuleImp
  val atlNode: TLNode = TLIdentityNode()
  val tlNode: TLNode = TLIdentityNode()
}

class LazyRoCCModuleImp(outer: LazyRoCC) extends LazyModuleImp(outer) {
  val io = IO(new RoCCIO(outer.nPTWPorts))
}

/** Mixins for including RoCC **/

trait HasLazyRoCC extends CanHavePTW { this: BaseTile =>
  val roccs = p(BuildRoCC).map(_(p))

  roccs.map(_.atlNode).foreach { atl => tlMasterXbar.node :=* atl }
  roccs.map(_.tlNode).foreach { tl => tlOtherMastersNode :=* tl }

  nPTWPorts += roccs.map(_.nPTWPorts).foldLeft(0)(_ + _)
  nDCachePorts += roccs.size
}

trait HasLazyRoCCModule extends CanHavePTWModule
    with HasCoreParameters { this: RocketTileModuleImp with HasFpuOpt =>

  val (respArb, cmdRouter) = if(outer.roccs.size > 0) {
    val respArb = Module(new RRArbiter(new RoCCResponse()(outer.p), outer.roccs.size))
    val cmdRouter = Module(new RoccCommandRouter(outer.roccs.map(_.opcodes))(outer.p))
    outer.roccs.zipWithIndex.foreach { case (rocc, i) =>
      ptwPorts ++= rocc.module.io.ptw
      rocc.module.io.cmd <> cmdRouter.io.out(i)
      val dcIF = Module(new SimpleHellaCacheIF()(outer.p))
      dcIF.io.requestor <> rocc.module.io.mem
      dcachePorts += dcIF.io.cache
      respArb.io.in(i) <> Queue(rocc.module.io.resp)
    }

    fpuOpt foreach { fpu =>
      val nFPUPorts = outer.roccs.filter(_.usesFPU).size
      if (usingFPU && nFPUPorts > 0) {
        val fpArb = Module(new InOrderArbiter(new FPInput()(outer.p), new FPResult()(outer.p), nFPUPorts))
        val fp_rocc_ios = outer.roccs.filter(_.usesFPU).map(_.module.io)
        fpArb.io.in_req <> fp_rocc_ios.map(_.fpu_req)
        fp_rocc_ios.zip(fpArb.io.in_resp).foreach {
          case (rocc, arb) => rocc.fpu_resp <> arb
        }
        fpu.io.cp_req <> fpArb.io.out_req
        fpArb.io.out_resp <> fpu.io.cp_resp
      } else {
        fpu.io.cp_req.valid := Bool(false)
        fpu.io.cp_resp.ready := Bool(false)
      }
    }
    (Some(respArb), Some(cmdRouter))
  } else {
    (None, None)
  }
}

class  AccumulatorExample(opcodes: OpcodeSet, val n: Int = 4)(implicit p: Parameters) extends LazyRoCC(opcodes) {
  override lazy val module = new AccumulatorExampleModuleImp(this)
}

class AccumulatorExampleModuleImp(outer: AccumulatorExample)(implicit p: Parameters) extends LazyRoCCModuleImp(outer)
    with HasCoreParameters {
  val mem_busy = RegInit(false.B)

  val cmd = Queue(io.cmd)
  val funct = cmd.bits.inst.funct
  val doWrite = funct === UInt(0)
  val doRead = funct === UInt(1)
  val doLoad = funct === UInt(2)
  val doAccum = funct === UInt(3)
  val doClear = funct === UInt(4)

  // controlpath
  val sIdle :: sLoad :: sWrite :: sAccum :: sClear :: sRead :: Nil = Enum(6)
  val state = RegInit(sIdle)

  val arg1  = RegInit(Vec(Seq.fill(256)(0.U(8.W))))
  val arg2  = RegInit(Vec(Seq.fill(256)(0.U(8.W))))
  val arg_valid = RegInit(Vec(Seq.fill(256)(false.B)))
  val dotP  = RegInit(0.U(16.W))
  val mem_addr = Reg(UInt(width = xLen))
  val idx   = RegInit(0.U(32.W))
  val arg_count = RegInit(0.U(32.W))
  val addend = cmd.bits.rs1

  val stallLoad = (!io.mem.req.ready || (state === sLoad) || (state === sWrite))
  val stallResp = !io.resp.ready && cmd.bits.inst.xd
  val stallComp = ((state === sAccum) || (state === sClear))

  cmd.ready := !stallLoad && !stallResp && !stallComp

  io.resp.valid := cmd.valid && !stallResp && !stallLoad && (state === sIdle)
  io.resp.bits.rd := cmd.bits.inst.rd
  io.resp.bits.data := dotP
  io.busy := cmd.valid
  io.interrupt := Bool(false)

  // State Logic - Load and Write Initialization
  when(cmd.fire() && (state === sIdle)) {
    when(doLoad) {
      state     := sLoad
      mem_addr  := cmd.bits.rs1
      arg_count := cmd.bits.rs2
    } .elsewhen(doWrite) {
      state     := sWrite
      mem_addr  := cmd.bits.rs1
      arg_count := cmd.bits.rs2
    } .elsewhen(doAccum) {
      state := sAccum
    } .elsewhen(doClear) {
      state := sClear
    } .elsewhen(doRead) {
      printf("dotP = %d\n",dotP)
    }
  }
  val products = Wire(Vec(256, UInt(16.W)))
  for(i <- 0 until 256) {
    products(i) := arg1(i) * arg2(i)
  }

  val adder_idx = RegInit(0.U(32.W))

  when (state === sAccum && adder_idx < arg_count) {
      when(arg_valid(adder_idx)) {
        printf("arg1(%d) = %d, arg2(%d) = %d, arg_valid(%d) = %b\n",idx,arg1(idx),idx, arg2(idx),idx,arg_valid(idx))
        dotP := dotP + (products(adder_idx))
        printf("dotP = %d\n", dotP)
        adder_idx := adder_idx + 1.U
    }
  } .elsewhen(state === sAccum && adder_idx >= arg_count) {
    adder_idx := 0.U
    state := sIdle
  }

  when (state === sClear) {
    dotP      := 0.U
    arg1      := Vec(Seq.fill(256)(0.U(8.W)))
    arg2      := Vec(Seq.fill(256)(0.U(8.W)))
    arg_valid := Vec(Seq.fill(256)(false.B))
    arg_count := 0.U
    idx       := 0.U
    mem_addr  := 0.U
    state     := sIdle
  }

  val load_done = (idx >= arg_count)

  // State Logic - Sending read requests to memory
  io.mem.req.bits.addr  := mem_addr
  io.mem.req.bits.tag   := mem_addr
  io.mem.req.bits.cmd   := M_XRD
  io.mem.req.bits.typ   := MT_B
  io.mem.req.bits.data  := Bits(0)
  io.mem.req.valid      := !stallResp && !mem_busy && (state === sLoad || state === sWrite) && !load_done

  // State Logic - Handling data read from memory
  when(state === sLoad && load_done) {
    state := sIdle
    idx   := 0.U
    printf("load done\n")
  } .elsewhen(state === sLoad && io.mem.resp.valid) {
    printf("loaded one\n")
    arg1(idx)       := io.mem.resp.bits.data
    arg_valid(idx)  := true.B
    idx             := idx + 1.U
    mem_busy        := false.B
    mem_addr        := mem_addr + 1.U
  }

  when(state === sWrite && load_done) {
    state := sIdle
    idx   := 0.U
    printf("write done\n")
  } .elsewhen(state === sWrite && io.mem.resp.valid) {
    printf("written\n")
    arg2(idx)       := io.mem.resp.bits.data
    arg_valid(idx)  := true.B
    idx             := idx + 1.U
    mem_busy        := false.B
    mem_addr        := mem_addr + 1.U
    printf("%b, %d, %d\n",load_done, arg_count, idx)
  }

  /*
  when(io.mem.resp.valid) {
    when(state === sLoad) {
      arg1(idx)       := io.mem.resp.bits.data
      arg_valid(idx)  := true.B
      idx := idx + 1.U
      mem_busy := false.B
      when(load_done) {
        state := sIdle
        idx   := 0.U
      }.otherwise {
        mem_addr := mem_addr + 1.U
      }
    } .elsewhen(state === sWrite) {
      arg2(idx)       := io.mem.resp.bits.data
      arg_valid(idx)  := true.B
      idx := idx + 1.U
      mem_busy := false.B
      when(load_done) {
        state := sIdle
        idx   := 0.U
      } .otherwise {
        mem_addr := mem_addr + 1.U
      }
    }
  }
  */
  // Memory Controller
  when(io.mem.req.fire()) {
    mem_busy := true.B
  }
  // datapath
/*
  val arg1  = RegInit(Vec(Seq.fill(n)(0.U(32.W))))
  val arg2  = RegInit(Vec(Seq.fill(n)(0.U(32.W))))
  val dotP  = RegInit(0.U)
  val mem_addr = Reg(UInt(width = xLen))
  val addend = cmd.bits.rs1
  val wdata = arg1 * arg2
  val route_mem = RegInit(false.B)



  when (cmd.fire() && doAccum) {
    dotP := dotP + wdata
  }

  when (cmd.fire() && doClear) {
    dotP := 0.U
    arg1 := 0.U
    arg2 := 0.U
  }

  when (cmd.fire() && doWrite) {
    route_mem := true.B
  }

  when (io.mem.resp.valid) {
    when (route_mem) {
      arg2 := io.mem.resp.bits.data
      route_mem := false.B
    } .otherwise {
    arg1 := io.mem.resp.bits.data
    }
    busy(memRespTag) := Bool(false)
  }

  // control
  when (io.mem.req.fire()) {
    busy(addr) := Bool(true)
  }

  val doResp = cmd.bits.inst.xd
  val stallReg = busy(addr)
  val stallLoad = (doWrite || doLoad) && !io.mem.req.ready
  val stallResp = doResp && !io.resp.ready

  cmd.ready := !stallReg && !stallLoad && !stallResp
    // command resolved if no stalls AND not issuing a load that will need a request

  // PROC RESPONSE INTERFACE
  io.resp.valid := cmd.valid && doResp && !stallReg && !stallLoad
    // valid response if valid command, need a response, and no stalls
  io.resp.bits.rd := cmd.bits.inst.rd
    // Must respond with the appropriate tag or undefined behavior
  io.resp.bits.data := dotP
    // Semantics is to always send out prior accumulator register value

  io.busy := cmd.valid || busy.reduce(_||_)
    // Be busy when have pending memory requests or committed possibility of pending requests
  io.interrupt := Bool(false)
    // Set this true to trigger an interrupt on the processor (please refer to supervisor documentation)

  // MEMORY REQUEST INTERFACE
  io.mem.req.valid := cmd.valid && (doLoad || doWrite) && !stallReg && !stallResp
  io.mem.req.bits.addr := addend
  io.mem.req.bits.tag := addr
  io.mem.req.bits.cmd := M_XRD // perform a load (M_XWR for stores)
  io.mem.req.bits.typ := MT_D // D = 8 bytes, W = 4, H = 2, B = 1
  io.mem.req.bits.data := Bits(0) // we're not performing any stores...
  io.mem.req.bits.phys := Bool(false)
*/
}

class  TranslatorExample(opcodes: OpcodeSet)(implicit p: Parameters) extends LazyRoCC(opcodes, nPTWPorts = 1) {
  override lazy val module = new TranslatorExampleModuleImp(this)
}

class TranslatorExampleModuleImp(outer: TranslatorExample)(implicit p: Parameters) extends LazyRoCCModuleImp(outer)
    with HasCoreParameters {
  val req_addr = Reg(UInt(width = coreMaxAddrBits))
  val req_rd = Reg(io.resp.bits.rd)
  val req_offset = req_addr(pgIdxBits - 1, 0)
  val req_vpn = req_addr(coreMaxAddrBits - 1, pgIdxBits)
  val pte = Reg(new PTE)

  val s_idle :: s_ptw_req :: s_ptw_resp :: s_resp :: Nil = Enum(Bits(), 4)
  val state = Reg(init = s_idle)

  io.cmd.ready := (state === s_idle)

  when (io.cmd.fire()) {
    req_rd := io.cmd.bits.inst.rd
    req_addr := io.cmd.bits.rs1
    state := s_ptw_req
  }

  private val ptw = io.ptw(0)

  when (ptw.req.fire()) { state := s_ptw_resp }

  when (state === s_ptw_resp && ptw.resp.valid) {
    pte := ptw.resp.bits.pte
    state := s_resp
  }

  when (io.resp.fire()) { state := s_idle }

  ptw.req.valid := (state === s_ptw_req)
  ptw.req.bits.addr := req_vpn

  io.resp.valid := (state === s_resp)
  io.resp.bits.rd := req_rd
  io.resp.bits.data := Mux(pte.leaf(), Cat(pte.ppn, req_offset), SInt(-1, xLen).asUInt)

  io.busy := (state =/= s_idle)
  io.interrupt := Bool(false)
  io.mem.req.valid := Bool(false)
}

class  CharacterCountExample(opcodes: OpcodeSet)(implicit p: Parameters) extends LazyRoCC(opcodes) {
  override lazy val module = new CharacterCountExampleModuleImp(this)
  override val atlNode = TLClientNode(Seq(TLClientPortParameters(Seq(TLClientParameters("CharacterCountRoCC")))))
}

class CharacterCountExampleModuleImp(outer: CharacterCountExample)(implicit p: Parameters) extends LazyRoCCModuleImp(outer)
  with HasCoreParameters
  with HasL1CacheParameters {
  val cacheParams = tileParams.icache.get

  private val blockOffset = blockOffBits
  private val beatOffset = log2Up(cacheDataBits/8)

  val needle = Reg(UInt(width = 8))
  val addr = Reg(UInt(width = coreMaxAddrBits))
  val count = Reg(UInt(width = xLen))
  val resp_rd = Reg(io.resp.bits.rd)

  val addr_block = addr(coreMaxAddrBits - 1, blockOffset)
  val offset = addr(blockOffset - 1, 0)
  val next_addr = (addr_block + UInt(1)) << UInt(blockOffset)

  val s_idle :: s_acq :: s_gnt :: s_check :: s_resp :: Nil = Enum(Bits(), 5)
  val state = Reg(init = s_idle)

  val (tl_out, edgesOut) = outer.atlNode.out(0)
  val gnt = tl_out.d.bits
  val recv_data = Reg(UInt(width = cacheDataBits))
  val recv_beat = Reg(UInt(width = log2Up(cacheDataBeats+1)), init = UInt(0))

  val data_bytes = Vec.tabulate(cacheDataBits/8) { i => recv_data(8 * (i + 1) - 1, 8 * i) }
  val zero_match = data_bytes.map(_ === UInt(0))
  val needle_match = data_bytes.map(_ === needle)
  val first_zero = PriorityEncoder(zero_match)

  val chars_found = PopCount(needle_match.zipWithIndex.map {
    case (matches, i) =>
      val idx = Cat(recv_beat - UInt(1), UInt(i, beatOffset))
      matches && idx >= offset && UInt(i) <= first_zero
  })
  val zero_found = zero_match.reduce(_ || _)
  val finished = Reg(Bool())

  io.cmd.ready := (state === s_idle)
  io.resp.valid := (state === s_resp)
  io.resp.bits.rd := resp_rd
  io.resp.bits.data := count
  tl_out.a.valid := (state === s_acq)
  tl_out.a.bits := edgesOut.Get(
                       fromSource = UInt(0),
                       toAddress = addr_block << blockOffset,
                       lgSize = UInt(lgCacheBlockBytes))._2
  tl_out.d.ready := (state === s_gnt)

  when (io.cmd.fire()) {
    addr := io.cmd.bits.rs1
    needle := io.cmd.bits.rs2
    resp_rd := io.cmd.bits.inst.rd
    count := UInt(0)
    finished := Bool(false)
    state := s_acq
  }

  when (tl_out.a.fire()) { state := s_gnt }

  when (tl_out.d.fire()) {
    recv_beat := recv_beat + UInt(1)
    recv_data := gnt.data
    state := s_check
  }

  when (state === s_check) {
    when (!finished) {
      count := count + chars_found
    }
    when (zero_found) { finished := Bool(true) }
    when (recv_beat === UInt(cacheDataBeats)) {
      addr := next_addr
      state := Mux(zero_found || finished, s_resp, s_acq)
    } .otherwise {
      state := s_gnt
    }
  }

  when (io.resp.fire()) { state := s_idle }

  io.busy := (state =/= s_idle)
  io.interrupt := Bool(false)
  io.mem.req.valid := Bool(false)
  // Tie off unused channels
  tl_out.b.ready := Bool(true)
  tl_out.c.valid := Bool(false)
  tl_out.e.valid := Bool(false)
}

class OpcodeSet(val opcodes: Seq[UInt]) {
  def |(set: OpcodeSet) =
    new OpcodeSet(this.opcodes ++ set.opcodes)

  def matches(oc: UInt) = opcodes.map(_ === oc).reduce(_ || _)
}

object OpcodeSet {
  def custom0 = new OpcodeSet(Seq(Bits("b0001011")))
  def custom1 = new OpcodeSet(Seq(Bits("b0101011")))
  def custom2 = new OpcodeSet(Seq(Bits("b1011011")))
  def custom3 = new OpcodeSet(Seq(Bits("b1111011")))
  def all = custom0 | custom1 | custom2 | custom3
}

class RoccCommandRouter(opcodes: Seq[OpcodeSet])(implicit p: Parameters)
    extends CoreModule()(p) {
  val io = new Bundle {
    val in = Decoupled(new RoCCCommand).flip
    val out = Vec(opcodes.size, Decoupled(new RoCCCommand))
    val busy = Bool(OUTPUT)
  }

  val cmd = Queue(io.in)
  val cmdReadys = io.out.zip(opcodes).map { case (out, opcode) =>
    val me = opcode.matches(cmd.bits.inst.opcode)
    out.valid := cmd.valid && me
    out.bits := cmd.bits
    out.ready && me
  }
  cmd.ready := cmdReadys.reduce(_ || _)
  io.busy := cmd.valid

  assert(PopCount(cmdReadys) <= UInt(1),
    "Custom opcode matched for more than one accelerator")
}
