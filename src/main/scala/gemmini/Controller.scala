
package gemmini

import java.nio.charset.StandardCharsets
import java.nio.file.{Files, Paths}
import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config._
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.tile._
import freechips.rocketchip.util.{BundleField, ClockGate}
import freechips.rocketchip.tilelink._
import GemminiISA._
import Util._

class GemminiCmd(rob_entries: Int)(implicit p: Parameters) extends Bundle {
  val cmd = new RoCCCommand
  val rob_id = UDValid(UInt(log2Up(rob_entries).W))
  val from_matmul_fsm = Bool()
  val from_conv_fsm = Bool()
}

class Gemmini[T <: Data : Arithmetic, U <: Data, V <: Data](val config: GemminiArrayConfig[T, U, V])
                                     (implicit p: Parameters)
  extends LazyRoCC (
    opcodes = config.opcodes,
    nPTWPorts = if (config.use_shared_tlb) 1 else 2) {

  Files.write(Paths.get(config.headerFilePath), config.generateHeader().getBytes(StandardCharsets.UTF_8))
  if (System.getenv("GEMMINI_ONLY_GENERATE_GEMMINI_H") == "1") {
    System.exit(1)
  }

  val xLen = p(TileKey).core.xLen
  val spad = LazyModule(new Scratchpad(config))

  val use_ext_tl_mem = config.use_shared_ext_mem && config.use_tl_ext_mem
  val num_ids = 32 // TODO (richard): move to config
  val spad_base = config.tl_ext_mem_base
  val spad_data_len = config.sp_width / 8
  val acc_data_len = config.sp_width / config.inputType.getWidth * config.accType.getWidth / 8
  val max_data_len = spad_data_len // max acc_data_len

  val mem_depth = config.sp_bank_entries * spad_data_len / max_data_len
  val mem_width = max_data_len
  println(f"unified shared memory size: ${mem_depth}x${mem_width}x${config.sp_banks}")

  // make scratchpad read and write clients, per bank
  //    _____  ________  _______  ___   ___
  //   / __/ |/_/_  __/ / __/ _ \/ _ | / _ \
  //  / _/_>  <  / /   _\ \/ ___/ __ |/ // /
  // /___/_/|_| /_/   /___/_/  /_/ |_/____/
  // ***************************************
  // HOW TO USE EXTERNAL SCRATCHPAD:
  // the scratchpad MUST BE INSTANTIATED ELSEWHERE if use_ext_tl_mem is enabled,
  // else elaboration will not pass. the scratchpad needs to be dual ported
  // and must be able to serve the entire scratchpad row (config.sp_width) in 1 cycle.
  // three nodes must be hooked up correctly: spad_read_nodes, spad_write_nodes, and spad.spad_writer.node
  // for deadlock avoidance, read and write should not be sharing a single channel anywhere until the SRAMs.
  // see RadianceCluster.scala for an example
  val spad_read_nodes = if (use_ext_tl_mem) TLClientNode(Seq.tabulate(config.sp_banks) {i =>
    TLMasterPortParameters.v1(Seq(TLMasterParameters.v1(
      name = s"spad_read_node_$i",
      sourceId = IdRange(0, num_ids),
      visibility = Seq(AddressSet(spad_base + i * mem_width * mem_depth, mem_width * mem_depth - 1)),
      supportsProbe = TransferSizes(mem_width, mem_width),
      supportsGet = TransferSizes(mem_width, mem_width)
    )))
  }) else TLIdentityNode()

  val spad_write_nodes = if (use_ext_tl_mem) TLClientNode(Seq.tabulate(config.sp_banks) { i =>
    TLMasterPortParameters.v1(Seq(TLMasterParameters.v1(
      name = s"spad_write_node_$i",
      sourceId = IdRange(0, num_ids),
      visibility = Seq(AddressSet(spad_base + i * mem_width * mem_depth, mem_width * mem_depth - 1)),
      supportsProbe = TransferSizes(mem_width, mem_width),
      supportsPutFull = TransferSizes(mem_width, mem_width),
      supportsPutPartial = TransferSizes(mem_width, mem_width)
    )))
  }) else TLIdentityNode()

  // val acc_read_nodes = if (create_tl_mem) TLClientNode(Seq.tabulate(config.acc_banks) { i =>
  //   TLMasterPortParameters.v1(Seq(TLMasterParameters.v1(name = s"acc_read_node_$i", sourceId = IdRange(0, numIDs))))
  // }) else TLIdentityNode()
  // val acc_write_nodes = if (create_tl_mem) TLClientNode(Seq.tabulate(config.acc_banks) { i =>
  //   TLMasterPortParameters.v1(Seq(TLMasterParameters.v1(name = s"acc_write_node_$i", sourceId = IdRange(0, numIDs))))
  // }) else TLIdentityNode()

  override lazy val module = new GemminiModule(this)
  override val tlNode = if (config.use_dedicated_tl_port) spad.id_node else TLIdentityNode()
  override val atlNode = if (config.use_dedicated_tl_port) TLIdentityNode() else spad.id_node

  val node = if (config.use_dedicated_tl_port) tlNode else atlNode
}

class GemminiModule[T <: Data: Arithmetic, U <: Data, V <: Data]
    (outer: Gemmini[T, U, V])
    extends LazyRoCCModuleImp(outer)
    with HasCoreParameters {

  import outer.config._
  import outer.spad

  val ext_mem_io = if (use_shared_ext_mem && !use_tl_ext_mem)
    Some(IO(new ExtSpadMemIO(sp_banks, acc_banks, acc_sub_banks))) else None

  if (outer.use_ext_tl_mem) {
    val ext_mem_spad = outer.spad.module.io.ext_mem.get.spad
    val ext_mem_acc = outer.spad.module.io.ext_mem.get.acc
    val source_counters = Seq.fill(4)(Counter(outer.num_ids))

    def connect(ext_mem: ExtMemIO, bank_base: Int, req_size: Int, r_node: TLBundle, r_edge: TLEdgeOut, r_source: Counter,
                w_node: TLBundle, w_edge: TLEdgeOut, w_source: Counter): Unit = {
      r_node.a.valid := ext_mem.read_req.valid
      r_node.a.bits := r_edge.Get(r_source.value,
        (ext_mem.read_req.bits << req_size.U).asUInt | bank_base.U | outer.spad_base.U,
        req_size.U)._2
      ext_mem.read_req.ready := r_node.a.ready

      val w_shifted_addr = (ext_mem.write_req.bits.addr << req_size.U).asUInt
      val w_mask = (ext_mem.write_req.bits.mask << (w_shifted_addr & (w_edge.manager.beatBytes - 1).U)).asUInt

      w_node.a.valid := ext_mem.write_req.valid
      w_node.a.bits := w_edge.Put(w_source.value,
        w_shifted_addr | bank_base.U | outer.spad_base.U,
        req_size.U, ext_mem.write_req.bits.data, w_mask)._2
      ext_mem.write_req.ready := w_node.a.ready

      ext_mem.read_resp.valid := r_node.d.valid
      ext_mem.read_resp.bits := r_node.d.bits.data
      r_node.d.ready := ext_mem.read_resp.ready

      w_node.d.ready := true.B // writes are not acknowledged in gemmini

      when(ext_mem.read_req.fire) { r_source.inc() }
      when(ext_mem.write_req.fire) { w_source.inc() }
    }
    (outer.spad_read_nodes.out zip outer.spad_write_nodes.out)
      .zipWithIndex.foreach{ case (((r_node, r_edge), (w_node, w_edge)), i) =>
        connect(ext_mem_spad(i), i * outer.mem_depth * outer.mem_width, log2Up(outer.spad_data_len),
          r_node, r_edge, source_counters(0), w_node, w_edge, source_counters(1))
    }


    ext_mem_acc.foreach(_.foreach(x => {
      x.read_resp.bits := 0.U(1.W)
      x.read_resp.valid := false.B
      x.read_req.ready := false.B
      x.write_req.ready := false.B
    }))
    // (outer.acc_read_nodes.out zip outer.acc_write_nodes.out)
    //   .zipWithIndex.foreach { case (((r_node, r_edge), (w_node, w_edge)), i) =>
    //     // TODO (richard): one subbank only for now
    //     connect(ext_mem_acc(i)(0), log2Up(outer.acc_data_len),
    //       r_node, r_edge, source_counters(2), w_node, w_edge, source_counters(3))
    // }
  } else if (use_shared_ext_mem) {
    ext_mem_io.foreach(_ <> outer.spad.module.io.ext_mem.get)
  }

  val tagWidth = 32

  // Counters
  val counters = Module(new CounterController(outer.config.num_counter, outer.xLen))
  io.resp <> counters.io.out  // Counter access command will be committed immediately
  counters.io.event_io.external_values(0) := 0.U
  counters.io.event_io.event_signal(0) := false.B
  counters.io.in.valid := false.B
  counters.io.in.bits := DontCare
  counters.io.event_io.collect(spad.module.io.counter)

  // TLB
  implicit val edge = outer.spad.id_node.edges.out.head
  val tlb = Module(new FrontendTLB(if (outer.config.use_tl_ext_mem) 3 else 2,
    tlb_size, dma_maxbytes, use_tlb_register_filter, use_firesim_simulation_counters, use_shared_tlb))
  (tlb.io.clients zip outer.spad.module.io.tlb).foreach(t => t._1 <> t._2)

  tlb.io.exp.foreach(_.flush_skip := false.B)
  tlb.io.exp.foreach(_.flush_retry := false.B)

  io.ptw <> tlb.io.ptw

  counters.io.event_io.collect(tlb.io.counter)

  spad.module.io.flush := tlb.io.exp.map(_.flush()).reduce(_ || _)

  val clock_en_reg = RegInit(true.B)
  val gated_clock = if (clock_gate) ClockGate(clock, clock_en_reg, "gemmini_clock_gate") else clock
  outer.spad.module.clock := gated_clock

  /*
  //=========================================================================
  // Frontends: Incoming commands and ROB
  //=========================================================================

  // forward cmd to correct frontend. if the rob is busy, do not forward new
  // commands to tiler, and vice versa
  val is_cisc_mode = RegInit(false.B)

  val raw_cmd = Queue(io.cmd)
  val funct = raw_cmd.bits.inst.funct

  val is_cisc_funct = (funct === CISC_CONFIG) ||
                      (funct === ADDR_AB) ||
                      (funct === ADDR_CD) ||
                      (funct === SIZE_MN) ||
                      (funct === SIZE_K) ||
                      (funct === RPT_BIAS) ||
                      (funct === RESET) ||
                      (funct === COMPUTE_CISC)

  val raw_cisc_cmd = WireInit(raw_cmd)
  val raw_risc_cmd = WireInit(raw_cmd)
  raw_cisc_cmd.valid := false.B
  raw_risc_cmd.valid := false.B
  raw_cmd.ready := false.B

  //-------------------------------------------------------------------------
  // cisc
  val cmd_fsm = CmdFSM(outer.config)
  cmd_fsm.io.cmd <> raw_cisc_cmd
  val tiler = TilerController(outer.config)
  tiler.io.cmd_in <> cmd_fsm.io.tiler

  //-------------------------------------------------------------------------
  // risc
  val unrolled_cmd = LoopUnroller(raw_risc_cmd, outer.config.meshRows * outer.config.tileRows)
  */

  val reservation_station = withClock (gated_clock) { Module(new ReservationStation(outer.config, new GemminiCmd(reservation_station_entries))) }
  counters.io.event_io.collect(reservation_station.io.counter)

  when (io.cmd.valid && io.cmd.bits.inst.funct === CLKGATE_EN && !io.busy) {
    clock_en_reg := io.cmd.bits.rs1(0)
  }

  val raw_cmd_q = Module(new Queue(new GemminiCmd(reservation_station_entries), entries = 2))
  raw_cmd_q.io.enq.valid := io.cmd.valid
  io.cmd.ready := raw_cmd_q.io.enq.ready
  raw_cmd_q.io.enq.bits.cmd := io.cmd.bits
  raw_cmd_q.io.enq.bits.rob_id := DontCare
  raw_cmd_q.io.enq.bits.from_conv_fsm := false.B
  raw_cmd_q.io.enq.bits.from_matmul_fsm := false.B

  val raw_cmd = raw_cmd_q.io.deq

  val max_lds = reservation_station_entries_ld
  val max_exs = reservation_station_entries_ex
  val max_sts = reservation_station_entries_st

  val (conv_cmd, loop_conv_unroller_busy) = if (has_loop_conv) withClock (gated_clock) { LoopConv(raw_cmd, reservation_station.io.conv_ld_completed, reservation_station.io.conv_st_completed, reservation_station.io.conv_ex_completed,
    meshRows*tileRows, coreMaxAddrBits, reservation_station_entries, max_lds, max_exs, max_sts, sp_banks * sp_bank_entries, acc_banks * acc_bank_entries,
    inputType.getWidth, accType.getWidth, dma_maxbytes,
    new ConfigMvinRs1(mvin_scale_t_bits, block_stride_bits, pixel_repeats_bits), new MvinRs2(mvin_rows_bits, mvin_cols_bits, local_addr_t),
    new ConfigMvoutRs2(acc_scale_t_bits, 32), new MvoutRs2(mvout_rows_bits, mvout_cols_bits, local_addr_t),
    new ConfigExRs1(acc_scale_t_bits), new PreloadRs(mvin_rows_bits, mvin_cols_bits, local_addr_t),
    new PreloadRs(mvout_rows_bits, mvout_cols_bits, local_addr_t),
    new ComputeRs(mvin_rows_bits, mvin_cols_bits, local_addr_t), new ComputeRs(mvin_rows_bits, mvin_cols_bits, local_addr_t),
    has_training_convs, has_max_pool, has_first_layer_optimizations, has_dw_convs) }
  else (raw_cmd, false.B)

  val (loop_cmd, loop_matmul_unroller_busy, loop_completed) = withClock (gated_clock) { LoopMatmul(if (has_loop_conv) conv_cmd else raw_cmd, reservation_station.io.matmul_ld_completed, reservation_station.io.matmul_st_completed, reservation_station.io.matmul_ex_completed,
    meshRows*tileRows, coreMaxAddrBits, reservation_station_entries, max_lds, max_exs, max_sts, sp_banks * sp_bank_entries, acc_banks * acc_bank_entries,
    inputType.getWidth, accType.getWidth, dma_maxbytes, new MvinRs2(mvin_rows_bits, mvin_cols_bits, local_addr_t),
    new PreloadRs(mvin_rows_bits, mvin_cols_bits, local_addr_t), new PreloadRs(mvout_rows_bits, mvout_cols_bits, local_addr_t),
    new ComputeRs(mvin_rows_bits, mvin_cols_bits, local_addr_t), new ComputeRs(mvin_rows_bits, mvin_cols_bits, local_addr_t),
    new MvoutSpadRs1(32, local_addr_t), new MvoutRs2(mvout_rows_bits, mvout_cols_bits, local_addr_t)) }

  val unrolled_cmd = Queue(loop_cmd)
  unrolled_cmd.ready := false.B

  // changed: LOOP_WS / LOOP_CONV_WS are illegal in sparse mode; track rs1[10] of the last full config_ex in program order
  if (nm_sparsity.isDefined) {
    val nm_loop_sparse = RegInit(false.B)
    val rc_funct = raw_cmd.bits.cmd.inst.funct
    val rc_cfg = raw_cmd.bits.cmd.rs1.asTypeOf(new ConfigExRs1(acc_scale_t_bits))
    when (raw_cmd.fire) {
      when (rc_funct === CONFIG_CMD && rc_cfg.cmd_type === CONFIG_EX && !rc_cfg.set_only_strides.asBool) {
        nm_loop_sparse := rc_cfg.nm_sparse.asBool
      }
      assert(!(nm_loop_sparse && (rc_funct === LOOP_WS || rc_funct === LOOP_CONV_WS)),
        "LOOP_WS / LOOP_CONV_WS issued while n:m sparse mode (config_ex rs1[10]) is on")
    }
  }
  counters.io.event_io.connectEventSignal(CounterEvent.LOOP_MATMUL_ACTIVE_CYCLES, loop_matmul_unroller_busy)

  // Wire up controllers to ROB
  reservation_station.io.alloc.valid := false.B
  reservation_station.io.alloc.bits := unrolled_cmd.bits

  val completion_io = IO(new Bundle {
    val completed = Output(loop_completed.cloneType)
  })

  completion_io.completed := loop_completed

  /*
  //-------------------------------------------------------------------------
  // finish muxing control signals to rob (risc) or tiler (cisc)
  when (raw_cmd.valid && is_cisc_funct && !rob.io.busy) {
    is_cisc_mode       := true.B
    raw_cisc_cmd.valid := true.B
    raw_cmd.ready      := raw_cisc_cmd.ready
  }
  .elsewhen (raw_cmd.valid && !is_cisc_funct && !tiler.io.busy) {
    is_cisc_mode       := false.B
    raw_risc_cmd.valid := true.B
    raw_cmd.ready      := raw_risc_cmd.ready
  }
  */

  //=========================================================================
  // Controllers
  //=========================================================================
  val load_controller = withClock (gated_clock) { Module(new LoadController(outer.config, coreMaxAddrBits, local_addr_t)) }
  val store_controller = withClock (gated_clock) { Module(new StoreController(outer.config, coreMaxAddrBits, local_addr_t)) }
  val ex_controller = withClock (gated_clock) { Module(new ExecuteController(xLen, tagWidth, outer.config)) }

  counters.io.event_io.collect(load_controller.io.counter)
  counters.io.event_io.collect(store_controller.io.counter)
  counters.io.event_io.collect(ex_controller.io.counter)

  /*
  tiler.io.issue.load.ready := false.B
  tiler.io.issue.store.ready := false.B
  tiler.io.issue.exec.ready := false.B
  */

  reservation_station.io.issue.ld.ready := false.B
  reservation_station.io.issue.st.ready := false.B
  reservation_station.io.issue.ex.ready := false.B

  /*
  when (is_cisc_mode) {
    load_controller.io.cmd  <> tiler.io.issue.load
    store_controller.io.cmd <> tiler.io.issue.store
    ex_controller.io.cmd  <> tiler.io.issue.exec
  }
  .otherwise {
    load_controller.io.cmd.valid := rob.io.issue.ld.valid
    rob.io.issue.ld.ready := load_controller.io.cmd.ready
    load_controller.io.cmd.bits.cmd := rob.io.issue.ld.cmd
    load_controller.io.cmd.bits.cmd.inst.funct := rob.io.issue.ld.cmd.inst.funct
    load_controller.io.cmd.bits.rob_id.push(rob.io.issue.ld.rob_id)

    store_controller.io.cmd.valid := rob.io.issue.st.valid
    rob.io.issue.st.ready := store_controller.io.cmd.ready
    store_controller.io.cmd.bits.cmd := rob.io.issue.st.cmd
    store_controller.io.cmd.bits.cmd.inst.funct := rob.io.issue.st.cmd.inst.funct
    store_controller.io.cmd.bits.rob_id.push(rob.io.issue.st.rob_id)

    ex_controller.io.cmd.valid := rob.io.issue.ex.valid
    rob.io.issue.ex.ready := ex_controller.io.cmd.ready
    ex_controller.io.cmd.bits.cmd := rob.io.issue.ex.cmd
    ex_controller.io.cmd.bits.cmd.inst.funct := rob.io.issue.ex.cmd.inst.funct
    ex_controller.io.cmd.bits.rob_id.push(rob.io.issue.ex.rob_id)
  }
  */

  load_controller.io.cmd.valid := reservation_station.io.issue.ld.valid
  reservation_station.io.issue.ld.ready := load_controller.io.cmd.ready
  load_controller.io.cmd.bits := reservation_station.io.issue.ld.cmd
  load_controller.io.cmd.bits.rob_id.push(reservation_station.io.issue.ld.rob_id)

  store_controller.io.cmd.valid := reservation_station.io.issue.st.valid
  reservation_station.io.issue.st.ready := store_controller.io.cmd.ready
  store_controller.io.cmd.bits := reservation_station.io.issue.st.cmd
  store_controller.io.cmd.bits.rob_id.push(reservation_station.io.issue.st.rob_id)

  ex_controller.io.cmd.valid := reservation_station.io.issue.ex.valid
  // changed: index rows for n:m sparse mode, driven by the dispatch below
  ex_controller.io.nm_meta_in.valid := false.B
  ex_controller.io.nm_meta_in.bits.rs1 := unrolled_cmd.bits.cmd.rs1
  ex_controller.io.nm_meta_in.bits.rs2 := unrolled_cmd.bits.cmd.rs2
  ex_controller.io.nm_clear := false.B
  reservation_station.io.issue.ex.ready := ex_controller.io.cmd.ready
  ex_controller.io.cmd.bits := reservation_station.io.issue.ex.cmd
  ex_controller.io.cmd.bits.rob_id.push(reservation_station.io.issue.ex.rob_id)

  // Wire up scratchpad to controllers
  spad.module.io.dma.read <> load_controller.io.dma
  spad.module.io.dma.write <> store_controller.io.dma
  ex_controller.io.srams.read <> spad.module.io.srams.read
  ex_controller.io.srams.write <> spad.module.io.srams.write
  spad.module.io.acc.read_req <> ex_controller.io.acc.read_req
  ex_controller.io.acc.read_resp <> spad.module.io.acc.read_resp
  ex_controller.io.acc.write <> spad.module.io.acc.write

  // Im2Col unit
  val im2col = withClock (gated_clock) { Module(new Im2Col(outer.config)) }

  // Wire up Im2col
  counters.io.event_io.collect(im2col.io.counter)
  // im2col.io.sram_reads <> spad.module.io.srams.read
  im2col.io.req <> ex_controller.io.im2col.req
  ex_controller.io.im2col.resp <> im2col.io.resp

  // Wire arbiter for ExecuteController and Im2Col scratchpad reads
  (ex_controller.io.srams.read, im2col.io.sram_reads, spad.module.io.srams.read).zipped.foreach { case (ex_read, im2col_read, spad_read) =>
    val req_arb = Module(new Arbiter(new ScratchpadReadReq(n=sp_bank_entries), 2))

    req_arb.io.in(0) <> ex_read.req
    req_arb.io.in(1) <> im2col_read.req

    spad_read.req <> req_arb.io.out

    // TODO if necessary, change how the responses are handled when fromIm2Col is added to spad read interface

    ex_read.resp.valid := spad_read.resp.valid
    im2col_read.resp.valid := spad_read.resp.valid

    ex_read.resp.bits := spad_read.resp.bits
    im2col_read.resp.bits := spad_read.resp.bits

    spad_read.resp.ready := ex_read.resp.ready || im2col_read.resp.ready
  }

  // Wire up controllers to ROB
  reservation_station.io.alloc.valid := false.B
  // rob.io.alloc.bits := compressed_cmd.bits
  reservation_station.io.alloc.bits := unrolled_cmd.bits

  /*
  //=========================================================================
  // committed insn return path to frontends
  //=========================================================================

  //-------------------------------------------------------------------------
  // cisc
  tiler.io.completed.exec.valid := ex_controller.io.completed.valid
  tiler.io.completed.exec.bits := ex_controller.io.completed.bits

  tiler.io.completed.load <> load_controller.io.completed
  tiler.io.completed.store <> store_controller.io.completed

  // mux with cisc frontend arbiter
  tiler.io.completed.exec.valid  := ex_controller.io.completed.valid && is_cisc_mode
  tiler.io.completed.load.valid  := load_controller.io.completed.valid && is_cisc_mode
  tiler.io.completed.store.valid := store_controller.io.completed.valid && is_cisc_mode
  */

  //-------------------------------------------------------------------------
  // risc
  // changed: Fix A, a 4th (lowest priority) input for the n:m index loader's completions
  val reservation_station_completed_arb = Module(new Arbiter(UInt(log2Up(reservation_station_entries).W), if (nm_sparsity.isDefined) 4 else 3))

  reservation_station_completed_arb.io.in(0).valid := ex_controller.io.completed.valid
  reservation_station_completed_arb.io.in(0).bits := ex_controller.io.completed.bits

  reservation_station_completed_arb.io.in(1) <> load_controller.io.completed
  reservation_station_completed_arb.io.in(2) <> store_controller.io.completed

  // mux with cisc frontend arbiter
  reservation_station_completed_arb.io.in(0).valid := ex_controller.io.completed.valid // && !is_cisc_mode
  reservation_station_completed_arb.io.in(1).valid := load_controller.io.completed.valid // && !is_cisc_mode
  reservation_station_completed_arb.io.in(2).valid := store_controller.io.completed.valid // && !is_cisc_mode
  if (nm_sparsity.isDefined) {
    reservation_station_completed_arb.io.in(3) <> ex_controller.io.nm_ld_completed // changed: Fix A
  } else {
    ex_controller.io.nm_ld_completed.ready := false.B // changed: Fix A, never valid on a dense build
  }

  reservation_station.io.completed.valid := reservation_station_completed_arb.io.out.valid
  reservation_station.io.completed.bits := reservation_station_completed_arb.io.out.bits
  reservation_station_completed_arb.io.out.ready := true.B

  // Wire up global RoCC signals
  io.busy := raw_cmd.valid || loop_conv_unroller_busy || loop_matmul_unroller_busy || reservation_station.io.busy || spad.module.io.busy || unrolled_cmd.valid || loop_cmd.valid || conv_cmd.valid

  io.interrupt := tlb.io.exp.map(_.interrupt).reduce(_ || _)

  // assert(!io.interrupt, "Interrupt handlers have not been written yet")

  // Cycle counters
  val incr_ld_cycles = load_controller.io.busy && !store_controller.io.busy && !ex_controller.io.busy
  val incr_st_cycles = !load_controller.io.busy && store_controller.io.busy && !ex_controller.io.busy
  val incr_ex_cycles = !load_controller.io.busy && !store_controller.io.busy && ex_controller.io.busy

  val incr_ld_st_cycles = load_controller.io.busy && store_controller.io.busy && !ex_controller.io.busy
  val incr_ld_ex_cycles = load_controller.io.busy && !store_controller.io.busy && ex_controller.io.busy
  val incr_st_ex_cycles = !load_controller.io.busy && store_controller.io.busy && ex_controller.io.busy

  val incr_ld_st_ex_cycles = load_controller.io.busy && store_controller.io.busy && ex_controller.io.busy

  counters.io.event_io.connectEventSignal(CounterEvent.MAIN_LD_CYCLES, incr_ld_cycles)
  counters.io.event_io.connectEventSignal(CounterEvent.MAIN_ST_CYCLES, incr_st_cycles)
  counters.io.event_io.connectEventSignal(CounterEvent.MAIN_EX_CYCLES, incr_ex_cycles)
  counters.io.event_io.connectEventSignal(CounterEvent.MAIN_LD_ST_CYCLES, incr_ld_st_cycles)
  counters.io.event_io.connectEventSignal(CounterEvent.MAIN_LD_EX_CYCLES, incr_ld_ex_cycles)
  counters.io.event_io.connectEventSignal(CounterEvent.MAIN_ST_EX_CYCLES, incr_st_ex_cycles)
  counters.io.event_io.connectEventSignal(CounterEvent.MAIN_LD_ST_EX_CYCLES, incr_ld_st_ex_cycles)

  // Issue commands to controllers
  // TODO we combinationally couple cmd.ready and cmd.valid signals here
  // when (compressed_cmd.valid) {
  when (unrolled_cmd.valid) {
    // val config_cmd_type = cmd.bits.rs1(1,0) // TODO magic numbers

    //val funct = unrolled_cmd.bits.inst.funct
    val risc_funct = unrolled_cmd.bits.cmd.inst.funct

    val is_flush = risc_funct === FLUSH_CMD
    val is_counter_op = risc_funct === COUNTER_OP
    val is_clock_gate_en = risc_funct === CLKGATE_EN
    val is_nm_meta = risc_funct === NM_META_CMD // changed: n:m index rows skip the reservation station

    /*
    val is_load = (funct === LOAD_CMD) || (funct === CONFIG_CMD && config_cmd_type === CONFIG_LOAD)
    val is_store = (funct === STORE_CMD) || (funct === CONFIG_CMD && config_cmd_type === CONFIG_STORE)
    val is_ex = (funct === COMPUTE_AND_FLIP_CMD || funct === COMPUTE_AND_STAY_CMD || funct === PRELOAD_CMD) ||
    (funct === CONFIG_CMD && config_cmd_type === CONFIG_EX)
    */

    when (is_flush) {
      val skip = unrolled_cmd.bits.cmd.rs1(0)
      tlb.io.exp.foreach(_.flush_skip := skip)
      tlb.io.exp.foreach(_.flush_retry := !skip)

      unrolled_cmd.ready := true.B // TODO should we wait for an acknowledgement from the TLB?
    }

    .elsewhen (is_counter_op) {
      // If this is a counter access/configuration command, execute immediately
      counters.io.in.valid := unrolled_cmd.valid
      unrolled_cmd.ready := counters.io.in.ready
      counters.io.in.bits := unrolled_cmd.bits.cmd
    }

    .elsewhen (is_clock_gate_en) {
      unrolled_cmd.ready := true.B
    }

    // changed: index rows skip the reservation station; preloads pop them in program order
    .elsewhen (is_nm_meta) {
      if (nm_sparsity.isEmpty) {
        assert(false.B, "NM_META_CMD (funct 26) on a dense build (nm_sparsity = None)")
      }
      when (unrolled_cmd.bits.cmd.inst.rd === 1.U) {
        // changed: rd == 1 is a clear: empty the index fifo and the row assembler (software fences first)
        ex_controller.io.nm_clear := true.B
        unrolled_cmd.ready := true.B
      }.elsewhen (nm_sparsity.isDefined.B && unrolled_cmd.bits.cmd.inst.rd === 2.U) {
        // changed: Fix A. rd == 2 loads index tiles from the scratchpad. It goes through the reservation station (ex
        // queue) so it is ordered against the mvins of its rows; the ExecuteController hands it to its index loader.
        reservation_station.io.alloc.valid := true.B
        when (reservation_station.io.alloc.fire) {
          unrolled_cmd.ready := true.B
        }
      }.elsewhen (nm_sparsity.isDefined.B && unrolled_cmd.bits.cmd.inst.rd =/= 0.U) {
        // changed: Fix A. rd 3..31 are reserved: assert and drop
        assert(false.B, "NM_META_CMD (funct 26): rd values 3..31 are reserved")
        unrolled_cmd.ready := true.B
      }.otherwise {
        ex_controller.io.nm_meta_in.valid := true.B
        unrolled_cmd.ready := ex_controller.io.nm_meta_in.ready
      }
    }

    .otherwise {
      reservation_station.io.alloc.valid := true.B

      when(reservation_station.io.alloc.fire) {
        // compressed_cmd.ready := true.B
        unrolled_cmd.ready := true.B
      }
    }
  }

  // changed: index fifo deadlock check. While index rows are blocked, any preload that could pop the fifo sits in the
  // RS with its compute (2+ entries, busy), so a blocked fifo with a non-busy RS can never drain.
  if (nm_sparsity.isDefined) {
    val nm_dead_cnt = RegInit(0.U(8.W))
    val nm_dead = ex_controller.io.nm_meta_in.valid && !ex_controller.io.nm_meta_in.ready && !reservation_station.io.busy
    nm_dead_cnt := Mux(nm_dead, nm_dead_cnt + 1.U, 0.U)
    assert(nm_dead_cnt =/= 255.U, "n:m index fifo deadlock: more than 4 index tiles sent ahead of their preloads")
  }

  // changed: Fix A. Program-order model of the index fifo, the same one the Spike model keeps, checked as commands
  // are dispatched (fix-a-spec.md sections 5 and 7). nm_q = tiles sent minus tiles popped by real sparse preloads;
  // nm_in_flight = a real sparse preload popped a tile and no compute has followed yet (its slot is still held).
  if (nm_sparsity.isDefined) {
    val c = unrolled_cmd.bits.cmd
    val f = c.inst.funct
    val is_nm = f === NM_META_CMD
    val nm_q = RegInit(0.U(4.W))
    val nm_in_flight = RegInit(false.B)
    val nm_mode = RegInit(false.B) // sparse mode of the last full config_ex dispatched
    val nm_ld_since_clear = RegInit(false.B)
    val nm_ld_pending = RegInit(0.U(log2Up(reservation_station_entries + 1).W)) // rd == 2 loads not finished
    val nm_room = nm_meta_fifo_depth.U -& (nm_q +& nm_in_flight)

    val sp_addr_bits = log2Ceil(sp_banks * sp_bank_entries)
    val sp_bank_row_bits = log2Ceil(sp_bank_entries)
    val ld_n = c.rs2(7, 0)
    val ld_first = c.rs1(sp_addr_bits - 1, 0)
    val ld_last = ld_first +& (ld_n * nm_spad_rows_per_tile.U) - 1.U

    val cfg = c.rs1.asTypeOf(new ConfigExRs1(acc_scale_t_bits))
    val cfg_is_ws = if (dataflow == Dataflow.BOTH) cfg.dataflow === Dataflow.WS.id.U else (dataflow == Dataflow.WS).B
    val d_garbage = c.rs1(31, 0).asTypeOf(local_addr_t).is_garbage()

    when (unrolled_cmd.fire) {
      when (is_nm && c.inst.rd === 2.U) {
        assert(ld_n >= 1.U && ld_n <= nm_meta_fifo_depth.U, "NM_META_CMD rd == 2: tile count rs2[7:0] must be 1..4")
        assert(c.rs1(63, 32) === 0.U && c.rs2(63, 8) === 0.U, "NM_META_CMD rd == 2: rs1[63:32] and rs2[63:8] must be 0")
        assert(c.rs1(31, sp_addr_bits) === 0.U,
          "NM_META_CMD rd == 2: rs1 must be a scratchpad row address (not accumulator, not GARBAGE)")
        assert(ld_n === 0.U || (ld_first >> sp_bank_row_bits) === (ld_last >> sp_bank_row_bits),
          "NM_META_CMD rd == 2: the index rows must all lie in one scratchpad bank")
        assert(!ex_controller.io.nm_asm_busy, "NM_META_CMD rd == 2 while a register-path index tile is half sent")
        assert(ld_n <= nm_room, "n:m index fifo: more than 4 index tiles ahead of their preloads")
        nm_q := nm_q + ld_n
        nm_ld_since_clear := true.B
      }
      when (is_nm && c.inst.rd === 1.U) {
        assert(nm_ld_pending === 0.U, "NM_META_CMD clear (rd == 1) while a scratchpad index load is outstanding: fence first")
        assert(!nm_in_flight, "NM_META_CMD clear (rd == 1) between a sparse preload and its compute: fence first")
        nm_q := 0.U
        nm_ld_since_clear := false.B
      }
      when (is_nm && c.inst.rd === 0.U) {
        assert(!nm_ld_since_clear,
          "NM_META_CMD register rows (rd == 0) after a scratchpad index load (rd == 2): gemmini_nm_reset() first")
      }
      when (f === CONFIG_CMD && cfg.cmd_type === CONFIG_EX && !cfg.set_only_strides.asBool) {
        nm_mode := cfg.nm_sparse.asBool && cfg_is_ws && !cfg.a_transpose.asBool && !cfg.b_transpose.asBool
      }
      when (f === PRELOAD_CMD && nm_mode && !d_garbage) {
        assert(nm_q =/= 0.U, "n:m sparse preload with no index tile sent: index tile must be sent before its preload")
        nm_q := nm_q - 1.U
        nm_in_flight := true.B
      }
      when (f === COMPUTE_AND_FLIP_CMD || f === COMPUTE_AND_STAY_CMD) {
        nm_in_flight := false.B
      }
    }
    // a register-path tile is complete in the same cycle its last row command is dispatched
    when (ex_controller.io.nm_reg_tile) {
      assert(nm_room =/= 0.U, "n:m index fifo: more than 4 index tiles ahead of their preloads")
      nm_q := nm_q + 1.U
    }

    val ld_dispatch = unrolled_cmd.fire && is_nm && c.inst.rd === 2.U
    nm_ld_pending := nm_ld_pending + ld_dispatch - ex_controller.io.nm_ld_completed.fire
    // a load can sit in the reservation station as its only entry while a lone preload makes it report not busy
    when (nm_ld_pending =/= 0.U) {
      io.busy := true.B
    }
  }

  // Debugging signals
  val pipeline_stall_counter = RegInit(0.U(32.W))
  when (io.cmd.fire) {
    pipeline_stall_counter := 0.U
  }.elsewhen(io.busy) {
    pipeline_stall_counter := pipeline_stall_counter + 1.U
  }
  assert(pipeline_stall_counter < 10000000.U, "pipeline stall")

  /*
  //=========================================================================
  // Wire up global RoCC signals
  //=========================================================================
  io.busy := raw_cmd.valid || unrolled_cmd.valid || rob.io.busy || spad.module.io.busy || tiler.io.busy
  io.interrupt := tlb.io.exp.interrupt

  // hack
  when(is_cisc_mode || !(unrolled_cmd.valid || rob.io.busy || tiler.io.busy)){
    tlb.io.exp.flush_retry := cmd_fsm.io.flush_retry
    tlb.io.exp.flush_skip  := cmd_fsm.io.flush_skip
  }
  */

  //=========================================================================
  // Performance Counters Access
  //=========================================================================

}
