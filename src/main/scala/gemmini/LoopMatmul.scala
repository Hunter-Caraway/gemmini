
package gemmini

import chisel3._
import chisel3.util._
import chisel3.experimental._
import freechips.rocketchip.tile.RoCCCommand
import org.chipsalliance.cde.config.Parameters
import GemminiISA._
import LocalAddr._
import Util._

// LdA

class LoopMatmulLdAReq(val block_size: Int, val coreMaxAddrBits: Int, val iterator_bitwidth: Int, val max_addr: Int, val concurrent_loops: Int) extends Bundle {
  val max_i = UInt(iterator_bitwidth.W)
  val max_k = UInt(iterator_bitwidth.W)
  val pad_i = UInt(log2Up(block_size).W)
  val pad_k = UInt(log2Up(block_size).W)
  val dram_addr = UInt(coreMaxAddrBits.W)
  val dram_stride = UInt(coreMaxAddrBits.W)
  val transpose = Bool()
  val addr_start = UInt(log2Up(max_addr).W)
  val loop_id = UInt(log2Up(concurrent_loops).W)
  val is_resadd = Bool()
}

class LoopMatmulLdA(block_size: Int, coreMaxAddrBits: Int, iterator_bitwidth: Int, max_addr: Int, input_w: Int,
                    max_block_len: Int, concurrent_loops: Int, mvin_rs2_t: MvinRs2)
                   (implicit p: Parameters) extends Module {
  val io = IO(new Bundle {
    val req = Flipped(Decoupled(new LoopMatmulLdAReq(block_size, coreMaxAddrBits, iterator_bitwidth, max_addr, concurrent_loops)))
    val cmd = Decoupled(Output(new RoCCCommand))
    val i = Output(UInt(iterator_bitwidth.W))
    val k = Output(UInt(iterator_bitwidth.W))
    val idle = Output(Bool())
    val rob_overloaded = Input(Bool())
    val loop_id = Output(UInt(log2Up(concurrent_loops).W))
  })

  object State extends ChiselEnum {
    val idle, ld = Value
  }
  import State._
  val state = RegInit(idle)

  val req = Reg(new LoopMatmulLdAReq(block_size, coreMaxAddrBits, iterator_bitwidth, max_addr, concurrent_loops))

  val i = Reg(UInt(iterator_bitwidth.W))
  val k = Reg(UInt(iterator_bitwidth.W))

  val row_iterator = Mux(req.transpose, k, i)
  val col_iterator = Mux(req.transpose, i, k)

  val max_row_iterator = Mux(req.transpose, req.max_k, req.max_i)
  val max_col_iterator = Mux(req.transpose, req.max_i, req.max_k)

  val row_pad = Mux(req.transpose, req.pad_k, req.pad_i)
  val col_pad = Mux(req.transpose, req.pad_i, req.pad_k)

  val max_col_dim = Mux(req.transpose, req.max_i, req.max_k)
  val max_blocks = Mux(max_col_dim <= max_block_len.U, max_col_dim, max_block_len.U)

  val sp_addr_start = req.addr_start

  val dram_offset = (row_iterator * req.dram_stride + col_iterator) * block_size.U * (input_w/8).U
  val dram_addr = req.dram_addr + LoopMatmul.castDramOffset(dram_offset)
  val sp_addr = sp_addr_start + (row_iterator * max_col_iterator + col_iterator) * block_size.U
  val blocks = Mux(col_iterator + max_blocks <= max_col_iterator, max_blocks, max_col_iterator-col_iterator)
  val cols = (blocks * block_size.U) - Mux(col_iterator + blocks >= max_col_iterator, col_pad, 0.U)
  val rows = block_size.U - Mux(row_iterator === max_row_iterator-1.U, row_pad, 0.U)

  val mvin_cmd = Wire(new RoCCCommand)
  mvin_cmd := DontCare
  mvin_cmd.inst.funct := LOAD_CMD
  mvin_cmd.rs1 := dram_addr

  val mvin_cmd_rs2 = Wire(mvin_rs2_t.cloneType)
  mvin_cmd_rs2 := DontCare
  mvin_cmd_rs2.num_rows := rows.asUInt
  mvin_cmd_rs2.num_cols := cols.asUInt
  mvin_cmd_rs2.local_addr := cast_to_sp_addr(mvin_cmd_rs2.local_addr, sp_addr)
  mvin_cmd.rs2 := mvin_cmd_rs2.asUInt
  when(req.is_resadd){
    mvin_cmd_rs2.local_addr := cast_to_acc_addr(mvin_cmd_rs2.local_addr, sp_addr, accumulate = false.B, read_full = false.B)
  }

  io.req.ready := state === idle
  io.i := i
  io.k := k
  io.idle := state === idle

  io.cmd.valid := state =/= idle && !io.rob_overloaded && req.dram_addr =/= 0.U
  io.cmd.bits := mvin_cmd

  io.loop_id := req.loop_id

  when(req.dram_addr === 0.U){
    state := idle
  }.elsewhen(io.cmd.fire) {
    // The order here is k, j, i
    val i_blocks = Mux(req.transpose, max_blocks, 1.U)
    val k_blocks = Mux(req.transpose, 1.U, max_blocks)

    val next_i = floorAdd(i, i_blocks, req.max_i)
    val next_k = floorAdd(k, k_blocks, req.max_k, next_i === 0.U)

    i := next_i
    k := next_k

    when (next_i === 0.U && next_k === 0.U) {
      state := idle
    }
  }

  when (io.req.fire) {
    req := io.req.bits
    state := ld
    i := 0.U
    k := 0.U
  }
}

// LdB

class LoopMatmulLdBReq(val block_size: Int, val coreMaxAddrBits: Int, val iterator_bitwidth: Int, val max_addr: Int, val concurrent_loops: Int) extends Bundle {
  val max_k = UInt(iterator_bitwidth.W)
  val max_j = UInt(iterator_bitwidth.W)
  val pad_k = UInt(log2Up(block_size).W)
  val pad_j = UInt(log2Up(block_size).W)
  val dram_addr = UInt(coreMaxAddrBits.W)
  val dram_stride = UInt(coreMaxAddrBits.W)
  val transpose = Bool()
  val addr_end = UInt(log2Up(max_addr+1).W)
  val loop_id = UInt(log2Up(concurrent_loops).W)
  val is_resadd = Bool()
}

class LoopMatmulLdB(block_size: Int, coreMaxAddrBits: Int, iterator_bitwidth: Int, max_addr: Int, input_w: Int,
                    max_block_len: Int, concurrent_loops: Int, mvin_rs2_t: MvinRs2)
                   (implicit p: Parameters) extends Module {
  val io = IO(new Bundle {
    val req = Flipped(Decoupled(new LoopMatmulLdBReq(block_size, coreMaxAddrBits, iterator_bitwidth, max_addr, concurrent_loops)))
    val cmd = Decoupled(Output(new RoCCCommand))

    val k = Output(UInt(iterator_bitwidth.W))
    val j = Output(UInt(iterator_bitwidth.W))

    val idle = Output(Bool())
    val rob_overloaded = Input(Bool())

    val loop_id = Output(UInt(log2Up(concurrent_loops).W))
  })

  object State extends ChiselEnum {
    val idle, ld = Value
  }
  import State._
  val state = RegInit(idle)

  val req = Reg(new LoopMatmulLdBReq(block_size, coreMaxAddrBits, iterator_bitwidth, max_addr, concurrent_loops))

  val k = Reg(UInt(iterator_bitwidth.W))
  val j = Reg(UInt(iterator_bitwidth.W))

  val row_iterator = Mux(req.transpose, j, k)
  val col_iterator = Mux(req.transpose, k, j)

  val max_row_iterator = Mux(req.transpose, req.max_j, req.max_k)
  val max_col_iterator = Mux(req.transpose, req.max_k, req.max_j)

  val row_pad = Mux(req.transpose, req.pad_j, req.pad_k)
  val col_pad = Mux(req.transpose, req.pad_k, req.pad_j)

  val max_col_dim = Mux(req.transpose, req.max_k, req.max_j)
  val max_blocks = Mux(max_col_dim <= max_block_len.U, max_col_dim, max_block_len.U)

  val sp_addr_start = Mux(req.is_resadd, req.addr_end, req.addr_end - req.max_k * req.max_j * block_size.U)

  val dram_offset = (row_iterator * req.dram_stride + col_iterator) * block_size.U * (input_w/8).U
  val dram_addr = req.dram_addr + LoopMatmul.castDramOffset(dram_offset)
  val sp_addr = sp_addr_start + (row_iterator * max_col_iterator + col_iterator) * block_size.U
  val blocks = Mux(col_iterator + max_blocks <= max_col_iterator, max_blocks, max_col_iterator-col_iterator)
  val cols = (blocks * block_size.U) - Mux(col_iterator + blocks >= max_col_iterator, col_pad, 0.U)
  val rows = block_size.U - Mux(max_row_iterator === max_row_iterator-1.U, row_pad, 0.U)

  val mvin_cmd = Wire(new RoCCCommand)
  mvin_cmd := DontCare
  mvin_cmd.inst.funct := LOAD2_CMD
  mvin_cmd.rs1 := dram_addr

  val mvin_cmd_rs2 = Wire(mvin_rs2_t.cloneType)
  mvin_cmd_rs2 := DontCare
  mvin_cmd_rs2.num_rows := rows.asUInt
  mvin_cmd_rs2.num_cols := cols.asUInt
  mvin_cmd_rs2.local_addr := cast_to_sp_addr(mvin_cmd_rs2.local_addr, sp_addr)
  mvin_cmd.rs2 := mvin_cmd_rs2.asUInt

  when (req.is_resadd){
    mvin_cmd_rs2.local_addr := cast_to_acc_addr(mvin_cmd_rs2.local_addr, sp_addr, accumulate = true.B, read_full = false.B)
  }

  io.req.ready := state === idle
  io.k := k
  io.j := j
  io.idle := state === idle

  io.cmd.valid := state =/= idle && !io.rob_overloaded && req.dram_addr =/= 0.U
  io.cmd.bits := mvin_cmd

  io.loop_id := req.loop_id

  when(req.dram_addr === 0.U){
    state := idle
  }.elsewhen(io.cmd.fire) {
    // The order here is k, j, i
    val j_blocks = Mux(req.transpose, 1.U, max_blocks)
    val k_blocks = Mux(req.transpose, max_blocks, 1.U)

    val next_j = floorAdd(j, j_blocks, req.max_j)
    val next_k = floorAdd(k, k_blocks, req.max_k, next_j === 0.U)

    j := next_j
    k := next_k

    when (next_j === 0.U && next_k === 0.U) {
      state := idle
    }
  }

  when (io.req.fire) {
    req := io.req.bits
    state := ld
    j := 0.U
    k := 0.U
  }
}

// LdD

class LoopMatmulLdDReq(val block_size: Int, val coreMaxAddrBits: Int, val iterator_bitwidth: Int, val max_acc_addr: Int, val concurrent_loops: Int) extends Bundle {
  val max_j = UInt(iterator_bitwidth.W)
  val max_i = UInt(iterator_bitwidth.W)
  val pad_j = UInt(log2Up(block_size).W)
  val pad_i = UInt(log2Up(block_size).W)
  val dram_addr = UInt(coreMaxAddrBits.W)
  val dram_stride = UInt(coreMaxAddrBits.W)
  val low_d = Bool()
  val addr_start = UInt(log2Up(max_acc_addr).W)
  val loop_id = UInt(log2Up(concurrent_loops).W)
}

class LoopMatmulLdD(block_size: Int, coreMaxAddrBits: Int, iterator_bitwidth: Int, max_acc_addr: Int, input_w: Int,
                    acc_w: Int, max_block_len: Int, max_block_len_acc: Int, concurrent_loops: Int, mvin_rs2_t: MvinRs2)
                   (implicit p: Parameters) extends Module {
  val io = IO(new Bundle {
    val req = Flipped(Decoupled(new LoopMatmulLdDReq(block_size, coreMaxAddrBits, iterator_bitwidth, max_acc_addr, concurrent_loops)))
    val cmd = Decoupled(Output(new RoCCCommand))

    val idle = Output(Bool())
    val rob_overloaded = Input(Bool())

    val loop_id = Output(UInt(log2Up(concurrent_loops).W))
  })

  object State extends ChiselEnum {
    val idle, ld = Value
  }
  import State._
  val state = RegInit(idle)

  val req = Reg(new LoopMatmulLdDReq(block_size, coreMaxAddrBits, iterator_bitwidth, max_acc_addr, concurrent_loops))

  val max_blocks = Mux(req.low_d, Mux(req.max_j <= max_block_len.U, req.max_j, max_block_len.U),
    Mux(req.max_j <= max_block_len_acc.U, req.max_j, max_block_len_acc.U))

  val j = Reg(UInt(iterator_bitwidth.W))
  val i = Reg(UInt(iterator_bitwidth.W))

  val acc_addr_start = req.addr_start

  val dram_offset = Mux(req.low_d, (i * req.dram_stride + j) * block_size.U * (input_w/8).U,
    (i * req.dram_stride + j) * block_size.U * (acc_w/8).U)
  val dram_addr = req.dram_addr + LoopMatmul.castDramOffset(dram_offset)
  val sp_addr = acc_addr_start + (i * req.max_j + j) * block_size.U
  val blocks = Mux(j + max_blocks <= req.max_j, max_blocks, req.max_j-j)
  val cols = (blocks * block_size.U) - Mux(j + blocks >= req.max_j, req.pad_j, 0.U)
  val rows = block_size.U - Mux(i === req.max_i-1.U, req.pad_i, 0.U)

  val mvin_cmd = Wire(new RoCCCommand)
  mvin_cmd := DontCare
  mvin_cmd.inst.funct := LOAD3_CMD
  mvin_cmd.rs1 := dram_addr

  val mvin_cmd_rs2 = Wire(mvin_rs2_t.cloneType)
  mvin_cmd_rs2 := DontCare
  mvin_cmd_rs2.num_rows := rows.asUInt
  mvin_cmd_rs2.num_cols := cols.asUInt
  mvin_cmd_rs2.local_addr := cast_to_acc_addr(mvin_cmd_rs2.local_addr, sp_addr, accumulate = false.B, read_full = false.B)
  mvin_cmd.rs2 := mvin_cmd_rs2.asUInt

  io.req.ready := state === idle
  io.idle := state === idle

  // The order here is k, j, i
  io.cmd.valid := state =/= idle && !io.rob_overloaded && req.dram_addr =/= 0.U
  io.cmd.bits := mvin_cmd

  io.loop_id := req.loop_id

  when (req.dram_addr === 0.U) {
    state := idle
  }.elsewhen (io.cmd.fire) {
    // The order here is k, j, i
    val next_i = floorAdd(i, 1.U, req.max_i)
    val next_j = floorAdd(j, max_blocks, req.max_j, next_i === 0.U)

    i := next_i
    j := next_j

    when (next_i === 0.U && next_j === 0.U) {
      state := idle
    }
  }

  when (io.req.fire) {
    req := io.req.bits
    state := ld
    j := 0.U
    i := 0.U
  }
}

// changed: sparse-loop-fix, LdIdx
// changed: sparse-loop-fix, this whole section (LdIdx) is new and only instantiated on n:m builds

class LoopMatmulLdIdxReq(val coreMaxAddrBits: Int, val iterator_bitwidth: Int, val max_addr: Int, val concurrent_loops: Int) extends Bundle { // changed: sparse-loop-fix, n:m builds only
  val n_tiles = UInt((iterator_bitwidth * 2).W) // changed: sparse-loop-fix, (max_k / 2) * max_j
  val dram_addr = UInt(coreMaxAddrBits.W) // changed: sparse-loop-fix, n:m builds only
  val addr_start = UInt(log2Up(max_addr+1).W) // changed: sparse-loop-fix, n:m builds only
  val loop_id = UInt(log2Up(concurrent_loops).W) // changed: sparse-loop-fix, n:m builds only
} // changed: sparse-loop-fix, n:m builds only

// changed: sparse-loop-fix, Mvins the index tiles of a sparse loop with LOAD_IDX_CMD (funct 27), in the order ex consumes them, as many whole
// changed: sparse-loop-fix, tiles per command as fit in block_size rows. DRAM and scratchpad both hold the tiles back to back.
class LoopMatmulLdIdx(block_size: Int, coreMaxAddrBits: Int, iterator_bitwidth: Int, max_addr: Int, input_w: Int, // changed: sparse-loop-fix, n:m builds only
                      concurrent_loops: Int, rows_per_tile: Int, mvin_rs2_t: MvinRs2) // changed: sparse-loop-fix, n:m builds only
                     (implicit p: Parameters) extends Module { // changed: sparse-loop-fix, n:m builds only
  val io = IO(new Bundle { // changed: sparse-loop-fix, n:m builds only
    val req = Flipped(Decoupled(new LoopMatmulLdIdxReq(coreMaxAddrBits, iterator_bitwidth, max_addr, concurrent_loops))) // changed: sparse-loop-fix, n:m builds only
    val cmd = Decoupled(Output(new RoCCCommand)) // changed: sparse-loop-fix, n:m builds only

    val u = Output(UInt((iterator_bitwidth * 2).W)) // changed: sparse-loop-fix, index tiles whose mvin has been issued
    val u_b = Input(UInt((iterator_bitwidth * 2).W)) // changed: sparse-loop-fix, weight tiles LdB has issued for the same loop
    val ldb_done = Input(Bool()) // changed: sparse-loop-fix, n:m builds only

    val idle = Output(Bool()) // changed: sparse-loop-fix, n:m builds only
    val rob_overloaded = Input(Bool()) // changed: sparse-loop-fix, n:m builds only

    val loop_id = Output(UInt(log2Up(concurrent_loops).W)) // changed: sparse-loop-fix, n:m builds only
  }) // changed: sparse-loop-fix, n:m builds only

  require(rows_per_tile > 0 && rows_per_tile <= block_size, "sparse-loop-fix: an index tile must fit in one mvin") // changed: sparse-loop-fix, n:m builds only
  val tiles_per_mvin = block_size / rows_per_tile // changed: sparse-loop-fix, n:m builds only
  val row_bytes = block_size * input_w / 8 // changed: sparse-loop-fix, n:m builds only

  object State extends ChiselEnum { // changed: sparse-loop-fix, n:m builds only
    val idle, ld = Value // changed: sparse-loop-fix, n:m builds only
  } // changed: sparse-loop-fix, n:m builds only
  import State._ // changed: sparse-loop-fix, n:m builds only
  val state = RegInit(idle) // changed: sparse-loop-fix, n:m builds only

  val req = Reg(new LoopMatmulLdIdxReq(coreMaxAddrBits, iterator_bitwidth, max_addr, concurrent_loops)) // changed: sparse-loop-fix, n:m builds only

  // changed: sparse-loop-fix, Counters step with each command so no address multiplier sits in the command path
  val u = Reg(UInt((iterator_bitwidth * 2).W)) // changed: sparse-loop-fix, n:m builds only
  val sp_addr = Reg(UInt(log2Up(max_addr+1).W)) // changed: sparse-loop-fix, n:m builds only
  val dram_addr = Reg(UInt(coreMaxAddrBits.W)) // changed: sparse-loop-fix, n:m builds only

  val remaining = req.n_tiles - u // changed: sparse-loop-fix, n:m builds only
  val tiles = Mux(remaining < tiles_per_mvin.U, remaining, tiles_per_mvin.U) // changed: sparse-loop-fix, n:m builds only
  val rows = (tiles * rows_per_tile.U)(log2Up(block_size + 1) - 1, 0) // changed: sparse-loop-fix, n:m builds only

  val mvin_cmd = Wire(new RoCCCommand) // changed: sparse-loop-fix, n:m builds only
  mvin_cmd := DontCare // changed: sparse-loop-fix, n:m builds only
  mvin_cmd.inst.funct := LOAD_IDX_CMD // changed: sparse-loop-fix, n:m builds only
  mvin_cmd.rs1 := dram_addr // changed: sparse-loop-fix, n:m builds only

  val mvin_cmd_rs2 = Wire(mvin_rs2_t.cloneType) // changed: sparse-loop-fix, n:m builds only
  mvin_cmd_rs2 := DontCare // changed: sparse-loop-fix, n:m builds only
  mvin_cmd_rs2.num_rows := rows.asUInt // changed: sparse-loop-fix, n:m builds only
  mvin_cmd_rs2._spacer2 := 0.U // changed: sparse-loop-fix, the Controller checks the full rows and cols fields (N12)
  mvin_cmd_rs2._spacer1 := 0.U // changed: sparse-loop-fix, as above
  mvin_cmd_rs2._spacer0 := 0.U // changed: sparse-loop-fix, as above (N13 checks the address bits)
  mvin_cmd_rs2.num_cols := block_size.U // changed: sparse-loop-fix, n:m builds only
  mvin_cmd_rs2.local_addr := cast_to_sp_addr(mvin_cmd_rs2.local_addr, sp_addr) // changed: sparse-loop-fix, n:m builds only
  mvin_cmd.rs2 := mvin_cmd_rs2.asUInt // changed: sparse-loop-fix, n:m builds only

  io.req.ready := state === idle // changed: sparse-loop-fix, n:m builds only
  io.u := u // changed: sparse-loop-fix, n:m builds only
  io.idle := state === idle // changed: sparse-loop-fix, n:m builds only

  // changed: sparse-loop-fix, Stay at most one mvin ahead of LdB, so this high-priority arbiter input cannot starve the A and B loads
  val throttle_ok = io.ldb_done || u <= io.u_b +& tiles_per_mvin.U // changed: sparse-loop-fix, n:m builds only

  io.cmd.valid := state =/= idle && !io.rob_overloaded && throttle_ok && req.n_tiles =/= 0.U // changed: sparse-loop-fix, n:m builds only
  io.cmd.bits := mvin_cmd // changed: sparse-loop-fix, n:m builds only

  io.loop_id := req.loop_id // changed: sparse-loop-fix, n:m builds only

  when (state =/= idle && req.n_tiles === 0.U) { // changed: sparse-loop-fix, n:m builds only
    state := idle // changed: sparse-loop-fix, n:m builds only
  }.elsewhen (io.cmd.fire) { // changed: sparse-loop-fix, n:m builds only
    u := u + tiles // changed: sparse-loop-fix, n:m builds only
    sp_addr := sp_addr + rows // changed: sparse-loop-fix, n:m builds only
    dram_addr := dram_addr + rows * row_bytes.U // changed: sparse-loop-fix, n:m builds only

    when (tiles === remaining) { // changed: sparse-loop-fix, n:m builds only
      state := idle // changed: sparse-loop-fix, n:m builds only
    } // changed: sparse-loop-fix, n:m builds only
  } // changed: sparse-loop-fix, n:m builds only

  when (io.req.fire) { // changed: sparse-loop-fix, n:m builds only
    req := io.req.bits // changed: sparse-loop-fix, n:m builds only
    state := ld // changed: sparse-loop-fix, n:m builds only
    u := 0.U // changed: sparse-loop-fix, n:m builds only
    sp_addr := io.req.bits.addr_start // changed: sparse-loop-fix, n:m builds only
    dram_addr := io.req.bits.dram_addr // changed: sparse-loop-fix, n:m builds only
  } // changed: sparse-loop-fix, n:m builds only
} // changed: sparse-loop-fix, n:m builds only

// Compute
class LoopMatmulExecuteReq(val block_size: Int, val coreMaxAddrBits: Int, val iterator_bitwidth: Int, val max_addr: Int, val max_acc_addr: Int, val concurrent_loops: Int, // changed: sparse-loop-fix, sparse loop fields of ex
                           val nm_on: Boolean = false) extends Bundle { // changed: sparse-loop-fix, nm_on adds the sparse loop fields (n:m builds only)
  val max_j = UInt(iterator_bitwidth.W)
  val max_k = UInt(iterator_bitwidth.W)
  val max_i = UInt(iterator_bitwidth.W)
  val pad_j = UInt(log2Up(block_size).W)
  val pad_k = UInt(log2Up(block_size).W)
  val pad_i = UInt(log2Up(block_size).W)
  val a_tranpose = Bool()
  val b_tranpose = Bool()
  val accumulate = Bool()
  val a_addr_start = UInt(log2Up(max_addr).W)
  val b_addr_end = UInt(log2Up(max_addr+1).W)
  val c_addr_start = UInt(log2Up(max_acc_addr).W)
  val loop_id = UInt(log2Up(concurrent_loops).W)
  val skip = Bool()
  val nm = if (nm_on) Some(Bool()) else None // changed: sparse-loop-fix, this loop is a sparse loop (LOOP_WS rs1[3])
  val max_k_b = if (nm_on) Some(UInt(iterator_bitwidth.W)) else None // changed: sparse-loop-fix, weight K tiles (max_k / 2); max_k stays the A tile count
}

class LoopMatmulExecute(block_size: Int, coreMaxAddrBits: Int, iterator_bitwidth: Int, max_addr: Int, max_acc_addr: Int, concurrent_loops: Int,
                        preload_rs1_t: PreloadRs, preload_rs2_t: PreloadRs,
                        compute_rs1_t: ComputeRs, compute_rs2_t: ComputeRs, // changed: sparse-loop-fix, sparse loop fields of ex
                        nm_rows_per_tile: Int = 0) // changed: sparse-loop-fix, scratchpad rows per index tile, 0 on a dense build
                       (implicit p: Parameters) extends Module {
  val nm_on = nm_rows_per_tile > 0 // changed: sparse-loop-fix, sparse loop hardware is generated only on n:m builds
  val io = IO(new Bundle {
    val req = Flipped(Decoupled(new LoopMatmulExecuteReq(block_size, coreMaxAddrBits, iterator_bitwidth, max_addr, max_acc_addr, concurrent_loops, nm_on))) // changed: sparse-loop-fix, nm_on
    val cmd = Decoupled(Output(new RoCCCommand))

    val k = Output(UInt(iterator_bitwidth.W))
    val j = Output(UInt(iterator_bitwidth.W))
    val i = Output(UInt(iterator_bitwidth.W))

    val ld_ka = Input(UInt(iterator_bitwidth.W))
    val ld_kb = Input(UInt(iterator_bitwidth.W))
    val ld_j = Input(UInt(iterator_bitwidth.W))
    val ld_i = Input(UInt(iterator_bitwidth.W))
    val lda_completed = Input(Bool())
    val ldb_completed = Input(Bool())
    val ldd_completed = Input(Bool())
    val ld_u_idx = if (nm_on) Some(Input(UInt((iterator_bitwidth * 2).W))) else None // changed: sparse-loop-fix, index tiles LdIdx has issued
    val ldidx_completed = if (nm_on) Some(Input(Bool())) else None // changed: sparse-loop-fix, LdIdx has issued all of this loop's index mvins
    val nm_first_load = if (nm_on) Some(Output(Bool())) else None // changed: sparse-loop-fix, cmd is the first index LOAD of a sparse loop (N10 in the Controller)

    val idle = Output(Bool())
    val rob_overloaded = Input(Bool())

    val loop_id = Output(UInt(log2Up(concurrent_loops).W))
  })

  object State extends ChiselEnum {
    val idle, pre, comp = Value
  }
  import State._
  val state = RegInit(idle)

  val req = Reg(new LoopMatmulExecuteReq(block_size, coreMaxAddrBits, iterator_bitwidth, max_addr, max_acc_addr, concurrent_loops, nm_on)) // changed: sparse-loop-fix, nm_on

  // changed: sparse-loop-fix, a sparse loop walks max_k / 2 weight tiles; on a dense build this is req.max_k itself
  val ex_max_k = if (nm_on) Mux(req.nm.get, req.max_k_b.get, req.max_k) else req.max_k // changed: sparse-loop-fix, sparse loop fields of ex

  val c_addr_start = /*(BigInt(1) << 31).U |*/ req.c_addr_start
  val b_addr_start = req.b_addr_end - ex_max_k * req.max_j * block_size.U // changed: sparse-loop-fix, ex_max_k: the sparse B region holds max_k / 2 tiles

  val k = Reg(UInt(iterator_bitwidth.W))
  val j = Reg(UInt(iterator_bitwidth.W))
  val i = Reg(UInt(iterator_bitwidth.W))

  val a_row = Mux(req.a_tranpose, k, i)
  val a_col = Mux(req.a_tranpose, i, k)
  val b_row = Mux(req.b_tranpose, j, k)
  val b_col = Mux(req.b_tranpose, k, j)

  val a_max_col = Mux(req.a_tranpose, req.max_i, req.max_k)
  val b_max_col = Mux(req.b_tranpose, req.max_k, req.max_j)

  val a_addr = req.a_addr_start + (a_row * a_max_col + a_col) * block_size.U
  val b_addr = b_addr_start + (b_row * b_max_col + b_col) * block_size.U
  val c_addr = c_addr_start + (i * req.max_j + j) * block_size.U

  val a_cols = block_size.U - Mux(k === req.max_k - 1.U, req.pad_k, 0.U)
  val a_rows = block_size.U - Mux(i === req.max_i - 1.U, req.pad_i, 0.U)
  val b_cols = block_size.U - Mux(j === req.max_j - 1.U, req.pad_j, 0.U)
  val b_rows = block_size.U - Mux(k === req.max_k - 1.U, req.pad_k, 0.U)
  val c_cols = block_size.U - Mux(j === req.max_j - 1.U, req.pad_j, 0.U)
  val c_rows = block_size.U - Mux(i === req.max_i - 1.U, req.pad_i, 0.U)

  val pre_cmd = Wire(new RoCCCommand)
  pre_cmd := DontCare
  pre_cmd.inst.funct := PRELOAD_CMD

  val pre_cmd_rs1 = Wire(preload_rs1_t.cloneType)
  pre_cmd_rs1 := DontCare
  pre_cmd_rs1.num_rows := b_rows.asUInt
  pre_cmd_rs1.num_cols := b_cols.asUInt
  pre_cmd_rs1.local_addr := Mux(i === 0.U, cast_to_sp_addr(pre_cmd_rs1.local_addr, b_addr),
    garbage_addr(pre_cmd_rs1.local_addr))

  val pre_cmd_rs2 = Wire(preload_rs2_t.cloneType)
  pre_cmd_rs2 := DontCare
  pre_cmd_rs2.num_rows := c_rows.asUInt
  pre_cmd_rs2.num_cols := c_cols.asUInt
  pre_cmd_rs2.local_addr := cast_to_acc_addr(pre_cmd_rs2.local_addr, c_addr, accumulate = req.accumulate || k =/= 0.U, read_full = false.B)

  pre_cmd.rs1 := pre_cmd_rs1.asUInt
  pre_cmd.rs2 := pre_cmd_rs2.asUInt

  val comp_cmd = Wire(new RoCCCommand())
  comp_cmd := DontCare
  comp_cmd.inst.funct := Mux(i === 0.U, COMPUTE_AND_FLIP_CMD, COMPUTE_AND_STAY_CMD)

  val comp_cmd_rs1 = Wire(compute_rs1_t.cloneType)
  comp_cmd_rs1 := DontCare
  comp_cmd_rs1.num_rows := a_rows.asUInt
  comp_cmd_rs1.num_cols := a_cols.asUInt
  comp_cmd_rs1.local_addr := cast_to_sp_addr(comp_cmd_rs1.local_addr, a_addr)

  val comp_cmd_rs2 = Wire(compute_rs2_t.cloneType)
  comp_cmd_rs2 := DontCare
  comp_cmd_rs2.num_rows := block_size.U
  comp_cmd_rs2.num_cols := block_size.U
  comp_cmd_rs2.local_addr := garbage_addr(comp_cmd_rs2.local_addr)

  comp_cmd.rs1 := comp_cmd_rs1.asUInt
  comp_cmd.rs2 := comp_cmd_rs2.asUInt

  io.req.ready := state === idle
  io.k := k
  io.j := j
  io.i := i
  io.idle := state === idle

  // The order here is k, j, i
  val lda_ahead = io.lda_completed || io.ld_ka > k || (io.ld_ka === k && io.ld_i > i)
  val ldb_ahead = io.ldb_completed || io.ld_kb > k || (io.ld_ka === k && io.ld_j > j)
  val ldd_ahead = io.ldd_completed
  val ld_ahead = lda_ahead && ldb_ahead && ldd_ahead

  // changed: sparse-loop-fix, this class is new. Sparse loop side of ex: operands of the sparse compute, and the index
  // changed: sparse-loop-fix, LOADs (NM_META_CMD rd == 2, fix-a-spec.md) that put index tiles in the fifo ahead of their preloads. A LOAD only
  // changed: sparse-loop-fix, leaves while the FSM is in `pre` at i == 0, so it is never between a preload and its compute.
  class NmLoad { // changed: sparse-loop-fix, sparse side of ex is new: sparse operands and index LOADs
    val nm = req.nm.get // changed: sparse-loop-fix, sparse side of ex is new: sparse operands and index LOADs
    val cnt_w = iterator_bitwidth * 2 // changed: sparse-loop-fix, sparse side of ex is new: sparse operands and index LOADs
    val n_tiles = req.max_k_b.get * req.max_j // changed: sparse-loop-fix, sparse side of ex is new: sparse operands and index LOADs

    val loaded = Reg(UInt(cnt_w.W)) // changed: sparse-loop-fix, index tiles named by the LOADs emitted so far
    val pre_cnt = Reg(UInt(cnt_w.W)) // changed: sparse-loop-fix, real preloads emitted so far, each pops one tile
    val ld_addr = Reg(UInt(log2Up(max_addr+1).W)) // changed: sparse-loop-fix, scratchpad row of index tile `loaded`
    val open_pre = RegInit(false.B) // changed: sparse-loop-fix, a preload was emitted and its compute has not followed yet

    // changed: sparse-loop-fix, Budget: the index fifo holds 4 tiles. One LOAD of 4 at the start, then 2 more whenever at most 2 are still
    // changed: sparse-loop-fix, ahead, so tiles ahead + in flight + n never exceeds 4 in program order (fix-a-spec.md rule P1).
    val first = loaded === 0.U && pre_cnt === 0.U // changed: sparse-loop-fix, sparse side of ex is new: sparse operands and index LOADs
    val ahead = loaded - pre_cnt // changed: sparse-loop-fix, sparse side of ex is new: sparse operands and index LOADs
    val remaining = n_tiles - loaded // changed: sparse-loop-fix, sparse side of ex is new: sparse operands and index LOADs
    val want_load = nm && state === pre && i === 0.U && loaded < n_tiles && ahead <= 2.U // changed: sparse-loop-fix, sparse side of ex is new: sparse operands and index LOADs
    val n = Mux(first, Mux(remaining < 4.U, remaining, 4.U), Mux(remaining < 2.U, remaining, 2.U))(2, 0) // changed: sparse-loop-fix, sparse side of ex is new: sparse operands and index LOADs

    // changed: sparse-loop-fix, The RS only orders the LOAD after the mvin of its rows if that mvin was allocated first
    val idx_issued = io.ldidx_completed.get || io.ld_u_idx.get >= loaded +& n // changed: sparse-loop-fix, sparse side of ex is new: sparse operands and index LOADs

    // changed: sparse-loop-fix, Weight tile k needs A tiles 2k and 2k+1, and LdB's k counts weight tiles like ex's k
    val a_hi_tile = Cat(k, 1.U(1.W)) // changed: sparse-loop-fix, sparse side of ex is new: sparse operands and index LOADs
    val lda_ahead_nm = io.lda_completed || io.ld_ka > a_hi_tile || (io.ld_ka === a_hi_tile && io.ld_i > i) // changed: sparse-loop-fix, sparse side of ex is new: sparse operands and index LOADs
    val ldb_ahead_nm = io.ldb_completed || io.ld_kb > k || (io.ld_kb === k && io.ld_j > j) // changed: sparse-loop-fix, sparse side of ex is new: sparse operands and index LOADs
    val ld_ahead_nm = lda_ahead_nm && ldb_ahead_nm && ldd_ahead // changed: sparse-loop-fix, sparse side of ex is new: sparse operands and index LOADs

    val cmd_gate = Mux(nm, Mux(want_load, idx_issued, ld_ahead_nm), ld_ahead) // changed: sparse-loop-fix, sparse side of ex is new: sparse operands and index LOADs

    val ld_cmd = Wire(new RoCCCommand) // changed: sparse-loop-fix, sparse side of ex is new: sparse operands and index LOADs
    ld_cmd := DontCare // changed: sparse-loop-fix, sparse side of ex is new: sparse operands and index LOADs
    ld_cmd.inst.funct := NM_META_CMD // changed: sparse-loop-fix, sparse side of ex is new: sparse operands and index LOADs
    ld_cmd.inst.rd := 2.U // changed: sparse-loop-fix, sparse side of ex is new: sparse operands and index LOADs
    ld_cmd.rs1 := ld_addr // changed: sparse-loop-fix, sparse side of ex is new: sparse operands and index LOADs
    ld_cmd.rs2 := n // changed: sparse-loop-fix, sparse side of ex is new: sparse operands and index LOADs

    val pre_or_ld_cmd = Mux(want_load, ld_cmd, pre_cmd) // changed: sparse-loop-fix, sparse side of ex is new: sparse operands and index LOADs

    io.nm_first_load.get := want_load && first // changed: sparse-loop-fix, sparse side of ex is new: sparse operands and index LOADs

    // changed: sparse-loop-fix, a_lo is A tile (i, 2k) in Berkeley's layout with the original max_k; a_hi is the next tile, block_size rows on
    val a_lo = req.a_addr_start + (i * req.max_k + Cat(k, 0.U(1.W))) * block_size.U // changed: sparse-loop-fix, sparse side of ex is new: sparse operands and index LOADs
    when (nm) { // changed: sparse-loop-fix, sparse side of ex is new: sparse operands and index LOADs
      comp_cmd_rs1.local_addr := cast_to_sp_addr(comp_cmd_rs1.local_addr, a_lo) // changed: sparse-loop-fix, sparse side of ex is new: sparse operands and index LOADs
      comp_cmd_rs2.num_rows := a_rows.asUInt // changed: sparse-loop-fix, sparse side of ex is new: sparse operands and index LOADs
      comp_cmd_rs2.num_cols := block_size.U // changed: sparse-loop-fix, sparse side of ex is new: sparse operands and index LOADs
      comp_cmd_rs2.local_addr := cast_to_sp_addr(comp_cmd_rs2.local_addr, a_lo + block_size.U) // changed: sparse-loop-fix, sparse side of ex is new: sparse operands and index LOADs
    } // changed: sparse-loop-fix, sparse side of ex is new: sparse operands and index LOADs

    // changed: sparse-loop-fix, Runs after the dense state update so that a LOAD keeps the FSM in `pre`
    def update(): Unit = { // changed: sparse-loop-fix, sparse side of ex is new: sparse operands and index LOADs
      val out_funct = io.cmd.bits.inst.funct // changed: sparse-loop-fix, sparse side of ex is new: sparse operands and index LOADs
      val out_is_load = out_funct === NM_META_CMD // changed: sparse-loop-fix, sparse side of ex is new: sparse operands and index LOADs
      val out_is_comp = out_funct === COMPUTE_AND_FLIP_CMD || out_funct === COMPUTE_AND_STAY_CMD // changed: sparse-loop-fix, sparse side of ex is new: sparse operands and index LOADs
      val last_comp = state === comp && i === req.max_i - 1.U && j === req.max_j - 1.U && k === ex_max_k - 1.U // changed: sparse-loop-fix, sparse side of ex is new: sparse operands and index LOADs

      when (io.cmd.fire && nm) { // changed: sparse-loop-fix, sparse side of ex is new: sparse operands and index LOADs
        when (want_load) { // changed: sparse-loop-fix, sparse side of ex is new: sparse operands and index LOADs
          state := pre // changed: sparse-loop-fix, sparse side of ex is new: sparse operands and index LOADs
          loaded := loaded + n // changed: sparse-loop-fix, sparse side of ex is new: sparse operands and index LOADs
          ld_addr := ld_addr + n * nm_rows_per_tile.U // changed: sparse-loop-fix, sparse side of ex is new: sparse operands and index LOADs
        }.elsewhen (state === pre && i === 0.U) { // changed: sparse-loop-fix, sparse side of ex is new: sparse operands and index LOADs
          pre_cnt := pre_cnt + 1.U // changed: sparse-loop-fix, sparse side of ex is new: sparse operands and index LOADs
        } // changed: sparse-loop-fix, sparse side of ex is new: sparse operands and index LOADs

        when (out_funct === PRELOAD_CMD) { // changed: sparse-loop-fix, sparse side of ex is new: sparse operands and index LOADs
          open_pre := true.B // changed: sparse-loop-fix, sparse side of ex is new: sparse operands and index LOADs
        }.elsewhen (out_is_comp) { // changed: sparse-loop-fix, sparse side of ex is new: sparse operands and index LOADs
          open_pre := false.B // changed: sparse-loop-fix, sparse side of ex is new: sparse operands and index LOADs
        } // changed: sparse-loop-fix, sparse side of ex is new: sparse operands and index LOADs

        assert(!(out_is_load && open_pre), "sparse-loop-fix: internal: LOAD between a preload and its compute") // changed: sparse-loop-fix, sparse side of ex is new: sparse operands and index LOADs
        assert(!(out_is_load && !idx_issued), "sparse-loop-fix: internal: LOAD before its index mvin was issued") // changed: sparse-loop-fix, sparse side of ex is new: sparse operands and index LOADs
        assert(!(out_is_load && (ahead +& n) > 4.U), "sparse-loop-fix: internal: loop finished with undelivered index tiles") // changed: sparse-loop-fix, sparse side of ex is new: sparse operands and index LOADs
        assert(!(last_comp && (loaded =/= n_tiles || pre_cnt =/= n_tiles)), // changed: sparse-loop-fix, sparse side of ex is new: sparse operands and index LOADs
          "sparse-loop-fix: internal: loop finished with undelivered index tiles") // changed: sparse-loop-fix, sparse side of ex is new: sparse operands and index LOADs
      } // changed: sparse-loop-fix, sparse side of ex is new: sparse operands and index LOADs

      when (io.req.fire) { // changed: sparse-loop-fix, sparse side of ex is new: sparse operands and index LOADs
        loaded := 0.U // changed: sparse-loop-fix, sparse side of ex is new: sparse operands and index LOADs
        pre_cnt := 0.U // changed: sparse-loop-fix, sparse side of ex is new: sparse operands and index LOADs
        ld_addr := io.req.bits.b_addr_end // changed: sparse-loop-fix, sparse side of ex is new: sparse operands and index LOADs
        open_pre := false.B // changed: sparse-loop-fix, sparse side of ex is new: sparse operands and index LOADs
      } // changed: sparse-loop-fix, sparse side of ex is new: sparse operands and index LOADs
    } // changed: sparse-loop-fix, sparse side of ex is new: sparse operands and index LOADs
  } // changed: sparse-loop-fix, sparse side of ex is new: sparse operands and index LOADs
  val nm_load = if (nm_on) Some(new NmLoad) else None // changed: sparse-loop-fix, n:m builds only

  // changed: sparse-loop-fix, on a dense build cmd_gate is ld_ahead and pre_or_ld_cmd is pre_cmd
  val cmd_gate = nm_load.map(_.cmd_gate).getOrElse(ld_ahead) // changed: sparse-loop-fix, n:m builds only
  val pre_or_ld_cmd = nm_load.map(_.pre_or_ld_cmd).getOrElse(pre_cmd) // changed: sparse-loop-fix

  io.cmd.valid := state =/= idle && !io.rob_overloaded && cmd_gate && !req.skip // changed: sparse-loop-fix, cmd_gate also gates the index LOAD
  io.cmd.bits := Mux(state === pre, pre_or_ld_cmd, comp_cmd) // changed: sparse-loop-fix, pre_or_ld_cmd: the index LOAD goes out in `pre`

  io.loop_id := req.loop_id

  when(req.skip) {
    state := idle
  }.elsewhen (io.cmd.fire) {
    when (state === pre) {
      state := comp
    }.otherwise {
      val next_i = floorAdd(i, 1.U, req.max_i)
      val next_j = floorAdd(j, 1.U, req.max_j, next_i === 0.U)
      val next_k = floorAdd(k, 1.U, ex_max_k, next_j === 0.U && next_i === 0.U) // changed: sparse-loop-fix, ex_max_k: max_k / 2 weight tiles in a sparse loop

      k := next_k
      j := next_j
      i := next_i

      state := Mux(next_k === 0.U && next_j === 0.U && next_i === 0.U, idle, pre)
    }
  }

  when (io.req.fire) {
    req := io.req.bits
    state := pre
    j := 0.U
    k := 0.U
    i := 0.U
  }

  nm_load.foreach(_.update()) // changed: sparse-loop-fix, LOAD bookkeeping, must follow the state updates above

  assert(!(state =/= idle && req.a_tranpose && req.b_tranpose))
}

// StC

class LoopMatmulStCReq(val block_size: Int, val coreMaxAddrBits: Int, val iterator_bitwidth: Int, val max_acc_addr: Int, val concurrent_loops: Int) extends Bundle {
  val max_k = UInt(iterator_bitwidth.W)
  val max_j = UInt(iterator_bitwidth.W)
  val max_i = UInt(iterator_bitwidth.W)
  val pad_j = UInt(log2Up(block_size).W)
  val pad_i = UInt(log2Up(block_size).W)
  val dram_addr = UInt(coreMaxAddrBits.W)
  val dram_stride = UInt(coreMaxAddrBits.W)
  val full_c = Bool()
  val act = UInt(Activation.bitwidth.W)
  val addr_start = UInt(log2Up(max_acc_addr).W)
  val loop_id = UInt(log2Up(concurrent_loops).W)
  val is_resadd = Bool()
}

class LoopMatmulStC(block_size: Int, coreMaxAddrBits: Int, iterator_bitwidth: Int, max_acc_addr: Int, input_w: Int, acc_w: Int, max_block_len: Int, concurrent_loops: Int, mvout_rs2_t: MvoutRs2)
                   (implicit p: Parameters) extends Module {
  val io = IO(new Bundle {
    val req = Flipped(Decoupled(new LoopMatmulStCReq(block_size, coreMaxAddrBits, iterator_bitwidth, max_acc_addr, concurrent_loops)))
    val cmd = Decoupled(Output(new RoCCCommand))

    val ex_k = Input(UInt(iterator_bitwidth.W))
    val ex_j = Input(UInt(iterator_bitwidth.W))
    val ex_i = Input(UInt(iterator_bitwidth.W))
    val ex_completed = Input(Bool())

    val j = Output(UInt(iterator_bitwidth.W))
    val i = Output(UInt(iterator_bitwidth.W))

    val idle = Output(Bool())
    val rob_overloaded = Input(Bool())

    val loop_id = Output(UInt(log2Up(concurrent_loops).W))
  })

  object State extends ChiselEnum {
    val idle, st, ln_config, ln_st = Value
  }
  import State._
  val state = RegInit(idle)

  val req = Reg(new LoopMatmulStCReq(block_size, coreMaxAddrBits, iterator_bitwidth, max_acc_addr, concurrent_loops))

  val max_blocks = Mux(req.full_c, 1.U, Mux(req.max_j <= max_block_len.U, req.max_j, max_block_len.U))

  // Non-normalization-related iterators and calculations
  val j = Reg(UInt(iterator_bitwidth.W))
  val i = Reg(UInt(iterator_bitwidth.W))

  val acc_addr_start = /*(BigInt(1) << 31).U | (req.full_c << 29.U).asUInt |*/ req.addr_start

  val dram_offset = Mux(req.full_c, (i * req.dram_stride + j) * block_size.U * (acc_w/8).U,
    (i * req.dram_stride + j) * block_size.U * (input_w/8).U)
  val dram_addr = req.dram_addr + LoopMatmul.castDramOffset(dram_offset)
  val sp_addr = acc_addr_start + (i * req.max_j + j) * block_size.U
  val blocks = Mux(j + max_blocks <= req.max_j, max_blocks, req.max_j-j)
  val cols = (blocks * block_size.U) - Mux(j + blocks >= req.max_j, req.pad_j, 0.U)
  val rows = block_size.U - Mux(i === req.max_i-1.U, req.pad_i, 0.U)

  val mvout_cmd = Wire(new RoCCCommand)
  mvout_cmd := DontCare
  mvout_cmd.inst.funct := STORE_CMD
  mvout_cmd.rs1 := dram_addr

  val mvout_cmd_rs2 = Wire(mvout_rs2_t.cloneType)
  mvout_cmd_rs2 := DontCare
  mvout_cmd_rs2.num_rows := rows.asUInt
  mvout_cmd_rs2.num_cols := cols.asUInt
  mvout_cmd_rs2.local_addr := cast_to_acc_addr(mvout_cmd_rs2.local_addr, sp_addr, accumulate = false.B, read_full = req.full_c)
  mvout_cmd.rs2 := mvout_cmd_rs2.asUInt

  // Layernorm iterators and calculations
  val ln_row = Reg(UInt(iterator_bitwidth.W))
  val ln_cmd = Reg(UInt(iterator_bitwidth.W))
  val ln_stat_id = Reg(UInt(iterator_bitwidth.W))

  val NORM_STAT_IDS = 2 // TODO magic number

  val ln_norm_cmds = VecInit(VecInit(NormCmd.SUM, NormCmd.MEAN), VecInit(NormCmd.VARIANCE, NormCmd.INV_STDDEV),
    VecInit(NormCmd.RESET, NormCmd.RESET))

  val sm_norm_cmds = VecInit(VecInit(NormCmd.MAX, NormCmd.MAX), VecInit(NormCmd.SUM_EXP, NormCmd.INV_SUM_EXP),
    VecInit(NormCmd.RESET, NormCmd.RESET))

  val ln_stat_ids = Mux(rows -& ln_row > NORM_STAT_IDS.U, NORM_STAT_IDS.U, rows -& ln_row)

  val ln_r = ln_row +& ln_stat_id

  val ln_sp_addr = acc_addr_start +& (i * req.max_j +& j) * block_size.U +& ln_r
  val ln_norm_cmd = Mux(j +& max_blocks >= req.max_j,
    Mux(req.act === Activation.LAYERNORM, ln_norm_cmds(ln_cmd)(1), sm_norm_cmds(ln_cmd)(1)),
    Mux(req.act === Activation.LAYERNORM, ln_norm_cmds(ln_cmd)(0), sm_norm_cmds(ln_cmd)(0)))

  // TODO we assume for now that full_C and layernorm aren't true at the same
  val ln_dram_offset = ((i * req.dram_stride +& j) * block_size.U +& ln_r * req.dram_stride) * (input_w/8).U
  val ln_dram_addr = req.dram_addr + LoopMatmul.castDramOffset(ln_dram_offset)

  val ln_config_norm_rs1 = Wire(new GemminiISA.ConfigNormRs1)
  ln_config_norm_rs1 := DontCare
  ln_config_norm_rs1.set_stats_id_only := 1.U
  ln_config_norm_rs1.cmd_type := CONFIG_NORM
  ln_config_norm_rs1.norm_stats_id := ln_stat_id

  val ln_config_norm = Wire(new RoCCCommand)
  ln_config_norm := DontCare
  ln_config_norm.inst.funct := CONFIG_CMD
  ln_config_norm.rs1 := ln_config_norm_rs1.asUInt
  ln_config_norm.rs2 := DontCare

  val ln_mvout_cmd = Wire(new RoCCCommand)
  ln_mvout_cmd := DontCare
  ln_mvout_cmd.inst.funct := STORE_CMD
  ln_mvout_cmd.rs1 := ln_dram_addr

  val ln_mvout_cmd_rs2 = Wire(mvout_rs2_t.cloneType)
  ln_mvout_cmd_rs2 := DontCare
  ln_mvout_cmd_rs2.num_rows := 1.U
  ln_mvout_cmd_rs2.num_cols := cols.asUInt
  ln_mvout_cmd_rs2.local_addr := cast_to_acc_addr(ln_mvout_cmd_rs2.local_addr, ln_sp_addr, accumulate = false.B, read_full = req.full_c)
  ln_mvout_cmd_rs2.local_addr.norm_cmd := ln_norm_cmd
  ln_mvout_cmd.rs2 := ln_mvout_cmd_rs2.asUInt

  io.req.ready := state === idle
  io.j := j
  io.i := i
  io.idle := state === idle

  // The order here is k, j, i when not doing LAYERNORM or SOFTMAX
  val ex_ahead = WireInit(io.ex_completed ||
    ((req.act =/= Activation.LAYERNORM) && (req.act =/= Activation.SOFTMAX) &&
      (io.ex_k === req.max_k - 1.U &&
        (io.ex_j >= j + blocks ||
          ((io.ex_j === j + blocks - 1.U) && io.ex_i > i)))))
  when(req.is_resadd){
    ex_ahead := io.ex_completed || (io.ex_i > i || (io.ex_i === i && io.ex_j >= j + blocks))
  }

  io.cmd.valid := state =/= idle && !io.rob_overloaded && ex_ahead && req.dram_addr =/= 0.U
  io.cmd.bits := MuxCase(mvout_cmd, Seq(
    (state === ln_config) -> ln_config_norm,
    (state === ln_st) -> ln_mvout_cmd,
  ))

  io.loop_id := req.loop_id

  when (req.dram_addr === 0.U) {
    state := idle
  }.elsewhen (io.cmd.fire && state === st) {
    // The order here is k, j, i
    val next_i = floorAdd(i, 1.U, req.max_i)
    val next_j = floorAdd(j, max_blocks, req.max_j, next_i === 0.U)

    i := next_i
    j := next_j

    when (next_i === 0.U && next_j === 0.U) {
      state := idle
    }
  }.elsewhen (io.cmd.fire && state === ln_config) {
    state := ln_st
  }.elsewhen (io.cmd.fire && state === ln_st) {
    val next_j = floorAdd(j, max_blocks, req.max_j)
    val next_stat_id = floorAdd(ln_stat_id, 1.U, ln_stat_ids, next_j === 0.U)
    val next_cmd = floorAdd(ln_cmd, 1.U, ln_norm_cmds.size.U, next_j === 0.U && next_stat_id === 0.U)
    val next_row = floorAdd(ln_row, NORM_STAT_IDS.U, rows, next_j === 0.U && next_stat_id === 0.U && next_cmd === 0.U)
    val next_i = floorAdd(i, 1.U, req.max_i,
      next_j === 0.U && next_stat_id === 0.U && next_cmd === 0.U && next_row === 0.U)

    j := next_j
    ln_stat_id := next_stat_id
    ln_cmd := next_cmd
    ln_row := next_row
    i := next_i

    when (next_i === 0.U && next_row === 0.U && next_cmd === 0.U && next_stat_id === 0.U && next_j === 0.U) {
      state := idle
    }.elsewhen (next_j === 0.U) {
      state := ln_config
    }
  }

  when (io.req.fire) {
    req := io.req.bits
    state := Mux((io.req.bits.act === Activation.LAYERNORM) || (io.req.bits.act === Activation.SOFTMAX), ln_config, st)

    j := 0.U
    i := 0.U
    ln_row := 0.U
    ln_cmd := 0.U
    ln_stat_id := 0.U
  }
}

// StCSpad
class LoopMatmulStCSpadReq(val block_size: Int, val iterator_bitwidth: Int, val max_addr: Int, val max_acc_addr: Int, val concurrent_loops: Int) extends Bundle {
  val max_k = UInt(iterator_bitwidth.W)
  val max_j = UInt(iterator_bitwidth.W)
  val max_i = UInt(iterator_bitwidth.W)
  val pad_j = UInt(log2Up(block_size).W)
  val pad_i = UInt(log2Up(block_size).W)
  val dst_addr = UInt(log2Up(max_addr).W)
  val src_addr = UInt(log2Up(max_acc_addr).W)
  val full_c = Bool()
  val act = UInt(Activation.bitwidth.W)
  val loop_id = UInt(log2Up(concurrent_loops).W)
  val is_resadd = Bool()
}

class LoopMatmulStCSpad(block_size: Int, iterator_bitwidth: Int, max_addr: Int, max_acc_addr: Int,
                        input_w: Int, acc_w: Int, max_block_len: Int, concurrent_loops: Int,
                        mvout_spad_rs1_t: MvoutSpadRs1, mvout_rs2_t: MvoutRs2)
                   (implicit p: Parameters) extends Module {
  val io = IO(new Bundle {
    val req = Flipped(Decoupled(new LoopMatmulStCSpadReq(block_size, iterator_bitwidth, max_addr, max_acc_addr, concurrent_loops)))
    val cmd = Decoupled(Output(new RoCCCommand))

    val ex_k = Input(UInt(iterator_bitwidth.W))
    val ex_j = Input(UInt(iterator_bitwidth.W))
    val ex_i = Input(UInt(iterator_bitwidth.W))
    val ex_completed = Input(Bool())

    val j = Output(UInt(iterator_bitwidth.W))
    val i = Output(UInt(iterator_bitwidth.W))

    val idle = Output(Bool())
    val rob_overloaded = Input(Bool())

    val loop_id = Output(UInt(log2Up(concurrent_loops).W))
  })

  object State extends ChiselEnum {
    val idle, st = Value
  }
  import State._
  val state = RegInit(idle)

  val req = Reg(new LoopMatmulStCSpadReq(block_size, iterator_bitwidth, max_addr, max_acc_addr, concurrent_loops))

  val max_blocks = Mux(req.full_c, 1.U, Mux(req.max_j <= max_block_len.U, req.max_j, max_block_len.U))
  assert(max_block_len == 1, "there might be hw bugs if block length > 1, disabled for now")

  // Non-normalization-related iterators and calculations
  val j = Reg(UInt(iterator_bitwidth.W))
  val i = Reg(UInt(iterator_bitwidth.W))

  val acc_addr_start = req.src_addr

  val dst_offset = Mux(req.full_c, (i * req.max_j + j) * block_size.U,
    (i * req.max_j + j) * block_size.U)
  val dst_addr = req.dst_addr + dst_offset

  val src_addr = acc_addr_start + (i * req.max_j + j) * block_size.U
  val blocks = Mux(j + max_blocks <= req.max_j, max_blocks, req.max_j-j)
  val cols = (blocks * block_size.U) - Mux(j + blocks >= req.max_j, req.pad_j, 0.U)
  val rows = block_size.U - Mux(i === req.max_i-1.U, req.pad_i, 0.U)

  val mvout_cmd = Wire(new RoCCCommand)
  mvout_cmd := DontCare
  mvout_cmd.inst.funct := STORE_SPAD_CMD

  val mvout_cmd_rs1 = Wire(mvout_spad_rs1_t.cloneType)
  mvout_cmd_rs1 := DontCare
  mvout_cmd_rs1.local_addr := cast_to_sp_addr(mvout_cmd_rs1.local_addr, dst_addr)
  mvout_cmd_rs1.stride := 1.U // TODO
  mvout_cmd.rs1 := mvout_cmd_rs1.asUInt

  val mvout_cmd_rs2 = Wire(mvout_rs2_t.cloneType)
  mvout_cmd_rs2 := DontCare
  mvout_cmd_rs2.num_rows := rows.asUInt
  mvout_cmd_rs2.num_cols := cols.asUInt
  mvout_cmd_rs2.local_addr := cast_to_acc_addr(mvout_cmd_rs2.local_addr, src_addr, accumulate = false.B, read_full = req.full_c)
  mvout_cmd.rs2 := mvout_cmd_rs2.asUInt

  io.req.ready := state === idle
  io.j := j
  io.i := i
  io.idle := state === idle

  // The order here is k, j, i when not doing LAYERNORM or SOFTMAX
  val ex_ahead = WireInit(io.ex_completed ||
    ((req.act =/= Activation.LAYERNORM) && (req.act =/= Activation.SOFTMAX) &&
      (io.ex_k === req.max_k - 1.U &&
        (io.ex_j >= j + blocks ||
          ((io.ex_j === j + blocks - 1.U) && io.ex_i > i)))))
  when(req.is_resadd){
    ex_ahead := io.ex_completed || (io.ex_i > i || (io.ex_i === i && io.ex_j >= j + blocks))
  }

  io.cmd.valid := state =/= idle && !io.rob_overloaded && ex_ahead
  io.cmd.bits := mvout_cmd
  io.loop_id := req.loop_id

  when (io.cmd.fire && state === st) {
    // The order here is k, j, i
    val next_i = floorAdd(i, 1.U, req.max_i)
    val next_j = floorAdd(j, max_blocks, req.max_j, next_i === 0.U)

    i := next_i
    j := next_j

    when(next_i === 0.U && next_j === 0.U) {
      state := idle
    }
  }

  when (io.req.fire) {
    req := io.req.bits
    state := st
    j := 0.U
    i := 0.U
  }
}

// Combined loop
class LoopMatmulState(val iterator_bitwidth: Int, val coreMaxAddrBits: Int, val max_addr: Int, val max_acc_addr: Int, // changed: sparse-loop-fix, sparse loop state
                      val nm_on: Boolean = false) extends Bundle { // changed: sparse-loop-fix, nm_on adds the sparse loop fields (n:m builds only)
  val max_k = UInt(iterator_bitwidth.W)
  val max_j = UInt(iterator_bitwidth.W)
  val max_i = UInt(iterator_bitwidth.W)

  val pad_k = UInt(iterator_bitwidth.W)
  val pad_j = UInt(iterator_bitwidth.W)
  val pad_i = UInt(iterator_bitwidth.W)

  val a_dram_addr = UInt(coreMaxAddrBits.W)
  val b_dram_addr = UInt(coreMaxAddrBits.W)
  val d_dram_addr = UInt(coreMaxAddrBits.W)
  val c_dram_addr = UInt(coreMaxAddrBits.W)

  val a_dram_stride = UInt(coreMaxAddrBits.W)
  val b_dram_stride = UInt(coreMaxAddrBits.W)
  val d_dram_stride = UInt(coreMaxAddrBits.W)
  val c_dram_stride = UInt(coreMaxAddrBits.W)

  val c_spad_addr = UInt(max_acc_addr.W)
  val inc_acc_addr = Bool()
  val spad_only = Bool()

  val a_transpose = Bool()
  val b_transpose = Bool()

  val act = UInt(Activation.bitwidth.W)

  val low_d = Bool()
  val full_c = Bool()
  val ex_accumulate = Bool()

  val a_ex_spad_id = UInt(2.W)
  val b_ex_spad_id = UInt(2.W)
  val configured = Bool()

  val running = Bool()

  val lda_started = Bool()
  val ldb_started = Bool()
  val ex_started = Bool()
  val ldd_started = Bool()
  val st_started = Bool()

  val lda_completed = Bool()
  val ldb_completed = Bool()
  val ex_completed = Bool()
  val ldd_completed = Bool()
  val st_completed = Bool()

  val nm = if (nm_on) Some(Bool()) else None // changed: sparse-loop-fix, sparse loop (LOOP_WS rs1[3])
  val idx_dram_addr = if (nm_on) Some(UInt(coreMaxAddrBits.W)) else None // changed: sparse-loop-fix, index tile 0 in DRAM (LOOP_WS_CONFIG_NM rs1)
  val ldidx_started = if (nm_on) Some(Bool()) else None // changed: sparse-loop-fix, LdIdx took this loop (preset on a dense loop)
  val ldidx_completed = if (nm_on) Some(Bool()) else None // changed: sparse-loop-fix, LdIdx issued all index mvins (preset on a dense loop)

  // changed: sparse-loop-fix, a sparse loop also waits for LdIdx; the dense build keeps the five-term expression
  def all_completed(dummy: Int=0): Bool = { // changed: sparse-loop-fix, sparse loop state
    val ws_completed = lda_completed && ldb_completed && ldd_completed && ex_completed && st_completed // changed: sparse-loop-fix, named so LdIdx can join
    ldidx_completed.map(_ && ws_completed).getOrElse(ws_completed) // changed: sparse-loop-fix
  } // changed: sparse-loop-fix, sparse loop state

  val a_addr_start = UInt(log2Up(max_addr).W)
  val b_addr_end = UInt(log2Up(max_addr+1).W)
  val resadd_addr_start = UInt(log2Up(max_acc_addr).W)

  def reset(): Unit = {
    configured := false.B

    running := false.B

    lda_started := false.B
    ldb_started := false.B
    ex_started := false.B
    ldd_started := false.B
    st_started := false.B

    lda_completed := false.B
    ldb_completed := false.B
    ex_completed := false.B
    ldd_completed := false.B
    st_completed := false.B

    spad_only := false.B

    nm.foreach(_ := false.B) // changed: sparse-loop-fix, a loop is dense until LOOP_WS says otherwise
    ldidx_started.foreach(_ := false.B) // changed: sparse-loop-fix
    ldidx_completed.foreach(_ := false.B) // changed: sparse-loop-fix

    //is_resadd := false.B
  }
}

class LoopMatmul(block_size: Int, coreMaxAddrBits: Int, reservation_station_size: Int, max_lds: Int, max_exs: Int, max_sts: Int,
                 max_addr: Int, max_acc_addr: Int, input_w: Int, acc_w: Int, dma_max_bytes: Int,
                 mvin_rs2_t: MvinRs2, preload_rs1_t: PreloadRs, preload_rs2_t: PreloadRs,
                 compute_rs1_t: ComputeRs, compute_rs2_t: ComputeRs, mvout_spad_rs1_t: MvoutSpadRs1, mvout_rs2_t: MvoutRs2, // changed: sparse-loop-fix, n:m builds only
                 nm_rows_per_tile: Int = 0, nm_bank_rows: Int = 0) // changed: sparse-loop-fix, scratchpad rows per index tile and per bank; 0 on a dense build
                (implicit p: Parameters) extends Module {
  val nm_on = nm_rows_per_tile > 0 // changed: sparse-loop-fix, sparse loop hardware is generated only on n:m builds
  val iterator_bitwidth = 16
  val max_block_len = (dma_max_bytes / (block_size * input_w / 8)) max 1
  val max_block_len_acc = (dma_max_bytes / (block_size * acc_w / 8)) max 1

  val io = IO(new Bundle {
    val in = Flipped(Decoupled(new GemminiCmd(reservation_station_size)))
    val out = Decoupled(new GemminiCmd(reservation_station_size))
    val ld_completed = Input(UInt(log2Up(reservation_station_size+1).W))
    val st_completed = Input(UInt(log2Up(reservation_station_size+1).W))
    val ex_completed = Input(UInt(log2Up(reservation_station_size+1).W))
    val busy = Output(Bool())
    val completed = Output(Vec(2, Bool()))
    val nm_first_load = if (nm_on) Some(Output(Bool())) else None // changed: sparse-loop-fix, io.out is the first index LOAD of a sparse loop
  })

  // Create states
  val concurrent_loops = 2
  val loops = Reg(Vec(concurrent_loops, new LoopMatmulState(iterator_bitwidth, coreMaxAddrBits, max_addr, max_acc_addr, nm_on))) // changed: sparse-loop-fix, nm_on
  val head_loop_id = Reg(UInt(log2Up(concurrent_loops).W))
  val tail_loop_id = (~head_loop_id).asUInt // This is the loop that we always try to configure if available
  val head_loop = loops(head_loop_id)
  val tail_loop = loops(tail_loop_id)

  val loop_configured = loops.map(_.configured).reduce(_ || _)

  val loop_being_configured_id = Mux(head_loop.configured, tail_loop_id, head_loop_id)
  val loop_being_configured = loops(loop_being_configured_id)

  val is_resadd = RegInit(false.B)

  val max_all_addr = if(max_addr > max_acc_addr) max_addr else max_acc_addr 
  // Create inner modules
  val ldA = Module(new LoopMatmulLdA(block_size, coreMaxAddrBits, iterator_bitwidth, max_all_addr, input_w, max_block_len, concurrent_loops, mvin_rs2_t))
  val ldB = Module(new LoopMatmulLdB(block_size, coreMaxAddrBits, iterator_bitwidth, max_all_addr, input_w, max_block_len, concurrent_loops, mvin_rs2_t))
  val ldD = Module(new LoopMatmulLdD(block_size, coreMaxAddrBits, iterator_bitwidth, max_acc_addr, input_w, acc_w, max_block_len, max_block_len_acc, concurrent_loops, mvin_rs2_t))
  val ex = Module(new LoopMatmulExecute(block_size, coreMaxAddrBits, iterator_bitwidth, max_addr, max_acc_addr, concurrent_loops, preload_rs1_t, preload_rs2_t, compute_rs1_t, compute_rs2_t, nm_rows_per_tile)) // changed: sparse-loop-fix, nm_rows_per_tile
  val stC = Module(new LoopMatmulStC(block_size, coreMaxAddrBits, iterator_bitwidth, max_acc_addr, input_w, acc_w, max_block_len, concurrent_loops, mvout_rs2_t))
  val stC_spad = Module(new LoopMatmulStCSpad(block_size, iterator_bitwidth, max_addr, max_acc_addr, input_w, acc_w, 1, concurrent_loops, mvout_spad_rs1_t, mvout_rs2_t))
  // changed: sparse-loop-fix, index tile mvins of a sparse loop, n:m builds only
  val ldIdx = if (nm_on) Some(Module(new LoopMatmulLdIdx(block_size, coreMaxAddrBits, iterator_bitwidth, max_addr, input_w, concurrent_loops, nm_rows_per_tile, mvin_rs2_t))) else None // changed: sparse-loop-fix, n:m builds only

  // Create command queue
  val cmd = Queue(io.in)

  io.busy := cmd.valid || loop_configured

  io.completed := 0.U.asTypeOf(io.completed.cloneType)

  // Create ld arbiters
  val ldab_arb = Module(new WeightedArbiter(new RoCCCommand(), maxWeightA=255, staticWeightAEnabled=true)) // TODO magic numbers
  ldab_arb.io.inA <> ldA.io.cmd
  ldab_arb.io.inB <> ldB.io.cmd
  val ab_loads_on_same_loop = ldA.io.loop_id === ldB.io.loop_id
  val forceA = !ab_loads_on_same_loop && ldA.io.loop_id === head_loop_id
  val forceB = !ab_loads_on_same_loop && ldB.io.loop_id === head_loop_id
  ldab_arb.io.forceA := Mux(is_resadd, ab_loads_on_same_loop && !ldA.io.idle, forceA)
  ldab_arb.io.forceB := Mux(is_resadd, forceB || ldA.io.idle, forceB) 
  ldab_arb.io.weightA := 0.U
  ldab_arb.io.inA_idle := ldA.io.idle
  ldab_arb.io.inB_idle := ldB.io.idle
  ldab_arb.io.inA_k := ldA.io.k
  ldab_arb.io.inA_i := ldA.io.i
  ldab_arb.io.inB_k := ldB.io.k
  ldab_arb.io.inB_j := ldB.io.j

  // Create global arbiter
  val arb = Module(new Arbiter(new RoCCCommand(), if (nm_on) 6 else 5)) // changed: sparse-loop-fix, one more input for LdIdx on n:m builds
  arb.io.in(0) <> stC.io.cmd
  arb.io.in(1) <> ex.io.cmd
  arb.io.in(2) <> ldD.io.cmd
  ldIdx.foreach(arb.io.in(3) <> _.io.cmd) // changed: sparse-loop-fix, above the A/B loads so a long A/B stream cannot starve it
  arb.io.in(if (nm_on) 4 else 3) <> ldab_arb.io.out // changed: sparse-loop-fix, moves down one on n:m builds
  arb.io.in(if (nm_on) 5 else 4) <> stC_spad.io.cmd // changed: sparse-loop-fix, moves down one on n:m builds
  val unrolled_cmd = arb.io.out

  // Create reservation station utilization counters
  val ld_utilization = RegInit(0.U(log2Up(max_lds+1).W))
  val st_utilization = RegInit(0.U(log2Up(max_sts+1).W))
  val ex_utilization = RegInit(0.U(log2Up(max_exs+1).W))

  ld_utilization := ld_utilization +& (ldA.io.cmd.fire || ldB.io.cmd.fire || ldD.io.cmd.fire) -& io.ld_completed
  st_utilization := st_utilization +& (stC.io.cmd.fire || stC_spad.io.cmd.fire) -& io.st_completed
  ex_utilization := ex_utilization +& ex.io.cmd.fire -& io.ex_completed
  // changed: sparse-loop-fix, a LOAD_IDX takes a load entry like any mvin (overrides the ld_utilization update above)
  ldIdx.foreach { l => // changed: sparse-loop-fix, LOAD_IDX takes a load entry
    ld_utilization := ld_utilization +& (ldA.io.cmd.fire || ldB.io.cmd.fire || ldD.io.cmd.fire || l.io.cmd.fire) -& io.ld_completed // changed: sparse-loop-fix, LOAD_IDX takes a load entry
  } // changed: sparse-loop-fix, LOAD_IDX takes a load entry

  assert(ld_utilization >= io.ld_completed, "ld utilization underflow")
  assert(st_utilization >= io.st_completed, "st utilization underflow")
  assert(ex_utilization >= io.ex_completed, "ex utilization underflow")

  // Wire up unrolled command output
  val is_loop_run_cmd = cmd.bits.cmd.inst.funct === LOOP_WS
  val is_loop_config_cmd_ws = (cmd.bits.cmd.inst.funct >= LOOP_WS_CONFIG_BOUNDS && cmd.bits.cmd.inst.funct <= LOOP_WS_CONFIG_STRIDES_DC) || // changed: sparse-loop-fix, renamed so funct 28 can join below
    (cmd.bits.cmd.inst.funct >= LOOP_WS_CONFIG_SPAD_AB && cmd.bits.cmd.inst.funct <= LOOP_WS_CONFIG_SPAD_C)
  // changed: sparse-loop-fix, LOOP_WS_CONFIG_NM (funct 28) is a loop config command on n:m builds only
  val is_loop_config_cmd = if (nm_on) is_loop_config_cmd_ws || cmd.bits.cmd.inst.funct === LOOP_WS_CONFIG_NM else is_loop_config_cmd_ws // changed: sparse-loop-fix, LOAD_IDX takes a load entry
  val is_loop_cmd = is_loop_run_cmd || is_loop_config_cmd

  io.out.bits.cmd := Mux(loop_configured, unrolled_cmd.bits, cmd.bits.cmd)
  io.out.bits.cmd.status := cmd.bits.cmd.status // TODO This is not guaranteed to be the correct fix! We must fix this
  io.out.bits.rob_id := DontCare
  io.out.bits.from_matmul_fsm := Mux(loop_configured, true.B, cmd.bits.from_matmul_fsm)
  io.out.bits.from_conv_fsm := Mux(loop_configured, false.B, cmd.bits.from_conv_fsm)
  io.out.valid := Mux(loop_configured, unrolled_cmd.valid, cmd.valid && !is_loop_config_cmd && !is_loop_run_cmd)

  cmd.ready := Mux(is_loop_cmd, !loop_being_configured.configured, !loop_configured && io.out.ready)
  arb.io.out.ready := io.out.ready

  // Wire up overloaded signals
  ldA.io.rob_overloaded := ld_utilization >= max_lds.U
  ldB.io.rob_overloaded := ld_utilization >= max_lds.U
  ex.io.rob_overloaded := ex_utilization >= max_exs.U
  ldD.io.rob_overloaded := ld_utilization >= max_lds.U
  stC.io.rob_overloaded := st_utilization >= max_sts.U
  stC_spad.io.rob_overloaded := st_utilization >= max_sts.U
  ldIdx.foreach(_.io.rob_overloaded := ld_utilization >= max_lds.U) // changed: sparse-loop-fix, throttled like the other loads

  // Wire up iterator inputs
  ex.io.lda_completed := (ldA.io.loop_id =/= ex.io.loop_id) || ldA.io.idle
  ex.io.ldb_completed := (ldB.io.loop_id =/= ex.io.loop_id) || ldB.io.idle
  ex.io.ldd_completed := (ldD.io.loop_id =/= ex.io.loop_id) || ldD.io.idle
  ex.io.ld_ka := ldA.io.k
  ex.io.ld_kb := ldB.io.k
  ex.io.ld_j := ldB.io.j
  ex.io.ld_i := ldA.io.i

  stC.io.ex_completed := (ex.io.loop_id =/= stC.io.loop_id) || ex.io.idle
  stC.io.ex_k := ex.io.k
  stC.io.ex_j := ex.io.j
  stC.io.ex_i := ex.io.i

  stC_spad.io.ex_completed := (ex.io.loop_id =/= stC_spad.io.loop_id) || ex.io.idle
  stC_spad.io.ex_k := ex.io.k
  stC_spad.io.ex_j := ex.io.j
  stC_spad.io.ex_i := ex.io.i

  // when loop matmul is used as resadd unroller
  // skip ex
  // track ldB instead of ex
  when(is_resadd){
    stC.io.ex_completed := (ldA.io.loop_id =/= stC.io.loop_id || ldA.io.idle) && (ldB.io.loop_id =/= stC.io.loop_id || ldB.io.idle)
    stC.io.ex_k := 0.U // req.max_k shall be 1
    stC.io.ex_j := ldB.io.j
    stC.io.ex_i := ldB.io.k
    //ldB.io.rob_overloaded := ld_utilization >= max_lds.U || !((ldA.io.loop_id =/= ldB.io.loop_id) || ldA.io.idle)
    // TODO(richard): does store to spad also support resadd?
  }

  val loops_configured = RegInit(0.U(16.W))
  dontTouch(loops_configured)

  // Create config registers
  when(cmd.valid && is_loop_cmd && !loop_being_configured.configured) {

    switch (cmd.bits.cmd.inst.funct) {
      is (LOOP_WS_CONFIG_BOUNDS) {
        loop_being_configured.max_k := cmd.bits.cmd.rs2(iterator_bitwidth * 3 - 1, iterator_bitwidth * 2)
        loop_being_configured.max_j := cmd.bits.cmd.rs2(iterator_bitwidth * 2 - 1, iterator_bitwidth)
        loop_being_configured.max_i := cmd.bits.cmd.rs2(iterator_bitwidth-1, 0)

        loop_being_configured.pad_k := cmd.bits.cmd.rs1(iterator_bitwidth * 3 - 1, iterator_bitwidth * 2)
        loop_being_configured.pad_j := cmd.bits.cmd.rs1(iterator_bitwidth * 2 - 1, iterator_bitwidth)
        loop_being_configured.pad_i := cmd.bits.cmd.rs1(iterator_bitwidth-1, 0)
      }

      is (LOOP_WS_CONFIG_ADDRS_AB) {
        loop_being_configured.spad_only := false.B
        loop_being_configured.a_dram_addr := cmd.bits.cmd.rs1
        loop_being_configured.b_dram_addr := cmd.bits.cmd.rs2
      }

      is (LOOP_WS_CONFIG_ADDRS_DC) {
        loop_being_configured.spad_only := false.B
        loop_being_configured.d_dram_addr := cmd.bits.cmd.rs1
        loop_being_configured.c_dram_addr := cmd.bits.cmd.rs2
      }

      is (LOOP_WS_CONFIG_STRIDES_AB) {
        loop_being_configured.a_dram_stride := cmd.bits.cmd.rs1
        loop_being_configured.b_dram_stride := cmd.bits.cmd.rs2
      }

      is (LOOP_WS_CONFIG_STRIDES_DC) {
        loop_being_configured.d_dram_stride := cmd.bits.cmd.rs1
        loop_being_configured.c_dram_stride := cmd.bits.cmd.rs2
      }

      is (LOOP_WS_CONFIG_SPAD_AB) {
        loop_being_configured.a_addr_start := cmd.bits.cmd.rs1
        loop_being_configured.b_addr_end := cmd.bits.cmd.rs2
      }

      is (LOOP_WS) {
        loop_being_configured.ex_accumulate := cmd.bits.cmd.rs1(0)
        loop_being_configured.full_c := cmd.bits.cmd.rs1(1)
        loop_being_configured.low_d := cmd.bits.cmd.rs1(2)
        loop_being_configured.act := cmd.bits.cmd.rs1(8+Activation.bitwidth-1, 8) // TODO magic numbers

        loop_being_configured.lda_started := cmd.bits.cmd.rs2(3)
        loop_being_configured.ldb_started := cmd.bits.cmd.rs2(4)
        loop_being_configured.ldd_started := cmd.bits.cmd.rs2(5)
        loop_being_configured.ex_started := cmd.bits.cmd.rs2(6)
        loop_being_configured.st_started := cmd.bits.cmd.rs2(7)
        loop_being_configured.lda_completed := cmd.bits.cmd.rs2(3)
        loop_being_configured.ldb_completed := cmd.bits.cmd.rs2(4)
        loop_being_configured.ldd_completed := cmd.bits.cmd.rs2(5)
        loop_being_configured.ex_completed := cmd.bits.cmd.rs2(6)
        loop_being_configured.st_completed := cmd.bits.cmd.rs2(7)
        loop_being_configured.inc_acc_addr := cmd.bits.cmd.rs2(8)
        loop_being_configured.spad_only := cmd.bits.cmd.rs2(9)

        loop_being_configured.c_spad_addr := cmd.bits.cmd.rs2(63, 32)

        loop_being_configured.a_ex_spad_id := cmd.bits.cmd.rs1(19, 18)
        loop_being_configured.b_ex_spad_id := cmd.bits.cmd.rs1(17, 16) 
        loop_being_configured.a_transpose := cmd.bits.cmd.rs2(0)
        loop_being_configured.b_transpose := cmd.bits.cmd.rs2(1)
        is_resadd := cmd.bits.cmd.rs2(2)

        loop_being_configured.configured := true.B

        loops_configured := loops_configured + 1.U
      }
    }
  }

  // changed: sparse-loop-fix, this block is new. Sparse loop config: LOOP_WS_CONFIG_NM (funct 28) gives the index DRAM
  // changed: sparse-loop-fix, address, LOOP_WS rs1[3] marks the loop sparse. A dense loop is marked as needing nothing from LdIdx.
  if (nm_on) { // changed: sparse-loop-fix, sparse loop config decode is new
    when (cmd.valid && is_loop_cmd && !loop_being_configured.configured) { // changed: sparse-loop-fix, sparse loop config decode is new
      when (cmd.bits.cmd.inst.funct === LOOP_WS_CONFIG_NM) { // changed: sparse-loop-fix, sparse loop config decode is new
        loop_being_configured.idx_dram_addr.get := cmd.bits.cmd.rs1 // changed: sparse-loop-fix, sparse loop config decode is new
      } // changed: sparse-loop-fix, sparse loop config decode is new

      when (cmd.bits.cmd.inst.funct === LOOP_WS) { // changed: sparse-loop-fix, sparse loop config decode is new
        val l = loop_being_configured // changed: sparse-loop-fix, sparse loop config decode is new
        val loop_nm = cmd.bits.cmd.rs1(3) // changed: sparse-loop-fix, sparse loop config decode is new
        l.nm.get := loop_nm // changed: sparse-loop-fix, sparse loop config decode is new
        l.ldidx_started.get := !loop_nm // changed: sparse-loop-fix, sparse loop config decode is new
        l.ldidx_completed.get := !loop_nm // changed: sparse-loop-fix, sparse loop config decode is new

        // changed: sparse-loop-fix, The bounds were set by an earlier LOOP_WS_CONFIG_BOUNDS; the products below keep their full width
        val kb = l.max_k >> 1 // changed: sparse-loop-fix, sparse loop config decode is new
        val a_rows = l.max_i * l.max_k * block_size.U // changed: sparse-loop-fix, sparse loop config decode is new
        val b_rows = kb * l.max_j * block_size.U // changed: sparse-loop-fix, sparse loop config decode is new
        val idx_rows = kb * l.max_j * nm_rows_per_tile.U // changed: sparse-loop-fix, sparse loop config decode is new
        when (loop_nm) { // changed: sparse-loop-fix, sparse loop config decode is new
          assert(!(cmd.bits.cmd.rs2(0) || cmd.bits.cmd.rs2(1) || cmd.bits.cmd.rs2(2) || cmd.bits.cmd.rs1(2) || cmd.bits.cmd.rs2(9)), // changed: sparse-loop-fix, sparse loop config decode is new
            "sparse-loop-fix: sparse loop does not support transposes, resadd, low_D or spad-only") // changed: sparse-loop-fix, sparse loop config decode is new
          assert(!l.max_k(0) && l.pad_k === 0.U, // changed: sparse-loop-fix, sparse loop config decode is new
            "sparse-loop-fix: sparse loop needs an even K tile count and pad_K == 0 (K %% 2*DIM)") // changed: sparse-loop-fix, sparse loop config decode is new
          assert((a_rows +& b_rows +& idx_rows) <= (max_addr / concurrent_loops).U && idx_rows <= nm_bank_rows.U, // changed: sparse-loop-fix, sparse loop config decode is new
            "sparse-loop-fix: A, B and index regions do not fit in the loop's scratchpad half (or index region exceeds one bank)") // changed: sparse-loop-fix, sparse loop config decode is new
        } // changed: sparse-loop-fix, sparse loop config decode is new
      } // changed: sparse-loop-fix, sparse loop config decode is new
    } // changed: sparse-loop-fix, sparse loop config decode is new
  } // changed: sparse-loop-fix, sparse loop config decode is new

  // Wire up request signals
  val ld_d_addr_start = RegInit(0.U(log2Up(max_acc_addr).W))
  val ex_c_addr_start = RegInit(0.U(log2Up(max_acc_addr).W))
  val st_c_addr_start = RegInit(0.U(log2Up(max_acc_addr).W))

  val loop_requesting_ldA_id = Mux(head_loop.lda_started, tail_loop_id, head_loop_id)
  val loop_requesting_ldA = loops(loop_requesting_ldA_id)
  ldA.io.req.bits.max_k := Mux(is_resadd, loop_requesting_ldA.max_j, loop_requesting_ldA.max_k)
  ldA.io.req.bits.max_i := loop_requesting_ldA.max_i
  ldA.io.req.bits.pad_k := Mux(is_resadd, loop_requesting_ldA.pad_j, loop_requesting_ldA.pad_k)
  ldA.io.req.bits.pad_i := loop_requesting_ldA.pad_i
  ldA.io.req.bits.dram_addr := loop_requesting_ldA.a_dram_addr
  ldA.io.req.bits.dram_stride := loop_requesting_ldA.a_dram_stride
  ldA.io.req.bits.transpose := loop_requesting_ldA.a_transpose
  ldA.io.req.bits.addr_start := Mux(loop_requesting_ldA.a_ex_spad_id === 0.U, loop_requesting_ldA.a_addr_start, (loop_requesting_ldA.a_ex_spad_id - 1.U) * (max_addr / concurrent_loops).U)
  ldA.io.req.bits.loop_id := loop_requesting_ldA_id
  ldA.io.req.bits.is_resadd := is_resadd

  ldA.io.req.valid := !loop_requesting_ldA.lda_started && loop_requesting_ldA.configured

  when (ldA.io.req.fire) {
    loop_requesting_ldA.running := true.B
    loop_requesting_ldA.lda_started := true.B
  }

  val loop_requesting_ldB_id = Mux(head_loop.ldb_started, tail_loop_id, head_loop_id)
  val loop_requesting_ldB = loops(loop_requesting_ldB_id)
  ldB.io.req.bits.max_j := loop_requesting_ldB.max_j
  ldB.io.req.bits.max_k := Mux(is_resadd, loop_requesting_ldB.max_i, loop_requesting_ldB.max_k)
  ldB.io.req.bits.pad_j := loop_requesting_ldB.pad_j
  ldB.io.req.bits.pad_k := Mux(is_resadd, loop_requesting_ldB.pad_i, loop_requesting_ldB.pad_k)
  ldB.io.req.bits.dram_addr := loop_requesting_ldB.b_dram_addr
  ldB.io.req.bits.dram_stride := loop_requesting_ldB.b_dram_stride
  ldB.io.req.bits.transpose := loop_requesting_ldB.b_transpose
  ldB.io.req.bits.addr_end := Mux(loop_requesting_ldB.b_ex_spad_id === 0.U, loop_requesting_ldB.b_addr_end, (loop_requesting_ldB.b_ex_spad_id) * (max_addr / concurrent_loops).U)
  ldB.io.req.bits.loop_id := loop_requesting_ldB_id
  ldB.io.req.bits.is_resadd := is_resadd

  ldB.io.req.valid := !loop_requesting_ldB.ldb_started && loop_requesting_ldB.configured

  when (ldB.io.req.fire) {
    loop_requesting_ldB.running := true.B
    loop_requesting_ldB.ldb_started := true.B
  }

  val loop_requesting_ex_id = Mux(head_loop.ex_started, tail_loop_id, head_loop_id)
  val loop_requesting_ex = loops(loop_requesting_ex_id)
  ex.io.req.bits.max_j := loop_requesting_ex.max_j
  ex.io.req.bits.max_k := loop_requesting_ex.max_k
  ex.io.req.bits.max_i := loop_requesting_ex.max_i
  ex.io.req.bits.pad_j := loop_requesting_ex.pad_j
  ex.io.req.bits.pad_k := loop_requesting_ex.pad_k
  ex.io.req.bits.pad_i := loop_requesting_ex.pad_i
  ex.io.req.bits.accumulate := loop_requesting_ex.ex_accumulate
  ex.io.req.bits.a_addr_start := Mux(loop_requesting_ex.spad_only || loop_requesting_ex.a_ex_spad_id === 0.U,
    loop_requesting_ex.a_addr_start, (loop_requesting_ex.a_ex_spad_id - 1.U) * (max_addr / concurrent_loops).U)
  ex.io.req.bits.b_addr_end := Mux(loop_requesting_ex.spad_only || loop_requesting_ex.b_ex_spad_id === 0.U,
    loop_requesting_ex.b_addr_end, (loop_requesting_ex.b_ex_spad_id) * (max_addr / concurrent_loops).U)
  ex.io.req.bits.a_tranpose := loop_requesting_ex.a_transpose
  ex.io.req.bits.b_tranpose := loop_requesting_ex.b_transpose
  ex.io.req.bits.c_addr_start := ex_c_addr_start
  ex.io.req.bits.loop_id := loop_requesting_ex_id
  ex.io.req.bits.skip := is_resadd

  ex.io.req.valid := !loop_requesting_ex.ex_started && loop_requesting_ex.lda_started &&
    loop_requesting_ex.ldb_started && loop_requesting_ex.ldd_started && loop_requesting_ex.configured 

  when (ex.io.req.fire) {
    loop_requesting_ex.running := true.B
    loop_requesting_ex.ex_started := true.B

    when (Mux(loop_requesting_ex.spad_only, loop_requesting_ex.inc_acc_addr, loop_requesting_ex.c_dram_addr =/= 0.U)) {
      ex_c_addr_start := floorAdd(ex_c_addr_start, (max_acc_addr / concurrent_loops).U, max_acc_addr.U)
    }
  }

  val loop_requesting_ldD_id = Mux(head_loop.ldd_started, tail_loop_id, head_loop_id)
  val loop_requesting_ldD = loops(loop_requesting_ldD_id)
  ldD.io.req.bits.max_j := loop_requesting_ldD.max_j
  ldD.io.req.bits.max_i := loop_requesting_ldD.max_i
  ldD.io.req.bits.pad_j := loop_requesting_ldD.pad_j
  ldD.io.req.bits.pad_i := loop_requesting_ldD.pad_i
  ldD.io.req.bits.dram_addr := loop_requesting_ldD.d_dram_addr
  ldD.io.req.bits.dram_stride := loop_requesting_ldD.d_dram_stride
  ldD.io.req.bits.low_d := loop_requesting_ldD.low_d
  ldD.io.req.bits.addr_start := ld_d_addr_start
  ldD.io.req.bits.loop_id := loop_requesting_ldD_id

  ldD.io.req.valid := !loop_requesting_ldD.ldd_started && loop_requesting_ldD.configured

  when (ldD.io.req.fire) {
    loop_requesting_ldD.running := true.B
    loop_requesting_ldD.ldd_started := true.B

    when (loop_requesting_ldD.c_dram_addr =/= 0.U) {
      ld_d_addr_start := floorAdd(ld_d_addr_start, (max_acc_addr / concurrent_loops).U, max_acc_addr.U)
    }
  }

  val loop_requesting_st_id = Mux(head_loop.st_started, tail_loop_id, head_loop_id)
  val loop_requesting_st = loops(loop_requesting_st_id)
  stC.io.req.bits.max_k := Mux(is_resadd, 1.U, loop_requesting_st.max_k)
  stC.io.req.bits.max_j := loop_requesting_st.max_j
  stC.io.req.bits.max_i := loop_requesting_st.max_i
  stC.io.req.bits.pad_j := loop_requesting_st.pad_j
  stC.io.req.bits.pad_i := loop_requesting_st.pad_i
  stC.io.req.bits.dram_addr := loop_requesting_st.c_dram_addr
  stC.io.req.bits.dram_stride := loop_requesting_st.c_dram_stride
  stC.io.req.bits.full_c := loop_requesting_st.full_c
  stC.io.req.bits.act := loop_requesting_st.act
  stC.io.req.bits.addr_start := st_c_addr_start
  stC.io.req.bits.loop_id := loop_requesting_st_id
  stC.io.req.bits.is_resadd := is_resadd

  stC_spad.io.req.bits.max_k := Mux(is_resadd, 1.U, loop_requesting_st.max_k)
  stC_spad.io.req.bits.max_j := loop_requesting_st.max_j
  stC_spad.io.req.bits.max_i := loop_requesting_st.max_i
  stC_spad.io.req.bits.pad_j := loop_requesting_st.pad_j
  stC_spad.io.req.bits.pad_i := loop_requesting_st.pad_i
  stC_spad.io.req.bits.src_addr := st_c_addr_start
  stC_spad.io.req.bits.dst_addr := loop_requesting_st.c_spad_addr
  stC_spad.io.req.bits.full_c := loop_requesting_st.full_c
  stC_spad.io.req.bits.act := loop_requesting_st.act
  stC_spad.io.req.bits.loop_id := loop_requesting_st_id
  stC_spad.io.req.bits.is_resadd := is_resadd

  stC.io.req.valid := !loop_requesting_st.st_started && loop_requesting_st.ex_started &&
    loop_requesting_st.configured && !loop_requesting_st.spad_only
  stC_spad.io.req.valid := !loop_requesting_st.st_started && loop_requesting_st.ex_started &&
    loop_requesting_st.configured && loop_requesting_st.spad_only

  when (stC.io.req.fire || stC_spad.io.req.fire) {
    loop_requesting_st.running := true.B
    loop_requesting_st.st_started := true.B

    when (Mux(loop_requesting_st.spad_only, loop_requesting_st.inc_acc_addr, loop_requesting_st.c_dram_addr =/= 0.U)) {
      st_c_addr_start := floorAdd(st_c_addr_start, (max_acc_addr / concurrent_loops).U, max_acc_addr.U)
    }
  }

  when(is_resadd){
    ldA.io.req.bits.addr_start := loop_requesting_ldA.resadd_addr_start
    ldB.io.req.bits.addr_end := loop_requesting_ldB.resadd_addr_start
    stC.io.req.bits.addr_start := loop_requesting_st.resadd_addr_start
    stC.io.req.valid := !loop_requesting_st.st_started && loop_requesting_st.configured
  }

  // changed: sparse-loop-fix, this block is new (n:m builds only). Scratchpad layout of a sparse loop: A as in a dense
  // changed: sparse-loop-fix, loop, the index region is the top idx_rows rows of the loop's half, and B (max_k / 2 tiles along K) ends where the
  // changed: sparse-loop-fix, index region starts. The connections below override the dense ones above for a sparse loop.
  ldIdx.foreach { li => // changed: sparse-loop-fix, sparse loop wiring is new
    def idx_rows(l: LoopMatmulState) = ((l.max_k >> 1) * l.max_j * nm_rows_per_tile.U)(log2Up(max_addr+1)-1, 0) // changed: sparse-loop-fix, sparse loop wiring is new
    def b_addr_end(l: LoopMatmulState) = Mux(l.b_ex_spad_id === 0.U, l.b_addr_end, (l.b_ex_spad_id) * (max_addr / concurrent_loops).U) // changed: sparse-loop-fix, sparse loop wiring is new
    def b_addr_end_eff(l: LoopMatmulState) = (b_addr_end(l) - idx_rows(l))(log2Up(max_addr+1)-1, 0) // changed: sparse-loop-fix, sparse loop wiring is new

    when (loop_requesting_ldB.nm.get) { // changed: sparse-loop-fix, sparse loop wiring is new
      ldB.io.req.bits.max_k := loop_requesting_ldB.max_k >> 1 // changed: sparse-loop-fix, sparse loop wiring is new
      ldB.io.req.bits.addr_end := b_addr_end_eff(loop_requesting_ldB) // changed: sparse-loop-fix, sparse loop wiring is new
    } // changed: sparse-loop-fix, sparse loop wiring is new

    ex.io.req.bits.nm.get := loop_requesting_ex.nm.get // changed: sparse-loop-fix, sparse loop wiring is new
    ex.io.req.bits.max_k_b.get := loop_requesting_ex.max_k >> 1 // changed: sparse-loop-fix, sparse loop wiring is new
    when (loop_requesting_ex.nm.get) { // changed: sparse-loop-fix, sparse loop wiring is new
      ex.io.req.bits.b_addr_end := b_addr_end_eff(loop_requesting_ex) // changed: sparse-loop-fix, sparse loop wiring is new
    } // changed: sparse-loop-fix, sparse loop wiring is new
    // changed: sparse-loop-fix, ex reads LdIdx's progress, so LdIdx must have taken the loop first (always true for a dense loop)
    when (!loop_requesting_ex.ldidx_started.get) { // changed: sparse-loop-fix, sparse loop wiring is new
      ex.io.req.valid := false.B // changed: sparse-loop-fix, sparse loop wiring is new
    } // changed: sparse-loop-fix, sparse loop wiring is new
    ex.io.ldidx_completed.get := (li.io.loop_id =/= ex.io.loop_id) || li.io.idle // changed: sparse-loop-fix, sparse loop wiring is new
    ex.io.ld_u_idx.get := li.io.u // changed: sparse-loop-fix, sparse loop wiring is new

    // changed: sparse-loop-fix, StC waits for ex to reach the last weight tile along K, which is max_k / 2 - 1 in a sparse loop
    when (loop_requesting_st.nm.get) { // changed: sparse-loop-fix, sparse loop wiring is new
      stC.io.req.bits.max_k := loop_requesting_st.max_k >> 1 // changed: sparse-loop-fix, sparse loop wiring is new
      stC_spad.io.req.bits.max_k := loop_requesting_st.max_k >> 1 // changed: sparse-loop-fix, sparse loop wiring is new
    } // changed: sparse-loop-fix, sparse loop wiring is new

    val loop_requesting_ldIdx_id = Mux(head_loop.ldidx_started.get, tail_loop_id, head_loop_id) // changed: sparse-loop-fix, sparse loop wiring is new
    val loop_requesting_ldIdx = loops(loop_requesting_ldIdx_id) // changed: sparse-loop-fix, sparse loop wiring is new
    li.io.req.bits.n_tiles := (loop_requesting_ldIdx.max_k >> 1) * loop_requesting_ldIdx.max_j // changed: sparse-loop-fix, sparse loop wiring is new
    li.io.req.bits.dram_addr := loop_requesting_ldIdx.idx_dram_addr.get // changed: sparse-loop-fix, sparse loop wiring is new
    li.io.req.bits.addr_start := b_addr_end_eff(loop_requesting_ldIdx) // changed: sparse-loop-fix, sparse loop wiring is new
    li.io.req.bits.loop_id := loop_requesting_ldIdx_id // changed: sparse-loop-fix, sparse loop wiring is new

    li.io.req.valid := !loop_requesting_ldIdx.ldidx_started.get && loop_requesting_ldIdx.configured // changed: sparse-loop-fix, sparse loop wiring is new

    when (li.io.req.fire) { // changed: sparse-loop-fix, sparse loop wiring is new
      loop_requesting_ldIdx.running := true.B // changed: sparse-loop-fix, sparse loop wiring is new
      loop_requesting_ldIdx.ldidx_started.get := true.B // changed: sparse-loop-fix, sparse loop wiring is new
    } // changed: sparse-loop-fix, sparse loop wiring is new

    // changed: sparse-loop-fix, LdB's progress on LdIdx's loop, in weight tiles, for LdIdx's throttle
    val ldidx_loop = loops(li.io.loop_id) // changed: sparse-loop-fix, sparse loop wiring is new
    val ldb_on_ldidx_loop = ldB.io.loop_id === li.io.loop_id && !ldB.io.idle // changed: sparse-loop-fix, sparse loop wiring is new
    li.io.ldb_done := ldidx_loop.ldb_completed || (ldidx_loop.ldb_started && !ldb_on_ldidx_loop) // changed: sparse-loop-fix, sparse loop wiring is new
    li.io.u_b := Mux(ldidx_loop.ldb_started && ldb_on_ldidx_loop, ldB.io.k * ldidx_loop.max_j + ldB.io.j, 0.U) // changed: sparse-loop-fix, sparse loop wiring is new

    when (li.io.idle && ldidx_loop.running && ldidx_loop.ldidx_started.get) { // changed: sparse-loop-fix, sparse loop wiring is new
      ldidx_loop.ldidx_completed.get := true.B // changed: sparse-loop-fix, sparse loop wiring is new
    } // changed: sparse-loop-fix, sparse loop wiring is new

    io.nm_first_load.get := loop_configured && arb.io.chosen === 1.U && ex.io.cmd.valid && ex.io.nm_first_load.get // changed: sparse-loop-fix, sparse loop wiring is new
  } // changed: sparse-loop-fix, sparse loop wiring is new

  // Handle completed signals
  when (ldA.io.idle && loops(ldA.io.loop_id).running && loops(ldA.io.loop_id).lda_started) {
    loops(ldA.io.loop_id).lda_completed := true.B
  }

  when (ldB.io.idle && loops(ldB.io.loop_id).running && loops(ldB.io.loop_id).ldb_started) {
    loops(ldB.io.loop_id).ldb_completed := true.B
  }

  when (ex.io.idle && loops(ex.io.loop_id).running && loops(ex.io.loop_id).ex_started) {
    loops(ex.io.loop_id).ex_completed := true.B
  }

  when (ldD.io.idle && loops(ldD.io.loop_id).running && loops(ldD.io.loop_id).ldd_started) {
    loops(ldD.io.loop_id).ldd_completed := true.B
  }

  when (stC.io.idle && loops(stC.io.loop_id).running && loops(stC.io.loop_id).st_started && !loops(stC.io.loop_id).spad_only) {
    loops(stC.io.loop_id).st_completed := true.B
  }

  when (stC_spad.io.idle && loops(stC_spad.io.loop_id).running &&
    loops(stC_spad.io.loop_id).st_started && loops(stC_spad.io.loop_id).spad_only) {
    loops(stC_spad.io.loop_id).st_completed := true.B
  }

  when (head_loop.running && head_loop.all_completed()) {
    head_loop.reset()
    io.completed(head_loop_id) := true.B
    head_loop_id := ~head_loop_id
  }

  // Resets
  when (reset.asBool) {
    loops.zipWithIndex.foreach { case (l, i) =>
      l.reset()
      l.a_addr_start := (i * (max_addr / concurrent_loops)).U
      l.b_addr_end := ((i+1) * (max_addr / concurrent_loops)).U
      l.resadd_addr_start := (i * (max_acc_addr / concurrent_loops)).U
    }
  }
}

object LoopMatmul {
  def apply(in: DecoupledIO[GemminiCmd], ld_completed: UInt, st_completed: UInt, ex_completed: UInt,
            block_size: Int, coreMaxAddrBits: Int, rob_size: Int, max_lds: Int, max_exs: Int, max_sts: Int,
            max_addr: Int, max_acc_addr: Int, input_w: Int, acc_w: Int, dma_max_bytes: Int,
            mvin_rs2_t: MvinRs2, preload_rs1_t: PreloadRs, preload_rs2_t: PreloadRs,
            compute_rs1_t: ComputeRs, compute_rs2_t: ComputeRs, mvout_spad_rs1_t: MvoutSpadRs1, mvout_rs2_t: MvoutRs2, // changed: sparse-loop-fix, n:m builds only
            nm_rows_per_tile: Int = 0, nm_bank_rows: Int = 0) // changed: sparse-loop-fix, 0 on a dense build
           (implicit p: Parameters): (DecoupledIO[GemminiCmd], Bool, Vec[Bool], Option[Bool]) = { // changed: sparse-loop-fix, 4th: first index LOAD of a sparse loop
    val mod = Module(new LoopMatmul(block_size, coreMaxAddrBits, rob_size, max_lds, max_exs, max_sts,
      max_addr, max_acc_addr, input_w, acc_w, dma_max_bytes,
      mvin_rs2_t, preload_rs1_t, preload_rs2_t, compute_rs1_t, compute_rs2_t, mvout_spad_rs1_t, mvout_rs2_t, // changed: sparse-loop-fix, n:m builds only
      nm_rows_per_tile, nm_bank_rows)) // changed: sparse-loop-fix
    mod.io.in <> in
    mod.io.ld_completed := ld_completed
    mod.io.st_completed := st_completed
    mod.io.ex_completed := ex_completed
    (mod.io.out, mod.io.busy, mod.io.completed, mod.io.nm_first_load) // changed: sparse-loop-fix, None on a dense build
  }

  def castDramOffset(dram_offset: UInt): UInt = {
    // Cast dram offsets to 32 bits max
    dram_offset & "hFFFFFFFF".U
  }
}
