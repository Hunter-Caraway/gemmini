// See README.md for license details.
package gemmini

import chisel3._
import chisel3.util._

class PEControl[T <: Data : Arithmetic](accType: T) extends Bundle {
  val dataflow = UInt(1.W) // TODO make this an Enum
  val propagate = UInt(1.W) // Which register should be propagated (and which should be accumulated)?
  val shift = UInt(log2Up(accType.getWidth).W) // TODO this isn't correct for Floats

}

class MacUnit[T <: Data](inputType: T, weightType: T, cType: T, dType: T) (implicit ev: Arithmetic[T]) extends Module {
  import ev._
  val io = IO(new Bundle {
    val in_a  = Input(inputType)
    val in_b  = Input(weightType)
    val in_c  = Input(cType)
    val out_d = Output(dType)
  })

  io.out_d := io.in_c.mac(io.in_a, io.in_b)
}

// TODO update documentation
/**
  * A PE implementing a MAC operation. Configured as fully combinational when integrated into a Mesh.
  * @param width Data width of operands
  */
class PE[T <: Data](inputType: T, weightType: T, outputType: T, accType: T, df: Dataflow.Value, max_simultaneous_matmuls: Int,
                    nm: Option[(Int, Int)] = None) // changed: added n:m option
                   (implicit ev: Arithmetic[T]) extends Module { // Debugging variables
  import ev._

  // changed: with n:m on, a is the whole group of m activations and the pe picks one
  val nm_idx_bits = nm.map { case (_, m) => log2Up(m) max 1 }.getOrElse(1)
  val aType: Data = nm match {
    case Some((_, m)) => Vec(m, inputType)
    case None => inputType
  }

  val io = IO(new Bundle {
    val in_a = Input(aType)
    val in_b = Input(outputType)
    val in_d = Input(outputType)
    val out_a = Output(aType)
    val out_b = Output(outputType)
    val out_c = Output(outputType)

    val in_control = Input(new PEControl(accType))
    val out_control = Output(new PEControl(accType))

    val in_id = Input(UInt(log2Up(max_simultaneous_matmuls).W))
    val out_id = Output(UInt(log2Up(max_simultaneous_matmuls).W))

    val in_last = Input(Bool())
    val out_last = Output(Bool())

    val in_valid = Input(Bool())
    val out_valid = Output(Bool())

    val bad_dataflow = Output(Bool())

    // changed: weight index, shifts down the column right next to in_d
    val in_meta = nm.map(_ => Input(UInt(nm_idx_bits.W)))
    val out_meta = nm.map(_ => Output(UInt(nm_idx_bits.W)))
  })

  val cType = if (df == Dataflow.WS) inputType else accType

  // When creating PEs that support multiple dataflows, the
  // elaboration/synthesis tools often fail to consolidate and de-duplicate
  // MAC units. To force mac circuitry to be re-used, we create a "mac_unit"
  // module here which just performs a single MAC operation
  val mac_unit = Module(new MacUnit(inputType, weightType,
    if (df == Dataflow.WS) outputType else accType, outputType))

  val b  = io.in_b
  val d  = io.in_d
  val c1 = Reg(cType)
  val c2 = Reg(cType)
  val dataflow = io.in_control.dataflow
  val prop  = io.in_control.propagate

  // changed: m1/m2 hold the index for whatever weight is sitting in c1/c2
  val m1 = nm.map(_ => Reg(UInt(nm_idx_bits.W)))
  val m2 = nm.map(_ => Reg(UInt(nm_idx_bits.W)))

  // changed: pick the activation, ws uses the index of the weight we're multiplying with
  // (c2 when propagating, c1 otherwise), os just takes slot 0 like a dense pe
  val a: T = nm match {
    case Some(_) =>
      val cur_meta = Mux(prop === 1.U, m2.get, m1.get)
      val sel = Mux(dataflow === Dataflow.WS.id.U(1.W), cur_meta, 0.U)
      io.in_a.asInstanceOf[Vec[T]](sel)
    case None => io.in_a.asInstanceOf[T]
  }
  val shift = io.in_control.shift
  val id = io.in_id
  val last = io.in_last
  val valid = io.in_valid

  io.out_a := io.in_a // changed: pass the whole group along, not just the picked one
  io.out_control.dataflow := dataflow
  io.out_control.propagate := prop
  io.out_control.shift := shift
  io.out_id := id
  io.out_last := last
  io.out_valid := valid

  mac_unit.io.in_a := a

  val last_s = RegEnable(prop, valid)
  val flip = last_s =/= prop
  val shift_offset = Mux(flip, shift, 0.U)

  // Which dataflow are we using?
  val OUTPUT_STATIONARY = Dataflow.OS.id.U(1.W)
  val WEIGHT_STATIONARY = Dataflow.WS.id.U(1.W)

  // Is c1 being computed on, or propagated forward (in the output-stationary dataflow)?
  val COMPUTE = 0.U(1.W)
  val PROPAGATE = 1.U(1.W)

  io.bad_dataflow := false.B
  io.out_meta.foreach(_ := m1.get) // changed: default, overwritten in ws below
  when ((df == Dataflow.OS).B || ((df == Dataflow.BOTH).B && dataflow === OUTPUT_STATIONARY)) {
    when(prop === PROPAGATE) {
      io.out_c := (c1 >> shift_offset).clippedToWidthOf(outputType)
      io.out_b := b
      mac_unit.io.in_b := b.asTypeOf(weightType)
      mac_unit.io.in_c := c2
      c2 := mac_unit.io.out_d
      c1 := d.withWidthOf(cType)
    }.otherwise {
      io.out_c := (c2 >> shift_offset).clippedToWidthOf(outputType)
      io.out_b := b
      mac_unit.io.in_b := b.asTypeOf(weightType)
      mac_unit.io.in_c := c1
      c1 := mac_unit.io.out_d
      c2 := d.withWidthOf(cType)
    }
  }.elsewhen ((df == Dataflow.WS).B || ((df == Dataflow.BOTH).B && dataflow === WEIGHT_STATIONARY)) {
    when(prop === PROPAGATE) {
      io.out_c := c1
      mac_unit.io.in_b := c2.asTypeOf(weightType)
      mac_unit.io.in_c := b
      io.out_b := mac_unit.io.out_d
      c1 := d
      // changed: index follows its weight down the column
      m1.foreach(_ := io.in_meta.get)
      io.out_meta.foreach(_ := m1.get)
    }.otherwise {
      io.out_c := c2
      mac_unit.io.in_b := c1.asTypeOf(weightType)
      mac_unit.io.in_c := b
      io.out_b := mac_unit.io.out_d
      c2 := d
      // changed: same thing for the other buffer
      m2.foreach(_ := io.in_meta.get)
      io.out_meta.foreach(_ := m2.get)
    }
  }.otherwise {
    io.bad_dataflow := true.B
    //assert(false.B, "unknown dataflow")
    io.out_c := DontCare
    io.out_b := DontCare
    mac_unit.io.in_b := b.asTypeOf(weightType)
    mac_unit.io.in_c := c2
  }

  when (!valid) {
    c1 := c1
    c2 := c2
    m1.foreach(r => r := r) // changed: hold the indexes too
    m2.foreach(r => r := r)
    mac_unit.io.in_b := DontCare
    mac_unit.io.in_c := DontCare
  }
}
