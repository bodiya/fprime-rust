@ Demo data types: one of every FPP type construct, with the values the
@ tests check byte for byte.
module Demo {

  @ A plain numeric constant
  constant COUNT = 4

  @ A derived constant
  constant DOUBLE_COUNT = COUNT * 2

  @ A hexadecimal constant
  constant MASK = 0xFF00

  @ A float constant
  constant SCALE = 1.5

  @ A string constant
  constant NAME = "demo"

  @ A boolean constant
  constant ENABLED = true

  @ An enum with implicit values and a default
  enum Mode : U8 {
    @ Idle
    IDLE
    @ Running
    RUNNING
    @ Faulted, explicitly numbered
    FAULTED = 7
    @ After an explicit value the sequence continues
    RECOVERING
  } default RUNNING

  @ An enum with the default representation type (I32)
  enum Level {
    LOW = -1
    MID = 0
    HIGH = 1
  }

  @ An enum constant used as a constant
  constant START_MODE = Mode.FAULTED

  @ A fixed array with a scalar default (filled)
  array Counts = [COUNT] U16 default 0xABCD

  @ A fixed array with an element-wise default
  array Gains = [3] F32 default [1.0, 2.5, -0.5]

  @ A struct with every member kind
  struct Reading {
    @ The mode
    mode: Mode
    @ Raw counts
    counts: Counts
    @ An inline array member
    samples: [2] I16
    @ A string member
    label: string size 8
    @ A flag
    valid: bool
  } default { mode = Mode.IDLE, samples = [3, -4], label = "abc", valid = true }

  @ A nested struct
  struct Frame {
    seq: U32
    reading: Reading
    level: Level
  }

  @ A type alias
  type Seq = U32

  @ A dictionary type alias
  dictionary type Ident = FwIdType

  @ A port with primitive, enum, struct, string and ref parameters and a
  @ return value
  port Measure(
    seq: Seq
    mode: Mode
    reading: Reading
    label: string size 8
    ref result: Frame
  ) -> Level

  @ A port with no parameters
  port Tick

  @ A port carrying a framework buffer (moves) and a framework time
  port Deliver(ref buffer: Fw.Buffer, ref timeTag: Fw.Time)

}
