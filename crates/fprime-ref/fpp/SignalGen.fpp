# The Rust port's `Ref.SignalGen` (`crates/fprime-ref/src/signal_gen.rs`),
# a deliberately simplified version of the upstream component: this file
# models exactly the commands, events, channels, parameter and data product
# the Rust implementation has, with the ids and wire types it uses. The
# upstream `Ref/SignalGen/SignalGen.fpp` must NOT be imported alongside it.

module Ref {

  @ Waveform selector (one byte on the wire)
  enum SignalType: U8 {
    Sine = 0
    Triangle = 1
  }

  @ Queued demo signal generator: async commands drain on the rate group's
  @ thread at `schedIn` time
  queued component SignalGen {

    sync input port schedIn: Svc.Sched

    command recv port cmdIn
    command reg port cmdRegOut
    command resp port cmdResponseOut
    event port logOut
    text event port logTextOut
    time get port timeGetOut
    telemetry port tlmOut
    param get port prmGetOut
    param set port prmSetOut
    product get port productGetOut
    product send port productSendOut

    @ Signal generator settings
    async command Settings(
      Frequency: U32 @< Ticks per period
      Amplitude: F32 @< Peak amplitude
      Phase: F32 @< Phase offset in radians
      SigType: SignalType @< Waveform
    ) opcode 0

    @ Toggle the generator on/off
    async command Toggle opcode 1

    @ Zero the next sample
    async command Skip opcode 2

    @ Produce one data product synchronously
    async command Dp(
      records: U32 @< Number of records
    ) opcode 3

    @ Peak amplitude applied by the parameter database
    param Amplitude: F32 default 0.0 id 0 set opcode 4 save opcode 5

    @ Settings were changed
    event SettingsChanged(
      frequency: U32
      amplitude: F32
      $phase: F32
      sigType: SignalType
    ) severity activity low id 0 \
      format "Settings changed: frequency {} amplitude {f} phase {f} type {}"

    @ The generator was toggled
    event Toggled(
      running: bool
    ) severity activity low id 1 format "Signal generator running: {}"

    @ A sample was skipped
    event SampleSkipped severity activity low id 2 \
      format "Sample skipped" throttle 3

    @ A data product was sent
    event DpSent(
      records: U32
      bytes: U32
    ) severity activity low id 3 format "Sent data product with {} records ({} bytes)"

    @ The data product buffer could not be allocated
    event DpBufferFailed(
      records: U32
    ) severity warning high id 4 format "Data product buffer allocation failed for {} records"

    @ The Amplitude parameter was updated
    event AmplitudeUpdated(
      val: F32
    ) severity activity high id 5 format "Amplitude parameter updated to {f}"

    @ The current sample
    telemetry SignalValue: F32 id 0

    @ The waveform in use
    telemetry SignalType: SignalType id 1

    @ Signal samples
    product container DataContainer id 0 default priority 10

    @ One sample
    product record DataRecord: F32 id 0

  }

}
