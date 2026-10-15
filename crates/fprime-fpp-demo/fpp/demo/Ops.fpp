@ The Ops deployment: a controller driven by an internal and an external
@ state machine, a console closing the special ports, and the hand-written
@ Svc.TlmPacketizer, configured from the topology's packet table by an
@ init specifier (as upstream deployments do it)
module Demo {

  @ Ticks the controller spends warming up
  constant WARMUP_TICKS = 3

  @ The controller's lifecycle
  state machine Lifecycle {

    @ Start running
    signal start

    @ Stop running
    signal stop

    @ One scheduler tick, with the tick count
    signal tick: U32

    @ Entered RUNNING
    action enterRunning

    @ Left RUNNING
    action exitRunning

    @ Record a tick
    action countTick: U32

    @ Whether the warm-up is over
    guard warmedUp: U32

    initial enter IDLE

    @ Not running
    state IDLE {
      on start enter RUNNING
    }

    @ Running: warming up, then active
    state RUNNING {
      entry do { enterRunning }
      exit do { exitRunning }
      initial enter WARMUP
      on stop enter IDLE

      @ Warming up: each tick checks
      state WARMUP {
        on tick enter CHECK
      }

      @ Warm-up check (typed by the tick)
      choice CHECK {
        if warmedUp do { countTick } enter ACTIVE else do { countTick } enter WARMUP
      }

      @ Active: ticks are counted
      state ACTIVE {
        on tick do { countTick }
      }
    }

  }

  @ A watchdog written by hand in Rust (an external state machine)
  state machine Watchdog

  @ The controller
  active component Controller {

    @ Scheduler tick
    async input port schedIn: Svc.Sched

    command recv port cmdIn
    command reg port cmdRegOut
    command resp port cmdResponseOut
    event port eventOut
    text event port textEventOut
    time get port timeGetOut
    telemetry port tlmOut

    @ Start the lifecycle
    async command START

    @ Stop the lifecycle
    async command STOP

    @ Kick the watchdog
    async command KICK

    @ Entered RUNNING
    event RunningEntered severity activity high format "Running"

    @ Left RUNNING
    event RunningExited severity activity high format "Stopped"

    @ The watchdog expired
    event WatchdogExpired severity warning high format "Watchdog expired"

    @ The lifecycle state
    telemetry State: Lifecycle.State

    @ Ticks counted
    telemetry Ticks: U32

    @ The lifecycle instance
    state machine instance lifecycle: Lifecycle priority 2 drop

    @ The watchdog instance
    state machine instance watchdog: Watchdog priority 1 hook

  }

  @ The console: commands in, everything else out
  passive component Console {

    sync input port cmdRegIn: [4] Fw.CmdReg
    output port cmdOut: [4] Fw.Cmd
    sync input port cmdRespIn: Fw.CmdResponse
    sync input port logIn: Fw.Log
    sync input port textLogIn: Fw.LogText
    sync input port timeIn: Fw.Time
    sync input port prmGetIn: Fw.PrmGet
    sync input port prmSetIn: Fw.PrmSet

    @ Telemetry packets
    sync input port pktIn: Fw.Com

    @ Scheduler ticks
    output port schedOut: [2] Svc.Sched

    match cmdOut with cmdRegIn

  }

  @ The controller instance
  instance controller: Controller base id 0x4000 type "crate::controller::Controller" \
    queue size 10 stack size 0 priority 1

  @ The console instance, built by an `instances` init specifier
  instance console: Console base id 0x6000 type "crate::console::Console" {
    phase Fpp.ToCpp.Phases.instances """
    crate::console::Console::new("ops console")
    """
  }

  @ The hand-written telemetry packetizer (bound to fprime_svc), configured
  @ from the Ops packet table in the configComponents phase
  instance tlmSend: Svc.TlmPacketizer base id 0x5000 queue size 10 stack size 0 priority 1 {
    phase Fpp.ToCpp.Phases.configConstants """
    /// The packetizer's start level.
    pub const START_LEVEL: u32 = 1;
    """
    phase Fpp.ToCpp.Phases.configComponents """
    self.tlm_send.set_packet_list(
        Ops_OpsPacketsTlmPackets::PACKET_LIST,
        Ops_OpsPacketsTlmPackets::OMITTED_CHANNELS,
        Ops_ConfigConstants::tlmSend::START_LEVEL,
    );
    """
  }

  @ The Ops topology
  deployment topology Ops {

    instance controller
    instance console
    instance tlmSend

    command connections instance console
    event connections instance console
    text event connections instance console
    time connections instance console
    param connections instance console
    telemetry connections instance tlmSend

    connections Ops {
      console.schedOut[0] -> controller.schedIn
      console.schedOut[1] -> tlmSend.Run
      tlmSend.PktSend[0] -> console.pktIn
    }

    @ The controller's telemetry
    telemetry packets OpsPackets {

      packet ControllerStatus group 1 {
        controller.State
        controller.Ticks
      }

    } omit {
      tlmSend.GroupConfigs
      tlmSend.SectionEnabled
    }

  }

}
