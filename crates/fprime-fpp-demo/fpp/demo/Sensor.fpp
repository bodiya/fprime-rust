@ Demo components and topology: a queued sensor with every kind of
@ component member, a passive ground stub that closes every port, and the
@ topology wiring them (patterns, port arrays, numbering).
module Demo {

  @ A report of one reading
  port Report(seq: U32, reading: Reading)

  @ A queued sensor, drained by a rate-group tick
  queued component Sensor {

    @ Rate-group tick: drains the queue, then reports
    sync input port tick: Svc.Sched

    @ A delivered buffer, queued (the buffer crosses the queue by escrow)
    async input port deliver: Deliver priority 2 drop

    @ A guarded measurement with a return value
    guarded input port measure: Measure

    @ The report output
    output port report: Report

    @ A fan-out port array
    output port fanout: [2] Tick

    command recv port cmdIn
    command reg port cmdRegOut
    command resp port cmdResponseOut
    event port eventOut
    text event port textEventOut
    time get port timeGetOut
    telemetry port tlmOut
    param get port prmGetOut
    param set port prmSetOut
    product get port productGetOut
    product send port productSendOut

    @ Configure the sensor (queued)
    async command CONFIGURE(
      mode: Mode @< The mode
      gain: F32 @< The gain
      label: string size 8 @< A label
    ) opcode 0x10 priority 3

    @ A synchronous no-op
    sync command PING

    @ A guarded reset
    guarded command RESET(count: U32)

    @ The sensor was configured
    event Configured(mode: Mode, gain: F32) severity activity high format "Configured {} with gain {.2f}"

    @ Too many resets, throttled
    event Overrun(n: U32) severity warning low format "Overrun {}" throttle 2

    @ A labelled reading
    event Labelled(label: string size 8, reading: Reading) severity diagnostic format "{} -> {}"

    @ The current value
    telemetry Value: F32 update always format "{f}"

    @ The current mode, only when it changes
    telemetry Mode: Mode update on change

    @ The label, only when it changes
    telemetry Label: string size 8 update on change

    @ The gain parameter
    param Gain: F32 default 2.5

    @ The threshold parameter, no default
    param Threshold: U32

    @ Sample container
    product container Samples id 0 default priority 9

    @ One reading
    product record Sample: Reading id 0

    @ Raw bytes
    product record Raw: U8 array id 1

    @ Internal recompute request
    internal port recompute(scale: F32) priority 1

  }

  @ The ground stub: the sink side of every special port, plus the ports
  @ needed to drive the sensor
  passive component Ground {

    sync input port cmdRegIn: [4] Fw.CmdReg
    output port cmdOut: [4] Fw.Cmd
    sync input port cmdRespIn: Fw.CmdResponse
    sync input port logIn: Fw.Log
    sync input port textLogIn: Fw.LogText
    sync input port tlmIn: Fw.Tlm
    sync input port timeIn: Fw.Time
    sync input port prmGetIn: Fw.PrmGet
    sync input port prmSetIn: Fw.PrmSet
    sync input port dpGetIn: Fw.DpGet
    sync input port dpSendIn: Fw.DpSend
    sync input port reportIn: Report
    sync input port tickIn: [2] Tick
    output port tickOut: Svc.Sched
    output port deliverOut: Deliver
    output port measureOut: Measure

    match cmdOut with cmdRegIn

  }

  @ The sensor instance (the type clause is the Rust implementation path)
  instance sensor: Sensor base id 0x1000 type "crate::sensor::Sensor" queue size 16

  @ The ground instance
  instance ground: Ground base id 0x2000 type "crate::ground::Ground"

  @ The demo topology
  topology Demo {

    instance sensor
    instance ground

    command connections instance ground
    event connections instance ground
    text event connections instance ground
    telemetry connections instance ground
    time connections instance ground
    param connections instance ground

    connections Data {
      sensor.report -> ground.reportIn
      sensor.fanout -> ground.tickIn
      sensor.fanout -> ground.tickIn[1]
      ground.tickOut -> sensor.tick
      ground.deliverOut -> sensor.deliver
      ground.measureOut -> sensor.measure
      sensor.productGetOut -> ground.dpGetIn
      sensor.productSendOut -> ground.dpSendIn
    }

  }

}
