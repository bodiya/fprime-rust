# The Rust reference deployment as an FPP model: the component instances of
# `crates/fprime-ref/src/topology.rs` with their base ids (the C++ ones of
# `TestDeploymentsProject/Ref` and the `CdhCore`, `ComFprime`,
# `FileHandling`, `DataProducts` and `ComLoggerTee` subtopologies). Its
# purpose is the ground dictionary (`fpp-to-rust --dict`), so the topology
# lists instances only; the wiring lives in `topology.rs`.
#
# Regenerate `crates/fprime-ref/dictionary/RefTopologyDictionary.json` with
# `crates/fprime-ref/fpp/generate-dictionary.sh <fprime checkout>`.

module Ref {

  module Default {
    constant QUEUE_SIZE = 10
    constant STACK_SIZE = 64 * 1024
  }

  # CdhCore
  instance cmdDisp: Svc.CommandDispatcher base id 0x01000000 \
    queue size 20 stack size Default.STACK_SIZE priority 30
  instance events: Svc.EventManager base id 0x01001000 \
    queue size Default.QUEUE_SIZE stack size Default.STACK_SIZE priority 25
  instance $health: Svc.Health base id 0x01002000 queue size 25
  instance textLogger: Svc.PassiveTextLogger base id 0x01003000
  instance fatalHandler: Svc.FatalHandler base id 0x01004000
  instance tlmSend: Svc.TlmChan base id 0x01005000 \
    queue size Default.QUEUE_SIZE stack size Default.STACK_SIZE priority 24

  # ComFprime
  instance comQueue: Svc.ComQueue base id 0x02000000 \
    queue size Default.QUEUE_SIZE stack size Default.STACK_SIZE priority 100
  instance frameAccumulator: Svc.FrameAccumulator base id 0x02001000
  instance bufferManager: Svc.BufferManager base id 0x02002000
  instance deframer: Svc.FprimeDeframer base id 0x02003000
  instance framer: Svc.FprimeFramer base id 0x02004000
  instance fprimeRouter: Svc.FprimeRouter base id 0x02005000
  instance comStub: Svc.ComStub base id 0x02006000

  # DataProducts
  instance dpCat: Svc.DpCatalog base id 0x04000000 \
    queue size Default.QUEUE_SIZE stack size Default.STACK_SIZE priority 25
  instance dpMgr: Svc.DpManager base id 0x04001000 \
    queue size Default.QUEUE_SIZE stack size Default.STACK_SIZE priority 25
  instance dpWriter: Svc.DpWriter base id 0x04002000 \
    queue size Default.QUEUE_SIZE stack size Default.STACK_SIZE priority 25
  instance dpBufferManager: Svc.BufferManager base id 0x04003000

  # FileHandling
  instance fileUplink: Svc.FileUplink base id 0x05000000 \
    queue size 30 stack size Default.STACK_SIZE priority 30
  instance fileDownlink: Svc.FileDownlink base id 0x05001000 \
    queue size 30 stack size Default.STACK_SIZE priority 20
  instance fileManager: Svc.FileManager base id 0x05002000 \
    queue size 30 stack size Default.STACK_SIZE priority 20
  instance prmDb: Svc.PrmDb base id 0x05003000 \
    queue size Default.QUEUE_SIZE stack size Default.STACK_SIZE priority 20

  # Ref (main)
  instance rateGroup1Comp: Svc.ActiveRateGroup base id 0x10001000 \
    queue size Default.QUEUE_SIZE stack size Default.STACK_SIZE priority 43
  instance rateGroup2Comp: Svc.ActiveRateGroup base id 0x10002000 \
    queue size Default.QUEUE_SIZE stack size Default.STACK_SIZE priority 42
  instance rateGroup3Comp: Svc.ActiveRateGroup base id 0x10003000 \
    queue size Default.QUEUE_SIZE stack size Default.STACK_SIZE priority 41
  instance cmdSeq: Svc.CmdSequencer base id 0x10006000 \
    queue size Default.QUEUE_SIZE stack size Default.STACK_SIZE priority 20
  instance signalGen: Ref.SignalGen base id 0x10011000 \
    queue size Default.QUEUE_SIZE
  instance posixTime: Svc.PosixTime base id 0x10020000
  instance rateGroupDriverComp: Svc.RateGroupDriver base id 0x10021000
  instance systemResources: Svc.SystemResources base id 0x10023000
  instance linuxTimer: Svc.LinuxTimer base id 0x10024000
  instance comDriver: Drv.TcpClient base id 0x10025000

  # ComLoggerTee
  instance comLog: Svc.ComLogger base id 0x10500000 \
    queue size Default.QUEUE_SIZE stack size Default.STACK_SIZE priority 20
  instance comSplitter: Svc.ComSplitter base id 0x10500100

  @ The Rust reference deployment
  deployment topology Ref {
    instance cmdDisp
    instance events
    instance $health
    instance textLogger
    instance fatalHandler
    instance tlmSend
    instance comQueue
    instance frameAccumulator
    instance bufferManager
    instance deframer
    instance framer
    instance fprimeRouter
    instance comStub
    instance dpCat
    instance dpMgr
    instance dpWriter
    instance dpBufferManager
    instance fileUplink
    instance fileDownlink
    instance fileManager
    instance prmDb
    instance rateGroup1Comp
    instance rateGroup2Comp
    instance rateGroup3Comp
    instance cmdSeq
    instance signalGen
    instance posixTime
    instance rateGroupDriverComp
    instance systemResources
    instance linuxTimer
    instance comDriver
    instance comLog
    instance comSplitter
  }

}
