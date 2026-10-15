//! # Two deployments joined by a GenericHub pair
//!
//! The hub pattern end to end (`Svc/GenericHub/docs/sdd.md`, "Command
//! routing" and "Buffer lifecycle"), with real components on both sides:
//!
//! ```text
//!   deployment A                                  deployment B
//!   ------------                                  ------------
//!   test --cmdDispIn[0]--> hubA ~~wire~~> hubB --cmdDispOut[0]--> CmdDispatcher
//!   test <-cmdRespOut[0]-- hubA <~~wire~~ hubB <-cmdRespIn[0]---- seqCmdStatus[0]
//!   test <--eventOut------ hubA <~~wire~~ hubB <-eventIn--------- CmdDispatcher events
//!   pingOut --serialIn[0]> hubA ~~wire~~> hubB --serialOut[0]---> CmdDispatcher.pingIn
//!   pingIn <-serialOut[0]- hubA <~~wire~~ hubB <-serialIn[0]----- CmdDispatcher.pingOut
//!   test --bufferIn[0]---> hubA ~~wire~~> hubB --bufferOut[0]---> consumer
//! ```
//!
//! Each side allocates transport buffers from its own `BufferManager`. The
//! "wire" is a buffer driver stand-in that copies the bytes into a buffer
//! from the FAR side's manager and returns the near one — only bytes cross,
//! as between address spaces. At the end both managers report zero buffers
//! outstanding: every move came home.

use fprime_comp::{
    BufferGetPort, BufferSendPort, CmdResponsePort, LogPort, OutputPort, PingPort, PortRef, TlmPort,
};
use fprime_config::{FwChanIdType, FwEventIdType, FwIndexType, FwOpcodeType, FwSizeType};
use fprime_fw::{Buffer, CmdResponse, ComBuffer, LogBuffer, LogSeverity, SerBuf, Time, TlmBuffer};
use fprime_os::task::TASK_DEFAULT;
use fprime_svc::buffer_manager::{BufferBin, BufferManager};
use fprime_svc::cmd_dispatcher::CmdDispatcher;
use fprime_svc::generic_hub::GenericHub;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

/// Buffer driver stand-in: one direction of the transport.
struct Wire {
    /// The far side's allocator (the receiving driver's buffers).
    far_allocate: OutputPort<dyn BufferGetPort>,
    /// The far hub's `fromBufferDriver`.
    far_deliver: OutputPort<dyn BufferSendPort>,
    /// The near hub's `toBufferDriverReturn`.
    near_return: OutputPort<dyn BufferSendPort>,
    /// Messages carried.
    carried: Mutex<usize>,
}

impl Wire {
    fn new() -> Arc<Self> {
        Arc::new(Self {
            far_allocate: OutputPort::new(),
            far_deliver: OutputPort::new(),
            near_return: OutputPort::new(),
            carried: Mutex::new(0),
        })
    }
}

impl BufferSendPort for Wire {
    fn invoke(&self, _port_num: FwIndexType, buffer: Buffer) {
        let alloc = self.far_allocate.get();
        let mut copy = alloc
            .target
            .invoke(alloc.port_num, buffer.size() as FwSizeType);
        assert_eq!(copy.size(), buffer.size(), "far side out of buffers");
        copy.data_mut().copy_from_slice(buffer.data());
        *self.carried.lock().unwrap() += 1;
        let back = self.near_return.get();
        back.target.invoke(back.port_num, buffer);
        let deliver = self.far_deliver.get();
        deliver.target.invoke(deliver.port_num, copy);
    }
}

/// Deployment A's receive-side sinks.
#[derive(Default)]
struct Ground {
    events: Mutex<Vec<(FwEventIdType, LogSeverity)>>,
    responses: Mutex<Vec<(FwIndexType, FwOpcodeType, u32, CmdResponse)>>,
    pings: Mutex<Vec<u32>>,
}

impl LogPort for Ground {
    fn invoke(
        &self,
        _port_num: FwIndexType,
        id: FwEventIdType,
        _time_tag: &mut Time,
        severity: LogSeverity,
        _args: &mut LogBuffer,
    ) {
        self.events.lock().unwrap().push((id, severity));
    }
}

impl CmdResponsePort for Ground {
    fn invoke(
        &self,
        port_num: FwIndexType,
        op_code: FwOpcodeType,
        cmd_seq: u32,
        response: CmdResponse,
    ) {
        self.responses
            .lock()
            .unwrap()
            .push((port_num, op_code, cmd_seq, response));
    }
}

impl PingPort for Ground {
    fn invoke(&self, _port_num: FwIndexType, key: u32) {
        self.pings.lock().unwrap().push(key);
    }
}

/// Deployment B's buffer consumer: checks the payload and returns it.
struct Consumer {
    hub: Arc<GenericHub>,
    received: Mutex<Vec<Vec<u8>>>,
}

impl BufferSendPort for Consumer {
    fn invoke(&self, port_num: FwIndexType, buffer: Buffer) {
        self.received.lock().unwrap().push(buffer.data().to_vec());
        let back = self.hub.buffer_out_return(port_num);
        back.target.invoke(back.port_num, buffer);
    }
}

/// Captures a manager's `CurrBuffs` channel.
#[derive(Default)]
struct CurrBuffs(Mutex<Option<u32>>);

impl TlmPort for CurrBuffs {
    fn invoke(
        &self,
        _port_num: FwIndexType,
        id: FwChanIdType,
        _time_tag: &mut Time,
        val: &mut TlmBuffer,
    ) {
        if id == BufferManager::CHANID_CURR_BUFFS {
            let mut value = 0u32;
            assert!(val.deserialize_u32_be(&mut value).is_ok());
            *self.0.lock().unwrap() = Some(value);
        }
    }
}

fn manager(name: &str, mgr_id: u16) -> (Arc<BufferManager>, Arc<CurrBuffs>) {
    let bm = BufferManager::new(name);
    let curr = Arc::new(CurrBuffs::default());
    bm.tlm.tlm_out.connect(curr.clone(), 0);
    bm.setup(
        mgr_id,
        &[BufferBin {
            buffer_size: 1024,
            num_buffers: 8,
        }],
    );
    (bm, curr)
}

/// Buffers outstanding, read through the manager's own telemetry.
fn outstanding(bm: &Arc<BufferManager>, curr: &CurrBuffs) -> u32 {
    let sched = bm.sched_in(0);
    sched.target.invoke(sched.port_num, 0);
    curr.0.lock().unwrap().unwrap()
}

fn wait_for(what: &str, mut done: impl FnMut() -> bool) {
    let deadline = Instant::now() + Duration::from_secs(5);
    while !done() {
        assert!(Instant::now() < deadline, "timed out waiting for {what}");
        std::thread::sleep(Duration::from_millis(5));
    }
}

#[test]
fn commands_events_serial_calls_and_buffers_cross_the_hub_pair() {
    const DISP_ID_BASE: u32 = 0x500;

    // -- Construct ---------------------------------------------------------
    let hub_a = GenericHub::new("hubA");
    let hub_b = GenericHub::new("hubB");
    let (bm_a, curr_a) = manager("bmA", 1);
    let (bm_b, curr_b) = manager("bmB", 2);
    let a_to_b = Wire::new();
    let b_to_a = Wire::new();
    let dispatcher = CmdDispatcher::new("cmdDispB");
    dispatcher.active.queued.base.set_id_base(DISP_ID_BASE);
    let ground = Arc::new(Ground::default());
    let consumer = Arc::new(Consumer {
        hub: hub_b.clone(),
        received: Mutex::new(Vec::new()),
    });

    // -- Wire the transport ---------------------------------------------------
    for (hub, bm) in [(&hub_a, &bm_a), (&hub_b, &bm_b)] {
        hub.allocate.connect_to(bm.buffer_get_callee_in(0));
        hub.deallocate.connect_to(bm.buffer_send_in(0));
        hub.from_buffer_driver_return
            .connect_to(bm.buffer_send_in(0));
    }
    hub_a.to_buffer_driver.connect(a_to_b.clone(), 0);
    a_to_b
        .near_return
        .connect_to(hub_a.to_buffer_driver_return(0));
    a_to_b.far_allocate.connect_to(bm_b.buffer_get_callee_in(0));
    a_to_b.far_deliver.connect_to(hub_b.from_buffer_driver(0));
    hub_b.to_buffer_driver.connect(b_to_a.clone(), 0);
    b_to_a
        .near_return
        .connect_to(hub_b.to_buffer_driver_return(0));
    b_to_a.far_allocate.connect_to(bm_a.buffer_get_callee_in(0));
    b_to_a.far_deliver.connect_to(hub_a.from_buffer_driver(0));

    // -- Wire the logical connections -----------------------------------------
    // Commands A -> B and responses B -> A.
    hub_b.cmd_disp_out[0].connect_to(dispatcher.seq_cmd_buff_in(0));
    dispatcher.seq_cmd_status[0].connect_to(hub_b.cmd_resp_in(0));
    hub_a.cmd_resp_out[0].connect(ground.clone(), 0);
    // B's dispatcher runs its own commands and sends its events to A.
    dispatcher
        .cmd
        .cmd_reg_out
        .connect_to(dispatcher.comp_cmd_reg_in(0));
    dispatcher.comp_cmd_send[0].connect_to(dispatcher.cmd_in(0));
    dispatcher
        .cmd
        .cmd_response_out
        .connect_to(dispatcher.comp_cmd_stat_in(0));
    dispatcher.evt.log_out.connect_to(hub_b.event_in(0));
    hub_a.event_out.connect(ground.clone(), 0);
    // A health ping across the hubs: typed Svc.Ping both ways.
    let ping_out: OutputPort<dyn PingPort> = OutputPort::new();
    ping_out.connect_serial(hub_a.serial_in(0));
    hub_b.serial_out[0].connect_typed(dispatcher.ping_in(0));
    dispatcher.ping_out.connect_serial(hub_b.serial_in(0));
    hub_a.serial_out[0].connect_typed(PortRef::new(ground.clone() as Arc<dyn PingPort>, 0));
    // Buffers A -> B.
    hub_b.buffer_out[0].connect(consumer.clone(), 0);
    hub_a.buffer_in_return[0].connect_to(bm_a.buffer_send_in(0));

    // -- Start ----------------------------------------------------------------
    dispatcher.init(10);
    dispatcher.reg_commands();
    dispatcher
        .active
        .start(&dispatcher, 50, TASK_DEFAULT, TASK_DEFAULT);

    // Registration events crossed already (on this thread).
    let registered = DISP_ID_BASE + CmdDispatcher::EVENTID_OP_CODE_REGISTERED;
    assert_eq!(
        ground
            .events
            .lock()
            .unwrap()
            .iter()
            .filter(|(id, _)| *id == registered)
            .count(),
        4
    );
    ground.events.lock().unwrap().clear();

    // -- A remote NO_OP: A's command splitter role is played by the test -------
    let op_code = DISP_ID_BASE + CmdDispatcher::OPCODE_CMD_NO_OP;
    let mut packet = ComBuffer::new();
    assert!(packet.serialize_u16_be(0).is_ok()); // FW_PACKET_COMMAND
    assert!(packet.serialize_u32_be(op_code).is_ok());
    let cmd = hub_a.cmd_disp_in(0);
    cmd.target.invoke(cmd.port_num, &mut packet, 0x77);
    wait_for("the command response", || {
        !ground.responses.lock().unwrap().is_empty()
    });
    assert_eq!(
        *ground.responses.lock().unwrap(),
        vec![(0, op_code, 0x77, CmdResponse::Ok)]
    );
    wait_for("the command events", || {
        ground.events.lock().unwrap().len() >= 3
    });
    let events = ground.events.lock().unwrap().clone();
    assert_eq!(
        events,
        vec![
            (
                DISP_ID_BASE + CmdDispatcher::EVENTID_OP_CODE_DISPATCHED,
                LogSeverity::Command
            ),
            (
                DISP_ID_BASE + CmdDispatcher::EVENTID_NO_OP_RECEIVED,
                LogSeverity::ActivityHi
            ),
            (
                DISP_ID_BASE + CmdDispatcher::EVENTID_OP_CODE_COMPLETED,
                LogSeverity::Command
            ),
        ]
    );

    // -- A ping round trip through both hubs' serial ports ---------------------
    let p = ping_out.get();
    p.target.invoke(p.port_num, 0x0BAD_CAFE);
    wait_for("the ping reply", || {
        !ground.pings.lock().unwrap().is_empty()
    });
    assert_eq!(*ground.pings.lock().unwrap(), vec![0x0BAD_CAFE]);

    // -- A buffer: copied out on A, delivered in place on B --------------------
    let get = bm_a.buffer_get_callee_in(0);
    let mut buffer = get.target.invoke(get.port_num, 300);
    for (i, byte) in buffer.data_mut().iter_mut().enumerate() {
        *byte = i as u8;
    }
    let expected = buffer.data().to_vec();
    let send = hub_a.buffer_in(0);
    send.target.invoke(send.port_num, buffer);
    assert_eq!(*consumer.received.lock().unwrap(), vec![expected]);

    // -- Stop; every buffer is home ---------------------------------------------
    dispatcher.active.exit();
    let _ = dispatcher.active.join();
    assert_eq!(outstanding(&bm_a, &curr_a), 0);
    assert_eq!(outstanding(&bm_b, &curr_b), 0);
    // Registration (4) + dispatched/received/completed (3) events, the
    // command, the response, ping + reply, and the buffer.
    assert_eq!(*a_to_b.carried.lock().unwrap(), 3);
    assert_eq!(*b_to_a.carried.lock().unwrap(), 4 + 3 + 1 + 1);
}
