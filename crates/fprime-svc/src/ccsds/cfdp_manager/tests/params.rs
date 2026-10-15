//! Parameter tests — ports of `CfdpManagerParameterTests.cpp`: every
//! parameter loads its FPP default (`DEFAULT`) and takes a `PRM_SET`
//! (`VALID`); plus `PRM_SAVE`, database values and the dictionary ids.

use super::*;

fn prm_set(
    h: &Harness,
    opcode: FwOpcodeType,
    write: impl FnOnce(&mut CmdArgBuffer),
) -> CmdResponse {
    let mut args = CmdArgBuffer::new();
    write(&mut args);
    CmdPort::invoke(h.comp.as_ref(), 0, ID_BASE + opcode, 1, &mut args);
    // Synchronous: answered before any dispatch.
    h.cmd_responses().last().unwrap().2
}

#[test]
fn every_parameter_loads_its_default() {
    let h = Harness::new();
    let c = &h.comp;
    assert_eq!(c.param_get_local_eid(), (42, ParamValid::Default));
    assert_eq!(
        c.param_get_outgoing_file_chunk_size(),
        (992, ParamValid::Default)
    );
    assert_eq!(
        c.param_get_rx_crc_calc_bytes_per_cycle(),
        (65536, ParamValid::Default)
    );
    assert_eq!(
        c.param_get_post_inactivity_send_retries(),
        (3, ParamValid::Default)
    );
    assert_eq!(
        c.param_get_file_in_default_channel(),
        (0, ParamValid::Default)
    );
    assert_eq!(
        c.param_get_file_in_default_dest_entity_id(),
        (100, ParamValid::Default)
    );
    assert_eq!(
        c.param_get_file_in_default_class(),
        (Class::Class2, ParamValid::Default)
    );
    assert_eq!(
        c.param_get_file_in_default_keep(),
        (Keep::Delete, ParamValid::Default)
    );
    assert_eq!(
        c.param_get_file_in_default_priority(),
        (0, ParamValid::Default)
    );
    let (cfg, valid) = c.param_get_channel_config();
    assert_eq!(valid, ParamValid::Default);
    for ch in 0..NUM_CHANNELS {
        let p = &cfg[ch];
        assert_eq!(
            (p.ack_limit, p.nack_limit, p.ack_timer, p.inactivity_timer),
            (10, 10, 2, 120)
        );
        assert_eq!(p.dequeue_enabled, Enabled::Enabled);
        assert_eq!(p.max_outgoing_pdus_per_cycle, 64);
        assert_eq!(p.move_dir.as_bytes(), b"");
        assert_eq!(p.tmp_dir.as_bytes(), b"/tmp");
        assert_eq!(p.fail_dir.as_bytes(), b"/fail");
    }
}

#[test]
fn every_parameter_takes_a_prm_set() {
    let h = Harness::new();
    let c = &h.comp;
    let u32_set = |op, v: u32| prm_set(&h, op, |a| assert!(a.serialize_u32_be(v).is_ok()));
    let u8_set = |op, v: u8| prm_set(&h, op, |a| assert!(a.serialize_u8_be(v).is_ok()));
    assert_eq!(
        u32_set(CfdpManager::OPCODE_LOCALEID_PRM_SET, 77),
        CmdResponse::Ok
    );
    assert_eq!(c.param_get_local_eid(), (77, ParamValid::Valid));
    assert_eq!(
        u32_set(CfdpManager::OPCODE_OUTGOINGFILECHUNKSIZE_PRM_SET, 512),
        CmdResponse::Ok
    );
    assert_eq!(
        c.param_get_outgoing_file_chunk_size(),
        (512, ParamValid::Valid)
    );
    assert_eq!(
        u32_set(CfdpManager::OPCODE_RXCRCCALCBYTESPERCYCLE_PRM_SET, 32768),
        CmdResponse::Ok
    );
    assert_eq!(
        c.param_get_rx_crc_calc_bytes_per_cycle(),
        (32768, ParamValid::Valid)
    );
    assert_eq!(
        u8_set(CfdpManager::OPCODE_POSTINACTIVITYSENDRETRIES_PRM_SET, 9),
        CmdResponse::Ok
    );
    assert_eq!(
        c.param_get_post_inactivity_send_retries(),
        (9, ParamValid::Valid)
    );
    assert_eq!(
        u8_set(CfdpManager::OPCODE_FILEINDEFAULTCHANNEL_PRM_SET, 1),
        CmdResponse::Ok
    );
    assert_eq!(
        c.param_get_file_in_default_channel(),
        (1, ParamValid::Valid)
    );
    assert_eq!(
        u32_set(CfdpManager::OPCODE_FILEINDEFAULTDESTENTITYID_PRM_SET, 200),
        CmdResponse::Ok
    );
    assert_eq!(
        c.param_get_file_in_default_dest_entity_id(),
        (200, ParamValid::Valid)
    );
    assert_eq!(
        u8_set(CfdpManager::OPCODE_FILEINDEFAULTCLASS_PRM_SET, 1),
        CmdResponse::Ok
    );
    assert_eq!(
        c.param_get_file_in_default_class(),
        (Class::Class1, ParamValid::Valid)
    );
    assert_eq!(
        u8_set(CfdpManager::OPCODE_FILEINDEFAULTKEEP_PRM_SET, 1),
        CmdResponse::Ok
    );
    assert_eq!(
        c.param_get_file_in_default_keep(),
        (Keep::Keep, ParamValid::Valid)
    );
    assert_eq!(
        u8_set(CfdpManager::OPCODE_FILEINDEFAULTPRIORITY_PRM_SET, 4),
        CmdResponse::Ok
    );
    assert_eq!(
        c.param_get_file_in_default_priority(),
        (4, ParamValid::Valid)
    );
    let mut cfg = ChannelArrayParams::fpp_default();
    cfg[1].ack_limit = 3;
    cfg[1].tmp_dir = PathString::from("/var/tmp");
    let rsp = prm_set(&h, CfdpManager::OPCODE_CHANNELCONFIG_PRM_SET, |a| {
        assert!(cfg.serialize_to(a, Endianness::Big).is_ok());
    });
    assert_eq!(rsp, CmdResponse::Ok);
    assert_eq!(c.get_ack_limit_param(1), 3);
    assert_eq!(c.get_tmp_dir_param(1).as_bytes(), b"/var/tmp");
    // Format errors: short, leftover, undeclared enum value.
    assert_eq!(
        prm_set(&h, CfdpManager::OPCODE_LOCALEID_PRM_SET, |_| {}),
        CmdResponse::FormatError
    );
    assert_eq!(
        prm_set(&h, CfdpManager::OPCODE_FILEINDEFAULTPRIORITY_PRM_SET, |a| {
            assert!(a.serialize_u16_be(1).is_ok());
        }),
        CmdResponse::FormatError
    );
    assert_eq!(
        u8_set(CfdpManager::OPCODE_FILEINDEFAULTCLASS_PRM_SET, 9),
        CmdResponse::FormatError
    );
    assert_eq!(
        c.param_get_file_in_default_class(),
        (Class::Class1, ParamValid::Valid)
    );
}

#[test]
fn prm_save_pushes_the_value_to_the_database() {
    let h = Harness::new();
    let rsp = prm_set(&h, CfdpManager::OPCODE_LOCALEID_PRM_SAVE, |_| {});
    assert_eq!(rsp, CmdResponse::Ok);
    let rsp = prm_set(&h, CfdpManager::OPCODE_FILEINDEFAULTKEEP_PRM_SAVE, |_| {});
    assert_eq!(rsp, CmdResponse::Ok);
    assert_eq!(
        *h.ground.prm_sets.lock(),
        [
            (CfdpManager::PARAMID_LOCAL_EID, vec![0, 0, 0, 42]),
            (CfdpManager::PARAMID_FILE_IN_DEFAULT_KEEP, vec![0]),
        ]
    );
}

#[test]
fn database_values_load_as_valid() {
    let h = Harness::with_params(|g| {
        let mut db = g.prm_db.lock();
        db.insert(CfdpManager::PARAMID_LOCAL_EID, vec![0, 0, 1, 0]);
        // A value that does not deserialize falls back to the default.
        db.insert(CfdpManager::PARAMID_OUTGOING_FILE_CHUNK_SIZE, vec![1]);
    });
    assert_eq!(h.comp.param_get_local_eid(), (256, ParamValid::Valid));
    assert_eq!(
        h.comp.param_get_outgoing_file_chunk_size(),
        (992, ParamValid::Default)
    );
}

#[test]
fn local_eid_parameter_drives_the_pdus() {
    let h = Harness::new();
    let rsp = prm_set(&h, CfdpManager::OPCODE_LOCALEID_PRM_SET, |a| {
        assert!(a.serialize_u32_be(0x1234).is_ok());
    });
    assert_eq!(rsp, CmdResponse::Ok);
    let dir = temp_dir();
    let src = path_in(&dir, "eid.bin");
    make_file(&src, 10);
    h.send_file_cmd(CH0, 7, Class::Class1, Keep::Keep, 0, &src, "/d");
    h.run_cycle();
    let md = decode_md(&h.sent()[0]);
    assert_eq!(md.as_header().get_source_eid(), 0x1234);
    assert_eq!(md.as_header().get_dest_eid(), 7);
}

#[test]
fn dictionary_ids_match_upstream() {
    // `fpp-to-rust --dict` on a topology instantiating
    // `Svc.Ccsds.Cfdp.CfdpManager` from the pinned upstream model.
    assert_eq!(CfdpManager::OPCODE_SEND_FILE, 0x0);
    assert_eq!(CfdpManager::OPCODE_RESET_COUNTERS, 0x8);
    assert_eq!(CfdpManager::OPCODE_LOCALEID_PRM_SET, 0x9);
    assert_eq!(CfdpManager::OPCODE_CHANNELCONFIG_PRM_SAVE, 0x1C);
    assert_eq!(CfdpManager::EVENTID_BUFFERS_EXHAUSTED, 0);
    assert_eq!(CfdpManager::EVENTID_RX_ACK_LIMIT_REACHED, 23);
    assert_eq!(CfdpManager::EVENTID_TX_ACK_LIMIT_REACHED, 41);
    assert_eq!(CfdpManager::EVENTID_METADATA_RECEIVED, 57);
    assert_eq!(CfdpManager::EVENTID_RESET_FREED_TRANSACTION, 68);
    assert_eq!(CfdpManager::EVENTID_TX_LATE_FIN_ACKED, 76);
    assert_eq!(CfdpManager::CHANID_CHANNEL_TELEMETRY, 0);
    assert_eq!(CfdpManager::PARAMID_CHANNEL_CONFIG, 9);
    assert_eq!(ChannelTelemetry::SERIALIZED_SIZE, 116);
    assert_eq!(ChannelTelemetryArray::SERIALIZED_SIZE, 232);
}
