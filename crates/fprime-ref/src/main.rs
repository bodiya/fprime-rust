//! # fprime-ref — reference deployment binary
//!
//! Rust port of `TestDeploymentsProject/Ref/Main.cpp`: parse `-a hostname
//! -p port` (both optional — without them the deployment runs standalone
//! with no comms) plus `-d data-dir` (this port's addition: the directory
//! the parameter database, uplinked files, data products and `.com` logs
//! live in, where C++ hard-codes `"PrmDb.dat"`, `"/tmp/uplink/"` and
//! `"./DpCat"`), set up the topology, run the blocking 1 Hz rate loop,
//! tear down, exit 0.
//!
//! Divergence from C++ (documented): C++ installs SIGINT/SIGTERM handlers
//! calling `stopRateGroups()`; zero-dependency safe Rust has no signal
//! API, so the graceful-stop path is a stdin watcher instead — the line
//! `quit` (or stdin EOF) requests the stop.

use std::io::BufRead;

use fprime_fw::{TimeInterval, fw_log};
use fprime_ref::topology::{RefTopology, TopologyConfig};

/// Print usage and exit with status 2 (getopt-style error path).
fn usage_error(program: &str, message: &str) -> ! {
    eprintln!("{message}");
    eprintln!("Usage: {program} [-a <hostname>] [-p <port>] [-d <data-dir>]");
    eprintln!("  -a, --address    GDS hostname (dotted-quad IPv4)");
    eprintln!("  -p, --port       GDS TCP port; comms need BOTH -a and -p");
    eprintln!("  -d, --data-dir   directory for the parameter database, uplinked");
    eprintln!("                   files, data products and .com logs");
    eprintln!("                   (default: <temp>/fprime-ref-<pid>)");
    std::process::exit(2);
}

/// Manual getopt-style parse of `-a <hostname> -p <port> -d <data-dir>`.
fn parse_args() -> TopologyConfig {
    let mut args = std::env::args();
    let program = args.next().unwrap_or_else(|| "fprime-ref".to_string());
    let mut config = TopologyConfig::default();
    while let Some(flag) = args.next() {
        match flag.as_str() {
            "-a" | "--address" => match args.next() {
                Some(hostname) => config.hostname = Some(hostname),
                None => usage_error(&program, "-a requires a hostname argument"),
            },
            "-p" | "--port" => match args.next().map(|v| v.parse::<u16>()) {
                Some(Ok(port)) if port != 0 => config.port = port,
                _ => usage_error(&program, "-p requires a port argument in [1, 65535]"),
            },
            "-d" | "--data-dir" => match args.next() {
                Some(dir) if !dir.is_empty() => config.data_dir = Some(dir),
                _ => usage_error(&program, "-d requires a non-empty directory argument"),
            },
            "-h" | "--help" => usage_error(&program, "fprime-ref reference deployment"),
            other => usage_error(&program, &format!("unknown option: {other}")),
        }
    }
    config
}

fn main() {
    // Os::init() equivalent: registers the console as the global fw logger.
    fprime_os::init();

    let config = parse_args();
    match config.hostname.as_deref() {
        Some(hostname) if config.port != 0 => {
            fw_log!(
                "Ref deployment starting (GDS at {}:{})\n",
                hostname,
                config.port
            );
        }
        _ => fw_log!("Ref deployment starting (no comms configured)\n"),
    }
    fw_log!(
        "Ref deployment data directory: {}\n",
        config.resolved_data_dir()
    );

    let topology = RefTopology::setup(&config);

    // Stdin watcher: 'quit' (or EOF) requests a graceful stop. It holds
    // only the timer Arc (the C++ signal handler's stopRateGroups
    // equivalent), so the topology stays uniquely owned by main. The
    // thread is detached; process exit reaps it if stdin stays open.
    let timer = topology.linux_timer.clone();
    let _ = std::thread::Builder::new()
        .name("stdinWatcher".to_string())
        .spawn(move || {
            let stdin = std::io::stdin();
            for line in stdin.lock().lines() {
                match line {
                    Ok(text) if text.trim() == "quit" => break,
                    Ok(_) => continue,
                    Err(_) => break,
                }
            }
            fw_log!("Ref deployment stop requested\n");
            timer.quit();
        });

    // Blocking 1 Hz rate loop (C++ startRateGroups(TimeInterval(1, 0))).
    fw_log!("Ref topology up; type 'quit' to exit\n");
    topology.start_rate_loop(TimeInterval::new(1, 0));

    topology.teardown();
    fw_log!("Ref deployment exiting\n");
}
