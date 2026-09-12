#!/usr/bin/env python3
"""Cross-compatibility check: the Rust reference deployment against the real
F Prime ground data system (fprime-gds).

Starts `fprime-gds` (no GUI, F Prime framing, TCP server) with the Rust
Ref's generated dictionary, starts the Rust Ref pointed at it, sends
commands through `fprime-cli`, and checks that the expected events and
telemetry channels come back decoded. Standard library only; needs a
Python environment with fprime-gds installed (`--gds-bin`).

    tools/gds-crosscheck.py --gds-bin ~/.venvs/fprime-gds/bin \
        --ref-bin target/debug/fprime-ref \
        --dictionary crates/fprime-ref/dictionary/RefTopologyDictionary.json
"""

import argparse
import json
import os
import shutil
import signal
import socket
import subprocess
import sys
import tempfile
import time


def wait_for_port(port, deadline_s):
    end = time.monotonic() + deadline_s
    while time.monotonic() < end:
        try:
            with socket.create_connection(("127.0.0.1", port), timeout=0.5):
                return True
        except OSError:
            time.sleep(0.2)
    return False


def run_cli(gds_bin, args, dictionary, logs, timeout_s):
    """Run fprime-cli, returning (stdout, timed_out)."""
    cmd = [os.path.join(gds_bin, "fprime-cli")] + args + [
        "--dictionary", dictionary, "-l", logs, "--log-directly", "--log-prefix", "",
    ]
    try:
        p = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout_s)
        return p.stdout + p.stderr, False
    except subprocess.TimeoutExpired as e:
        out = (e.stdout or b"").decode(errors="replace") + (e.stderr or b"").decode(errors="replace")
        return out, True


def start_listener(gds_bin, kind, dictionary, logs, out_path):
    """Start `fprime-cli events|channels` streaming to a file (text mode:
    the JSON mode of fprime-gds 4.3 cannot serialize enum arguments)."""
    env = dict(os.environ, PYTHONUNBUFFERED="1")
    cmd = [os.path.join(gds_bin, "fprime-cli"), kind,
           "--dictionary", dictionary, "-l", logs, "--log-directly", "--log-prefix", ""]
    out = open(out_path, "w")
    return subprocess.Popen(cmd, stdout=out, stderr=subprocess.STDOUT, env=env)


def stop(proc, timeout_s=15):
    if proc is None or proc.poll() is not None:
        return
    proc.send_signal(signal.SIGINT)
    try:
        proc.wait(timeout=timeout_s)
    except subprocess.TimeoutExpired:
        proc.kill()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--gds-bin", required=True, help="directory with fprime-gds and fprime-cli")
    ap.add_argument("--ref-bin", required=True, help="the fprime-ref binary")
    ap.add_argument("--dictionary", required=True, help="RefTopologyDictionary.json")
    ap.add_argument("--port", type=int, default=50000, help="GDS TCP server port")
    ap.add_argument("--keep", action="store_true", help="keep the scratch directory")
    args = ap.parse_args()

    # Short paths: command string arguments are capped at
    # FW_CMD_STRING_MAX_SIZE (40) characters, on both sides.
    scratch = tempfile.mkdtemp(prefix="gds-", dir="/tmp")
    logs = os.path.join(scratch, "logs")
    data = os.path.join(scratch, "d")
    os.makedirs(logs)
    os.makedirs(data)
    dictionary = os.path.abspath(args.dictionary)
    failures = []
    gds = ref = events = channels = None
    try:
        gds_cmd = [
            os.path.join(args.gds_bin, "fprime-gds"), "-n", "-g", "none",
            "--dictionary", dictionary, "--framing-selection", "fprime",
            "--ip-address", "127.0.0.1", "--ip-port", str(args.port),
            "-l", logs, "--log-directly", "--log-prefix", "",
        ]
        gds_out = open(os.path.join(scratch, "gds.out"), "w")
        # Own session: the GDS spawns helper processes, and the whole group
        # is torn down at the end.
        gds = subprocess.Popen(gds_cmd, stdout=gds_out, stderr=subprocess.STDOUT, cwd=scratch,
                               start_new_session=True)
        if not wait_for_port(args.port, 30):
            failures.append("fprime-gds did not open the TCP server port")
            raise SystemExit
        time.sleep(2.0)  # let the GDS finish loading the dictionary

        # Listeners first, so the start-up events (command registration)
        # are captured too.
        events_path = os.path.join(scratch, "events.txt")
        channels_path = os.path.join(scratch, "channels.txt")
        events = start_listener(args.gds_bin, "events", dictionary, logs, events_path)
        channels = start_listener(args.gds_bin, "channels", dictionary, logs, channels_path)
        time.sleep(3.0)

        ref_out = open(os.path.join(scratch, "ref.out"), "w")
        ref = subprocess.Popen(
            [os.path.abspath(args.ref_bin), "-a", "127.0.0.1", "-p", str(args.port), "-d", data],
            stdin=subprocess.PIPE, stdout=ref_out, stderr=subprocess.STDOUT, cwd=scratch,
        )
        time.sleep(4.0)  # connect, register commands, first rate-group cycles
        if ref.poll() is not None:
            failures.append(f"fprime-ref exited early with status {ref.returncode}")
            raise SystemExit

        made = os.path.join(data, "made")
        commands = [
            ["Ref.cmdDisp.CMD_NO_OP"],
            ["Ref.cmdDisp.CMD_NO_OP_STRING", "--arguments", "hello from the GDS"],
            ["Ref.signalGen.Settings", "--arguments", "5", "3.5", "0.25", "Triangle"],
            ["Ref.signalGen.Toggle"],
            ["Ref.signalGen.Skip"],
            ["Ref.fileManager.CreateDirectory", "--arguments", made],
            ["Ref.health.HLTH_ENABLE", "--arguments", "DISABLED"],
        ]
        for c in commands:
            out, timed_out = run_cli(args.gds_bin, ["command-send"] + c, dictionary, logs, 30)
            if timed_out or "rror" in out:
                failures.append(f"command-send {' '.join(c)}: {out.strip()}")
            time.sleep(0.5)
        time.sleep(4.0)  # let responses and telemetry come down
        stop(events)
        stop(channels)
        events_out = open(events_path, errors="replace").read()
        channels_out = open(channels_path, errors="replace").read()

        expected_events = [
            "Ref.cmdDisp.OpCodeDispatched",
            "Ref.cmdDisp.OpCodeCompleted",
            "Ref.cmdDisp.NoOpReceived",
            "Ref.cmdDisp.NoOpStringReceived",
            "Ref.signalGen.SettingsChanged",
            "Ref.signalGen.Toggled",
            "Ref.signalGen.SampleSkipped",
            "Ref.fileManager.CreateDirectoryStarted",
            "Ref.fileManager.CreateDirectorySucceeded",
            "Ref.health.HLTH_CHECK_ENABLE",
        ]
        for name in expected_events:
            if name not in events_out:
                failures.append(f"event {name} not decoded by the GDS")
        if "hello from the GDS" not in events_out:
            failures.append("NoOpStringReceived did not carry the string argument")
        if "OpCodeError" in events_out:
            failures.append("a command completed with an error")
        expected_channels = [
            "Ref.cmdDisp.CommandsDispatched",
            "Ref.signalGen.SignalValue",
            "Ref.signalGen.SignalType",
            "Ref.rateGroup1Comp.RgMaxTime",
        ]
        for name in expected_channels:
            if name not in channels_out:
                failures.append(f"channel {name} not decoded by the GDS")
        if "Triangle" not in channels_out:
            failures.append("signalGen.SignalType did not decode to Triangle")
        if not os.path.isdir(made):
            failures.append("CreateDirectory did not create the directory")

        # Decoder trouble shows up in the GDS logs.
        for root, _, files in os.walk(logs):
            for f in files:
                if f.endswith(".log"):
                    text = open(os.path.join(root, f), errors="replace").read()
                    for line in text.splitlines():
                        if "ERROR" in line or "Traceback" in line or "Exception" in line:
                            failures.append(f"{f}: {line.strip()}")
        for name, text in (("events", events_out), ("channels", channels_out)):
            if "Traceback" in text:
                failures.append(f"the fprime-cli {name} listener crashed")
        n_events = sum(1 for line in events_out.splitlines() if "Ref." in line)
        n_channels = sum(1 for line in channels_out.splitlines() if "Ref." in line)
        print(f"events decoded: {n_events}; channel samples decoded: {n_channels}")
    except SystemExit:
        pass
    finally:
        stop(events)
        stop(channels)
        if ref is not None and ref.poll() is None:
            try:
                ref.stdin.write(b"quit\n")
                ref.stdin.flush()
                ref.wait(timeout=15)
            except Exception:
                ref.kill()
        if gds is not None:
            # The comm helper ignores SIGTERM; finish the group with SIGKILL.
            for sig in (signal.SIGTERM, signal.SIGKILL):
                try:
                    os.killpg(gds.pid, sig)
                except ProcessLookupError:
                    break
                time.sleep(1.0)
            try:
                gds.wait(timeout=5)
            except subprocess.TimeoutExpired:
                pass

    if failures:
        print("FAILURES:")
        for f in failures:
            print(" -", f)
        print(f"scratch directory: {scratch}")
        return 1
    print("OK: the real fprime-gds commanded the Rust Ref and decoded its events and telemetry")
    if not args.keep:
        shutil.rmtree(scratch, ignore_errors=True)
    else:
        print(f"scratch directory: {scratch}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
