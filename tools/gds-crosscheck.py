#!/usr/bin/env python3
"""Cross-compatibility check: the Rust reference deployment against the real
F Prime ground data system (fprime-gds).

Starts `fprime-gds` (no GUI, TCP server) with the Rust Ref's generated
dictionary, starts the Rust Ref pointed at it, and drives the stock GDS
tools against it. Each check passes or fails on its own:

  commands  commands sent with `fprime-cli command-send` (no-arg, string,
            enum, float arguments) execute; their events and the Ref's
            telemetry decode with the stock decoders
  params    `<PARAM>_PRM_SET`/`_PRM_SAVE` + `prmDb.PRM_SAVE_FILE`; the
            parameter file decodes with `fprime-prm-decode`; after a restart
            on the same data directory the value is back in the component;
            a file written by `fprime-prm-write` loads too
  files     `fprime-cli file-uplink` and `fileDownlink.SendFile`, byte for
            byte
  dp        `signalGen.Dp` and `fileManager.GenerateDp` products, downlinked
            by `dpCat`, decode and validate with `fprime-dp`
  seq       a sequence compiled by `fprime-seqgen` runs on `cmdSeq`
            (`CS_VALIDATE`, `CS_RUN`)
  ccsds     the Ref with `--framing ccsds` against the GDS's
            `space-packet-space-data-link` framing: commands, events,
            telemetry, and a file both ways

Standard library only; needs a Python environment with fprime-gds installed
(`--gds-bin`). All checks run by default; `--checks` selects a subset.

    tools/gds-crosscheck.py --gds-bin ~/.venvs/fprime-gds/bin \\
        --ref-bin target/debug/fprime-ref \\
        --dictionary crates/fprime-ref/dictionary/RefTopologyDictionary.json
"""

import argparse
import json
import os
import random
import re
import shutil
import signal
import socket
import subprocess
import sys
import tempfile
import time

CHECKS = ["commands", "params", "files", "dp", "seq", "ccsds"]

# GDS framing plugin for each Ref `--framing` selection.
GDS_FRAMING = {"fprime": "fprime", "ccsds": "space-packet-space-data-link"}


def wait_for_port(port, deadline_s):
    end = time.monotonic() + deadline_s
    while time.monotonic() < end:
        try:
            with socket.create_connection(("127.0.0.1", port), timeout=0.5):
                return True
        except OSError:
            time.sleep(0.2)
    return False


def stop(proc, timeout_s=15):
    if proc is None or proc.poll() is not None:
        return
    proc.send_signal(signal.SIGINT)
    try:
        proc.wait(timeout=timeout_s)
    except subprocess.TimeoutExpired:
        proc.kill()


def read(path):
    try:
        with open(path, errors="replace") as f:
            return f.read()
    except OSError:
        return ""


class CheckFailed(Exception):
    """Aborts the current check (its failures are already recorded)."""


class Session:
    """One fprime-gds instance (with event and channel listeners) and the Rust
    Ref connected to it. Everything lives under `root`; the ZMQ transport is
    private to the session so concurrent runs cannot cross-talk."""

    def __init__(self, args, root, framing):
        self.args = args
        self.root = root
        self.framing = framing
        self.gds_bin = args.gds_bin
        self.dictionary = os.path.abspath(args.dictionary)
        self.port = args.port if framing == "fprime" else args.port + 1
        self.logs = os.path.join(root, "logs")
        # Short paths: command string arguments are capped at
        # FW_CMD_STRING_MAX_SIZE (40) characters, on both sides.
        self.data = os.path.join(root, "d")
        self.files = os.path.join(root, "gds-files")
        self.events_path = os.path.join(root, "events.txt")
        self.channels_path = os.path.join(root, "channels.txt")
        for d in (self.logs, self.data, self.files):
            os.makedirs(d, exist_ok=True)
        self.zmq = ["--zmq-transport", "ipc://" + os.path.join(root, "zmq-in"),
                    "ipc://" + os.path.join(root, "zmq-out")]
        self.gds = self.ref = self.events = self.channels = None
        self.failures = []
        # GDS log warnings a check provokes on purpose.
        self.expected_log = []

    # -- process management ------------------------------------------------

    def common(self):
        return ["--dictionary", self.dictionary, "-l", self.logs, "--log-directly",
                "--log-prefix", "", "--file-storage-directory", self.files] + self.zmq

    def start(self):
        gds_cmd = [
            os.path.join(self.gds_bin, "fprime-gds"), "-n", "-g", "none",
            "--framing-selection", GDS_FRAMING[self.framing],
            "--ip-address", "127.0.0.1", "--ip-port", str(self.port),
        ] + self.common()
        gds_out = open(os.path.join(self.root, "gds.out"), "w")
        # Own session: the GDS spawns helper processes, and the whole group
        # is torn down at the end.
        self.gds = subprocess.Popen(gds_cmd, stdout=gds_out, stderr=subprocess.STDOUT,
                                    cwd=self.root, start_new_session=True)
        if not wait_for_port(self.port, 30):
            self.fail("fprime-gds did not open the TCP server port")
            return False
        time.sleep(2.0)  # let the GDS finish loading the dictionary
        # Listeners first, so the start-up events (command registration)
        # are captured too.
        self.events = self.listener("events", self.events_path)
        self.channels = self.listener("channels", self.channels_path)
        time.sleep(3.0)
        return self.start_ref()

    def listener(self, kind, out_path):
        """`fprime-cli events|channels` streaming to a file (text mode: the
        JSON mode of fprime-gds 4.3 cannot serialize enum arguments)."""
        env = dict(os.environ, PYTHONUNBUFFERED="1")
        cmd = [os.path.join(self.gds_bin, "fprime-cli"), kind] + self.common()
        out = open(out_path, "a")
        return subprocess.Popen(cmd, stdout=out, stderr=subprocess.STDOUT, env=env)

    def start_ref(self):
        ref_out = open(os.path.join(self.root, "ref.out"), "a")
        self.ref = subprocess.Popen(
            [os.path.abspath(self.args.ref_bin), "-a", "127.0.0.1", "-p", str(self.port),
             "-d", self.data, "-f", self.framing],
            stdin=subprocess.PIPE, stdout=ref_out, stderr=subprocess.STDOUT, cwd=self.root,
        )
        time.sleep(4.0)  # connect, register commands, first rate-group cycles
        if self.ref.poll() is not None:
            self.fail(f"fprime-ref exited early with status {self.ref.returncode}")
            return False
        return True

    def stop_ref(self):
        if self.ref is not None and self.ref.poll() is None:
            try:
                self.ref.stdin.write(b"quit\n")
                self.ref.stdin.flush()
                self.ref.wait(timeout=15)
            except Exception:
                self.ref.kill()
        self.ref = None

    def restart_ref(self):
        self.stop_ref()
        time.sleep(1.0)
        return self.start_ref()

    def close(self):
        stop(self.events)
        stop(self.channels)
        self.stop_ref()
        if self.gds is not None:
            # The comm helper ignores SIGTERM; finish the group with SIGKILL.
            for sig in (signal.SIGTERM, signal.SIGKILL):
                try:
                    os.killpg(self.gds.pid, sig)
                except ProcessLookupError:
                    break
                time.sleep(1.0)
            try:
                self.gds.wait(timeout=5)
            except subprocess.TimeoutExpired:
                pass

    # -- tools -------------------------------------------------------------

    def fail(self, message):
        self.failures.append(message)

    def tool(self, name, argv, timeout_s=60):
        """Run a GDS tool, returning (status, output); status None on timeout."""
        cmd = [os.path.join(self.gds_bin, name)] + argv
        try:
            p = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout_s,
                               cwd=self.root)
            return p.returncode, p.stdout + p.stderr
        except subprocess.TimeoutExpired as e:
            out = (e.stdout or b"").decode(errors="replace") + (e.stderr or b"").decode(errors="replace")
            return None, out

    def cli(self, argv, timeout_s=30):
        return self.tool("fprime-cli", argv + self.common(), timeout_s)

    def command(self, name, *arguments):
        """`fprime-cli command-send`; records a failure if the GDS rejects it."""
        argv = ["command-send", name]
        if arguments:
            argv += ["--arguments"] + [str(a) for a in arguments]
        status, out = self.cli(argv)
        if status is None or status != 0 or "rror" in out:
            self.fail(f"command-send {name} {' '.join(map(str, arguments))}: {out.strip()}")
            return False
        time.sleep(0.5)
        return True

    # -- the downlinked event stream ----------------------------------------

    def mark(self):
        """A position in the event stream, for `wait_event`/`events_since`."""
        return len(read(self.events_path))

    def events_since(self, mark):
        return read(self.events_path)[mark:]

    def wait_event(self, mark, pattern, timeout_s=20.0, what=None):
        """Wait for an event line matching `pattern` (a regex) after `mark`;
        returns the match, or records a failure and returns None."""
        rx = re.compile(pattern)
        end = time.monotonic() + timeout_s
        while True:
            m = rx.search(self.events_since(mark))
            if m:
                return m
            if time.monotonic() > end:
                self.fail(f"[{self.framing}] no event matching {what or pattern!r}")
                return None
            time.sleep(0.25)

    def expect_event(self, mark, pattern, timeout_s=20.0, what=None):
        m = self.wait_event(mark, pattern, timeout_s, what)
        if m is None:
            raise CheckFailed
        return m

    def no_errors_since(self, mark, what):
        text = self.events_since(mark)
        for name in ("OpCodeError", "WARNING_HI", "FATAL"):
            for line in text.splitlines():
                if name in line:
                    self.fail(f"[{self.framing}] {what}: {line.strip()}")

    def log_problems(self):
        """Decoder and file-transfer trouble shows up in the GDS logs."""
        problems = []
        for root, _, files in os.walk(self.logs):
            for f in files:
                if not f.endswith(".log"):
                    continue
                for line in read(os.path.join(root, f)).splitlines():
                    if any(e in line for e in self.expected_log):
                        continue
                    if ("ERROR" in line or "Traceback" in line or "Exception" in line
                            or ("WARNING" in line and ("packet" in line or "sequence" in line))):
                        problems.append(f"{f}: {line.strip()}")
        for name, path in (("events", self.events_path), ("channels", self.channels_path)):
            if "Traceback" in read(path):
                problems.append(f"the fprime-cli {name} listener crashed")
        return problems


# ---------------------------------------------------------------------------
# Checks
# ---------------------------------------------------------------------------


def check_commands(s):
    """Commands with every argument kind; events and telemetry decode."""
    mark = s.mark()
    made = os.path.join(s.data, "made")
    commands = [
        ["Ref.cmdDisp.CMD_NO_OP"],
        ["Ref.cmdDisp.CMD_NO_OP_STRING", "hello from the GDS"],
        ["Ref.signalGen.Settings", "5", "3.5", "0.25", "Triangle"],
        ["Ref.signalGen.Toggle"],
        ["Ref.signalGen.Skip"],
        ["Ref.fileManager.CreateDirectory", made],
        ["Ref.health.HLTH_ENABLE", "DISABLED"],
        # Both BufferAccumulator enum arguments; the zero-count drain
        # answers at once with BA_PartialDrainDone(0), and entering DRAIN
        # writes BA_NumQueuedBuffers.
        ["Ref.dpBufferAccumulator.BA_SetMode", "ACCUMULATE"],
        ["Ref.dpBufferAccumulator.BA_DrainBuffers", "0", "NOBLOCK"],
        ["Ref.dpBufferAccumulator.BA_SetMode", "DRAIN"],
    ]
    for c in commands:
        s.command(*c)
    s.wait_event(mark, r"BA_PartialDrainDone")
    time.sleep(4.0)  # let telemetry come down
    events_out = s.events_since(mark)
    # Everything since the session started: the telemetry store only sends
    # channels that changed, and some settle before this check runs.
    channels_out = read(s.channels_path)
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
        "Ref.dpBufferAccumulator.BA_PartialDrainDone",
    ]
    for name in expected_events:
        if name not in events_out:
            s.fail(f"[{s.framing}] event {name} not decoded by the GDS")
    if "hello from the GDS" not in events_out:
        s.fail(f"[{s.framing}] NoOpStringReceived did not carry the string argument")
    s.no_errors_since(mark, "commands")
    expected_channels = [
        "Ref.cmdDisp.CommandsDispatched",
        "Ref.signalGen.SignalValue",
        "Ref.signalGen.SignalType",
        "Ref.rateGroup1Comp.RgMaxTime",
        "Ref.dpBufferAccumulator.BA_NumQueuedBuffers",
    ]
    for name in expected_channels:
        if name not in channels_out:
            s.fail(f"[{s.framing}] channel {name} not decoded by the GDS")
    if "Triangle" not in channels_out:
        s.fail(f"[{s.framing}] signalGen.SignalType did not decode to Triangle")
    if not os.path.isdir(made):
        s.fail(f"[{s.framing}] CreateDirectory did not create the directory")
    # Leave the generator stopped, as the other checks expect.
    s.command("Ref.signalGen.Toggle")
    n_events = sum(1 for line in events_out.splitlines() if "Ref." in line)
    n_channels = sum(1 for line in channels_out.splitlines() if "Ref." in line)
    return f"{n_events} events and {n_channels} channel samples decoded"


def decode_prm_file(s, dat):
    """`fprime-prm-decode` the Ref's parameter file; returns the JSON."""
    out_json = os.path.join(s.root, "prm.json")
    status, out = s.tool("fprime-prm-decode", ["-d", s.dictionary, "-f", "json", "-o",
                                               out_json, dat])
    if status != 0:
        s.fail(f"fprime-prm-decode {dat}: {out.strip()}")
        raise CheckFailed
    with open(out_json) as f:
        return json.load(f)


def amplitude_in(s, decoded):
    try:
        return decoded["Ref.signalGen"]["Amplitude"]
    except (KeyError, TypeError):
        s.fail(f"fprime-prm-decode output has no Ref.signalGen.Amplitude: {decoded}")
        raise CheckFailed


def save_component_parameter(s):
    """`AMPLITUDE_PRM_SAVE` (the component's own copy into prmDb) and
    `PRM_SAVE_FILE`: afterwards the file holds what the component holds."""
    mark = s.mark()
    s.command("Ref.signalGen.AMPLITUDE_PRM_SAVE")
    s.expect_event(mark, r"Ref\.prmDb\.PrmId(Added|Updated)")
    mark = s.mark()
    s.command("Ref.prmDb.PRM_SAVE_FILE")
    s.expect_event(mark, r"Ref\.prmDb\.PrmFileSaveComplete.*Wrote 1 records")


def check_params(s):
    """Parameter round trip through prmDb and the GDS parameter tools."""
    dat = os.path.join(s.data, "PrmDb.dat")
    value = 7.25
    mark = s.mark()
    s.command("Ref.signalGen.AMPLITUDE_PRM_SET", value)
    s.expect_event(mark, r"Ref\.signalGen\.AmplitudeUpdated.*updated to 7\.25")
    save_component_parameter(s)
    if amplitude_in(s, decode_prm_file(s, dat)) != value:
        s.fail(f"fprime-prm-decode does not show Amplitude = {value}")
        raise CheckFailed

    # Restart on the same data directory: the Ref loads the file before its
    # components read their parameters. Saving the component's copy back
    # proves it holds the reloaded value (a default would overwrite it).
    mark = s.mark()
    if not s.restart_ref():
        raise CheckFailed
    s.expect_event(mark, r"Ref\.prmDb\.PrmFileLoadComplete.*Records: 1", 30)
    os.remove(dat)
    save_component_parameter(s)
    if amplitude_in(s, decode_prm_file(s, dat)) != value:
        s.fail("Amplitude did not survive the restart")
        raise CheckFailed

    # A parameter file written by the GDS (`fprime-prm-write dat`).
    written = 2.5
    src = os.path.join(s.root, "prm-in.json")
    with open(src, "w") as f:
        json.dump({"Ref.signalGen": {"Amplitude": written}}, f)
    status, out = s.tool("fprime-prm-write", ["dat", src, "-d", s.dictionary, "-o", dat])
    if status != 0:
        s.fail(f"fprime-prm-write: {out.strip()}")
        raise CheckFailed
    mark = s.mark()
    if not s.restart_ref():
        raise CheckFailed
    s.expect_event(mark, r"Ref\.prmDb\.PrmFileLoadComplete.*Records: 1", 30)
    save_component_parameter(s)
    if amplitude_in(s, decode_prm_file(s, dat)) != written:
        s.fail("a parameter file written by fprime-prm-write did not load")
        raise CheckFailed
    s.no_errors_since(mark, "params")
    return f"Amplitude {value} survived a restart; a fprime-prm-write file loaded {written}"


def random_file(path, size, seed):
    rng = random.Random(seed)
    with open(path, "wb") as f:
        f.write(bytes(rng.getrandbits(8) for _ in range(size)))


def same_bytes(a, b):
    try:
        with open(a, "rb") as fa, open(b, "rb") as fb:
            return fa.read() == fb.read()
    except OSError:
        return False


def downlink(s, source, dest_name, timeout_s=60):
    """`fileDownlink.SendFile`; returns the path the GDS wrote."""
    mark = s.mark()
    s.command("Ref.fileDownlink.SendFile", source, dest_name)
    s.expect_event(mark, r"Ref\.fileDownlink\.FileSent", timeout_s)
    landed = os.path.join(s.files, "fprime-downlink", dest_name)
    end = time.monotonic() + 10
    while not os.path.exists(landed) and time.monotonic() < end:
        time.sleep(0.25)
    return landed


def uplink(s, original, dest):
    """`fprime-cli file-uplink` (which waits for `FileReceived`) and a byte
    comparison of what landed on board."""
    # The GDS uplinker deletes the file it was handed once it is sent.
    staged = original + ".up"
    shutil.copyfile(original, staged)
    mark = s.mark()
    status, out = s.cli(["file-uplink", staged, dest, "--timeout", "90"], timeout_s=120)
    if status != 0:
        s.fail(f"[{s.framing}] fprime-cli file-uplink {os.path.basename(original)}: {out.strip()}")
        raise CheckFailed
    s.expect_event(mark, r"Ref\.fileUplink\.FileReceived")
    if not same_bytes(original, dest):
        s.fail(f"[{s.framing}] the uplinked {os.path.basename(original)} differs from the original")
        raise CheckFailed


def check_files(s):
    """File uplink with `fprime-cli file-uplink`, downlink with
    `fileDownlink.SendFile` and `SendPartial`, byte for byte both ways."""
    tag = s.framing[0]  # keeps on-board paths under the 40-character cap
    mark = s.mark()
    size = 5000 + 37  # several uplink chunks and downlink packets, odd tail
    original = os.path.join(s.root, f"orig-{tag}.bin")
    random_file(original, size, seed=size)
    dest = os.path.join(s.data, "uplink", f"u{tag}.bin")
    uplink(s, original, dest)
    # An empty file is a START and an END packet with nothing between.
    empty = os.path.join(s.root, f"empty-{tag}.bin")
    open(empty, "wb").close()
    uplink(s, empty, os.path.join(s.data, "uplink", f"e{tag}.bin"))

    landed = downlink(s, dest, f"down-{tag}.bin")
    if not same_bytes(original, landed):
        s.fail(f"[{s.framing}] the downlinked file {landed} differs from the original")
        raise CheckFailed
    # SendPartial: an interior byte range. Its DATA packets carry absolute
    # file offsets (upstream FileDownlink does the same), so the GDS writes
    # the range in place after a hole, and says so in its log.
    offset, length = 1000, 2345
    s.expected_log.append(f"Missing data packet at offset: 0. Gap of {offset} bytes")
    pmark = s.mark()
    s.command("Ref.fileDownlink.SendPartial", dest, f"part-{tag}.bin", offset, length)
    s.expect_event(pmark, r"Ref\.fileDownlink\.FileSent", 60)
    landed = os.path.join(s.files, "fprime-downlink", f"part-{tag}.bin")
    with open(original, "rb") as f:
        want = f.read()[offset:offset + length]
    try:
        with open(landed, "rb") as f:
            got = f.read()
    except OSError:
        got = b""
    if len(got) != offset + length or got[offset:] != want:
        s.fail(f"[{s.framing}] SendPartial({offset}, {length}) downlinked "
               f"{len(got)} bytes, not the range at its offset")
    s.no_errors_since(mark, "files")
    return f"{size} bytes and an empty file up, the file and a {length}-byte range back down"


def dp_files(s):
    d = os.path.join(s.data, "DpCat")
    return sorted(f for f in os.listdir(d) if f.endswith(".fdp"))


def fprime_dp(s, fdp):
    """`fprime-dp validate` and `decode` one product; returns the JSON."""
    status, out = s.tool("fprime-dp", ["validate", "-b", fdp, "-d", s.dictionary])
    if status != 0 or "rror" in out or "nvalid" in out:
        s.fail(f"fprime-dp validate {os.path.basename(fdp)}: {out.strip()}")
    out_json = fdp + ".json"
    status, out = s.tool("fprime-dp", ["decode", "-b", fdp, "-d", s.dictionary, "-o", out_json])
    if status != 0 or not os.path.exists(out_json):
        s.fail(f"fprime-dp decode {os.path.basename(fdp)}: {out.strip()}")
        raise CheckFailed
    with open(out_json) as f:
        return json.load(f)


def check_dp(s):
    """Data products from both producers, downlinked by dpCat and decoded
    with the GDS's data-product tooling."""
    before = set(dp_files(s))
    records = 6
    mark = s.mark()
    s.command("Ref.signalGen.Dp", records)
    s.expect_event(mark, r"Ref\.signalGen\.DpSent.*with %d records" % records)
    s.expect_event(mark, r"Ref\.dpWriter\.FileWritten")

    # fileManager.GenerateDp: a file chunked into FileChunk records.
    source = os.path.join(s.data, "dp.bin")
    random_file(source, 700, seed=700)  # 256 + 256 + 188: three containers
    mark = s.mark()
    s.command("Ref.fileManager.GenerateDp", source, 256, 0, 0, 5, "IMMEDIATE")
    s.expect_event(mark, r"Ref\.fileManager\.GenerateDpComplete.* in 3 chunks")
    end = time.monotonic() + 10
    while len(set(dp_files(s)) - before) < 2 and time.monotonic() < end:
        time.sleep(0.25)
    new = sorted(set(dp_files(s)) - before)
    if len(new) != 4:
        s.fail(f"expected four new .fdp files in DpCat, found {new}")
        raise CheckFailed

    mark = s.mark()
    s.command("Ref.dpCat.BUILD_CATALOG")
    s.expect_event(mark, r"Ref\.dpCat\.CatalogBuildComplete")
    mark = s.mark()
    s.command("Ref.dpCat.START_XMIT_CATALOG", "NO_WAIT", "False")
    s.expect_event(mark, r"Ref\.dpCat\.CatalogXmitCompleted", 120)

    decoded = {}
    for name in new:
        on_board = os.path.join(s.data, "DpCat", name)
        landed = os.path.join(s.files, "fprime-downlink", name)
        if not os.path.exists(landed):
            # dpCat downlinks under the on-board path; the GDS flattens it.
            flat = on_board.replace(os.sep, "_")
            landed = os.path.join(s.files, "fprime-downlink", flat)
        if not same_bytes(on_board, landed):
            s.fail(f"downlinked data product {name} missing or different ({landed})")
            continue
        decoded[name] = fprime_dp(s, landed)

    samples = 0
    chunks = {}
    for name, d in decoded.items():
        recs = d.get("Records", [])
        names = [r["Record"]["record_name"] for r in recs]
        if names and all(n == "Ref.signalGen.DataRecord" for n in names):
            samples += len(names)
            if len(names) != records:
                s.fail(f"{name}: {len(names)} DataRecord entries decoded, expected {records}")
        elif names == ["Ref.fileManager.FileChunkHeaderRecord",
                       "Ref.fileManager.FileChunkDataRecord"]:
            header, data = recs[0]["Data"], recs[1]["Data"]
            if header["fileName"]["value"] != source:
                s.fail(f"{name}: chunk of {header['fileName']['value']}, expected {source}")
            chunks[header["offset"]["value"]] = bytes(data["values"])
            if header["dataSize"]["value"] != len(data["values"]):
                s.fail(f"{name}: chunk header size {header['dataSize']['value']} != "
                       f"{len(data['values'])} data bytes")
        else:
            s.fail(f"{name}: unexpected records {names}")
    with open(source, "rb") as f:
        original = f.read()
    rebuilt = b"".join(chunks[k] for k in sorted(chunks))
    if rebuilt != original:
        s.fail(f"the decoded GenerateDp chunks do not reassemble {source}")
    s.no_errors_since(mark, "dp")
    return (f"4 products downlinked, validated and decoded: {samples} signalGen samples, "
            f"{len(original)} file bytes in {len(chunks)} chunk containers")


def seqgen(s, text, name, timebase=None):
    """Compile `text` with fprime-seqgen into the Ref's data directory."""
    src = os.path.join(s.root, name + ".seq")
    with open(src, "w") as f:
        f.write(text)
    binary = os.path.join(s.data, name + ".bin")
    argv = ["--dictionary", s.dictionary, src, binary]
    if timebase is not None:
        argv = ["-t", str(timebase)] + argv
    status, out = s.tool("fprime-seqgen", argv)
    if status != 0 or not os.path.exists(binary):
        s.fail(f"fprime-seqgen {name}: {out.strip()}")
        raise CheckFailed
    return binary


def check_seq(s):
    """Sequences compiled by fprime-seqgen run on cmdSeq: relative time tags
    with a don't-care time base, then an absolute time tag in the Ref's own
    time base (TB_WORKSTATION_TIME, 2)."""
    binary = seqgen(s, "\n".join([
        "; built by tools/gds-crosscheck.py",
        "R00:00:00 Ref.cmdDisp.CMD_NO_OP",
        'R00:00:01 Ref.cmdDisp.CMD_NO_OP_STRING "from a sequence"',
        "R00:00:00 Ref.signalGen.Settings 4 1.5 0.0 Sine",
        "R00:00:00 Ref.cmdDisp.CMD_NO_OP",
        "",
    ]), "s")
    mark = s.mark()
    s.command("Ref.cmdSeq.CS_VALIDATE", binary)
    s.expect_event(mark, r"Ref\.cmdSeq\.CS_SequenceValid")
    mark = s.mark()
    s.command("Ref.cmdSeq.CS_RUN", binary, "BLOCK")
    s.expect_event(mark, r"Ref\.cmdSeq\.CS_SequenceComplete", 30)
    out = s.events_since(mark)
    for what in ("Ref.cmdSeq.CS_SequenceLoaded", "from a sequence",
                 "Ref.signalGen.SettingsChanged", "Ref.cmdSeq.CS_CommandComplete"):
        if what not in out:
            s.fail(f"sequence run: no {what!r} in the events")
    n_noop = out.count("Ref.cmdDisp.NoOpReceived")
    if n_noop != 2:
        s.fail(f"sequence run: {n_noop} NoOpReceived events, expected 2")

    # Absolute time tag a few seconds ahead (seqgen reads it as UTC, the
    # workstation time base counts Unix seconds).
    at = int(time.time()) + 6
    binary = seqgen(s, "\n".join([
        'R00:00:00 Ref.cmdDisp.CMD_NO_OP_STRING "before the absolute"',
        f'A{time.strftime("%Y-%jT%H:%M:%S", time.gmtime(at))} '
        'Ref.cmdDisp.CMD_NO_OP_STRING "at the absolute"',
        "",
    ]), "a", timebase=2)
    mark = s.mark()
    s.command("Ref.cmdSeq.CS_RUN", binary, "NO_BLOCK")
    m = s.expect_event(mark, r"\(2\(0\)-(\d+):\d+\)[^\n]*string=at the absolute", 30,
                       what="the absolute-time command")
    if "before the absolute" not in s.events_since(mark):
        s.fail("absolute sequence: the relative command did not run")
    if int(m.group(1)) < at:
        s.fail(f"absolute sequence: the command ran at {m.group(1)}, before its time tag {at}")
    s.expect_event(mark, r"Ref\.cmdSeq\.CS_SequenceComplete", 10)
    s.no_errors_since(mark, "seq")
    return "relative and absolute (time base 2) fprime-seqgen sequences ran on cmdSeq"


def run_checks(s, checks, results):
    for name, fn in checks:
        before = len(s.failures)
        t0 = time.monotonic()
        detail = ""
        try:
            detail = fn(s) or ""
        except CheckFailed:
            pass
        except Exception as e:  # a bug in this script, or a tool crashed
            s.fail(f"{name}: {type(e).__name__}: {e}")
        failed = s.failures[before:]
        results.append((name, not failed, detail, failed, time.monotonic() - t0))
        status = "PASS" if not failed else "FAIL"
        print(f"[{status}] {name} ({time.monotonic() - t0:.0f} s){': ' + detail if detail and not failed else ''}",
              flush=True)
        for f in failed:
            print(f"       - {f}", flush=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--gds-bin", required=True, help="directory with fprime-gds and fprime-cli")
    ap.add_argument("--ref-bin", required=True, help="the fprime-ref binary")
    ap.add_argument("--dictionary", required=True, help="RefTopologyDictionary.json")
    ap.add_argument("--port", type=int, default=50000,
                    help="GDS TCP server port (the ccsds check uses PORT+1)")
    ap.add_argument("--checks", default=",".join(CHECKS),
                    help="comma-separated subset of: " + ", ".join(CHECKS) + " (default: all)")
    ap.add_argument("--keep", action="store_true", help="keep the scratch directory")
    args = ap.parse_args()
    selected = [c.strip() for c in args.checks.split(",") if c.strip()]
    unknown = [c for c in selected if c not in CHECKS]
    if unknown or not selected:
        ap.error(f"unknown check(s) {unknown}; choose from {CHECKS}")

    scratch = tempfile.mkdtemp(prefix="gds-", dir="/tmp")
    functions = {"commands": check_commands, "params": check_params, "files": check_files,
                 "dp": check_dp, "seq": check_seq}
    results = []
    all_failures = []
    plans = []
    fprime_checks = [c for c in CHECKS if c in selected and c != "ccsds"]
    if fprime_checks:
        plans.append(("fprime", [(c, functions[c]) for c in fprime_checks]))
    if "ccsds" in selected:
        # Same expectations, other framing: commands/events/telemetry and a
        # file each way through TC/TM frames.
        plans.append(("ccsds", [("ccsds", lambda s: "; ".join([check_commands(s), check_files(s)]))]))
    for framing, checks in plans:
        root = os.path.join(scratch, framing[0])
        os.makedirs(root)
        s = Session(args, root, framing)
        try:
            if s.start():
                run_checks(s, checks, results)
            else:
                for name, _ in checks:
                    results.append((name, False, "", list(s.failures), 0))
                    print(f"[FAIL] {name}: session did not start: {s.failures}", flush=True)
        finally:
            s.close()
        problems = s.log_problems()
        for p in problems:
            print(f"[FAIL] GDS log ({framing}): {p}", flush=True)
        all_failures += s.failures + problems

    passed = [r[0] for r in results if r[1]]
    failed = [r[0] for r in results if not r[1]]
    if all_failures:
        print(f"FAILED: {', '.join(failed) or 'GDS logs'} (passed: {', '.join(passed) or 'none'})")
        print(f"scratch directory: {scratch}")
        return 1
    print(f"OK: the real fprime-gds drove the Rust Ref through: {', '.join(passed)}")
    if not args.keep:
        shutil.rmtree(scratch, ignore_errors=True)
    else:
        print(f"scratch directory: {scratch}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
