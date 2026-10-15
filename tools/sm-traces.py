#!/usr/bin/env python3
"""Golden call traces of an FPP state machine, from the C++ that the
reference compiler generates for it.

Reads a generated `<M>StateMachineAc.cpp` and its `<M>_StateEnumAc.hpp`
and prints, for `initBase` and for every (leaf state, signal) pair, every
path through the guards:

    init [outcomes] -> trace ; FINAL
    LEAF signal [outcomes] -> trace ; FINAL

The trace lists the actions (`a:<name>`) and guards (`g:<name>`) called;
`outcomes` are the guard results in call order (`1`/`0`); FINAL is the
state after the step (`-`: unchanged). It follows the `enter_*` functions
and both branches of every `if (this->guard_*(..))`; the generated code is
regular enough that nothing else is needed. Standard library only.

`crates/fprime-comp/tests/sm_traces.rs` replays these files against the
`fpp_state_machine!` versions of the same machines. To regenerate one
(fpp-to-cpp from the `fprime-fpp` version upstream pins, 3.3.0 at
4dee010):

    fpp-to-cpp -i <framework .fpp imports> Model.fpp     # in a scratch dir
    tools/sm-traces.py MStateMachineAc.cpp M_StateEnumAc.hpp \
        > crates/fprime-comp/tests/sm_traces/M.txt
"""

import re
import sys

src = open(sys.argv[1]).read()
enum_hdr = open(sys.argv[2]).read()

leaves = [m.group(1) for m in re.finditer(r"^\s*(\w+) = (\d+),", enum_hdr, re.M)]
leaves = [l for l in leaves if l not in ("__FPRIME_UNINITIALIZED", "NUM_CONSTANTS", "SERIALIZED_SIZE")]

# Function bodies: name -> list of lines
funcs = {}
for m in re.finditer(r"::\s*\n\s*(\w+)\(([^)]*)\)\s*\n( *)\{\n(.*?)\n\3\}\n", src, re.S):
    funcs[m.group(1)] = m.group(4).split("\n")


def parse_block(lines, i):
    """Parse statements from lines[i] until a closing brace; return (stmts, i)."""
    stmts = []
    while i < len(lines):
        line = lines[i].strip()
        i += 1
        if not line or line.startswith("//"):
            continue
        if line == "}":
            return stmts, i
        m = re.match(r"if \(this->guard_(\w+)\(", line)
        if m:
            then, i = parse_block(lines, i)
            els = []
            if i < len(lines) and lines[i].strip() == "else {":
                els, i = parse_block(lines, i + 1)
            stmts.append(("if", m.group(1), then, els))
            continue
        m = re.match(r"this->action_(\w+)\(", line)
        if m:
            stmts.append(("action", m.group(1)))
            continue
        m = re.match(r"this->enter_(\w+)\(", line)
        if m:
            stmts.append(("enter", m.group(1)))
            continue
        m = re.match(r"this->m_state = State::(\w+);", line)
        if m:
            stmts.append(("state", m.group(1)))
            continue
        if line in ("break;",):
            stmts.append(("break",))
            continue
        if line.startswith("this->m_id") or line.startswith("FW_ASSERT") or line.startswith("default:"):
            continue
        raise SystemExit("unparsed: " + line)
    return stmts, i


def body(name):
    stmts, _ = parse_block(funcs[name] + ["}"], 0)
    return stmts


def signal_cases(name):
    """case State::X: statements until break."""
    cases = {}
    cur = None
    lines = funcs[name]
    i = 0
    while i < len(lines):
        line = lines[i].strip()
        m = re.match(r"case State::(\w+):", line)
        if m:
            cur = m.group(1)
            # collect until 'break;' at this indentation level
            depth_lines = []
            i += 1
            level = 0
            while True:
                l = lines[i].strip()
                if l.endswith("{"):
                    level += 1
                if l == "}":
                    level -= 1
                if l == "break;" and level == 0:
                    break
                depth_lines.append(lines[i])
                i += 1
            stmts, _ = parse_block(depth_lines + ["}"], 0)
            cases[cur] = stmts
        i += 1
    return cases


def run(stmts, outcomes, trace, state):
    """Yield (outcomes, trace, state) for every guard path."""
    if not stmts:
        yield outcomes, trace, state
        return
    s, rest = stmts[0], stmts[1:]
    if s[0] == "action":
        yield from run(rest, outcomes, trace + ["a:" + s[1]], state)
    elif s[0] == "state":
        yield from run(rest, outcomes, trace, s[1])
    elif s[0] == "enter":
        for o, t, st in run(body("enter_" + s[1]), outcomes, trace, state):
            yield from run(rest, o, t, st)
    elif s[0] == "if":
        for val, branch in (("1", s[2]), ("0", s[3])):
            yield from run(branch + rest, outcomes + val, trace + ["g:" + s[1]], state)
    elif s[0] == "break":
        yield outcomes, trace, state


def fmt(head, o, t, st):
    return f"{head} [{o}] -> {' '.join(t)} ; {st}"


for o, t, st in run(body("initBase"), "", [], "-"):
    print(fmt("init", o, t, st))
signals = sorted(n[len("sendSignal_"):] for n in funcs if n.startswith("sendSignal_"))
for leaf in leaves:
    for sig in signals:
        cases = signal_cases("sendSignal_" + sig)
        for o, t, st in run(cases.get(leaf, []), "", [], "-"):
            print(fmt(f"{leaf} {sig}", o, t, st))
