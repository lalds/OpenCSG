#!/usr/bin/env python3
"""Inventory surviving Mach-O symbols and direct calls without executing code.

Requires LLVM llvm-nm and llvm-objdump. Call edges are static observations:
indirect calls and inlined functions are not resolved. Cross-block unconditional
jumps are listed separately as possible tail calls, not proven call edges.
"""

import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess


ROOT = Path(__file__).resolve().parents[1]
BINARY = ROOT / "Plugins/Editor/External/RealtimeCSG[1_559].bundle/Contents/MacOS/RealtimeCSG[1_559]"


def inspect():
    symbols_text = subprocess.check_output(
        ["llvm-nm", "--demangle", "--defined-only", str(BINARY)], text=True
    )
    symbols = []
    for line in symbols_text.splitlines():
        match = re.fullmatch(r"([0-9a-fA-F]+) ([A-Za-z]) (.+)", line)
        if not match:
            raise ValueError("Unparsed llvm-nm line: " + line)
        symbols.append({"address": hex(int(match[1], 16)), "type": match[2], "name": match[3]})
    assembly = subprocess.check_output(
        ["llvm-objdump", "-d", "--demangle", "--no-show-raw-insn", str(BINARY)], text=True
    )
    blocks = re.split(r"(?m)^([0-9a-fA-F]+) <(.+)>:\n", assembly)
    functions = []
    bodies = {}
    for i in range(1, len(blocks), 3):
        address, name, body = blocks[i:i + 3]
        calls = []
        indirect = []
        tail_jumps = []
        block_start = int(address, 16)
        block_end = int(blocks[i + 3], 16) if i + 3 < len(blocks) else None
        for line in body.splitlines():
            jump = re.search(r"^\s*([0-9a-fA-F]+):\s+jmpq?\s+0x([0-9a-fA-F]+) <(.+)>", line)
            if jump:
                target_address = int(jump[2], 16)
                if target_address < block_start or (block_end is not None and target_address >= block_end):
                    tail_jumps.append({"site": hex(int(jump[1], 16)),
                                       "target_address": hex(target_address), "target_label": jump[3]})
            match = re.search(r"^\s*([0-9a-fA-F]+):\s+callq?\s+(.+)", line)
            if not match:
                continue
            target = re.fullmatch(r"0x([0-9a-fA-F]+) <(.+)>", match[2].strip())
            if target:
                calls.append({"site": hex(int(match[1], 16)),
                              "target_address": hex(int(target[1], 16)),
                              "target_label": target[2]})
            else:
                indirect.append({"site": hex(int(match[1], 16)), "operand": match[2].strip()})
        functions.append({"address": hex(int(address, 16)), "name": name,
                          "direct_calls": calls, "unresolved_calls": indirect,
                          "tail_jump_candidates": tail_jumps})
        bodies[name] = address + " <" + name + ">:\n" + body
    if not functions:
        raise ValueError("No function blocks parsed from llvm-objdump")
    strings = re.findall(rb"[\x20-\x7e]{8,}", BINARY.read_bytes())
    paths = sorted({s.decode("ascii") for s in strings
                    if s.startswith(b"/") and s.endswith((b".cpp", b".h", b".hpp"))})
    report = {
        "binary": str(BINARY.relative_to(ROOT)),
        "sha256": hashlib.sha256(BINARY.read_bytes()).hexdigest(),
        "defined_symbol_count": len(symbols),
        "code_symbol_count": sum(s["type"] in ("t", "T") for s in symbols),
        "code_symbols_mentioning_realtimecsg": sum(s["type"] in ("t", "T") and "RealtimeCSG::" in s["name"] for s in symbols),
        "limitations": "Symbol blocks, not recovered source. Direct calls and cross-block unconditional jump candidates only; imported stub labels may be misleading. No proof of behavioral equivalence to Windows.",
        "source_paths": paths, "symbols": symbols, "functions": functions,
    }
    return report, bodies


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, help="Write JSON inventory and direct calls")
    parser.add_argument("--disassemble", metavar="NAME", help="Print one exact demangled symbol block")
    args = parser.parse_args()
    report, bodies = inspect()
    if args.disassemble and args.disassemble not in bodies:
        parser.error("Symbol block not found: " + args.disassemble)
    if args.output:
        args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    if args.disassemble:
        print(bodies[args.disassemble], end="")
    elif not args.output:
        print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
