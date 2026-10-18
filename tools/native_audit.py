#!/usr/bin/env python3
"""Inventory the checked-in C# native contract and Windows PE exports.

Requires Python 3 and GNU objdump. Does not load or execute the plugin.
This is a source inventory, not a C# compiler or a native ABI specification.
"""

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess


ROOT = Path(__file__).resolve().parents[1]
DLL = ROOT / "Plugins/Editor/External/x64/RealtimeCSG[1_559].dll"
IMPORT = re.compile(
    r"\[DllImport\((?P<attribute>[^\n]*?)\)\]"
    r"(?P<declaration>[^;{}]*?\bextern\s+(?P<return>\w+)\s+"
    r"(?P<name>\w+)\s*\([^;{}]*?\))\s*;",
    re.DOTALL,
)
# Preserve offsets for line numbers, and preserve string literals containing //.
COMMENTS = re.compile(r'@"(?:""|[^"])*"|"(?:\\.|[^"\\])*"|//[^\n]*|/\*.*?\*/', re.DOTALL)


def source_imports():
    declarations = []
    for path in sorted((ROOT / "Plugins").rglob("*.cs")):
        source = path.read_text(encoding="utf-8-sig")
        source = COMMENTS.sub(
            lambda m: re.sub(r"[^\n]", " ", m[0]) if m[0].startswith("/") else m[0],
            source,
        )
        matches = list(IMPORT.finditer(source))
        # Fail if a new declaration exceeds this deliberately limited parser.
        if len(matches) != len(re.findall(r"\[DllImport\(", source)):
            raise ValueError("Unparsed DllImport in " + str(path.relative_to(ROOT)))
        for match in matches:
            attribute = match["attribute"]
            if "NativePluginName" not in attribute:
                continue
            entry = re.search(r'EntryPoint\s*=\s*"([^"]+)"', attribute)
            declarations.append({
                "path": str(path.relative_to(ROOT)),
                "line": source.count("\n", 0, match.start()) + 1,
                "managed_name": match["name"],
                "entry_point": entry[1] if entry else match["name"],
                "return_type": match["return"],
                "attribute": attribute.strip(),
                "declaration": " ".join(match["declaration"].split()),
            })
    return declarations


def pe_exports(dll):
    env = dict(os.environ, LC_ALL="C")
    output = subprocess.check_output(["objdump", "-p", str(dll)], text=True, env=env)
    base = re.search(r"^ImageBase\s+([0-9a-fA-F]+)", output, re.MULTILINE)
    addresses = {
        int(index): int(rva, 16)
        for index, rva in re.findall(
            r"\[\s*(\d+)\]\s+\+base\[\s*\d+\]\s+([0-9a-fA-F]+) Export RVA", output
        )
    }
    table = output.split("[Ordinal/Name Pointer] Table", 1)[1].split("\n\n", 1)[0]
    names = re.findall(
        r"^\s*\[\s*(\d+)\]\s+(?:\+base\[\s*\d+\]\s+[0-9a-fA-F]+\s+)?(\w+)\s*$",
        table, re.MULTILINE,
    )
    exports = {name: addresses[int(index)] for index, name in names}
    if base is None or not exports or len(exports) != len(addresses):
        raise ValueError("Incomplete PE export parsing; check GNU objdump output")
    return int(base[1], 16), exports


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, help="Write the JSON inventory to this file")
    parser.add_argument("--disassemble", metavar="EXPORT", help="Inspect an export up to the next export address")
    args = parser.parse_args()
    image_base, exports = pe_exports(DLL)
    declarations = source_imports()
    required = {item["entry_point"] for item in declarations}
    report = {
        "binary": str(DLL.relative_to(ROOT)),
        "sha256": hashlib.sha256(DLL.read_bytes()).hexdigest(),
        "image_base": hex(image_base),
        "declaration_count": len(declarations),
        "required_entry_point_count": len(required),
        "export_count": len(exports),
        "missing_exports": sorted(required - exports.keys()),
        "exports_without_managed_imports": sorted(exports.keys() - required),
        "exports": {name: {"rva": hex(rva), "address": hex(image_base + rva)}
                    for name, rva in sorted(exports.items())},
        "imports": declarations,
    }
    if args.output:
        args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    if args.disassemble:
        if args.disassemble not in exports:
            parser.error("Unknown export: " + args.disassemble)
        start = exports[args.disassemble]
        # An export interval can include internal functions. It is not a function boundary.
        end = min((rva for rva in exports.values() if rva > start), default=start + 256)
        subprocess.run([
            "objdump", "-d", "-M", "intel",
            "--start-address=" + hex(image_base + start),
            "--stop-address=" + hex(image_base + end), str(DLL),
        ], check=True)
    elif not args.output:
        print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
