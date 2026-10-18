#!/usr/bin/env python3
"""Compare reconstructed category program outcomes with the original generator.

Compares category algebra to binary constants and program evaluation to native
GenerateCategorizationTable on synthesized valid trees. State IDs/row order are
deliberately not compared: the portable decision compiler uses different reduction.
"""

import argparse
import ctypes as C
import itertools
import json
from pathlib import Path
import random
import struct

from original_categorization import BINARY, CategoryNode, CategorizationOracle, SHA256


class TreeNode(C.Structure):
    _fields_ = [("first_child", C.c_int32), ("child_count", C.c_int32),
                ("operation", C.c_uint8), ("is_brush", C.c_uint8),
                ("infinite", C.c_uint8), ("reserved", C.c_uint8)]


COMPILE = C.CFUNCTYPE(C.c_int32, C.POINTER(TreeNode), C.c_int32, C.POINTER(C.c_int32),
                      C.c_int32, C.c_int32, C.POINTER(C.c_uint8), C.c_int32,
                      C.POINTER(CategoryNode), C.c_int32, C.POINTER(C.c_uint16), C.POINTER(C.c_int32))
EVALUATE = C.CFUNCTYPE(C.c_int32, C.POINTER(CategoryNode), C.c_int32, C.c_uint16,
                       C.POINTER(C.c_uint8), C.c_int32)
COMBINE = C.CFUNCTYPE(C.c_int32, C.c_uint8, C.c_uint8, C.c_uint8)


def native_evaluate(rows, categories):
    state = 0
    index = 0
    while index < len(rows):
        brush = rows[index]["node"]
        end = index + 1
        while end < len(rows) and rows[end]["node"] == brush:
            end += 1
        for row in rows[index:end]:
            if row["state"] == state:
                state = row["next"][categories[brush]]
                break
        index = end
    if state not in (0, 1, 2, 3):
        raise AssertionError("Original program ended in nonterminal state: " + str(state))
    # Empty original output leaves initial 0; no subject surface survives.
    return state or 1


def compile_program(function, nodes, target, hints):
    flat = []
    records = (TreeNode * len(nodes))()
    for index, node in enumerate(nodes):
        children = node.get("children", [])
        records[index] = TreeNode(len(flat), len(children), node.get("operation", 0),
                                  int("children" not in node), int(node.get("infinite", False)), 0)
        flat += children
    indices = (C.c_int32 * len(flat))(*flat)
    flags = (C.c_uint8 * len(nodes))(*hints)
    initial, count = C.c_uint16(), C.c_int32()
    size = function(records, len(nodes), indices, len(flat), target, flags, 0, None, 0,
                     C.byref(initial), C.byref(count))
    if size < 0:
        raise AssertionError(f"Compile rejected tree with {size}: {nodes}")
    rows = (CategoryNode * max(size, 1))()
    actual = function(records, len(nodes), indices, len(flat), target, flags, 0, rows, size,
                       C.byref(initial), C.byref(count))
    assert actual == size
    return rows, size, initial.value, count.value


def random_tree(rng, index):
    count = rng.randrange(2, 9)
    nodes = [{"children": list(range(1, count + 1))}]
    nodes += [{"operation": rng.randrange(3)} for _ in range(count)]
    # Nested groups are formed from adjacent child ranges. Every group currently
    # has an additive child; empty/all-nonadditive group behavior is unresolved.
    if index % 2:
        for _ in range(rng.randrange(1, 4)):
            children = nodes[0]["children"]
            if len(children) < 2:
                break
            start = rng.randrange(len(children) - 1)
            length = rng.randrange(2, len(children) - start + 1)
            group = children[start:start + length]
            nodes[group[0]]["operation"] = 0
            nodes.append({"children": group, "operation": rng.randrange(3)})
            children[start:start + length] = [len(nodes) - 1]
    root_children = nodes[0]["children"]
    nodes[rng.choice(root_children)]["operation"] = 0
    # Sometimes keep leading non-additive children before the first additive.
    if index % 3 != 0:
        nodes[root_children[0]]["operation"] = 0
    if index % 7 == 0:
        leaf = rng.randrange(1, count + 1)
        nodes[leaf]["infinite"] = True
    return nodes


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--library", type=Path, required=True)
    parser.add_argument("--trees", type=int, default=300)
    parser.add_argument("--samples", type=int, default=128)
    parser.add_argument("--seed", type=int, default=1559)
    args = parser.parse_args()
    if args.trees < 1 or args.samples < 1:
        parser.error("Tree/sample counts must be positive")
    oracle = CategorizationOracle()
    library = C.CDLL(str(args.library.resolve()))
    compile_function = COMPILE(("rcsg_recovered_compile_categories", library))
    evaluate = EVALUATE(("rcsg_recovered_evaluate_categories", library))
    combine = COMBINE(("rcsg_recovered_combine_categories", library))
    matrices = struct.unpack_from("<3I", BINARY.read_bytes(), 0x3c750)
    for operation, packed in enumerate(matrices):
        for left, right in itertools.product(range(4), repeat=2):
            expected = (packed >> (left * 8 + right * 2)) & 3
            assert combine(operation, left, right) == expected
    rng = random.Random(args.seed)
    programs = comparisons = 0
    maximum_rows = maximum_native_rows = 0
    outcomes = {1: 0, 2: 0, 3: 0}
    for index in range(args.trees):
        nodes = random_tree(rng, index)
        leaves = [i for i, n in enumerate(nodes) if "children" not in n]
        for target in leaves:
            hints = [1] * len(nodes)
            if index % 4 == 0:
                for leaf in leaves:
                    if leaf != target:
                        hints[leaf] = rng.randrange(4)
            native = oracle.generate(nodes, target, hints)
            rows, size, initial, state_count = compile_program(compile_function, nodes, target, hints)
            programs += 1
            maximum_rows, maximum_native_rows = max(maximum_rows, size), max(maximum_native_rows, len(native["rows"]))
            varying = [leaf for leaf in leaves if leaf != target]
            samples = (list(itertools.product(range(4), repeat=len(varying))) if len(varying) <= 3
                       else [tuple(rng.randrange(4) for _ in varying) for _ in range(args.samples)])
            for values in samples:
                categories = [0] * len(nodes)
                categories[target] = 1
                for leaf, value in zip(varying, values):
                    categories[leaf] = value
                expected = native_evaluate(native["rows"], categories)
                array = (C.c_uint8 * len(nodes))(*categories)
                actual = evaluate(rows, size, initial, array, len(nodes))
                if actual != expected:
                    raise AssertionError(json.dumps({"tree": index, "seed": args.seed, "target": target,
                        "nodes": nodes, "hints": hints, "categories": categories,
                        "expected": expected, "actual": actual, "native": native,
                        "portable": {"initial": initial, "state_count": state_count,
                                     "rows": [{"node": r.node_index, "state": r.state, "next": list(r.next)}
                                              for r in rows[:size]]}}))
                outcomes[actual] += 1
                comparisons += 1
    print(json.dumps({"original_sha256": SHA256, "seed": args.seed,
                      "native_functions_mapped": oracle.mapped_functions, "operation_table_checks": 48,
                      "trees": args.trees, "programs": programs, "evaluations": comparisons,
                      "outcomes": outcomes, "maximum_rows": maximum_rows,
                      "maximum_native_rows": maximum_native_rows, "mismatches": 0}, indent=2))


if __name__ == "__main__":
    main()
