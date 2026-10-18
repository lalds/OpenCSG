#!/usr/bin/env python3
"""Research oracle for the original CategoryStackNode table generator on Linux x64.

Only the generator's static code dependency closure and its constant pool are
mapped. malloc/free/memory operations use the host libc; next_prime is supplied
as a mathematical helper. No Mach-O initializers, exception paths or Unity code
are executed. CSG node manager data is synthesized from explicit test inputs.
"""

import ctypes as C
import hashlib
import mmap
import platform
import struct
import sys

from compare_recovered_geometry import BINARY, SHA256
from macho_audit import inspect


class NativeVector(C.Structure):
    _fields_ = [("begin", C.c_void_p), ("end", C.c_void_p), ("capacity", C.c_void_p)]


class NodePrefix(C.Structure):
    _fields_ = [("vtable", C.c_void_p), ("children", C.POINTER(C.c_int32)),
                ("child_count", C.c_int32), ("unknown_14", C.c_int32),
                ("node_index", C.c_int32), ("tree_index", C.c_int32)]


class TwoBitSet(C.Structure):
    _fields_ = [("bits", C.c_int32), ("length", C.c_int32), ("words", C.POINTER(C.c_uint32))]


class CategoryNode(C.Structure):
    _fields_ = [("node_index", C.c_int32), ("state", C.c_int16),
                ("next", C.c_int16 * 4), ("padding", C.c_int16)]


GENERATE = C.CFUNCTYPE(None, C.c_int32, C.POINTER(NodePrefix), C.POINTER(TwoBitSet),
                      C.POINTER(NativeVector), C.POINTER(NativeVector), C.POINTER(C.c_int32))


class CategorizationOracle:
    def __init__(self):
        if sys.platform != "linux" or platform.machine() not in ("x86_64", "AMD64"):
            raise RuntimeError("Original-code oracle requires Linux x86_64")
        data = BINARY.read_bytes()
        if hashlib.sha256(data).hexdigest() != SHA256:
            raise ValueError("Original binary changed; re-audit before execution")
        report, _ = inspect()
        functions = {int(f["address"], 16): f for f in report["functions"]}
        starts = sorted(functions)
        ends = dict(zip(starts, starts[1:] + [0x3c140]))
        selected, pending = set(), [0x25456]
        while pending:
            address = pending.pop()
            if address in selected:
                continue
            if address not in functions:
                if address >= 0x3c140:  # Imported stub, bound separately below.
                    continue
                raise ValueError("Unknown code dependency: " + hex(address))
            selected.add(address)
            function = functions[address]
            pending += [int(edge["target_address"], 16)
                        for edge in function["direct_calls"] + function["tail_jump_candidates"]]

        self.memory = mmap.mmap(-1, 0x43000, prot=mmap.PROT_READ | mmap.PROT_WRITE)
        self.address = C.addressof(C.c_char.from_buffer(self.memory))
        executable_pages = set()
        for first in sorted(selected):
            last = ends[first]
            self.memory[first:last] = data[first:last]
            executable_pages.update(range(first // 4096, (last - 1) // 4096 + 1))
        self.memory[0x3c140:0x3c25a] = data[0x3c140:0x3c25a]  # Import stubs.
        self.memory[0x3c420:0x3c844] = data[0x3c420:0x3c844]  # Constant pool.
        executable_pages.add(0x3c)
        self.libc = C.CDLL(None, use_errno=True)
        self.libc.malloc.argtypes = [C.c_size_t]
        self.libc.malloc.restype = C.c_void_p
        self.libc.free.argtypes = [C.c_void_p]
        self.libc.free.restype = None

        @C.CFUNCTYPE(C.c_size_t, C.c_size_t)
        def next_prime(value):
            candidate = max(2, value)
            while any(candidate % divisor == 0 for divisor in range(2, int(candidate**0.5) + 1)):
                candidate += 1
            return candidate

        self.next_prime = next_prime
        self._write_pointer(0x40070, C.cast(next_prime, C.c_void_p).value)
        bindings = {0x400a0: "free", 0x400a8: "free", 0x400b0: "malloc", 0x400b8: "malloc",
                    0x40100: "bzero", 0x40168: "free", 0x40180: "malloc", 0x40188: "memcpy",
                    0x40190: "memmove", 0x40198: "memset", 0x401a0: "realloc"}
        for address, name in bindings.items():
            self._write_pointer(address, C.cast(getattr(self.libc, name), C.c_void_p).value)
        self.libc.mprotect.argtypes = [C.c_void_p, C.c_size_t, C.c_int]
        self.libc.mprotect.restype = C.c_int
        self._protect(0, 0x40000, mmap.PROT_READ)
        for page in sorted(executable_pages):
            self._protect(page * 4096, 4096, mmap.PROT_READ | mmap.PROT_EXEC)
        self.generate_function = GENERATE(self.address + 0x25456)
        self.mapped_functions = len(selected)

    def _protect(self, offset, size, protection):
        if self.libc.mprotect(self.address + offset, size, protection):
            raise OSError(C.get_errno(), "mprotect failed")

    def _write_pointer(self, offset, value):
        struct.pack_into("<Q", self.memory, offset, value or 0)

    def generate(self, nodes, target, intersections=None, root=0):
        """nodes: list of {children: [indices]} branches or {operation: 0..2} brushes.

        Node kind/flags follow the original CreateTree/CreateBrush observations.
        intersections are raw 2-bit hints (0 outside, 2 inside, 1/3 unresolved).
        Returned transitions are native state IDs, not CategoryIndex values.
        """
        count = len(nodes)
        prefixes = (NodePrefix * count)()
        pointers = (C.c_void_p * count)()
        flags = (C.c_uint32 * count)()
        child_arrays = []
        for index, node in enumerate(nodes):
            children = node.get("children", [])
            array = (C.c_int32 * len(children))(*children)
            child_arrays.append(array)
            prefixes[index] = NodePrefix(None, array, len(children), 0, index, root)
            pointers[index] = C.addressof(prefixes[index])
            operation = node.get("operation", 0)
            type_flags = 0 if "children" in node else 0xc0 if node.get("infinite", False) else 0x80
            flags[index] = ((operation | type_flags) << 16) | ((1 if index == root else 2 if "children" in node else 3) << 24)
        self._write_pointer(0x404b0, C.addressof(flags))
        self._write_pointer(0x404f8, C.addressof(pointers))
        self._write_pointer(0x40500, C.addressof(pointers) + count * 8)
        words = (C.c_uint32 * ((count + 15) // 16))()
        for index, value in enumerate(intersections or [1] * count):
            words[index // 16] |= value << ((index % 16) * 2)
        used = TwoBitSet(2, count, words)
        # UpdateCategorizationTable reserves 6 * global node count before calling
        # the generator (0x46ae..0x46be). The generator keeps pointers into this
        # workspace while appending: calling it with an empty-capacity vector is
        # outside its preconditions and can invalidate those pointers.
        workspace_bytes = max(1, count * 6) * C.sizeof(CategoryNode)
        workspace = self.libc.malloc(workspace_bytes)
        if not workspace:
            raise MemoryError("Native category workspace allocation failed")
        working = NativeVector(workspace, workspace, workspace + workspace_bytes)
        output = NativeVector()
        state_count = C.c_int32(0)
        try:
            self.generate_function(root, C.byref(prefixes[target]), C.byref(used),
                                   C.byref(working), C.byref(output), C.byref(state_count))
            size = ((output.end or 0) - (output.begin or 0)) // C.sizeof(CategoryNode)
            rows = (CategoryNode * size).from_address(output.begin) if size else []
            return {"state_count": state_count.value,
                    "rows": [{"node": r.node_index, "state": r.state, "next": list(r.next)} for r in rows]}
        finally:
            self.libc.free(working.begin)
            self.libc.free(output.begin)
