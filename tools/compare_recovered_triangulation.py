#!/usr/bin/env python3
"""Execute isolated, hash-pinned native convex triangulation on Linux x86_64.

Only convex triangulation and FlagInvisiblePolygons are mapped. The static invalid-edge guard is initialized
as after its original constructor. Capacity is preallocated, so allocation and
guard imports are deliberately absent. Compare all triangle/vertex/output bytes,
including untouched fields, existing neighbors, and unused capacity. This does
not execute TriangleMesh::Create, concave triangulation, or gap repair.
"""

import argparse
import ctypes as C
import hashlib
import json
import mmap
import platform
import random
import struct
import sys

from compare_recovered_geometry import BINARY, SHA256, Vector3


class Triangle(C.Structure):
    _fields_ = [("vertices", C.c_int32*3), ("twins", C.c_uint32*3),
                ("normals", Vector3*3), ("edge_source", C.c_int32*3),
                ("unknown_48", C.c_byte*36), ("surface", C.c_int32),
                ("flags", C.c_uint8), ("unknown_71", C.c_uint8), ("state", C.c_uint16)]


class Vertex(C.Structure):
    _fields_ = [("edge", C.c_uint32), ("position", Vector3)]


class Mesh(C.Structure):
    _fields_ = [("triangles", C.POINTER(Triangle)), ("vertices", C.POINTER(Vertex)),
                ("unknown_10", C.c_void_p), ("unknown_18", C.c_void_p),
                ("triangle_count", C.c_int32), ("vertex_count", C.c_int32),
                ("triangle_capacity", C.c_int32), ("vertex_capacity", C.c_int32)]


class NativeVector(C.Structure):
    _fields_ = [("begin", C.c_void_p), ("end", C.c_void_p), ("capacity", C.c_void_p)]

    @classmethod
    def of(cls, array):
        start = C.addressof(array)
        return cls(start, start+C.sizeof(array), start+C.sizeof(array))


NATIVE = C.CFUNCTYPE(None, C.POINTER(Mesh), C.c_void_p,
                    *([C.POINTER(NativeVector)]*4), C.c_int32, C.c_uint8, C.c_uint16)
FLAG_NATIVE = C.CFUNCTYPE(None,C.c_void_p)


def original_function():
    if sys.platform != "linux" or platform.machine() not in ("x86_64", "AMD64"):
        raise RuntimeError("Original-code comparison requires Linux x86_64")
    data = BINARY.read_bytes()
    if hashlib.sha256(data).hexdigest() != SHA256:
        raise ValueError("Original hash changed; re-audit before execution")
    memory = mmap.mmap(-1, 0x43000, prot=mmap.PROT_READ | mmap.PROT_WRITE)
    memory[0x2f4d4:0x2f822] = data[0x2f4d4:0x2f822]
    memory[0x208d4:0x20960] = data[0x208d4:0x20960]  # Includes the 4-entry jump table at 0x20950.
    memory[0x42768] = 1  # Completed static initialization guard.
    memory[0x42770:0x42774] = struct.pack("<I",1)  # Original InvalidIndex storage -> -1 after sign extension.
    address = C.addressof(C.c_byte.from_buffer(memory))
    libc = C.CDLL(None, use_errno=True)
    libc.mprotect.argtypes = [C.c_void_p,C.c_size_t,C.c_int]
    libc.mprotect.restype = C.c_int
    for start, size, mode in ((address,0x43000,mmap.PROT_READ),
                              (address+0x20000,0x1000,mmap.PROT_READ | mmap.PROT_EXEC),
                              (address+0x2f000,0x1000,mmap.PROT_READ | mmap.PROT_EXEC)):
        if libc.mprotect(start,size,mode):
            raise OSError(C.get_errno(),"mprotect failed")
    return memory,NATIVE(address+0x2f4d4),FLAG_NATIVE(address+0x208d4)


class Fixture:
    def __init__(self, triangles, vertices, initial_count, output):
        self.triangles = (Triangle*128).from_buffer_copy(triangles)
        self.vertices = (Vertex*128).from_buffer_copy(vertices)
        self.output = (C.c_uint32*(len(output)//4)).from_buffer_copy(output)
        self.mesh = Mesh(self.triangles,self.vertices,0x1234,0x5678,initial_count,128,128,128)

    def snapshot(self):
        return (bytes(self.triangles),bytes(self.vertices),bytes(self.output),
                (self.mesh.unknown_10,self.mesh.unknown_18,self.mesh.triangle_count,
                 self.mesh.vertex_count,self.mesh.triangle_capacity,self.mesh.vertex_capacity))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--library",required=True)
    parser.add_argument("--cases",type=int,default=5000)
    parser.add_argument("--seed",type=int,default=1559)
    args = parser.parse_args()
    if args.cases < 1:
        parser.error("cases must be positive")
    assert C.sizeof(Triangle) == 116 and C.sizeof(Vertex) == 16 and C.sizeof(Mesh) == 48
    code,native,flag_native = original_function()
    lib = C.CDLL(args.library)
    recovered = lib.rcsg_recovered_triangulate_convex
    recovered.argtypes = [C.POINTER(Mesh),C.POINTER(C.c_int32),C.POINTER(Vector3),
                           C.POINTER(C.c_uint32),C.POINTER(C.c_uint32),C.c_int32,C.c_int32,C.c_uint8,C.c_uint16]
    recovered.restype = C.c_int32
    flag_recovered = lib.rcsg_recovered_flag_triangle_visibility
    flag_recovered.argtypes = [C.POINTER(Mesh)]
    flag_recovered.restype = C.c_int32
    rng = random.Random(args.seed)
    emitted = 0
    neighbor_cases = 0
    for case in range(args.cases):
        count = rng.randint(3,64)
        initial = rng.randint(0,16)
        triangle_bytes = rng.randbytes(128*C.sizeof(Triangle))
        vertex_bytes = rng.randbytes(128*C.sizeof(Vertex))
        output_bytes = rng.randbytes(count*4)
        first = Fixture(triangle_bytes,vertex_bytes,initial,output_bytes)
        second = Fixture(triangle_bytes,vertex_bytes,initial,output_bytes)
        indices = (C.c_int32*count)(*rng.sample(range(128),count))
        # Arbitrary bit patterns also verify that corner normals are merely
        # copied: no normalization, NaN canonicalization, or geometric predicate.
        normals = (Vector3*count).from_buffer_copy(rng.randbytes(12*count))
        invalids = (0xffffffff,3,7,0x1234567b)
        twins = (C.c_uint32*count)(*[
            (rng.randrange(initial)*4+rng.randrange(3)) if initial and rng.random()<0.5
            else rng.choice(invalids) for _ in range(count)])
        neighbor_cases += any(t & 3 != 3 for t in twins)
        surface,flags,state = rng.randint(-2000,2000),rng.randrange(256),rng.randrange(65536)
        iv,nv,tv,ov = (NativeVector.of(x) for x in (indices,normals,twins,first.output))
        native(C.byref(first.mesh),None,C.byref(iv),C.byref(nv),C.byref(tv),C.byref(ov),surface,flags,state)
        result = recovered(C.byref(second.mesh),indices,normals,twins,second.output,count,surface,flags,state)
        if result != count-2 or first.snapshot() != second.snapshot():
            labels = ("triangle_bytes","vertex_bytes","boundary_edges","mesh_metadata")
            mismatch = [name for name,a,b in zip(labels,first.snapshot(),second.snapshot()) if a != b]
            raise AssertionError(f"case {case}, corners {count}, prefix {initial}, return {result}, mismatches {mismatch}")
        emitted += count-2
    # Exhaust every signed 16-bit state, preserving every byte except flags.
    # Four sweeps vary high input flag bits, exercising all four preserved
    # combinations for each state, without allocating millions of records.
    flag_checks = 0
    for high in (0,0x40,0x80,0xc0):
        count = 65536
        before = (Triangle*count).from_buffer_copy(rng.randbytes(C.sizeof(Triangle)*count))
        for state,t in enumerate(before):
            t.state = state
            t.flags = high | (state & 0x3f)
        after = (Triangle*count).from_buffer_copy(bytes(before))
        original_mesh = Mesh(before,None,None,None,count,0,count,0)
        recovered_mesh = Mesh(after,None,None,None,count,0,count,0)
        brush = (C.c_void_p*13)()
        brush[12] = C.addressof(original_mesh)  # Observed brush->triangleMesh at +0x60.
        flag_native(C.byref(brush))
        result = flag_recovered(C.byref(recovered_mesh))
        if result != 0 or bytes(before) != bytes(after):
            raise AssertionError(f"FlagInvisiblePolygons differs for high flags {high}")
        flag_checks += count
    print(json.dumps({"original_sha256":SHA256,"seed":args.seed,"polygon_cases":args.cases,
                      "triangles_emitted":emitted,"cases_with_existing_neighbors":neighbor_cases,
                      "visibility_state_flag_checks":flag_checks,
                      "mismatches":0},indent=2))
    del native,flag_native
    code.close()


if __name__ == "__main__":
    main()
