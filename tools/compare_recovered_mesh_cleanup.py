#!/usr/bin/env python3
"""Differential execution of isolated native compaction and ConnectGaps.

Linux x86_64 only. Hash-pinned inspected functions, libc allocation/free/bzero,
and a signed-int sorting callback replacing libc++ __sort. No Unity, native
initializers, optimization planner, exception paths, or geometric FixGaps run.
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

from compare_recovered_geometry import BINARY,SHA256
from compare_recovered_triangulation import Mesh,Triangle,Vertex,NativeVector


class Pair(C.Structure):
    _fields_ = [("brush",C.c_int32),("edge",C.c_uint32)]


class GapBrush(C.Structure):
    _fields_ = [("mesh",C.POINTER(Mesh)),("node",C.c_int32),("plane_count",C.c_int32)]


def links(triangle):
    return (Pair*3).from_address(C.addressof(triangle)+72)


CLEANUP = C.CFUNCTYPE(None,C.POINTER(Mesh),C.c_void_p,C.c_void_p)
CONNECT = C.CFUNCTYPE(None,C.POINTER(Mesh),C.c_int32,C.c_void_p)
SORT = C.CFUNCTYPE(None,C.c_void_p,C.c_void_p,C.c_void_p)


class Oracle:
    def __init__(self):
        if sys.platform != "linux" or platform.machine() not in ("x86_64","AMD64"):
            raise RuntimeError("Original-code oracle requires Linux x86_64")
        data = BINARY.read_bytes()
        if hashlib.sha256(data).hexdigest() != SHA256:
            raise ValueError("Hash changed; re-audit before executing")
        self.memory = mmap.mmap(-1,0x43000,prot=mmap.PROT_READ | mmap.PROT_WRITE)
        self.address = C.addressof(C.c_char.from_buffer(self.memory))
        pages = set()
        for first,last in ((0xfeea,0xff29),(0x32ab0,0x32b0e),(0x32b82,0x330a7),
                           (0x3a24c,0x3a335),(0x3c140,0x3c25a)):
            self.memory[first:last] = data[first:last]
            pages.update(range(first//4096,(last-1)//4096+1))
        self.memory[0x42768] = 1
        struct.pack_into('<I',self.memory,0x42770,1)
        self.libc = C.CDLL(None,use_errno=True)
        for offset,name in ((0x400a0,'free'),(0x400a8,'free'),(0x400b0,'malloc'),
                            (0x400b8,'malloc'),(0x40100,'bzero')):
            struct.pack_into('<Q',self.memory,offset,C.cast(getattr(self.libc,name),C.c_void_p).value)

        @SORT
        def sort_ints(first,last,comparator):
            count = ((last or 0)-(first or 0))//4
            if count:
                values = (C.c_int32*count).from_address(first)
                values[:] = sorted(values)
        self.sort = sort_ints
        struct.pack_into('<Q',self.memory,0x40088,C.cast(sort_ints,C.c_void_p).value)
        self.libc.mprotect.argtypes = [C.c_void_p,C.c_size_t,C.c_int]
        self.libc.mprotect.restype = C.c_int
        for offset,size,prot in [(0,0x40000,mmap.PROT_READ)]+[
                (page*4096,4096,mmap.PROT_READ | mmap.PROT_EXEC) for page in sorted(pages)]:
            if self.libc.mprotect(self.address+offset,size,prot):
                raise OSError(C.get_errno(),'mprotect')
        self.cleanup = CLEANUP(self.address+0x32b82)
        self.connect = CONNECT(self.address+0x3a24c)

    def manager(self,meshes,node_ids):
        nodes = [(C.c_byte*104)() for _ in meshes]
        pointers = (C.c_void_p*len(nodes))()
        for i,(node,mesh,node_id) in enumerate(zip(nodes,meshes,node_ids)):
            if node_id is None: continue
            struct.pack_into('<i',node,24,node_id)
            struct.pack_into('<Q',node,96,C.addressof(mesh))
            pointers[i] = C.addressof(node)
        start = C.addressof(pointers)
        struct.pack_into('<QQ',self.memory,0x404f8,start,start+C.sizeof(pointers))
        return nodes,pointers


class Fixture:
    def __init__(self,triangles,vertices,tcount,vcount):
        self.triangles = (Triangle*128).from_buffer_copy(triangles)
        self.vertices = (Vertex*128).from_buffer_copy(vertices)
        self.mesh = Mesh(self.triangles,self.vertices,0x1234,0x5678,tcount,vcount,128,128)

    def snapshot(self):
        return (bytes(self.triangles),bytes(self.vertices),self.mesh.triangle_count,self.mesh.vertex_count,
                self.mesh.unknown_10,self.mesh.unknown_18,self.mesh.triangle_capacity,self.mesh.vertex_capacity)


def cleanup_case(oracle,recovered,rng,case):
    vc,tc = rng.randrange(65),rng.randrange(65)
    if vc < 3: tc = 0
    removed_v = rng.sample(range(vc),rng.randrange(vc+1))
    survivors = [v for v in range(vc) if v not in removed_v]
    removed_t = rng.sample(range(tc),rng.randrange(tc+1)) if len(survivors)>=3 else list(range(tc))
    rng.shuffle(removed_t)
    fixture = Fixture(rng.randbytes(128*116),rng.randbytes(128*16),tc,vc)
    for v in range(vc):
        fixture.vertices[v].edge = rng.randrange(tc)*4+rng.randrange(3) if tc else 0xffffffff
    for p in range(tc):
        t = fixture.triangles[p]
        t.surface = -1 if p in removed_t else rng.randrange(100)
        t.vertices[:] = rng.sample(survivors,3) if p not in removed_t else [0,0,0]
        t.twins[:] = [rng.randrange(tc)*4+rng.randrange(3) if rng.random()<0.8 else 0xffffffff for _ in range(3)]
    other = Fixture(bytes(fixture.triangles),bytes(fixture.vertices),tc,vc)
    v1,v2 = [(C.c_int32*len(removed_v))(*removed_v) for _ in range(2)]
    t1,t2 = [(C.c_int32*len(removed_t))(*removed_t) for _ in range(2)]
    context = (NativeVector*6)()
    context[1],context[3] = NativeVector.of(t1),NativeVector.of(v1)
    oracle.cleanup(C.byref(fixture.mesh),None,C.byref(context))
    result = recovered(C.byref(other.mesh),v2,len(v2),t2,len(t2))
    a,b = fixture.snapshot(),other.snapshot()
    if result != 0 or a != b or bytes(v1) != bytes(v2) or bytes(t1) != bytes(t2):
        mismatch = [i for i,(x,y) in enumerate(zip(a,b)) if x != y]
        raise AssertionError(f'cleanup case {case}: V={vc}, T={tc}, removeV={removed_v}, removeT={removed_t}, return={result}, mismatch={mismatch}, counts {a[2:4]} vs {b[2:4]}')
    return (vc-len(removed_v)==0 and vc>0,
            bool(removed_v) and fixture.mesh.vertex_count != vc-len(removed_v))


def connect_case(oracle,recovered,rng,case):
    count = rng.randrange(1,8)
    source = rng.randrange(count)
    fixtures = [Fixture(rng.randbytes(128*116),rng.randbytes(128*16),rng.randrange(1,30),3) for _ in range(count)]
    node_ids = [i if rng.random()<0.7 else None if rng.random()<0.5 else i+100 for i in range(count)]
    node_ids[source] = source
    planes = rng.randrange(1,10)
    for f in fixtures:
        for t in f.triangles[:f.mesh.triangle_count]:
            t.surface = rng.randrange(-2,planes+3)
            for pair in links(t):
                pair.brush = rng.randrange(-2,count+2)
                pair.edge = (rng.randrange(fixtures[pair.brush].mesh.triangle_count)*4+rng.randrange(3)
                             if 0 <= pair.brush < count else 0xffffffff)
    copies = [Fixture(bytes(f.triangles),bytes(f.vertices),f.mesh.triangle_count,f.mesh.vertex_count) for f in fixtures]
    # Existing self-links/cycles deliberately exercise mutation aliasing and
    # multiple writes to the same target slot, in the original loop order.
    keepalive = oracle.manager([f.mesh for f in fixtures],node_ids)
    surfaces = (C.c_byte*44)()
    struct.pack_into('<i',surfaces,40,planes)
    oracle.connect(C.byref(fixtures[source].mesh),source,C.byref(surfaces))
    manager = (GapBrush*count)(*[
        GapBrush(C.pointer(f.mesh) if node is not None else None,node or 0,planes)
        for f,node in zip(copies,node_ids)])
    result = recovered(C.byref(copies[source].mesh),source,planes,manager,count)
    if result != 0 or any(a.snapshot()!=b.snapshot() for a,b in zip(fixtures,copies)):
        raise AssertionError(f'ConnectGaps case {case}, source {source}, nodes {node_ids}, return {result}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--library',required=True)
    parser.add_argument('--cases',type=int,default=5000)
    parser.add_argument('--seed',type=int,default=1559)
    args = parser.parse_args()
    if args.cases < 1: parser.error('cases must be positive')
    oracle = Oracle()
    lib = C.CDLL(args.library)
    cleanup = lib.rcsg_recovered_cleanup_mesh
    cleanup.argtypes = [C.POINTER(Mesh),C.POINTER(C.c_int32),C.c_int32,C.POINTER(C.c_int32),C.c_int32]
    cleanup.restype = C.c_int32
    connect = lib.rcsg_recovered_connect_gaps
    connect.argtypes = [C.POINTER(Mesh),C.c_int32,C.c_int32,C.POINTER(GapBrush),C.c_int32]
    connect.restype = C.c_int32
    rng = random.Random(args.seed)
    empty_vertex_cases = 0
    skipped_count_cases = 0
    for case in range(args.cases):
        empty,skipped = cleanup_case(oracle,cleanup,rng,case)
        empty_vertex_cases += empty
        skipped_count_cases += skipped
        connect_case(oracle,connect,rng,case)
    print(json.dumps({'original_sha256':SHA256,'seed':args.seed,'cleanup_cases':args.cases,
                      'connect_gap_cases':args.cases,'all_vertex_removal_cases':empty_vertex_cases,
                      'native_vertex_count_skip_cases':skipped_count_cases,'mismatches':0},indent=2))


if __name__ == '__main__': main()
