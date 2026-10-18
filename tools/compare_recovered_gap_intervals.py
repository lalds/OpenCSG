#!/usr/bin/env python3
"""Observe original FixGaps interval collection without patching its code.

The foreign BrushEdge has no triangle edges, so original FixGaps builds its
local intervals then skips matching/splitting. Allocator wrappers poison new
storage and capture it at deletion, exposing defined interval fields. This is
NOT a comparison of the full FixGaps mutation, visibility arbitration or repair.
Requires Linux x86_64 and the hash-pinned inspected Mach-O.
"""

import argparse
import ctypes as C
import hashlib
import json
import math
import mmap
import platform
import random
import struct
import sys

from compare_recovered_geometry import BINARY,SHA256,Vector3,f32
from compare_recovered_triangulation import Mesh,Triangle,Vertex,NativeVector


class Line(C.Structure):
    _fields_ = [('direction',Vector3),('origin',Vector3)]


class Interval(C.Structure):
    _fields_ = [('lower',C.c_float),('upper',C.c_float),('a',C.c_int32),('b',C.c_int32),
                ('edge',C.c_uint32),('reverse',C.c_uint8),('padding',C.c_byte*3)]


ALLOCATE = C.CFUNCTYPE(C.c_void_p,C.c_size_t)
DELETE = C.CFUNCTYPE(None,C.c_void_p)
FIX = C.CFUNCTYPE(None,C.POINTER(Mesh),C.c_int32,C.c_void_p,C.POINTER(NativeVector))


def set_vector(buffer,offset,array):
    start = C.addressof(array)
    struct.pack_into('<QQQ',buffer,offset,start,start+C.sizeof(array),start+C.sizeof(array))


class Oracle:
    def __init__(self,include_splitting=False):
        if sys.platform != 'linux' or platform.machine() not in ('x86_64','AMD64'):
            raise RuntimeError('Original-code oracle requires Linux x86_64')
        data = BINARY.read_bytes()
        if hashlib.sha256(data).hexdigest() != SHA256: raise ValueError('Hash changed; re-audit before execution')
        self.memory = mmap.mmap(-1,0x43000,prot=mmap.PROT_READ | mmap.PROT_WRITE)
        self.address = C.addressof(C.c_char.from_buffer(self.memory))
        pages = set()
        ranges = [(0x380ca,0x39f36),(0x3c140,0x3c25a)]
        if include_splitting:
            ranges += [(0x4d2c,0x4dd4),(0x2f36a,0x2f4d4),(0x37196,0x380ca)]
        for start,end in ranges:
            self.memory[start:end] = data[start:end]
            pages.update(range(start//4096,(end-1)//4096+1))
        self.memory[0x3c420:0x3c844] = data[0x3c420:0x3c844]
        self.memory[0x42768] = 1
        struct.pack_into('<I',self.memory,0x42770,1)
        self.libc = C.CDLL(None,use_errno=True)
        self.libc.malloc.argtypes = [C.c_size_t]; self.libc.malloc.restype = C.c_void_p
        self.libc.free.argtypes = [C.c_void_p]; self.libc.free.restype = None
        self.allocations,self.freed,self.errors = {},[],[]

        @ALLOCATE
        def allocate(size):
            pointer = self.libc.malloc(size)
            if pointer:
                C.memset(pointer,0xcc,size)
                self.allocations[pointer] = size
            return pointer

        @DELETE
        def delete(pointer):
            if not pointer: return
            size = self.allocations.pop(pointer,None)
            if size is None:
                self.errors.append('Unexpected deletion outside interval allocator')
                return
            self.freed.append(C.string_at(pointer,size))
            self.libc.free(pointer)

        self.allocate,self.delete = allocate,delete
        for offset,callback in ((0x400b8,allocate),(0x400a8,delete)):
            struct.pack_into('<Q',self.memory,offset,C.cast(callback,C.c_void_p).value)
        if include_splitting:
            for offset,name in ((0x40100,'bzero'),(0x40168,'free'),(0x40180,'malloc'),(0x40188,'memcpy')):
                struct.pack_into('<Q',self.memory,offset,C.cast(getattr(self.libc,name),C.c_void_p).value)
        self.libc.mprotect.argtypes = [C.c_void_p,C.c_size_t,C.c_int]; self.libc.mprotect.restype = C.c_int
        for offset,size,mode in [(0,0x40000,mmap.PROT_READ)]+[
                (p*4096,4096,mmap.PROT_READ | mmap.PROT_EXEC) for p in sorted(pages)]:
            if self.libc.mprotect(self.address+offset,size,mode): raise OSError(C.get_errno(),'mprotect')
        self.fix = FIX(self.address+0x380ca)

    def collect(self,mesh,line,edges):
        self.freed.clear(); self.errors.clear()
        assert not self.allocations
        source,foreign = (C.c_byte*104)(),(C.c_byte*104)()
        set_vector(source,24,edges)
        # One connected foreign BrushEdge. Its own triangle-edge list is empty,
        # so code after local collection takes 0x387cf -> 0x398f3.
        connected = (C.c_int32*2)(1,0)
        set_vector(source,48,connected)
        C.memmove(C.addressof(source)+80,C.addressof(line),24)
        C.memmove(C.addressof(foreign)+80,C.addressof(line),24)
        empty = (C.c_uint32*0)()
        set_vector(foreign,24,empty)
        node = (C.c_byte*224)()
        struct.pack_into('<i',node,24,1)
        struct.pack_into('<Q',node,96,C.addressof(mesh))
        set_vector(node,160,foreign)
        struct.pack_into('<i',node,216,0)
        pointers = (C.c_void_p*2)(None,C.addressof(node))
        metadata = (C.c_byte*8)(); metadata[7] = 3
        surface_record = (C.c_byte*104)(); struct.pack_into('<i',surface_record,40,1)
        surfaces = (C.c_byte*44)()
        struct.pack_into('<Q',self.memory,0x404b0,C.addressof(metadata))
        set_vector(self.memory,0x404f8,pointers)
        set_vector(self.memory,0x40670,surface_record)
        source_vector = NativeVector.of(source)
        self.fix(C.byref(mesh),0,C.byref(surfaces),C.byref(source_vector))
        if self.errors or self.allocations: raise AssertionError((self.errors,self.allocations))
        if not self.freed: raise AssertionError('No interval allocation observed')
        raw = self.freed[-1]
        result = []
        for offset in range(0,len(raw),24):
            if struct.unpack_from('<I',raw,offset+16)[0] == 0xcccccccc: break
            result.append(raw[offset:offset+21]) # Three unspecified padding bytes excluded.
        return result


def fixture(rng,case):
    tc = rng.randint(1,40)
    triangles = (Triangle*tc)()
    vertices = (Vertex*(tc*3))()
    world = (Vector3*(tc*3))()
    if case%3 == 0:
        direction = [0.0,0.0,0.0]; direction[rng.randrange(3)] = rng.choice((-1.0,1.0))
    else:
        direction = [rng.uniform(-1,1) for _ in range(3)]
        length = math.sqrt(sum(v*v for v in direction))
        direction = [f32(v/length) for v in direction]
    origin = [f32(rng.uniform(-4,4)) for _ in range(3)]
    line = Line(Vector3(*direction),Vector3(*origin))
    handles = []
    for p,t in enumerate(triangles):
        t.vertices[:] = [3*p,3*p+1,3*p+2]
        t.flags = rng.randrange(256)
        t.twins[:] = [0xffffffff]*3
        c = rng.randrange(3)
        for j in range(3):
            parameter = rng.choice((0.0,0.5,-1.0,2.0,rng.uniform(-8,8)))
            coords = [f32(o+f32(d*parameter)) for o,d in zip(origin,direction)]
            if rng.random()<0.3:
                k = rng.randrange(3)
                coords[k] = f32(coords[k]+rng.choice((0.0001,0.0003,0.00031622776,0.00032,0.001)))
            world[3*p+j] = Vector3(*coords)
        h = p*4+c
        handles.append(h)
        if rng.random()<0.3: handles.append(h) # Native retains duplicate same handles.
        if p and rng.random()<0.5:
            previous = rng.choice(handles[:-1])
            triangles[previous>>2].twins[previous&3] = h
            t.twins[c] = previous
    rng.shuffle(handles)
    # Include explicit zero-length and hidden edges through random equal
    # parameters above; flags do not gate the collector.
    mesh = Mesh(triangles,vertices,C.addressof(world),None,tc,tc*3,tc,tc*3)
    edges = (C.c_uint32*len(handles))(*handles)
    return mesh,line,edges,(triangles,vertices,world)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--library',required=True)
    parser.add_argument('--cases',type=int,default=5000)
    parser.add_argument('--seed',type=int,default=1559)
    args = parser.parse_args()
    if args.cases < 1: parser.error('cases must be positive')
    oracle = Oracle(); library = C.CDLL(args.library)
    recovered = library.rcsg_recovered_collect_gap_intervals
    recovered.argtypes = [C.POINTER(Mesh),C.POINTER(Line),C.POINTER(C.c_uint32),C.c_int32,C.POINTER(Interval),C.c_int32]
    recovered.restype = C.c_int32
    rng = random.Random(args.seed)
    compared = large = 0
    for case in range(args.cases):
        mesh,line,edges,keepalive = fixture(rng,case)
        expected = oracle.collect(mesh,line,edges)
        output = (Interval*len(edges))()
        count = recovered(C.byref(mesh),C.byref(line),edges,len(edges),output,len(output))
        actual = [bytes(item)[:21] for item in output[:max(0,count)]]
        if count != len(expected) or actual != expected:
            raise AssertionError(f'case {case}: return {count}, expected {len(expected)}, records equal {actual == expected}')
        compared += count; large += count>8
    print(json.dumps({'original_sha256':SHA256,'seed':args.seed,'collector_cases':args.cases,
                      'interval_records':compared,'cases_growing_native_interval_buffer':large,'mismatches':0},indent=2))


if __name__ == '__main__': main()
