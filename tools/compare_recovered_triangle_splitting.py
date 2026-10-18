#!/usr/bin/env python3
"""Compare original SplitTriangleEdgeIndex, AddWorldVertex and brush transform.

Hash-pinned isolated Mach-O code on Linux x86_64. Native triangle/vertex/list
capacities are preallocated; original AddWorldVertex still reallocates its world
position/lookup block through host libc. No native exception/growth paths, Unity
initializers or geometric FixGaps are executed. Run as a separate process.
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

from compare_recovered_geometry import BINARY,SHA256,Vector3
from compare_recovered_triangulation import Mesh,Triangle,Vertex,NativeVector


class SourceList(C.Structure):
    _fields_ = [('edges',C.POINTER(C.c_uint32)),('count',C.c_int32),('capacity',C.c_int32)]


class Transform(C.Structure):
    _fields_ = [('values',C.c_float*16)]


NATIVE = C.CFUNCTYPE(C.c_void_p,C.POINTER(C.c_uint32),C.POINTER(Mesh),C.c_int32,
                     C.POINTER(NativeVector),C.POINTER(C.c_uint32),C.POINTER(Vector3),C.c_float)


class Oracle:
    def __init__(self):
        if sys.platform != 'linux' or platform.machine() not in ('x86_64','AMD64'):
            raise RuntimeError('Original-code oracle requires Linux x86_64')
        data = BINARY.read_bytes()
        if hashlib.sha256(data).hexdigest() != SHA256:
            raise ValueError('Original hash changed; re-audit before execution')
        self.memory = mmap.mmap(-1,0x43000,prot=mmap.PROT_READ | mmap.PROT_WRITE)
        self.address = C.addressof(C.c_char.from_buffer(self.memory))
        pages = set()
        for start,end in ((0x4d2c,0x4dd4),(0x2f36a,0x2f4d4),(0x37196,0x37fad),(0x3c140,0x3c25a)):
            self.memory[start:end] = data[start:end]
            pages.update(range(start//4096,(end-1)//4096+1))
        self.memory[0x3c420:0x3c844] = data[0x3c420:0x3c844]
        self.memory[0x42768] = 1
        struct.pack_into('<I',self.memory,0x42770,1)
        self.libc = C.CDLL(None,use_errno=True)
        self.libc.malloc.argtypes = [C.c_size_t]
        self.libc.malloc.restype = C.c_void_p
        self.libc.free.argtypes = [C.c_void_p]
        self.libc.free.restype = None
        for offset,name in ((0x40168,'free'),(0x40180,'malloc'),(0x40188,'memcpy')):
            struct.pack_into('<Q',self.memory,offset,C.cast(getattr(self.libc,name),C.c_void_p).value)
        self.libc.mprotect.argtypes = [C.c_void_p,C.c_size_t,C.c_int]
        self.libc.mprotect.restype = C.c_int
        for offset,size,mode in [(0,0x40000,mmap.PROT_READ)]+[
                (page*4096,4096,mmap.PROT_READ | mmap.PROT_EXEC) for page in sorted(pages)]:
            if self.libc.mprotect(self.address+offset,size,mode):
                raise OSError(C.get_errno(),'mprotect')
        self.split = NATIVE(self.address+0x37196)

    def matrix(self,brush,matrix):
        transforms = (C.c_byte*((brush+1)*256))()
        C.memmove(C.addressof(transforms)+brush*256+64,C.addressof(matrix),64)
        struct.pack_into('<Q',self.memory,0x404c8,C.addressof(transforms))
        return transforms


def compare(oracle,recovered,rng,case):
    vc,tc = rng.randint(4,24),rng.randint(6,24)
    triangles = (Triangle*64).from_buffer_copy(rng.randbytes(64*116))
    vertices = (Vertex*64).from_buffer_copy(rng.randbytes(64*16))
    a,b = rng.sample(range(tc),2)
    c,d = rng.randrange(3),rng.randrange(3)
    ap,an,bp,bn = (c+2)%3,(c+1)%3,(d+2)%3,(d+1)%3
    u,v,wa,wb = rng.sample(range(vc),4)
    for vertex in vertices:
        vertex.position = Vector3(*(rng.uniform(-8,8) for _ in range(3)))
        vertex.edge = rng.randrange(tc)*4+rng.randrange(3)
    for t in (triangles[a],triangles[b]):
        t.surface,t.flags,t.state = rng.randrange(20),rng.randrange(256),rng.randrange(65536)
        for normal in t.normals:
            normal.x,normal.y,normal.z = [rng.uniform(-4,4) for _ in range(3)]
    if case%50 == 0:
        # Exact cancellation and a zero second-side endpoint exercise native
        # zero-length normalization, including the resulting NaN bit patterns.
        triangles[a].normals[ap] = Vector3(1,0,0)
        triangles[a].normals[c] = Vector3(-1,0,0)
        triangles[b].normals[d] = Vector3(0,0,0)
    triangles[a].vertices[ap],triangles[a].vertices[c],triangles[a].vertices[an] = u,v,wa
    triangles[b].vertices[bp],triangles[b].vertices[d],triangles[b].vertices[bn] = v,u,wb
    triangles[a].twins[c],triangles[b].twins[d] = b*4+d,a*4+c
    other = [p for p in range(tc) if p not in (a,b)]
    rng.shuffle(other)
    for p,corner,n in zip((a,a,b,b),(ap,an,bp,bn),other):
        k = rng.randrange(3)
        triangles[p].twins[corner] = n*4+k if rng.random()<0.8 else 0xffffffff
        triangles[n].twins[k] = p*4+corner
    source_count = 5
    source_data,source_sizes = [],[]
    for _ in range(source_count):
        values = [rng.randrange(tc)*4+rng.randrange(3) for _ in range(rng.randrange(20))]
        values += [a*4+an,b*4+bp] if rng.random()<0.7 else []
        source_data.append(values)
        source_sizes.append(len(values))
    for t in (triangles[a],triangles[b]):
        t.edge_source[:] = [rng.randrange(-1,source_count) for _ in range(3)]
    arrays1,arrays2 = [],[]
    source_records = (C.c_byte*(104*source_count)).from_buffer_copy(rng.randbytes(104*source_count))
    sources = (SourceList*source_count)()
    for i,values in enumerate(source_data):
        raw = rng.randbytes(64*4)
        first,second = [(C.c_uint32*64).from_buffer_copy(raw) for _ in range(2)]
        first[:len(values)] = second[:len(values)] = values
        arrays1.append(first); arrays2.append(second)
        start = C.addressof(first)
        struct.pack_into('<QQQ',source_records,i*104+24,start,start+len(values)*4,start+64*4)
        sources[i] = SourceList(second,len(values),64)
    original_source_bytes = bytes(source_records)
    native_sources = NativeVector.of(source_records)
    other_triangles = (Triangle*64).from_buffer_copy(bytes(triangles))
    other_vertices = (Vertex*64).from_buffer_copy(bytes(vertices))
    world = (Vector3*64).from_buffer_copy(rng.randbytes(64*12))
    lookup = (C.c_int32*64).from_buffer_copy(rng.randbytes(64*4))
    native_world = oracle.libc.malloc(vc*16)
    if not native_world: raise MemoryError()
    C.memmove(native_world,C.addressof(world),vc*12)
    C.memmove(native_world+vc*12,C.addressof(lookup),vc*4)
    first_mesh = Mesh(triangles,vertices,native_world,native_world+vc*12,tc,vc,64,64)
    second_mesh = Mesh(other_triangles,other_vertices,C.addressof(world),C.addressof(lookup),tc,vc,64,64)
    matrix = Transform((C.c_float*16)(*(rng.uniform(-2,2) for _ in range(16))))
    for i in (3,7,11): matrix.values[i] = 0
    matrix.values[15] = 1
    brush = rng.randrange(8)
    transform_buffer = oracle.matrix(brush,matrix)
    point = Vector3(*(rng.uniform(-8,8) for _ in range(3)))
    weight = rng.choice((0,1,0.5,0.125,0.25,0.75,0.875,-0.25,1.25,rng.random()))
    if case%50 == 0: weight = 0.5
    native_input,native_output,output = C.c_uint32(a*4+c),C.c_uint32(0x12345678),C.c_uint32(0x12345678)
    saved_world,saved_lookup = bytes(world),bytes(lookup)
    try:
        returned = oracle.split(C.byref(native_output),C.byref(first_mesh),brush,C.byref(native_sources),
                                 C.byref(native_input),C.byref(point),weight)
        result = recovered(C.byref(second_mesh),C.byref(matrix),sources,source_count,a*4+c,C.byref(point),weight,C.byref(output))
        errors = []
        if result or returned != C.addressof(native_output) or output.value != native_output.value: errors.append('return')
        if bytes(triangles) != bytes(other_triangles):
            offsets = [i for i,(x,y) in enumerate(zip(bytes(triangles),bytes(other_triangles))) if x != y]
            errors.append('triangle bytes '+str(offsets[:25]))
        if bytes(vertices) != bytes(other_vertices): errors.append('vertex bytes')
        if (first_mesh.triangle_count,first_mesh.vertex_count) != (second_mesh.triangle_count,second_mesh.vertex_count): errors.append('counts')
        n = vc+1
        if C.string_at(first_mesh.unknown_10,n*12) != bytes(world)[:n*12]: errors.append('world positions')
        if C.string_at(first_mesh.unknown_18,n*4) != bytes(lookup)[:n*4]: errors.append('world lookup')
        if bytes(world)[n*12:] != saved_world[n*12:] or bytes(lookup)[n*4:] != saved_lookup[n*4:]: errors.append('unused world capacity')
        native_sizes = []
        for i in range(source_count):
            start,end,capacity = struct.unpack_from('<QQQ',source_records,i*104+24)
            size = (end-start)//4
            native_sizes.append(size)
            if size != sources[i].count or bytes(arrays1[i]) != bytes(arrays2[i]): errors.append('source list '+str(i))
            before = original_source_bytes[i*104:(i+1)*104]
            after = bytes(source_records)[i*104:(i+1)*104]
            if before[:32] != after[:32] or before[40:] != after[40:]: errors.append('source metadata '+str(i))
        if errors:
            raise AssertionError(f'case {case}, a/c={a}/{c}, b/d={b}/{d}, weight={weight}, rc={result}: {errors}')
        return c,d
    finally:
        oracle.libc.free(first_mesh.unknown_10)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--library',required=True)
    parser.add_argument('--cases',type=int,default=5000)
    parser.add_argument('--seed',type=int,default=1559)
    args = parser.parse_args()
    if args.cases < 1: parser.error('cases must be positive')
    oracle = Oracle()
    lib = C.CDLL(args.library)
    function = lib.rcsg_recovered_split_triangle_edge
    function.argtypes = [C.POINTER(Mesh),C.POINTER(Transform),C.POINTER(SourceList),C.c_int32,C.c_uint32,
                         C.POINTER(Vector3),C.c_float,C.POINTER(C.c_uint32)]
    function.restype = C.c_int32
    rng = random.Random(args.seed)
    pairs = set()
    for case in range(args.cases): pairs.add(compare(oracle,function,rng,case))
    print(json.dumps({'original_sha256':SHA256,'seed':args.seed,'split_cases':args.cases,
                      'corner_pairs':len(pairs),'zero_normal_cases':(args.cases+49)//50,'mismatches':0},indent=2))


if __name__ == '__main__': main()
