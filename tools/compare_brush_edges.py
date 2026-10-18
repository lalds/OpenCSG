#!/usr/bin/env python3
"""Compare recovered FindBrushEdges with unchanged original machine code.

The internal compiler-specialized ABI passes the source normal in xmm0..2;
the symbol's original C++ signature alone does not describe that call ABI.
"""
import argparse
import ctypes as C
import json
import mmap
import random
import struct

from compare_recovered_geometry import BINARY,SHA256,Plane,Vector3
from compare_recovered_splitting import Fixture,Memory
from compare_recovered_triangulation import NativeVector
from compare_recovered_triangle_splitting import Transform
from compare_recovered_gap_intervals import Oracle
from compare_fix_gaps import Ref


class Bounds(C.Structure):
    _fields_ = [(name,C.c_float) for name in ('min_x','max_x','min_y','max_y','min_z','max_z')]


class Group(C.Structure):
    _fields_ = [('refs',C.POINTER(Ref)),('count',C.c_int32),('capacity',C.c_int32),
                ('a',C.c_int32),('b',C.c_int32),('direction',Vector3),('origin',Vector3)]


NATIVE = C.CFUNCTYPE(None,C.c_int32,C.POINTER(Memory),C.c_int32,C.POINTER(NativeVector),Bounds,
                     C.c_int32,C.c_int32,C.POINTER(NativeVector),C.c_float,C.c_float,C.c_float)


def original():
    oracle = Oracle(include_splitting=True); data = BINARY.read_bytes()
    ranges = ((0x1b90,0x1bf6),(0x4b12,0x4d2c),(0x20b53,0x21c32))
    pages = {p for a,b in ranges for p in range(a//4096,(b-1)//4096+1)}
    for p in pages:
        if oracle.libc.mprotect(oracle.address+p*4096,4096,mmap.PROT_READ | mmap.PROT_WRITE): raise OSError('mprotect load')
    for a,b in ranges: oracle.memory[a:b] = data[a:b]
    for p in pages:
        if oracle.libc.mprotect(oracle.address+p*4096,4096,mmap.PROT_READ | mmap.PROT_EXEC): raise OSError('mprotect execute')
    struct.pack_into('<Q',oracle.memory,0x40190,C.cast(oracle.libc.memmove,C.c_void_p).value)
    oracle.find = NATIVE(oracle.address+0x20b53)
    return oracle


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--library',required=True); p.add_argument('--cases',type=int,default=5000)
    p.add_argument('--seed',type=int,default=1559); args = p.parse_args()
    if args.cases < 1: p.error('cases must be positive')
    oracle = original(); lib = C.CDLL(args.library)
    recovered = lib.rcsg_recovered_find_brush_edges
    recovered.argtypes = [C.POINTER(Memory),C.c_int32,C.POINTER(Plane),C.POINTER(Plane),C.c_int32,
        C.POINTER(Bounds),C.POINTER(Transform),C.c_int32,C.c_int32,C.POINTER(Group),C.POINTER(C.c_int32),C.c_int32]
    recovered.restype = C.c_int32
    rng = random.Random(args.seed); calls = groups_total = refs_total = 0
    for case in range(args.cases):
        count = rng.randrange(2,10)
        points = [(x/8,0.,0.) for x in sorted(rng.sample(range(1,16),count))]
        points += [(2.,0.,0.),(2.,1.,0.),(0.,1.,0.),(0.,0.,0.)]
        faces = [list(range(len(points))),list(reversed(range(len(points))))]
        local_planes = [Plane(Vector3(0,0,1),0),Plane(Vector3(0,0,-1),0)]
        a,b = [Fixture(points,faces,local_planes) for _ in range(2)]
        buffers = [(Ref*64)() for _ in range(64)]
        groups = (Group*64)(*[Group(buf,0,64,0,0,Vector3(),Vector3()) for buf in buffers]); group_count = C.c_int32()
        native_groups = (C.c_byte*(104*64))(); native_vector = NativeVector(C.addressof(native_groups),C.addressof(native_groups),C.addressof(native_groups)+C.sizeof(native_groups))
        matrix = Transform((C.c_float*16)(*[rng.uniform(-2,2) for _ in range(16)]))
        matrices = (C.c_byte*256)(); C.memmove(matrices,C.byref(matrix),64)
        struct.pack_into('<Q',oracle.memory,0x404c8,C.addressof(matrices))
        try:
            for step in range(rng.randrange(2,7)):
                existing = (native_vector.end-native_vector.begin)//104
                if existing and rng.randrange(2):
                    e = rng.randrange(a.memory.edge_count); twin = a.edges[e].twin_edge
                    ids = [rng.randrange(-1,existing),rng.randrange(-1,existing)]
                    for fixture in (a,b):
                        fixture.links[e].unknown_04 = ids[0]
                        fixture.links[twin].unknown_04 = ids[1]
                polygon = rng.randrange(2); local = local_planes[polygon]
                if case%13 == 0: local = Plane(Vector3(0,1,0),0) # Parallel early return.
                jitter = rng.choice([0.,0.,0.,0.00059,-0.00059,0.0006,-0.0006,0.001])
                planes = (Plane*3)(Plane(Vector3(0,-1,0),jitter),Plane(Vector3(0,1,0),1+jitter),
                    Plane(Vector3(1,0,0),rng.choice([3.,3.,0.5])))
                bounds = Bounds(rng.choice([-3.,0.,0.5]),3.,-2.,2.,-2.,2.)
                brush = rng.randrange(1,5); surface = rng.randrange(2)
                pv = NativeVector.of(planes)
                oracle.find(0,C.byref(a.memory),polygon,C.byref(pv),bounds,brush,surface,C.byref(native_vector),
                            local.normal.x,local.normal.y,local.normal.z)
                rc = recovered(C.byref(b.memory),polygon,C.byref(local),planes,3,C.byref(bounds),C.byref(matrix),
                               brush,surface,groups,C.byref(group_count),64)
                if rc: raise AssertionError(('recovered error',case,step,rc))
                native_count = (native_vector.end-native_vector.begin)//104
                if group_count.value != native_count: raise AssertionError(('group count',case,step,native_count,group_count.value))
                if a.snapshot() != b.snapshot(): raise AssertionError(('mesh buffer mismatch',case,step))
                for g in range(native_count):
                    start,end,_ = struct.unpack_from('<QQQ',native_groups,g*104)
                    expected = C.string_at(start,end-start) if end != start else b''
                    actual = C.string_at(groups[g].refs,groups[g].count*8)
                    fields = C.string_at(C.addressof(native_groups)+g*104+72,32)
                    if fields != C.string_at(C.addressof(groups[g])+16,32) or expected != actual:
                        raise AssertionError(('group mismatch',case,step,g,fields.hex(),C.string_at(C.addressof(groups[g])+16,32).hex()))
                    for offset in (24,48):
                        if struct.unpack_from('<QQQ',native_groups,g*104+offset) != (0,0,0): raise AssertionError('Unexpected triangle/connection writes')
                calls += 1
            groups_total += group_count.value
            refs_total += sum(groups[g].count for g in range(group_count.value))
        finally:
            for address in list(oracle.allocations): oracle.delete(address)
            if oracle.errors: raise AssertionError(oracle.errors)
            oracle.freed.clear()
    print(json.dumps({'original_sha256':SHA256,'seed':args.seed,'mesh_cases':args.cases,'original_calls':calls,
        'final_groups':groups_total,'final_surface_refs':refs_total,'mesh_and_group_mismatches':0},indent=2))


if __name__ == '__main__': main()
