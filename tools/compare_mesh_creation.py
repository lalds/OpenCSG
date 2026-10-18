#!/usr/bin/env python3
"""Compare complete original Create with caller-owned portable mesh creation.

Runs unchanged, hash-pinned Mach-O code with host allocator imports. Includes
capacity growth in the original, removed digons and repeated plane splits.
Portable allocation policy/capacities are deliberately not compared.
"""
import argparse
import ctypes as C
import json
import random
import struct

from compare_find_connections import original
from compare_recovered_geometry import Edge,EdgeLink,Plane,Polygon,Surfaces,Vector3,SHA256
from compare_recovered_splitting import Fixture,Memory,cube_inputs,polygon_inputs
from compare_recovered_triangulation import Mesh,Triangle,Vertex


def add_digon(fixture, edge, count=2):
    m = fixture.memory
    twin = fixture.edges[edge].twin_edge
    a,b,p = m.edge_count,m.edge_count+1,m.polygon_count
    fixture.polygons[p] = Polygon(count,a,0,17,0x9234,0xabcd)
    fixture.edges[a] = Edge(b,fixture.edges[twin].vertex_index,edge,p)
    fixture.edges[b] = Edge(a,fixture.edges[edge].vertex_index,twin,p)
    fixture.edges[edge].twin_edge = a; fixture.edges[twin].twin_edge = b
    fixture.links[a] = EdgeLink(b,51); fixture.links[b] = EdgeLink(a,52)
    m.polygon_count += 1; m.edge_count += 2


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--library',required=True); parser.add_argument('--cases',type=int,default=5000)
    parser.add_argument('--seed',type=int,default=1559); args = parser.parse_args()
    if args.cases < 1: parser.error('cases must be positive')
    oracle = original(0x30810)
    # Create uses both operator new/delete and malloc/free. Track all buffers,
    # poison newly allocated bytes, and check their release after each case.
    for offset,callback in ((0x40180,oracle.allocate),(0x40168,oracle.delete)):
        struct.pack_into('<Q',oracle.memory,offset,C.cast(callback,C.c_void_p).value)
    create = C.CFUNCTYPE(C.POINTER(Mesh),C.POINTER(Surfaces),C.POINTER(Memory))(oracle.address+0x30810)
    lib = C.CDLL(args.library)
    recovered = lib.rcsg_recovered_create_triangle_mesh
    recovered.argtypes = [C.POINTER(Mesh),C.POINTER(Memory),C.POINTER(Surfaces),C.c_int32]
    recovered.restype = C.c_int32
    split = lib.rcsg_recovered_split_plane
    split.argtypes = [C.POINTER(Memory),C.POINTER(Surfaces),C.c_int32,C.POINTER(Plane),C.POINTER(C.c_int32)]
    split.restype = C.c_uint8
    rng = random.Random(args.seed)
    total = digons = split_calls = growth_cases = open_cases = empty_cases = singletons = 0
    for case in range(args.cases):
        inputs = cube_inputs(rng) if case%3 == 0 else polygon_inputs(rng.randrange(3,25),rng.uniform(.01,10),rng.uniform(-2,2))
        a,b = [Fixture(*inputs) for _ in range(2)]
        if case%4 == 0:
            for step in range(rng.randrange(1,5)):
                p = rng.randrange(a.memory.polygon_count); output = C.c_int32(-1)
                plane = Plane(Vector3(*[rng.uniform(-1,1) for _ in range(3)]),rng.uniform(-2,2))
                rc = split(C.byref(a.memory),C.byref(a.surfaces),p,C.byref(plane),C.byref(output))
                if rc == 255: raise AssertionError(('split setup failed',case,step))
                split_calls += 1
        if case%5 in (1,2):
            for _ in range(rng.randrange(1,5)):
                add_digon(a,rng.randrange(a.memory.edge_count),rng.choice([0,2]))
                digons += 1
        if case%13 == 0:
            # Open contours are accepted by native: negative twins are skipped
            # when recording neighbors. Avoid digons which need real twins.
            for e in range(a.memory.edge_count): a.edges[e].twin_edge = -1
            open_cases += 1
        if case%13 == 0 and case%5 in (1,2):
            # Removing a digon would dereference its negative twins. Use a
            # fresh open fixture instead of executing this invalid topology.
            a = Fixture(*inputs)
            for e in range(a.memory.edge_count): a.edges[e].twin_edge = -1
        if case%17 == 0:
            a.memory.polygon_count = 0
            if case%34 == 0: a.memory.vertex_count = a.memory.edge_count = 0
            empty_cases += 1
        if case%7 == 1:
            # A self-twin one-edge loop tests the <=2 branch without dangling
            # pointers. This is a synthetic indexed input, not a valid solid.
            e,p = a.memory.edge_count,a.memory.polygon_count
            a.polygons[p] = Polygon(1,e,0,0,0,0)
            a.edges[e] = Edge(e,0,e,p); a.links[e] = EdgeLink(e,-1)
            a.memory.edge_count += 1; a.memory.polygon_count += 1; singletons += 1
        for p in range(a.memory.polygon_count):
            a.polygons[p].unknown_10 = rng.randrange(65536)
            a.polygons[p].unknown_12 = rng.randrange(65536)
            if a.polygons[p].edge_count >= 3:
                for _ in range(rng.randrange(a.polygons[p].edge_count)):
                    a.polygons[p].first_edge = a.edges[a.polygons[p].first_edge].next_edge
        for e in range(a.memory.edge_count): a.links[e].unknown_04 = rng.randrange(-1,25)
        for array_a,array_b in zip((a.polygons,a.edges,a.links,a.vertices),(b.polygons,b.edges,b.links,b.vertices)):
            C.memmove(array_b,array_a,C.sizeof(array_a))
        for name,_ in Memory._fields_[4:]: setattr(b.memory,name,getattr(a.memory,name))
        # Arbitrary normal bits verify pure copying, including NaNs.
        C.memmove(a.planes,rng.randbytes(C.sizeof(a.planes)),C.sizeof(a.planes))
        C.memmove(b.planes,a.planes,C.sizeof(a.planes))
        triangles = (Triangle*512)(); vertices = (Vertex*128)()
        C.memset(triangles,0xcc,C.sizeof(triangles)); C.memset(vertices,0xcc,C.sizeof(vertices))
        mesh = Mesh(triangles,vertices,None,None,0,0,512,128)
        try:
            expected = create(C.byref(a.surfaces),C.byref(a.memory)).contents
            if oracle.errors: raise AssertionError(oracle.errors)
            rc = recovered(C.byref(mesh),C.byref(b.memory),C.byref(b.surfaces),len(b.planes))
            if rc: raise AssertionError(('portable Create failed',case,rc))
            tc,vc = expected.triangle_count,expected.vertex_count
            if (mesh.triangle_count,mesh.vertex_count) != (tc,vc): raise AssertionError(('counts',case))
            if C.string_at(expected.triangles,tc*116) != C.string_at(triangles,tc*116):
                raw = C.string_at(expected.triangles,tc*116); actual = bytes(triangles)[:tc*116]
                first = next(i for i,(x,y) in enumerate(zip(raw,actual)) if x != y)
                raise AssertionError(('triangles',case,first//116,first%116,raw[first],actual[first]))
            if C.string_at(expected.vertices,vc*16) != C.string_at(vertices,vc*16): raise AssertionError(('vertices',case))
            if a.snapshot() != b.snapshot(): raise AssertionError(('source mutation',case))
            if bytes(triangles)[tc*116:] != b'\xcc'*((512-tc)*116): raise AssertionError(('unused triangles',case))
            if bytes(vertices)[vc*16:] != b'\xcc'*((128-vc)*16): raise AssertionError(('unused vertices',case))
            if expected.unknown_10 or expected.unknown_18: raise AssertionError('Unexpected native world buffers')
            total += tc
            growth_cases += expected.triangle_capacity > max(0,sum(a.polygons[p].edge_count-2 for p in range(a.memory.polygon_count)))
        finally:
            for pointer in list(oracle.allocations): oracle.delete(pointer)
            if oracle.errors: raise AssertionError(oracle.errors)
            oracle.freed.clear()
    print(json.dumps({'original_sha256':SHA256,'seed':args.seed,'cases':args.cases,'triangles':total,
        'inserted_digons_in_setup':digons,'synthetic_singletons':singletons,
        'split_setup_calls':split_calls,'native_capacity_growth_cases':growth_cases,
        'open_cases':open_cases,'empty_cases':empty_cases,'mesh_and_source_mismatches':0},indent=2))


if __name__ == '__main__': main()
