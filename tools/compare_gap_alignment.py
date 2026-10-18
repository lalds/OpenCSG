#!/usr/bin/env python3
"""Compare scoped endpoint alignment with an actual full original FixGaps call.

Controlled single-line groups on closed tetrahedra, known source connections,
identity transforms. Compare resulting segment partitions, counts, volume, area
and closed local twins. Cross-brush link selection and normal arbitration are
deliberately NOT asserted equivalent to the new alignment-only adapter.
"""

import argparse
import ctypes as C
import json
import math
import random
import struct

from compare_recovered_geometry import SHA256,Vector3,Plane
from compare_recovered_triangulation import Mesh,Triangle,Vertex,NativeVector
from compare_recovered_triangle_splitting import SourceList,Transform
from compare_recovered_gap_intervals import Oracle,Line,set_vector


class Body:
    def __init__(self):
        self.triangles,self.vertices = (Triangle*64)(),(Vertex*64)()
        self.world,self.lookup,self.edges = (Vector3*64)(),(C.c_int32*64)(),(C.c_uint32*64)()
        self.world[:4] = [Vector3(0,0,0),Vector3(2,0,0),Vector3(0,2,0),Vector3(0,0,2)]
        self.mesh = Mesh(self.triangles,self.vertices,C.addressof(self.world),C.addressof(self.lookup),4,4,64,64)
        self.matrix = Transform((C.c_float*16)(1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1))
        self.source = SourceList(self.edges,0,64)
        self.line = Line(Vector3(1,0,0),Vector3(0,0,0))
        faces = ((0,2,1),(0,1,3),(0,3,2),(1,2,3))
        n = 1/math.sqrt(3)
        self.planes = (Plane*4)(Plane(Vector3(0,0,-1),0),Plane(Vector3(0,-1,0),0),
                                Plane(Vector3(-1,0,0),0),Plane(Vector3(n,n,n),2*n))
        for v in range(4): self.vertices[v] = Vertex(0xffffffff,self.world[v])
        for p,face in enumerate(faces):
            t = self.triangles[p]
            t.vertices[:] = face; t.surface = p; t.flags = 9; t.state = 2
            for c in range(3):
                start,end = face[(c+2)%3],face[c]
                t.normals[c] = self.planes[p].normal
                t.edge_source[c] = 0 if {start,end} == {0,1} else -1
                if t.edge_source[c] == 0:
                    self.edges[self.source.count] = p*4+c; self.source.count += 1
                self.vertices[end].edge = p*4+c
                struct.pack_into('<iI',t,72+8*c,-1,0xffffffff)
                for q,other in enumerate(faces):
                    for d in range(3):
                        if other[d] == start and other[(d+2)%3] == end: t.twins[c] = q*4+d


def segments(body):
    result = set()
    for edge in body.edges[:body.source.count]:
        t = body.triangles[edge>>2]; c = edge&3
        u,v = t.vertices[(c+2)%3],t.vertices[c]
        world = C.cast(body.mesh.unknown_10,C.POINTER(Vector3))
        result.add(tuple(sorted((world[u].x,world[v].x))))
    return sorted(result)


def moments_and_topology(body):
    area = volume = 0.0
    for p,t in enumerate(body.triangles[:body.mesh.triangle_count]):
        for c,twin in enumerate(t.twins):
            if twin&3 == 3 or twin>>2 >= body.mesh.triangle_count: raise AssertionError('Missing closed twin')
            other = body.triangles[twin>>2]; d = twin&3
            if other.twins[d] != p*4+c or t.vertices[c] != other.vertices[(d+2)%3] or t.vertices[(c+2)%3] != other.vertices[d]:
                raise AssertionError('Broken reciprocal triangle edge')
        a,b,c = [body.vertices[i].position for i in t.vertices]
        x = (b.y-a.y)*(c.z-a.z)-(b.z-a.z)*(c.y-a.y)
        y = (b.z-a.z)*(c.x-a.x)-(b.x-a.x)*(c.z-a.z)
        z = (b.x-a.x)*(c.y-a.y)-(b.y-a.y)*(c.x-a.x)
        area += math.sqrt(x*x+y*y+z*z)/2
        volume += (a.x*x+a.y*y+a.z*z)/6
    if abs(volume-4/3)>1e-6 or abs(area-(6+2*math.sqrt(3)))>1e-6: raise AssertionError('Volume/area changed')


def original_fix(oracle,local,foreign):
    # A list supplies multiple connected brushes in deterministic native order.
    foreign = foreign if isinstance(foreign,list) else [foreign]
    bodies = [local]+foreign
    size = len(bodies)
    edge_records = [(C.c_byte*104)() for _ in bodies]
    source = edge_records[0]
    def edge_vector(record,body):
        start = C.addressof(body.edges)
        struct.pack_into('<QQQ',record,24,start,start+body.source.count*4,start+64*4)
        C.memmove(C.addressof(record)+80,C.addressof(body.line),24)
    for record,body in zip(edge_records,bodies): edge_vector(record,body)
    connected = (C.c_int32*(2*len(foreign)))(*[v for i in range(1,size) for v in (i,0)])
    set_vector(source,48,connected)
    nodes = [(C.c_byte*224)() for _ in bodies]
    records = (C.c_byte*(104*size))()
    for i,(body,node,edge_record) in enumerate(zip(bodies,nodes,edge_records)):
        struct.pack_into('<i',node,24,i); struct.pack_into('<Q',node,96,C.addressof(body.mesh))
        set_vector(node,160,edge_record); struct.pack_into('<i',node,216,i)
        struct.pack_into('<Q',records,i*104+32,C.addressof(body.planes)); struct.pack_into('<i',records,i*104+40,4)
    pointers = (C.c_void_p*size)(*[C.addressof(n) for n in nodes])
    metadata = (C.c_byte*(4*size))()
    for i in range(size): metadata[4*i+3] = 3
    struct.pack_into('<Q',oracle.memory,0x404b0,C.addressof(metadata))
    set_vector(oracle.memory,0x404f8,pointers); set_vector(oracle.memory,0x40670,records)
    matrices = (C.c_byte*(256*size))()
    for i,body in enumerate(bodies): C.memmove(C.addressof(matrices)+i*256+64,C.addressof(body.matrix),64)
    struct.pack_into('<Q',oracle.memory,0x404c8,C.addressof(matrices))
    surfaces = (C.c_byte*44)()
    struct.pack_into('<Q',surfaces,32,C.addressof(local.planes)); struct.pack_into('<i',surfaces,40,4)
    vc = local.mesh.vertex_count
    block = oracle.libc.malloc(vc*16)
    if not block: raise MemoryError()
    C.memmove(block,C.addressof(local.world),vc*12); C.memmove(block+vc*12,C.addressof(local.lookup),vc*4)
    local.mesh.unknown_10,local.mesh.unknown_18 = block,block+vc*12
    source_vector = NativeVector.of(source)
    try:
        oracle.freed.clear(); oracle.errors.clear()
        oracle.fix(C.byref(local.mesh),0,C.byref(surfaces),C.byref(source_vector))
        if oracle.errors or oracle.allocations: raise AssertionError((oracle.errors,oracle.allocations))
        start,end,_ = struct.unpack_from('<QQQ',source,24)
        local.source.count = (end-start)//4
        world = C.cast(local.mesh.unknown_10,C.POINTER(Vector3))
        lookup = C.cast(local.mesh.unknown_18,C.POINTER(C.c_int32))
        for v in range(local.mesh.vertex_count): local.world[v],local.lookup[v] = world[v],lookup[v]
    finally:
        oracle.libc.free(local.mesh.unknown_10)
        local.mesh.unknown_10,local.mesh.unknown_18 = C.addressof(local.world),C.addressof(local.lookup)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--library',required=True); parser.add_argument('--cases',type=int,default=200)
    parser.add_argument('--seed',type=int,default=1559); args = parser.parse_args()
    if args.cases < 1: parser.error('cases must be positive')
    oracle = Oracle(include_splitting=True); lib = C.CDLL(args.library)
    split = lib.rcsg_recovered_split_triangle_edge
    split.argtypes = [C.POINTER(Mesh),C.POINTER(Transform),C.POINTER(SourceList),C.c_int32,C.c_uint32,
                      C.POINTER(Vector3),C.c_float,C.POINTER(C.c_uint32)]; split.restype = C.c_int32
    align = lib.rcsg_recovered_align_gap_intervals
    align.argtypes = [C.POINTER(Mesh),C.POINTER(Transform),C.POINTER(SourceList),C.c_int32,C.c_int32,
                      C.POINTER(Line),C.POINTER(Mesh),C.POINTER(C.c_uint32),C.c_int32,C.c_int32]; align.restype = C.c_int32
    rng = random.Random(args.seed); total = 0
    for case in range(args.cases):
        cuts = sorted(x/64 for x in rng.sample(range(1,128),rng.randrange(1,9)))
        local,foreign,recovered = Body(),Body(),Body()
        edge = 5
        for x in cuts:
            output = C.c_uint32(); point = Vector3(x,0,0)
            if split(C.byref(foreign.mesh),C.byref(foreign.matrix),C.byref(foreign.source),1,edge,C.byref(point),0.5,C.byref(output)):
                raise AssertionError('Cannot prepare foreign segmentation')
            edge = output.value
        original_fix(oracle,local,foreign)
        count = align(C.byref(recovered.mesh),C.byref(recovered.matrix),C.byref(recovered.source),1,0,C.byref(recovered.line),
                      C.byref(foreign.mesh),foreign.edges,foreign.source.count,64)
        expected = list(zip([0]+cuts,cuts+[2]))
        if count != len(cuts) or segments(local) != expected or segments(recovered) != expected:
            raise AssertionError(f'case {case}, cuts {cuts}, rc {count}: native {segments(local)}, recovered {segments(recovered)}')
        if (local.mesh.vertex_count,local.mesh.triangle_count) != (recovered.mesh.vertex_count,recovered.mesh.triangle_count):
            raise AssertionError('Native/adapter counts differ')
        moments_and_topology(local); moments_and_topology(recovered)
        total += count
    print(json.dumps({'original_sha256':SHA256,'seed':args.seed,'scoped_fix_gap_cases':args.cases,
                      'new_vertices_per_implementation':total,'segment_count_volume_area_twins_mismatches':0,
                      'cross_brush_link_and_normal_arbitration_equivalence':'not asserted'},indent=2))


if __name__ == '__main__': main()
