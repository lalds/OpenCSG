#!/usr/bin/env python3
"""Compare complete FixGaps traversal on three independently subdivided groups.

Executes unpatched hash-pinned native code. Supplies native manager metadata,
source records and connected BrushEdges, including skipped nodes/connections.
No brush-edge discovery, Unity runtime or native capacity-growth paths.
"""
import argparse
import ctypes as C
import json
import random
import struct

from compare_gap_alignment import Body, moments_and_topology
from compare_gap_neighbors import View
from compare_recovered_geometry import SHA256, Vector3, Plane
from compare_recovered_gap_intervals import Oracle, Line, set_vector
from compare_recovered_triangulation import Mesh, NativeVector
from compare_recovered_triangle_splitting import Transform, SourceList


class Ref(C.Structure):
    _fields_ = [('brush',C.c_int32),('edge',C.c_uint32)]


class Connections(C.Structure):
    _fields_ = [('data',C.POINTER(Ref)),('count',C.c_int32)]


class Node(C.Structure):
    _fields_ = [('brush',View),('sources',C.POINTER(SourceList)),('lines',C.POINTER(Line)),
                ('count',C.c_int32),('kind',C.c_int32),('valid_record',C.c_int32)]


class MultiBody(Body):
    def __init__(self):
        super().__init__()
        self.edge_buffers = [(C.c_uint32*64)() for _ in range(3)]
        self.sources = (SourceList*3)(*[SourceList(e,0,64) for e in self.edge_buffers])
        self.lines = (Line*3)(*[Line(Vector3(*[float(k == j) for k in range(3)]),Vector3(0,0,0)) for j in range(3)])
        for p,t in enumerate(self.triangles[:4]):
            for c in range(3):
                a,b = t.vertices[(c+2)%3],t.vertices[c]
                group = max(a,b)-1 if min(a,b) == 0 else -1
                t.edge_source[c] = group
                if group >= 0:
                    s = self.sources[group]; s.edges[s.count] = p*4+c; s.count += 1


def native_fix(oracle,bodies,links,modes):
    size = len(bodies)
    edge_records = [(C.c_byte*(104*3))() for _ in bodies]
    connected = []
    for i,body in enumerate(bodies):
        for g in range(3):
            s = body.sources[g]; start = C.addressof(body.edge_buffers[g])
            struct.pack_into('<QQQ',edge_records[i],104*g+24,start,start+s.count*4,start+64*4)
            C.memmove(C.addressof(edge_records[i])+104*g+80,C.addressof(body.lines[g]),24)
            refs = (Ref*len(links[g]))(*[Ref(*ref) for ref in links[g]]) if i == 0 else (Ref*0)()
            connected.append(refs); set_vector(edge_records[i],104*g+48,refs)
    nodes = [(C.c_byte*224)() for _ in bodies]
    records = (C.c_byte*(104*size))(); metadata = (C.c_byte*(4*size))()
    for i,(body,node,mode) in enumerate(zip(bodies,nodes,modes)):
        metadata[4*i+3] = 1 if mode == 'kind' else 3
        struct.pack_into('<i',node,24,i+100 if mode == 'index' else i)
        struct.pack_into('<Q',node,96,0 if mode == 'mesh' else C.addressof(body.mesh))
        set_vector(node,160,edge_records[i]); struct.pack_into('<i',node,216,-1 if mode == 'record' else i)
        struct.pack_into('<Q',records,104*i+32,C.addressof(body.planes))
        struct.pack_into('<i',records,104*i+40,0 if mode == 'planes' else 4)
    pointers = (C.c_void_p*size)(*[None if mode == 'absent' else C.addressof(node) for node,mode in zip(nodes,modes)])
    struct.pack_into('<Q',oracle.memory,0x404b0,C.addressof(metadata))
    set_vector(oracle.memory,0x404f8,pointers); set_vector(oracle.memory,0x40670,records)
    matrices = (C.c_byte*(256*size))()
    for i,body in enumerate(bodies): C.memmove(C.addressof(matrices)+i*256+64,C.addressof(body.matrix),64)
    struct.pack_into('<Q',oracle.memory,0x404c8,C.addressof(matrices))
    local = bodies[0]; surfaces = (C.c_byte*44)()
    struct.pack_into('<Q',surfaces,32,C.addressof(local.planes)); struct.pack_into('<i',surfaces,40,4)
    vc = local.mesh.vertex_count; block = oracle.libc.malloc(vc*16)
    if not block: raise MemoryError()
    C.memmove(block,local.world,vc*12); C.memmove(block+vc*12,local.lookup,vc*4)
    local.mesh.unknown_10,local.mesh.unknown_18 = block,block+vc*12
    vector = NativeVector.of(edge_records[0])
    try:
        oracle.freed.clear(); oracle.errors.clear()
        oracle.fix(C.byref(local.mesh),0,C.byref(surfaces),C.byref(vector))
        if oracle.errors or oracle.allocations: raise AssertionError((oracle.errors,oracle.allocations))
        for g in range(3):
            start,end,_ = struct.unpack_from('<QQQ',edge_records[0],104*g+24)
            local.sources[g].count = (end-start)//4
        C.memmove(local.world,local.mesh.unknown_10,local.mesh.vertex_count*12)
        C.memmove(local.lookup,local.mesh.unknown_18,local.mesh.vertex_count*4)
    finally:
        oracle.libc.free(local.mesh.unknown_10)
        local.mesh.unknown_10,local.mesh.unknown_18 = C.addressof(local.world),C.addressof(local.lookup)


def snapshot(body):
    return tuple(bytes(getattr(body,attr)) for attr in ('triangles','vertices','world','lookup'))+tuple(bytes(e) for e in body.edge_buffers)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--library',required=True); p.add_argument('--cases',type=int,default=2000)
    p.add_argument('--seed',type=int,default=1559); args = p.parse_args()
    if args.cases < 1: p.error('cases must be positive')
    lib = C.CDLL(args.library); oracle = Oracle(include_splitting=True)
    split = lib.rcsg_recovered_split_triangle_edge
    split.argtypes = [C.POINTER(Mesh),C.POINTER(Transform),C.POINTER(SourceList),C.c_int32,C.c_uint32,
        C.POINTER(Vector3),C.c_float,C.POINTER(C.c_uint32)]; split.restype = C.c_int32
    fix = lib.rcsg_recovered_fix_gaps
    fix.argtypes = [C.POINTER(Mesh),C.POINTER(Plane),C.c_int32,C.POINTER(Transform),C.POINTER(SourceList),
        C.c_int32,C.POINTER(Line),C.POINTER(Connections),C.POINTER(Node),C.c_int32,C.c_int32]; fix.restype = C.c_int32
    rng = random.Random(args.seed); total = refs = 0; mode_counts = {}
    # Use exact float neighbors of the native direction-alignment constant.
    threshold_bits = struct.unpack('<I',struct.pack('<f',0.9999899864196777))[0]
    scales = [1.,-1.,0.,float('nan')]+[struct.unpack('<f',struct.pack('<I',threshold_bits+k))[0] for k in (-1,0,1)]
    for case in range(args.cases):
        bodies = [MultiBody() for _ in range(rng.randrange(2,5))]
        modes = ['valid']+[rng.choice(['valid']*5+['absent','kind','index','mesh','record','planes']) for _ in bodies[1:]]
        for mode in modes[1:]: mode_counts[mode] = mode_counts.get(mode,0)+1
        for i,body in enumerate(bodies):
            for g in range(3):
                for x in sorted(rng.sample(range(1,16),rng.randrange(3))):
                    x /= 8
                    def coordinate(v): return (body.world[v].x,body.world[v].y,body.world[v].z)[g]
                    e = next(e for e in body.sources[g].edges[:body.sources[g].count]
                        if coordinate(body.triangles[e>>2].vertices[((e&3)+2)%3]) < x < coordinate(body.triangles[e>>2].vertices[e&3]))
                    point = Vector3(*[x if k == g else 0 for k in range(3)]); output = C.c_uint32()
                    if split(C.byref(body.mesh),C.byref(body.matrix),body.sources,3,e,C.byref(point),.5,C.byref(output)):
                        raise AssertionError('Fixture split failed')
                s = body.sources[g]; handles = list(s.edges[:s.count]); rng.shuffle(handles)
                for j,e in enumerate(handles): s.edges[j] = e
                if i:
                    scale = rng.choice(scales)
                    body.lines[g].direction = Vector3(*[scale if k == g else 0 for k in range(3)])
            for t in body.triangles[:body.mesh.triangle_count]:
                t.flags = rng.randrange(256)
                for c in range(3): struct.pack_into('<iI',t,72+8*c,99,0x12345678)
            for plane in body.planes: plane.normal = Vector3(*[rng.uniform(-2,2) for _ in range(3)])
        links = []
        for g in range(3):
            group = [(i,g) for i in range(1,len(bodies))]
            group += [(rng.randrange(1,len(bodies)),rng.randrange(3)) for _ in range(rng.randrange(4))]
            group += [(-1,0),(len(bodies)+7,0)]
            rng.shuffle(group); links.append(group); refs += len(group)
        local = bodies[0]; recovered = MultiBody()
        for attr in ('triangles','vertices','world','lookup','planes'):
            C.memmove(getattr(recovered,attr),getattr(local,attr),C.sizeof(getattr(local,attr)))
        for g in range(3):
            C.memmove(recovered.edge_buffers[g],local.edge_buffers[g],C.sizeof(local.edge_buffers[g]))
            recovered.sources[g].count = local.sources[g].count
        recovered.mesh.vertex_count,recovered.mesh.triangle_count = local.mesh.vertex_count,local.mesh.triangle_count
        old_count = local.mesh.vertex_count; before_foreign = [snapshot(b) for b in bodies[1:]]
        native_fix(oracle,bodies,links,modes)
        nodes = []
        for i,(body,mode) in enumerate(zip([recovered]+bodies[1:],modes)):
            view = View(C.pointer(body.mesh) if mode != 'mesh' else None,body.planes,
                        i+100 if mode in ('index','absent') else i,0 if mode == 'planes' else 4)
            nodes.append(Node(view,body.sources,body.lines,3,1 if mode == 'kind' else 3,int(mode != 'record')))
        nodes = (Node*len(nodes))(*nodes)
        arrays = [(Ref*len(g))(*[Ref(*ref) for ref in g]) for g in links]
        connections = (Connections*3)(*[Connections(a,len(a)) for a in arrays])
        result = fix(C.byref(recovered.mesh),recovered.planes,4,C.byref(recovered.matrix),recovered.sources,3,
                     recovered.lines,connections,nodes,len(nodes),64)
        if result < 0: raise AssertionError(('dispatch failed',case,result,modes))
        if snapshot(local) != snapshot(recovered):
            for n,(a,b) in enumerate(zip(snapshot(local),snapshot(recovered))):
                if a != b:
                    at = next(i for i,(x,y) in enumerate(zip(a,b)) if x != y)
                    raise AssertionError(('buffer mismatch',case,n,at,modes))
        if (local.mesh.vertex_count,local.mesh.triangle_count,[s.count for s in local.sources]) != (
            recovered.mesh.vertex_count,recovered.mesh.triangle_count,[s.count for s in recovered.sources]):
            raise AssertionError(('count mismatch',case))
        if result != local.mesh.vertex_count-old_count: raise AssertionError('Wrong split count')
        if before_foreign != [snapshot(b) for b in bodies[1:]]: raise AssertionError('Foreign buffers changed')
        moments_and_topology(local); moments_and_topology(recovered); total += result
    print(json.dumps({'original_sha256':SHA256,'seed':args.seed,'multi_group_cases':args.cases,
        'source_groups_per_brush':3,'connection_visits':refs,'foreign_node_modes':mode_counts,
        'new_vertices_per_implementation':total,'complete_mesh_buffer_mismatches':0,
        'closed_twins_volume_area_mismatches':0},indent=2))


if __name__ == '__main__': main()
