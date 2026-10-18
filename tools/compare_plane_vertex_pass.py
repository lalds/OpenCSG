#!/usr/bin/env python3
"""Compare the full first Optimize pass, including sequential removals.

Original traversal/removal/triangulation/compaction code is unchanged. Its
external libc++ signed-int sorting import is provided by a host callback.
"""
import argparse
import ctypes as C
import json
import math
import random
import struct

from compare_find_connections import original,Output
from compare_brush_edge_cleanup import prepare_groups,compare_groups
from compare_brush_edges import Group
from compare_recovered_geometry import SHA256,Plane,Surfaces,Vector3
from compare_recovered_triangulation import Mesh,Triangle,Vertex,NativeVector
from compare_recovered_triangle_splitting import SourceList
from compare_plane_vertex_removal import Context,ContextView
from compare_recovered_mesh_cleanup import SORT


def geometry(rng,case,gc):
    points,faces,adjacency = [],[],[]
    local_normals = [(0,0,1),(0,0,1)]
    local_distances = [0.,0.]
    def face(vertices,surface=0):
        faces.append((vertices,surface)); adjacency.append([0xffffffff]*3)
        return len(faces)-1
    def degree_two(offset):
        base = len(points); points.extend([(offset,0,0),(offset-1,0,0),(offset+1,0,0),(offset,2,0),(offset,-2,0)])
        c,a,b,x,y = range(base,base+5)
        t0,t1,u0,u1 = face([c,a,b]),face([c,b,a]),face([a,x,b],1),face([b,y,a],1)
        adjacency[t0] = [t1*4+1,t1*4,u0*4]
        adjacency[t1] = [t0*4+1,t0*4,u1*4]
        adjacency[u0][0] = t0*4+2; adjacency[u1][0] = t1*4+2
    if case%5 != 0:
        width,height = rng.randrange(2,6),rng.randrange(2,6)
        start = len(points)
        for y in range(height+1):
            for x in range(width+1):
                jitter = rng.uniform(-.08,.08) if 0 < x < width and 0 < y < height else 0
                points.append((x+jitter,y+jitter*.5,0))
        begin = len(faces)
        for y in range(height):
            for x in range(width):
                a = start+y*(width+1)+x; b = a+1; c = b+width+1; d = a+width+1
                for vertices in ([a,b,c],[a,c,d]):
                    surface = rng.randrange(2) if case%7 == 0 else 0
                    face(vertices,surface)
        if case%2 == 0:
            # Close the planar top with a pyramid underneath. Every top
            # boundary edge then has a real outside neighbor, as in a brush.
            perimeter = [start+x for x in range(width+1)]
            perimeter += [start+y*(width+1)+width for y in range(1,height+1)]
            perimeter += [start+height*(width+1)+x for x in range(width-1,-1,-1)]
            perimeter += [start+y*(width+1) for y in range(height-1,0,-1)]
            apex = len(points); points.append((width*.5,height*.5,-1))
            for i,current in enumerate(perimeter):
                next_vertex = perimeter[(i+1)%len(perimeter)]
                a = tuple(points[next_vertex][k]-points[apex][k] for k in range(3))
                b = tuple(points[current][k]-points[apex][k] for k in range(3))
                n = (a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0])
                length = math.sqrt(sum(value*value for value in n))
                face([apex,next_vertex,current],len(local_normals))
                local_normals.append(tuple(value/length for value in n))
                local_distances.append(sum(local_normals[-1][k]*points[apex][k] for k in range(3)))
        edges = {(vertices[(c+2)%3],vertices[c]):(i,c)
                 for i,(vertices,_) in enumerate(faces[begin:],begin) for c in range(3)}
        for (a,b),(i,c) in edges.items():
            if (b,a) in edges:
                other,corner = edges[(b,a)]; adjacency[i][c] = other*4+corner
    for i in range(rng.randrange(1,4) if case%3 != 0 else 0): degree_two(10+i*4)
    if case%29 == 0: points,faces,adjacency = [],[],[]
    # Permute all vertex and triangle indices and independently rotate corners.
    vc,tc = len(points),len(faces)
    vm = rng.sample(range(vc),vc); tm = rng.sample(range(tc),tc); rotations = [rng.randrange(3) for _ in faces]
    ta = (Triangle*128).from_buffer_copy(rng.randbytes(128*116)); va = (Vertex*96).from_buffer_copy(rng.randbytes(96*16))
    a,b = rng.uniform(-3,3),rng.uniform(-3,3)
    ca,sa,cb,sb = math.cos(a),math.sin(a),math.cos(b),math.sin(b)
    u,v,normal = (ca*cb,sa*cb,-sb),(-sa,ca,0),(ca*sb,sa*sb,cb)
    origin = [rng.uniform(-2,2) for _ in range(3)]; scale = rng.choice([.01,1.,10.])
    for old,point in enumerate(points):
        x,y,z = point; va[vm[old]].position = Vector3(*[origin[k]+scale*(x*u[k]+y*v[k]+z*normal[k]) for k in range(3)])
        va[vm[old]].edge = 0xffffffff
    for old,(vertices,surface) in enumerate(faces):
        record = ta[tm[old]]; record.surface = surface; record.flags = 8; record.state = 2
        for c in range(3):
            corner = (c+rotations[old])%3; record.vertices[corner] = vm[vertices[c]]
            neighbor = adjacency[old][c]
            record.twins[corner] = 0xffffffff if neighbor&3 == 3 else tm[neighbor>>2]*4+((neighbor&3)+rotations[neighbor>>2])%3
            record.edge_source[corner] = rng.randrange(-1,gc)
            va[record.vertices[corner]].edge = tm[old]*4+corner
    transformed = [tuple(n[0]*u[k]+n[1]*v[k]+n[2]*normal[k] for k in range(3)) for n in local_normals]
    planes = [Plane(Vector3(*n),sum(n[k]*origin[k] for k in range(3))+scale*d) for n,d in zip(transformed,local_distances)]
    return ta,va,Mesh(ta,va,0x1234,0x5678,tc,vc,128,96),planes


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--library',required=True); parser.add_argument('--cases',type=int,default=5000)
    parser.add_argument('--seed',type=int,default=1559); args = parser.parse_args()
    if args.cases < 1: parser.error('cases must be positive')
    oracle = original(0x33112); guard = C.c_uint64(0x12345678)
    struct.pack_into('<Q',oracle.memory,0x40038,C.addressof(guard))
    @SORT
    def sort_ints(first,last,comparator):
        n = ((last or 0)-(first or 0))//4
        if n:
            values = (C.c_int32*n).from_address(first); values[:] = sorted(values)
    struct.pack_into('<Q',oracle.memory,0x40088,C.cast(sort_ints,C.c_void_p).value)
    for offset,callback in ((0x400a0,oracle.delete),(0x400b0,oracle.allocate)):
        struct.pack_into('<Q',oracle.memory,offset,C.cast(callback,C.c_void_p).value)
    native = C.CFUNCTYPE(None,C.POINTER(Mesh),C.c_int32,C.POINTER(Surfaces),C.c_void_p,C.POINTER(NativeVector))(oracle.address+0x33112)
    recovered = C.CDLL(args.library).rcsg_recovered_remove_plane_vertices
    recovered.argtypes = [C.POINTER(Mesh),C.POINTER(Surfaces),C.c_int32,C.POINTER(ContextView),
        C.POINTER(Group),C.POINTER(SourceList),C.POINTER(Output),C.POINTER(C.c_int32)]
    recovered.restype = C.c_int32
    rng = random.Random(args.seed); removed_t = removed_v = merged = multi = empty = closed = 0
    for case in range(args.cases):
        gc = rng.randrange(9)
        description = [([(rng.randrange(-1,4),rng.randrange(4)) for _ in range(rng.randrange(5))],
            [rng.randrange(32) for _ in range(rng.randrange(3))],[],(i,i+1),(1,0,0),(i,0,0)) for i in range(gc)]
        ga,gb = prepare_groups(oracle,description)
        ta,va,ma,plane_values = geometry(rng,case,gc)
        tb,vb = (Triangle*128).from_buffer_copy(ta),(Vertex*96).from_buffer_copy(va)
        mb = Mesh(tb,vb,0x1234,0x5678,ma.triangle_count,ma.vertex_count,128,96)
        tc,vc = ma.triangle_count,ma.vertex_count
        ca = Context(rng); cb = Context(copy=ca); count = C.c_int32(gc)
        planes = (Plane*len(plane_values))(*plane_values); surfaces = Surfaces(); surfaces.planes = planes
        try:
            native(C.byref(ma),rng.randrange(-2,5),C.byref(surfaces),C.byref(ca.native),C.byref(ga.vector))
            if oracle.errors: raise AssertionError(oracle.errors)
            rc = recovered(C.byref(mb),C.byref(surfaces),len(planes),C.byref(cb.view),gb.view,gb.sources,gb.results,C.byref(count))
            if rc: raise AssertionError(('portable pass',case,tc,vc,rc))
            compare_groups(ga,gb,count.value,ta,tb,case)
            if bytes(va) != bytes(vb):
                diff = next(i for i,(a,b) in enumerate(zip(bytes(va),bytes(vb))) if a != b)
                raise AssertionError(('vertex bytes',case,diff))
            if bytes(ma)[32:] != bytes(mb)[32:] or mb.unknown_10 != 0x1234 or mb.unknown_18 != 0x5678:
                raise AssertionError(('mesh metadata',case,ma.triangle_count,mb.triangle_count,ma.vertex_count,mb.vertex_count))
            ca.compare(cb,case)
            removed_t += tc-ma.triangle_count; removed_v += vc-ma.vertex_count
            merged += gc-count.value; multi += cb.view.removed_vertices.count > 1; empty += tc == 0
            closed += tc > 0 and len(planes) > 2
        finally:
            for pointer in list(oracle.allocations): oracle.delete(pointer)
            if oracle.errors: raise AssertionError(oracle.errors)
            oracle.freed.clear()
    print(json.dumps({'original_sha256':SHA256,'seed':args.seed,'cases':args.cases,'removed_triangles':removed_t,
        'vertex_count_reduction':removed_v,'removed_groups':merged,'multiple_vertex_removal_cases':multi,
        'empty_inputs':empty,'closed_grid_components':closed,'mesh_context_and_group_mismatches':0},indent=2))


if __name__ == '__main__': main()
