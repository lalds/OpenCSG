#!/usr/bin/env python3
"""Compare recovered connected-group cuts AND links with original full FixGaps.

Uses closed tetrahedra, one connected source group per brush, independently
ordered subdivisions, explicit manager and fixed capacities. Native FixGaps
owns its temporary interval vector. No code patch or Unity runtime is used.
"""
import argparse
import ctypes as C
import json
import random

from compare_gap_alignment import Body, original_fix, moments_and_topology
from compare_gap_neighbors import View
from compare_recovered_geometry import SHA256, Vector3, Plane
from compare_recovered_gap_intervals import Oracle, Line
from compare_recovered_triangulation import Mesh
from compare_recovered_triangle_splitting import Transform, SourceList


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--library',required=True); p.add_argument('--cases',type=int,default=2000)
    p.add_argument('--seed',type=int,default=1559); args = p.parse_args()
    if args.cases < 1: p.error('cases must be positive')
    lib = C.CDLL(args.library); oracle = Oracle(include_splitting=True)
    split = lib.rcsg_recovered_split_triangle_edge
    split.argtypes = [C.POINTER(Mesh),C.POINTER(Transform),C.POINTER(SourceList),C.c_int32,C.c_uint32,
                      C.POINTER(Vector3),C.c_float,C.POINTER(C.c_uint32)]; split.restype = C.c_int32
    reset = lib.rcsg_recovered_reset_gap_neighbors
    reset.argtypes = [C.POINTER(Mesh)]; reset.restype = C.c_int32
    fix = lib.rcsg_recovered_fix_gap_group
    fix.argtypes = [C.POINTER(Mesh),C.POINTER(Plane),C.c_int32,C.POINTER(Transform),C.POINTER(SourceList),
        C.c_int32,C.c_int32,C.POINTER(Line),C.c_int32,C.POINTER(C.c_uint32),C.c_int32,C.POINTER(View),C.c_int32,C.c_int32]
    fix.restype = C.c_int32
    rng = random.Random(args.seed); total = neighbors = 0
    for case in range(args.cases):
        bodies = [Body() for _ in range(rng.randrange(2,6))]
        for body in bodies:
            # Independent partitions and visitation order for each brush.
            for x in sorted(rng.sample(range(1,32),rng.randrange(0,6))):
                x /= 16
                selected = next(e for e in body.edges[:body.source.count]
                    if body.world[body.triangles[e>>2].vertices[((e&3)+2)%3]].x < x
                    < body.world[body.triangles[e>>2].vertices[e&3]].x)
                point = Vector3(x,0,0); result = C.c_uint32()
                rc = split(C.byref(body.mesh),C.byref(body.matrix),C.byref(body.source),1,selected,C.byref(point),0.5,C.byref(result))
                if rc: raise AssertionError(('fixture split failed',case,rc))
            handles = list(body.edges[:body.source.count]); rng.shuffle(handles)
            body.edges[:body.source.count] = handles
            for t in body.triangles[:body.mesh.triangle_count]: t.flags = rng.randrange(256) if case%3 else 9
            for plane in body.planes:
                plane.normal = Vector3(*[rng.choice([-1.,0.,1.]) if case%5 == 0 else rng.uniform(-2,2) for _ in range(3)])
                if case%11 == 0: plane.normal.x = float('nan')
        # Rotate the fixture onto all three axes, including negative direction,
        # and translate it. These exact rotations preserve tetrahedron moments.
        axis = case%3; sign = -1 if case%2 else 1
        offset = [rng.randrange(-3,4) for _ in range(3)]
        def rotate(v):
            xyz = [sign*v.x,sign*v.y,v.z]
            return xyz[axis:]+xyz[:axis]
        direction = rotate(Vector3(1,0,0))
        for body in bodies:
            body.line.direction = Vector3(*direction); body.line.origin = Vector3(*offset)
            for v in range(body.mesh.vertex_count):
                point = Vector3(*[x+y for x,y in zip(rotate(body.world[v]),offset)])
                body.world[v] = point; body.vertices[v].position = point
        local = bodies[0]; recovered = Body()
        for attr in ('triangles','vertices','world','lookup','edges','planes'):
            C.memmove(getattr(recovered,attr),getattr(local,attr),C.sizeof(getattr(local,attr)))
        recovered.mesh.vertex_count = local.mesh.vertex_count
        recovered.mesh.triangle_count = local.mesh.triangle_count
        recovered.source.count = local.source.count
        recovered.line = Line(local.line.direction,local.line.origin)
        old_count = local.mesh.vertex_count
        before_foreign = [(bytes(b.triangles),bytes(b.vertices),bytes(b.world),bytes(b.edges)) for b in bodies[1:]]
        original_fix(oracle,local,bodies[1:])
        views = (View*len(bodies))(*[View(C.pointer(b.mesh),b.planes,i,4) for i,b in enumerate([recovered]+bodies[1:])])
        if reset(C.byref(recovered.mesh)): raise AssertionError('reset failed')
        splits = 0
        for i,body in enumerate(bodies[1:],1):
            count = fix(C.byref(recovered.mesh),recovered.planes,4,C.byref(recovered.matrix),C.byref(recovered.source),1,0,
                        C.byref(recovered.line),i,body.edges,body.source.count,views,len(views),64)
            if count < 0: raise AssertionError(('fix failed',case,count))
            splits += count
        if (local.mesh.vertex_count,local.mesh.triangle_count,local.source.count) != (
            recovered.mesh.vertex_count,recovered.mesh.triangle_count,recovered.source.count):
            raise AssertionError(('count mismatch',case,local.mesh.vertex_count,recovered.mesh.vertex_count))
        for attr in ('triangles','vertices','world','lookup','edges'):
            expected,actual = bytes(getattr(local,attr)),bytes(getattr(recovered,attr))
            # Original reallocates world/lookup only to active counts; wrapper
            # copies active data back, preserving the same unused Python bytes.
            if expected != actual:
                offset = next(i for i,(a,b) in enumerate(zip(expected,actual)) if a != b)
                raise AssertionError(('buffer mismatch',case,attr,offset,expected[offset:offset+16].hex(),actual[offset:offset+16].hex()))
        if splits != local.mesh.vertex_count-old_count: raise AssertionError('Wrong split count')
        if before_foreign != [(bytes(b.triangles),bytes(b.vertices),bytes(b.world),bytes(b.edges)) for b in bodies[1:]]:
            raise AssertionError('Foreign buffers changed')
        moments_and_topology(local); moments_and_topology(recovered)
        total += splits; neighbors += len(bodies)-1
    print(json.dumps({'original_sha256':SHA256,'seed':args.seed,'connected_group_cases':args.cases,
                      'foreign_brushes':neighbors,'new_vertices_per_implementation':total,
                      'complete_mesh_buffer_mismatches':0,'closed_twins_volume_area_mismatches':0},indent=2))


if __name__ == '__main__': main()
