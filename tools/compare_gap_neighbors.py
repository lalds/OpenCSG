#!/usr/bin/env python3
"""Execute unpatched original FixGaps on aligned groups; compare complete triangles.

Multiple foreign brushes exercise incumbent arbitration. No local/foreign cuts
are required; this tests visibility, orientation, source normals and tie order,
not the native temporary interval update during splitting.
"""
import argparse
import ctypes as C
import json
import random
import struct

from compare_gap_alignment import Body, original_fix
from compare_recovered_geometry import SHA256, Plane, Vector3
from compare_recovered_triangulation import Mesh
from compare_recovered_gap_intervals import Oracle, Line


class View(C.Structure):
    _fields_ = [('mesh',C.POINTER(Mesh)),('planes',C.POINTER(Plane)),
                ('node',C.c_int32),('count',C.c_int32)]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--library',required=True); p.add_argument('--cases',type=int,default=5000)
    p.add_argument('--seed',type=int,default=1559); args = p.parse_args()
    if args.cases < 1: p.error('cases must be positive')
    lib = C.CDLL(args.library); oracle = Oracle(include_splitting=True)
    reset = lib.rcsg_recovered_reset_gap_neighbors
    reset.argtypes = [C.POINTER(Mesh)]; reset.restype = C.c_int32
    match = lib.rcsg_recovered_match_gap_neighbors
    match.argtypes = [C.POINTER(Mesh),C.POINTER(Plane),C.c_int32,C.POINTER(Line),C.POINTER(C.c_uint32),
                     C.c_int32,C.c_int32,C.POINTER(C.c_uint32),C.c_int32,C.POINTER(View),C.c_int32]
    match.restype = C.c_int32
    rng = random.Random(args.seed); candidates = changes = 0
    for case in range(args.cases):
        bodies = [Body() for _ in range(rng.randrange(2,9))]
        for body in bodies:
            for i,t in enumerate(body.triangles[:4]):
                t.flags = rng.randrange(256)
                # Source planes, not per-corner normals, determine selection.
                values = [rng.choice([-1.,0.,1.]) for _ in range(3)] if case%3 == 0 else [rng.uniform(-2,2) for _ in range(3)]
                if case%11 == 0: values[rng.randrange(3)] = float('nan')
                if case%17 == 0: values[rng.randrange(3)] = float('inf')
                body.planes[i].normal = Vector3(*values)
                for c in range(3): t.normals[c] = Vector3(123,456,789)
            if rng.randrange(2): body.edges[0],body.edges[1] = body.edges[1],body.edges[0]
            # Duplicated foreign handles stay in the traversal order.
            if rng.randrange(3) == 0:
                body.edges[2] = body.edges[0]; body.source.count = 3
        local = bodies[0]; recovered = Body()
        C.memmove(recovered.triangles,local.triangles,C.sizeof(local.triangles))
        C.memmove(recovered.planes,local.planes,C.sizeof(local.planes))
        C.memmove(recovered.edges,local.edges,C.sizeof(local.edges)); recovered.source.count = local.source.count
        unchanged = [bytes(body.triangles) for body in bodies[1:]]
        original_fix(oracle,local,bodies[1:])
        views = (View*len(bodies))(*[View(C.pointer(b.mesh),b.planes,i,4) for i,b in enumerate([recovered]+bodies[1:])])
        if reset(C.byref(recovered.mesh)): raise AssertionError('reset failed')
        for i,body in enumerate(bodies[1:],1):
            result = match(C.byref(recovered.mesh),recovered.planes,4,C.byref(recovered.line),
                           recovered.edges,recovered.source.count,i,body.edges,body.source.count,views,len(views))
            if result: raise AssertionError(('match failed',case,result))
        if bytes(local.triangles) != bytes(recovered.triangles):
            detail = []
            for t in range(4):
                detail.append((t,local.triangles[t].flags,
                    struct.unpack_from('<iIiIiI',local.triangles[t],72),
                    struct.unpack_from('<iIiIiI',recovered.triangles[t],72)))
            raise AssertionError((case,detail))
        if unchanged != [bytes(body.triangles) for body in bodies[1:]]: raise AssertionError('Foreign triangles mutated')
        if bytes(local.vertices) != bytes(recovered.vertices): raise AssertionError('Vertices mutated')
        for t in local.triangles[:4]:
            changes += sum(struct.unpack_from('<i',t,72+8*c)[0]>=0 for c in range(3))
        candidates += len(bodies)-1
    print(json.dumps({'original_sha256':SHA256,'seed':args.seed,'aligned_group_cases':args.cases,
                      'foreign_brushes':candidates,'selected_local_links':changes,
                      'complete_triangle_buffer_mismatches':0,'foreign_triangle_mutations':0},indent=2))


if __name__ == '__main__': main()
