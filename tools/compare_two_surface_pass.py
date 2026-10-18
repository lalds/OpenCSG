#!/usr/bin/env python3
"""Compare the full second Optimize pass with unchanged original machine code.

Original traversal/removal/triangulation/compaction code is unchanged. Its
external libc++ signed-int sorting import is provided by a host callback.
"""
import argparse
import ctypes as C
import json
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
from compare_plane_vertex_pass import geometry as base_geometry


def geometry(rng,case,gc):
    mode=case%8
    if mode==0: source_case=29
    elif mode==1: source_case=14
    elif mode==4: source_case=1
    elif mode==5: source_case=5
    else:
        source_case=2+case*2
        while source_case%5==0 or source_case%7==0 or source_case%29==0: source_case+=2
    ta,va,old,planes=base_geometry(rng,source_case,gc)
    # Merge coplanar conical side facets into one surface per side, giving
    # the closed grid genuine two-surface seam vertices along its perimeter.
    canonical={}; mapping={0:0,1:1}
    for i,p in enumerate(planes[2:],2):
        key=tuple(round(v,5) for v in (p.normal.x,p.normal.y,p.normal.z,p.distance))
        mapping[i]=canonical.setdefault(key,i)
    for i in range(old.triangle_count):
        ta[i].surface=mapping[ta[i].surface]
        if case%6==0: ta[i].flags=rng.choice([8,9])
    triangles=(Triangle*512).from_buffer_copy(bytes(ta)+rng.randbytes((512-len(ta))*116))
    vertices=(Vertex*256).from_buffer_copy(bytes(va)+rng.randbytes((256-len(va))*16))
    return triangles,vertices,Mesh(triangles,vertices,0x1234,0x5678,old.triangle_count,old.vertex_count,512,256),planes


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--library',required=True); parser.add_argument('--cases',type=int,default=5000)
    parser.add_argument('--seed',type=int,default=1559)
    parser.add_argument('--both-passes',action='store_true',help='Execute the original two passes in Optimize order')
    parser.add_argument('--original-wrapper',action='store_true',help='Execute original Optimize including context allocation; implies --both-passes')
    args = parser.parse_args()
    if args.original_wrapper: args.both_passes=True
    if args.cases < 1: parser.error('cases must be positive')
    oracle = original(0x34c44 if args.both_passes else 0x339fe); guard = C.c_uint64(0x12345678)
    struct.pack_into('<Q',oracle.memory,0x40038,C.addressof(guard))
    @SORT
    def sort_ints(first,last,comparator):
        n = ((last or 0)-(first or 0))//4
        if n:
            values = (C.c_int32*n).from_address(first); values[:] = sorted(values)
    struct.pack_into('<Q',oracle.memory,0x40088,C.cast(sort_ints,C.c_void_p).value)
    for offset,callback in ((0x400a0,oracle.delete),(0x400b0,oracle.allocate)):
        struct.pack_into('<Q',oracle.memory,offset,C.cast(callback,C.c_void_p).value)
    native = C.CFUNCTYPE(None,C.POINTER(Mesh),C.c_int32,C.POINTER(Surfaces),C.c_void_p,C.POINTER(NativeVector))(oracle.address+0x339fe)
    wrapper_native = C.CFUNCTYPE(None,C.POINTER(Mesh),C.c_int32,C.POINTER(Surfaces),C.POINTER(NativeVector))(oracle.address+0x34c44)
    first_native = C.CFUNCTYPE(None,C.POINTER(Mesh),C.c_int32,C.POINTER(Surfaces),C.c_void_p,C.POINTER(NativeVector))(oracle.address+0x33112)
    recovered = getattr(C.CDLL(args.library),'rcsg_recovered_optimize_mesh' if args.both_passes else 'rcsg_recovered_remove_two_surface_vertices')
    recovered.argtypes = [C.POINTER(Mesh),C.POINTER(Surfaces),C.c_int32,C.POINTER(ContextView),
        C.POINTER(Group),C.POINTER(SourceList),C.POINTER(Output),C.POINTER(C.c_int32)]
    recovered.restype = C.c_int32
    rng = random.Random(args.seed); removed_t = removed_v = merged = multi = empty = closed = 0
    inserted_vertices=0
    for case in range(args.cases):
        gc = case%3
        description = [([(rng.randrange(-1,4),rng.randrange(4)) for _ in range(rng.randrange(5))],
            [rng.randrange(32) for _ in range(rng.randrange(3))],[],(i,i+1),(1,0,0),(i,0,0)) for i in range(gc)]
        ga,gb = prepare_groups(oracle,description)
        ta,va,ma,plane_values = geometry(rng,case,gc)
        tb,vb = (Triangle*512).from_buffer_copy(ta),(Vertex*256).from_buffer_copy(va)
        mb = Mesh(tb,vb,0x1234,0x5678,ma.triangle_count,ma.vertex_count,512,256)
        tc,vc = ma.triangle_count,ma.vertex_count
        ca = Context(rng); cb = Context(copy=ca); count = C.c_int32(gc)
        planes = (Plane*len(plane_values))(*plane_values); surfaces = Surfaces(); surfaces.planes = planes
        try:
            if args.both_passes and not args.original_wrapper: first_native(C.byref(ma),0,C.byref(surfaces),C.byref(ca.native),C.byref(ga.vector))
            if args.original_wrapper: wrapper_native(C.byref(ma),0,C.byref(surfaces),C.byref(ga.vector))
            else: native(C.byref(ma),rng.randrange(-2,5),C.byref(surfaces),C.byref(ca.native),C.byref(ga.vector))
            if oracle.errors: raise AssertionError(oracle.errors)
            rc = recovered(C.byref(mb),C.byref(surfaces),len(planes),C.byref(cb.view),gb.view,gb.sources,gb.results,C.byref(count))
            if rc: raise AssertionError(('portable pass',case,tc,vc,rc))
            compare_groups(ga,gb,count.value,ta,tb,case)
            if bytes(va) != bytes(vb):
                diff = next(i for i,(a,b) in enumerate(zip(bytes(va),bytes(vb))) if a != b)
                raise AssertionError(('vertex bytes',case,diff))
            if bytes(ma)[32:] != bytes(mb)[32:] or mb.unknown_10 != 0x1234 or mb.unknown_18 != 0x5678:
                raise AssertionError(('mesh metadata',case,ma.triangle_count,mb.triangle_count,ma.vertex_count,mb.vertex_count))
            if not args.original_wrapper: ca.compare(cb,case)
            removed_t += tc-ma.triangle_count; removed_v += vc-ma.vertex_count
            inserted_vertices += ma.vertex_count-vc+cb.view.removed_vertices.count
            merged += gc-count.value; multi += cb.view.removed_vertices.count > 1; empty += tc == 0
            closed += tc > 0 and len(planes) > 2
        finally:
            for pointer in list(oracle.allocations): oracle.delete(pointer)
            if oracle.errors: raise AssertionError(oracle.errors)
            oracle.freed.clear()
    print(json.dumps({'original_sha256':SHA256,'both_passes':args.both_passes,'original_wrapper':args.original_wrapper,'context_compared':not args.original_wrapper,'seed':args.seed,'cases':args.cases,'removed_triangles':removed_t,
        'second_pass_inferred_temporary_vertices':inserted_vertices if not args.both_passes else None,'vertex_count_reduction':removed_v,'removed_groups':merged,'multiple_vertex_removal_cases':multi,
        'empty_inputs':empty,'closed_grid_components':closed,'mesh_and_group_mismatches':0,'context_mismatches':None if args.original_wrapper else 0},indent=2))


if __name__ == '__main__': main()
