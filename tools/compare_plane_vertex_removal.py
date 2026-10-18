#!/usr/bin/env python3
"""Compare complete original RemoveVertexInMiddleOfPlane on bounded fans.

Includes native scratch-vector mutations on rejection, reused triangles and
the two-corner deletion/BrushEdge merge path whose native result is false.
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
from compare_concave_triangulation import PoolView


class Normals(C.Structure):
    _fields_ = [('values',C.POINTER(Vector3)),('count',C.c_int32),('capacity',C.c_int32)]


class ContextView(C.Structure):
    _fields_ = [('boundary',SourceList),('removed',PoolView),('incident',PoolView),
                ('removed_vertices',PoolView),('vertices',PoolView),('normals',Normals)]


class Context:
    def __init__(self,rng=None,copy=None):
        kinds = [C.c_uint32,C.c_int32,C.c_int32,C.c_int32,C.c_int32,Vector3]
        self.arrays = [type(a).from_buffer_copy(a) for a in copy.arrays] if copy else [
            (kind*128).from_buffer_copy(rng.randbytes(128*C.sizeof(kind))) for kind in kinds]
        self.counts = list(copy.counts) if copy else [rng.randrange(5) for _ in kinds]
        self.native = (NativeVector*6)(*[NativeVector(C.addressof(a),C.addressof(a)+count*C.sizeof(kind),C.addressof(a)+C.sizeof(a))
            for a,count,kind in zip(self.arrays,self.counts,kinds)])
        self.view = ContextView(*[view(a,count,128) for view,a,count in zip(
            (SourceList,PoolView,PoolView,PoolView,PoolView,Normals),self.arrays,self.counts)])

    def compare(self,other,case):
        fields = [name for name,_ in ContextView._fields_]
        for i,(a,b,name) in enumerate(zip(self.arrays,other.arrays,fields)):
            length = (self.native[i].end-self.native[i].begin)//C.sizeof(a._type_)
            actual = getattr(other.view,name).count
            if length != actual or bytes(a) != bytes(b):
                byte = next((j for j,(x,y) in enumerate(zip(bytes(a),bytes(b))) if x != y),None)
                raise AssertionError(('context',case,name,length,actual,'byte',byte))


def fan(rng,case,gc,removal_cases=False):
    n = 1 if case%97 == 0 else 2 if case%4 == 0 else rng.randrange(3,21)
    tc,vc = 2*n+6,2*n+8
    triangles = (Triangle*64).from_buffer_copy(rng.randbytes(64*116))
    vertices = (Vertex*64).from_buffer_copy(rng.randbytes(64*16))
    centers = rng.sample(range(tc),n); outside = rng.sample([i for i in range(tc) if i not in centers],n)
    center,*ids = rng.sample(range(vc),n+1)
    extras = rng.sample([i for i in range(vc) if i != center and i not in ids],n)
    corners = [rng.randrange(3) for _ in range(n)]; outcorners = [rng.randrange(3) for _ in range(n)]
    if n == 2 and case%8 == 0:
        # Bounded synthetic case: both outside edges occupy different corners
        # of one record, exercising the native extra-removal branch.
        outside[1] = outside[0]; outcorners = [0,1]
    normal = (0,0,1)
    if case%3:
        a,b = rng.uniform(-3,3),rng.uniform(-3,3)
        ca,sa,cb,sb = math.cos(a),math.sin(a),math.cos(b),math.sin(b)
        u,v,normal = (ca*cb,sa*cb,-sb),(-sa,ca,0),(ca*sb,sa*sb,cb)
    else: u,v = (1,0,0),(0,1,0)
    origin = [rng.uniform(-2,2) for _ in range(3)]; vertices[center].position = Vector3(*origin)
    for i,id in enumerate(ids):
        angle = i*2*math.pi/n; radius = rng.choice([.5,1.,2.])
        vertices[id].position = Vector3(*[origin[k]+radius*(math.cos(angle)*u[k]+math.sin(angle)*v[k]) for k in range(3)])
        vertices[extras[i]].position = Vector3(*[origin[k]+3*(math.cos(angle+math.pi/n)*u[k]+math.sin(angle+math.pi/n)*v[k]) for k in range(3)])
    flags,state,surface = rng.randrange(256),rng.randrange(65536),rng.randrange(3)
    for i,(index,c) in enumerate(zip(centers,corners)):
        t = triangles[index]; nexti = (i+1)%n
        t.vertices[c] = center; t.vertices[(c+1)%3] = ids[i]; t.vertices[(c+2)%3] = ids[nexti]
        t.twins[c] = centers[nexti]*4+(corners[nexti]+1)%3
        t.twins[(c+1)%3] = centers[(i-1)%n]*4+corners[(i-1)%n]
        t.twins[(c+2)%3] = outside[i]*4+outcorners[i]
        t.surface,t.flags,t.state = surface,flags,state
        for j in range(3): t.edge_source[j] = rng.randrange(-1,gc)
        ext = triangles[outside[i]]; d = outcorners[i]
        ext.vertices[d] = ids[i]; ext.vertices[(d+2)%3] = ids[nexti]; ext.vertices[(d+1)%3] = extras[i]
        ext.twins[d] = index*4+(c+2)%3; ext.edge_source[d] = rng.randrange(-1,gc)
    start = rng.randrange(n); vertices[center].edge = centers[start]*4+corners[start]
    mode = 0 if removal_cases else (case//4)%11
    bad = (start+max(1,n//2))%n
    if mode == 1: vertices[center].edge = 0xffffffff
    if mode == 2: vertices[center].edge = (tc+3)*4
    if mode == 3: triangles[centers[start]].surface = -1
    if mode == 4: triangles[centers[bad]].surface = (surface+1)%3
    if mode == 5: triangles[centers[bad]].flags ^= 1
    if mode == 6: triangles[centers[bad]].state ^= 1
    if mode == 7: triangles[centers[bad]].twins[(corners[bad]+2)%3] = 0xffffffff
    if mode == 8: triangles[centers[bad]].vertices[(corners[bad]+1)%3] = ids[start]
    if mode == 9: triangles[centers[bad]].twins[(corners[bad]+2)%3] = centers[start]*4+corners[start]
    if mode == 10: triangles[centers[bad]].twins[(corners[bad]+2)%3] = outside[bad]*4+3
    return triangles,vertices,Mesh(triangles,vertices,0x1234,0x5678,tc,vc,64,64),center,normal,n,mode


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--library',required=True); parser.add_argument('--cases',type=int,default=5000)
    parser.add_argument('--seed',type=int,default=1559)
    parser.add_argument('--removal-cases',action='store_true',help='Disable deliberately rejecting fixture modes')
    args = parser.parse_args()
    if args.cases < 1: parser.error('cases must be positive')
    oracle = original(0x316a2)
    guard = C.c_uint64(0x12345678)
    struct.pack_into('<Q',oracle.memory,0x40038,C.addressof(guard))
    native = C.CFUNCTYPE(C.c_uint8,C.POINTER(Mesh),C.POINTER(Surfaces),C.c_int32,C.c_void_p,C.POINTER(NativeVector))(oracle.address+0x316a2)
    recovered = C.CDLL(args.library).rcsg_recovered_remove_plane_vertex
    recovered.argtypes = [C.POINTER(Mesh),C.POINTER(Surfaces),C.c_int32,C.c_int32,C.POINTER(ContextView),
        C.POINTER(Group),C.POINTER(SourceList),C.POINTER(Output),C.POINTER(C.c_int32)]
    recovered.restype = C.c_int32
    rng = random.Random(args.seed); accepted = changed_false = group_merges = two_corner = single_corner = 0
    for case in range(args.cases):
        gc = rng.randrange(9)
        description = [([(rng.randrange(-1,4),rng.randrange(4)) for _ in range(rng.randrange(5))],
            [rng.randrange(32) for _ in range(rng.randrange(3))],[],(i,i+1),(1,0,0),(i,0,0)) for i in range(gc)]
        ga,gb = prepare_groups(oracle,description)
        ta,va,ma,vertex,normal,n,mode = fan(rng,case,gc,args.removal_cases)
        tb,vb = (Triangle*64).from_buffer_copy(ta),(Vertex*64).from_buffer_copy(va)
        mb = Mesh(tb,vb,0x1234,0x5678,ma.triangle_count,ma.vertex_count,64,64)
        ca = Context(rng); cb = Context(copy=ca); count = C.c_int32(gc)
        planes = (Plane*3)(*[Plane(Vector3(*normal),0) for _ in range(3)])
        surfaces = Surfaces(); surfaces.planes = planes
        before = bytes(ta),bytes(va)
        try:
            expected = native(C.byref(ma),C.byref(surfaces),vertex,C.byref(ca.native),C.byref(ga.vector))
            if oracle.errors: raise AssertionError(oracle.errors)
            rc = recovered(C.byref(mb),C.byref(surfaces),3,vertex,C.byref(cb.view),gb.view,gb.sources,gb.results,C.byref(count))
            if rc != expected: raise AssertionError(('return',case,n,mode,expected,rc))
            compare_groups(ga,gb,count.value,ta,tb,case)
            if bytes(va) != bytes(vb):
                diff = next(i for i,(a,b) in enumerate(zip(bytes(va),bytes(vb))) if a != b)
                raise AssertionError(('vertices',case,n,mode,diff))
            ca.compare(cb,case)
            if bytes(ma)[32:] != bytes(mb)[32:] or mb.unknown_10 != 0x1234 or mb.unknown_18 != 0x5678:
                raise AssertionError(('mesh metadata',case))
            accepted += expected
            changed_false += not expected and before != (bytes(ta),bytes(va))
            group_merges += count.value < gc; two_corner += n == 2; single_corner += n == 1
        finally:
            for pointer in list(oracle.allocations): oracle.delete(pointer)
            if oracle.errors: raise AssertionError(oracle.errors)
            oracle.freed.clear()
    print(json.dumps({'original_sha256':SHA256,'seed':args.seed,'cases':args.cases,'native_true':accepted,
        'native_false_with_mesh_changes':changed_false,'cases_with_group_merge':group_merges,
        'two_corner_fans':two_corner,'synthetic_single_corner_fans':single_corner,
        'removal_cases':args.removal_cases,'mesh_context_and_group_mismatches':0},indent=2))


if __name__ == '__main__': main()
