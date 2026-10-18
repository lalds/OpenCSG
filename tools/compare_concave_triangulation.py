#!/usr/bin/env python3
"""Compare original concave triangulation, reused slots and mutable vectors.

Executes unchanged hash-pinned Mach-O x86_64 code on Linux. Boundary handles
with invalid low bits still have bounded triangle indices: native copies their
source fields unconditionally. Out-of-bounds invalid handles are not executed.
"""
import argparse
import ctypes as C
import json
import math
import random

from compare_find_connections import original
from compare_recovered_geometry import SHA256,Plane,Surfaces,Vector3
from compare_recovered_triangulation import Mesh,Triangle,Vertex,NativeVector,NATIVE


class PolygonView(C.Structure):
    _fields_ = [('vertices',C.POINTER(C.c_int32)),('normals',C.POINTER(Vector3)),
                ('twins',C.POINTER(C.c_uint32)),('count',C.c_int32),('capacity',C.c_int32)]


class PoolView(C.Structure):
    _fields_ = [('indices',C.POINTER(C.c_int32)),('count',C.c_int32),('capacity',C.c_int32)]


def shape(rng,case,count):
    mode = case%10
    scale = rng.choice([.0001,.01,1.,100.,1e8])
    result = []
    for i in range(count):
        radius = scale*(.3 if mode == 1 and i%2 else 1.)
        angle = 2*math.pi*i/max(count,1)
        result.append((radius*math.cos(angle),radius*math.sin(angle),0.))
    if mode == 2 and count >= 6:
        result[:6] = [(0,0,0),(2*scale,0,0),(2*scale,scale,0),(scale,scale,0),(scale,2*scale,0),(0,2*scale,0)]
    if mode == 3: result = [(i*scale,0.,0.) for i in range(count)]
    if mode == 4:
        for i in range(1,count,3): result[i] = result[i-1]
    if mode == 5: result.reverse()
    if mode == 6: rng.shuffle(result)
    if mode == 7 and count: result[rng.randrange(count)] = (float('nan'),0,0)
    if mode == 8: result = [(x*.000001,y*.000001,z) for x,y,z in result]
    return result


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--library',required=True); p.add_argument('--cases',type=int,default=5000)
    p.add_argument('--seed',type=int,default=1559); args = p.parse_args()
    if args.cases < 1: p.error('cases must be positive')
    oracle = original(0x2f822); native = NATIVE(oracle.address+0x2f822)
    recovered = C.CDLL(args.library).rcsg_recovered_triangulate_concave
    recovered.argtypes = [C.POINTER(Mesh),C.POINTER(Surfaces),C.c_int32,C.POINTER(PolygonView),
        C.POINTER(PoolView),C.c_int32,C.c_uint8,C.c_uint16]
    recovered.restype = C.c_int32
    rng = random.Random(args.seed); emitted = invalids = short = rotated = repeated = 0
    for case in range(args.cases):
        n = (case//31)%3 if case%31 == 0 else rng.randrange(3,33)
        needed = max(0,n-2); vc = n+8; tc = n+needed+8
        ta = (Triangle*96).from_buffer_copy(rng.randbytes(96*116))
        va = (Vertex*64).from_buffer_copy(rng.randbytes(64*16))
        ids = rng.sample(range(vc),n)
        positions = shape(rng,case,n); normal = (0.,0.,1.)
        if case%4:
            a,b = rng.uniform(-math.pi,math.pi),rng.uniform(-math.pi,math.pi)
            ca,sa,cb,sb = math.cos(a),math.sin(a),math.cos(b),math.sin(b)
            u,v,normal = (ca*cb,sa*cb,-sb),(-sa,ca,0.),(ca*sb,sa*sb,cb)
            origin = tuple(rng.uniform(-2,2) for _ in range(3))
            positions = [tuple(origin[k]+x*u[k]+y*v[k]+z*normal[k] for k in range(3)) for x,y,z in positions]
            rotated += 1
        for i,position in zip(ids,positions): va[i].position = Vector3(*position)
        indices = (C.c_int32*40).from_buffer_copy(rng.randbytes(40*4))
        normals = (Vector3*40).from_buffer_copy(rng.randbytes(40*12))
        twins = (C.c_uint32*40).from_buffer_copy(rng.randbytes(40*4))
        for i,id in enumerate(ids):
            indices[i] = ids[i-1] if case%23 == 0 and i%4 == 1 else id
            repeated += indices[i] != id
            c = 3 if case%7 == 0 else rng.randrange(3)
            twins[i] = i*4+c; invalids += c == 3
        pool_count = needed+rng.randrange(4)
        pool = (C.c_int32*40).from_buffer_copy(rng.randbytes(40*4))
        pool[:pool_count] = rng.sample(range(n,tc),pool_count)
        arrays = (indices,normals,twins,pool)
        copies = [type(a).from_buffer_copy(a) for a in arrays]
        tb = (Triangle*96).from_buffer_copy(ta); vb = (Vertex*64).from_buffer_copy(va)
        ma,mb = [Mesh(t,v,0x1234,0x5678,tc,vc,96,64) for t,v in ((ta,va),(tb,vb))]
        vectors = [NativeVector(C.addressof(a),C.addressof(a)+length*C.sizeof(a._type_),C.addressof(a)+C.sizeof(a))
                   for a,length in zip(arrays,(n,n,n,pool_count))]
        polygon = PolygonView(*copies[:3],n,40); slots = PoolView(copies[3],pool_count,40)
        planes = (Plane*3)()
        for plane in planes: plane.normal = Vector3(*normal)
        if case%10 == 9:
            for plane in planes: plane.normal = Vector3(*rng.choice([(0,0,0),tuple(-v for v in normal),(float('nan'),0,1)]))
        surfaces = Surfaces(); surfaces.planes = planes
        surface,flags,state = rng.randrange(3),rng.randrange(256),rng.randrange(65536)
        native(C.byref(ma),C.byref(surfaces),*[C.byref(v) for v in vectors],surface,flags,state)
        rc = recovered(C.byref(mb),C.byref(surfaces),3,C.byref(polygon),C.byref(slots),surface,flags,state)
        if rc != needed: raise AssertionError(('return',case,n,needed,rc))
        if bytes(ta) != bytes(tb):
            i = next(i for i,(a,b) in enumerate(zip(bytes(ta),bytes(tb))) if a != b)
            raise AssertionError(('triangle',case,n,i//116,i%116,bytes(ta)[i],bytes(tb)[i]))
        if bytes(va) != bytes(vb): raise AssertionError(('vertices',case,n))
        for name,a,b in zip(('indices','normals','twins','pool'),arrays,copies):
            if bytes(a) != bytes(b): raise AssertionError(('array',case,n,name))
        lengths = [(v.end-v.begin)//C.sizeof(a._type_) for v,a in zip(vectors,arrays)]
        expected_count = 0 if n >= 3 else n
        if lengths != [expected_count]*3+[pool_count-needed] or polygon.count != expected_count or slots.count != pool_count-needed:
            raise AssertionError(('vector lengths',case,n,lengths,polygon.count,slots.count))
        if bytes(ma)[32:] != bytes(mb)[32:] or ma.unknown_10 != mb.unknown_10 or ma.unknown_18 != mb.unknown_18:
            raise AssertionError(('mesh metadata',case))
        if oracle.errors or oracle.allocations: raise AssertionError((oracle.errors,oracle.allocations))
        emitted += needed; short += n < 3
    print(json.dumps({'original_sha256':SHA256,'seed':args.seed,'cases':args.cases,
        'triangles_reused':emitted,'short_polygon_noops':short,'bounded_invalid_boundary_handles':invalids,
        'rotated_plane_cases':rotated,'repeated_vertex_ids':repeated,
        'mesh_vector_and_pool_mismatches':0},indent=2))


if __name__ == '__main__': main()
