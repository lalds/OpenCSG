#!/usr/bin/env python3
"""Compare original Swizzle with the portable diagonal replacement helper.

Executes unchanged hash-pinned macOS x86_64 machine code on Linux. Not a test
of complete Optimize. All nine corner pairs and unrelated record bytes vary.
"""
import argparse
import ctypes as C
import json
import math
import random
import struct

from compare_find_connections import original
from compare_recovered_geometry import SHA256,Plane,Surfaces,Vector3
from compare_recovered_triangulation import Mesh,Triangle,Vertex


def points(rng,case):
    scale = rng.choice([.0001,.0006,.001,1.,100.,1e10])
    mode = case%10
    if mode == 0: return [(scale,0,0),(0,0,0),(0,scale,0),(scale,scale,0)]
    if mode == 1: return [(scale,0,0),(0,0,0),(0,scale,0),(-scale,-scale,0)]
    if mode == 2: return [(scale,0,0),(0,0,0),(0,scale,0),(2*scale,0,0)]
    if mode == 3: return [(0,0,0)]*4
    if mode == 4: return [(0,0,0),(scale,0,0),(2*scale,0,0),(3*scale,0,0)]
    result = [tuple(rng.uniform(-2,2)*scale for _ in range(3)) for _ in range(4)]
    if mode == 5: result[rng.randrange(4)] = (float('nan'),0,0)
    if mode == 6: result[rng.randrange(4)] = (float('inf'),0,0)
    if mode == 7: result[2] = result[1]
    if mode == 8: result[3] = result[2]
    return result


def adjacent_float(value,offset):
    bits = struct.unpack('<I',struct.pack('<f',value))[0]
    return struct.unpack('<f',struct.pack('<I',bits+offset))[0]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--library',required=True); p.add_argument('--cases',type=int,default=5000)
    p.add_argument('--seed',type=int,default=1559); args = p.parse_args()
    if args.cases < 1: p.error('cases must be positive')
    oracle = original(0x331ae)
    swizzle = C.CFUNCTYPE(C.c_uint8,C.POINTER(Mesh),C.POINTER(Surfaces),C.POINTER(C.c_uint32))(oracle.address+0x331ae)
    recovered = C.CDLL(args.library).rcsg_recovered_swizzle_triangle_edge
    recovered.argtypes = [C.POINTER(Mesh),C.POINTER(Surfaces),C.c_int32,C.c_uint32]
    recovered.restype = C.c_int32
    rng = random.Random(args.seed); accepted = nonfinite = missing = boundary = 0; pairs = set()
    for case in range(args.cases):
        tc = rng.randrange(6,20); vc = rng.randrange(4,20)
        ta = (Triangle*24).from_buffer_copy(rng.randbytes(24*116))
        va = (Vertex*24).from_buffer_copy(rng.randbytes(24*16))
        ai,bi,*others = rng.sample(range(tc),6)
        ac,bc = case%3,(case//3)%3; pairs.add((ac,bc))
        pi,qi,ri,si = rng.sample(range(vc),4)
        positions = points(rng,case); family = case%31; step = (case//31)%3-1
        boundary_normal = None
        if family == 0:
            positions = [(1,0,0),(0,0,0),(0,1,0),(1,1,0)]
            boundary_normal = (0,0,-adjacent_float(9.999999747378752e-6,step))
        elif family == 1:
            t = adjacent_float(.0006,step)
            positions = [(0,0,0),(t,0,0),(t,1,0),(0,1,0)]; boundary_normal = (0,0,1)
        elif family == 2:
            t = adjacent_float(.00001,step)
            positions = [(0,0,0),(1,0,0),(2,t,0),(0,t,0)]; boundary_normal = (0,0,1)
        elif family == 3:
            t = adjacent_float(1.,step+1)
            positions = [(0,0,0),(t,0,0),(t-1.,0,0),(0,0,0)]; boundary_normal = (0,0,1)
        boundary += boundary_normal is not None
        for index,point in zip((pi,qi,ri,si),positions): va[index].position = Vector3(*point)
        nonfinite += any(not math.isfinite(value) for i in (pi,qi,ri,si)
                         for value in (va[i].position.x,va[i].position.y,va[i].position.z))
        for offset,v in enumerate((si,pi,qi)): ta[ai].vertices[(ac+offset)%3] = v
        for offset,v in enumerate((qi,ri,si)): ta[bi].vertices[(bc+offset)%3] = v
        ta[ai].twins[ac] = bi*4+bc; ta[bi].twins[bc] = ai*4+ac
        for slot,(index,c) in enumerate(((ai,(ac+1)%3),(ai,(ac+2)%3),(bi,(bc+1)%3),(bi,(bc+2)%3))):
            if rng.randrange(3) == 0:
                ta[index].twins[c] = rng.choice([3,7,0xffffffff,0x1234567b]); missing += 1
            else:
                target = others[slot]*4+rng.randrange(3); ta[index].twins[c] = target
                ta[target>>2].twins[target&3] = index*4+c
        planes = (Plane*4)()
        for plane in planes:
            plane.normal = Vector3(*[rng.choice([0.,1.,-1.,rng.uniform(-2,2)]) for _ in range(3)])
        if case%11 == 0: planes[rng.randrange(4)].normal = Vector3(float('nan'),0,1)
        if boundary_normal is not None:
            for plane in planes: plane.normal = Vector3(*boundary_normal)
        ta[ai].surface,ta[bi].surface = rng.randrange(4),rng.randrange(4)
        tb = (Triangle*24).from_buffer_copy(ta); vb = (Vertex*24).from_buffer_copy(va)
        ma,mb = [Mesh(t,v,0x1234,0x5678,tc,vc,24,24) for t,v in ((ta,va),(tb,vb))]
        surfaces = Surfaces(); surfaces.planes = planes
        edge = C.c_uint32(ai*4+ac)
        expected = swizzle(C.byref(ma),C.byref(surfaces),C.byref(edge))
        result = recovered(C.byref(mb),C.byref(surfaces),4,edge.value)
        if result != expected or bytes(ta) != bytes(tb) or bytes(va) != bytes(vb):
            diff = next((i for i,(x,y) in enumerate(zip(bytes(ta),bytes(tb))) if x != y),None)
            raise AssertionError(('mismatch',case,ac,bc,expected,result,'triangle byte',diff,
                'vertex differences',[(i,va[i].edge,vb[i].edge) for i in range(vc) if va[i].edge != vb[i].edge]))
        if bytes(ma)[32:] != bytes(mb)[32:] or ma.unknown_10 != mb.unknown_10 or ma.unknown_18 != mb.unknown_18:
            raise AssertionError(('mesh metadata changed',case))
        if oracle.errors or oracle.allocations: raise AssertionError((oracle.errors,oracle.allocations))
        accepted += expected
    print(json.dumps({'original_sha256':SHA256,'seed':args.seed,'cases':args.cases,'changed':accepted,
        'rejected':args.cases-accepted,'nonfinite_position_cases':nonfinite,'invalid_outer_handles':missing,
        'corner_pairs':len(pairs),'threshold_boundary_cases':boundary,'return_and_buffer_mismatches':0},indent=2))


if __name__ == '__main__': main()
