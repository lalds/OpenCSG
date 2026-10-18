#!/usr/bin/env python3
"""Compare two-surface decisions up to the first mutating action.

Intercept RemoveInPlane, Swizzle and Split entries plus rejection. Original
traversal, topology and float instructions run, but no action/compaction runs.
Enough capacity is supplied so native capacity growth is not exercised.
"""
import argparse
import ctypes as C
import json
import mmap
import random
import struct
from compare_find_connections import original
from compare_recovered_geometry import SHA256,Vector3,Plane,Surfaces
from compare_recovered_triangulation import Mesh,Triangle,Vertex,NativeVector
from compare_two_surface_candidates import Candidate,fixture

class Plan(C.Structure):
    _fields_=[('boundary',Candidate),('swizzle',C.c_uint32),('vertex',C.c_int32)]

def decision_oracle():
    oracle=original(0x339fe); guard=C.c_uint64(0x12345678)
    struct.pack_into('<Q',oracle.memory,0x40038,C.addressof(guard))
    epilogue=bytes.fromhex('4881c4380200005b415c415d415e415f5dc3')
    common=bytes.fromhex('4c8b85d0fdffff8b85b4feffff4189008b85a0feffff418940048b850cfeffff4189400c45895008')
    patches={0x41300:bytes.fromhex('31c0')+epilogue}
    entries=((0x345c3,0x41000,1),(0x34533,0x41100,2),(0x345e8,0x41200,3),(0x34bf6,0x41300,0))
    for site,target,action in entries:
        patches[site]=b'\xe9'+struct.pack('<i',target-site-5)
        if action:
            save_edge=bytes.fromhex('448b12') if action==2 else bytes.fromhex('41baffffffff')
            patches[target]=save_edge+common+b'\xb8'+struct.pack('<I',action)+epilogue
    pages={a//4096 for a in patches}
    for p in pages:
        if oracle.libc.mprotect(oracle.address+p*4096,4096,mmap.PROT_READ|mmap.PROT_WRITE): raise OSError('patch protection')
    for address,code in patches.items(): oracle.memory[address:address+len(code)]=code
    for p in pages:
        if oracle.libc.mprotect(oracle.address+p*4096,4096,mmap.PROT_READ|mmap.PROT_EXEC): raise OSError('execute protection')
    call=C.CFUNCTYPE(C.c_int32,C.POINTER(Mesh),C.c_int32,C.POINTER(Surfaces),C.c_void_p,C.POINTER(Plan))(oracle.address+0x339fe)
    return oracle,guard,call

def geometry(rng,case):
    ta,va,old,vertex,mode=fixture(rng,case)
    tc,vc=old.triangle_count,old.vertex_count
    triangles=(Triangle*(tc+4)).from_buffer_copy(bytes(ta)+rng.randbytes(4*116))
    vertices=(Vertex*(vc+4)).from_buffer_copy(bytes(va)+rng.randbytes(4*16))
    # Break a cross-boundary endpoint while leaving center handles/region
    # traversal valid. Endpoint consistency must reject after prefix acceptance.
    if case%17==0:
        i=rng.randrange(tc)
        # The ring's center corner is identified by vertex identity.
        c=next(j for j in range(3) if triangles[i].vertices[j]==vertex)
        triangles[i].vertices[(c+1)%3]=vertex
    mesh=Mesh(triangles,vertices,0x1234,0x5678,tc,vc,tc+4,vc+4)
    values=[Plane(Vector3(*[rng.choice([-1.,0.,1.,float('nan'),float('inf')]) for _ in range(3)]),rng.uniform(-1,1)) for _ in range(3)]
    planes=(Plane*3)(*values); surfaces=Surfaces(); surfaces.planes=planes
    return triangles,vertices,mesh,vertex,mode,planes,surfaces

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--library',required=True); p.add_argument('--cases',type=int,default=5000); p.add_argument('--seed',type=int,default=1559)
    args=p.parse_args()
    if args.cases<1: p.error('cases must be positive')
    oracle,guard,native=decision_oracle()
    portable=C.CDLL(args.library).rcsg_recovered_plan_two_surface_vertex
    portable.argtypes=[C.POINTER(Mesh),C.POINTER(Surfaces),C.c_int32,C.c_int32,C.POINTER(Plan)]; portable.restype=C.c_int32
    rng=random.Random(args.seed); actions=[0]*4; corners=set(); same_surface=0
    for case in range(args.cases):
        ta,va,mesh,vertex,mode,planes,surfaces=geometry(rng,case)
        before=(bytes(ta),bytes(va),bytes(mesh),bytes(planes),bytes(surfaces))
        native_mesh=Mesh.from_buffer_copy(mesh); native_mesh.vertex_count=vertex+1
        scratch=(C.c_uint32*6)(); context=(NativeVector*6)()
        for i in range(6): context[i]=NativeVector(C.addressof(scratch)+4*i,C.addressof(scratch)+4*i,C.addressof(scratch)+4*i)
        a=Plan.from_buffer_copy(b'\xa5'*16); b=Plan.from_buffer_copy(a)
        expected=native(C.byref(native_mesh),0,C.byref(surfaces),C.byref(context),C.byref(a))
        actual=portable(C.byref(mesh),C.byref(surfaces),3,vertex,C.byref(b))
        if expected!=actual or bytes(a)!=bytes(b): raise AssertionError((case,mode,expected,actual,bytes(a).hex(),bytes(b).hex()))
        if before!=(bytes(ta),bytes(va),bytes(mesh),bytes(planes),bytes(surfaces)): raise AssertionError(('input mutation',case))
        if oracle.errors or oracle.allocations: raise AssertionError((oracle.errors,oracle.allocations))
        actions[actual]+=1
        if actual:
            corners.add((a.boundary.first&3,a.boundary.second&3))
            same_surface+=ta[a.boundary.first>>2].surface==ta[a.boundary.second>>2].surface
    print(json.dumps(dict(source_sha256=SHA256,scope='decision prefix only; four intercepted exits; no capacity growth',seed=args.seed,cases=args.cases,actions=dict(zip(['rejected','remove_in_plane','swizzle','split'],actions)),boundary_corner_pairs=len(corners),same_surface_actions=same_surface,mismatches=0),indent=2))
if __name__=='__main__': main()
