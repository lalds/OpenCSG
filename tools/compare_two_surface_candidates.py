#!/usr/bin/env python3
"""Compare only the read-only candidate prefix of the second Optimize pass.

Two exit jumps intercept acceptance at 0x3407f and rejection at 0x34bf6.
The traversal and floating-point instructions being compared are unchanged;
this is NOT execution or validation of the complete mutating pass.
"""
import argparse
import ctypes as C
import json
import math
import mmap
import random
import struct
from compare_find_connections import original
from compare_recovered_geometry import SHA256,Vector3
from compare_recovered_triangulation import Mesh,Triangle,Vertex,NativeVector

class Candidate(C.Structure):
    _fields_=[('first',C.c_uint32),('second',C.c_uint32)]

def prefix_oracle():
    oracle=original(0x339fe)
    guard=C.c_uint64(0x12345678)
    struct.pack_into('<Q',oracle.memory,0x40038,C.addressof(guard))
    epilogue=bytes.fromhex('4881c4380200005b415c415d415e415f5dc3')
    accepted=bytes.fromhex('4c8b85d0fdffff8b85b4feffff4189008b85a0feffff41894004b801000000')+epilogue
    rejected=bytes.fromhex('31c0')+epilogue
    patches={0x41000:accepted,0x41100:rejected}
    for site,target in ((0x3407f,0x41000),(0x34bf6,0x41100)):
        patches[site]=b'\xe9'+struct.pack('<i',target-site-5)
    pages={address//4096 for address in patches}
    for p in pages:
        if oracle.libc.mprotect(oracle.address+p*4096,4096,mmap.PROT_READ|mmap.PROT_WRITE): raise OSError('patch protection')
    for address,code in patches.items(): oracle.memory[address:address+len(code)]=code
    for p in pages:
        if oracle.libc.mprotect(oracle.address+p*4096,4096,mmap.PROT_READ|mmap.PROT_EXEC): raise OSError('execute protection')
    call=C.CFUNCTYPE(C.c_int32,C.POINTER(Mesh),C.c_int32,C.c_void_p,C.c_void_p,C.POINTER(Candidate))(oracle.address+0x339fe)
    return oracle,guard,call

def fixture(rng,case):
    n=rng.randrange(2,23); split=rng.randrange(1,n)
    ta=(Triangle*n).from_buffer_copy(rng.randbytes(n*116)); va=(Vertex*(n+1))()
    vm=rng.sample(range(n+1),n+1); tm=rng.sample(range(n),n); rotations=[rng.randrange(3) for _ in range(n)]
    points=[(0,0,0)]+[(math.cos(2*math.pi*i/n),math.sin(2*math.pi*i/n),rng.uniform(-1,1)) for i in range(n)]
    points[1]=(-1,0,0); points[split+1]=(1,0,0)
    mode=case%12
    if mode==1: points[split+1]=(1,rng.uniform(.01,1),0)
    if mode==2: points[split+1]=(0,0,0)
    if mode==3: points[split+1]=(rng.choice([float('nan'),float('inf')]),0,0)
    if mode==4: points[split+1]=(1,rng.choice([.0001,.0004,.0006,.001]),0)
    scale=rng.choice([1e-5,.01,1.,1000.]); angle=rng.uniform(-math.pi,math.pi)
    co,si=math.cos(angle),math.sin(angle); origin=[rng.uniform(-1,1) for _ in range(3)]
    for i,(x,y,z) in enumerate(points):
        va[vm[i]].position=Vector3(origin[0]+scale*(co*x-si*y),origin[1]+scale*(si*x+co*y),origin[2]+scale*z)
        va[vm[i]].edge=0xffffffff
    for i in range(n):
        t=ta[tm[i]]; r=rotations[i]
        t.surface=0 if i<split else 1; t.state=2
        if mode==5: t.surface=0
        if mode==6: t.surface=i%3
        if mode==7: t.state=rng.randrange(3)
        if mode==8: t.surface=0; t.state=0 if i<split else 1
        if mode==9: t.surface=-1
        for c,v in enumerate([0,i+1,(i+1)%n+1]): t.vertices[(c+r)%3]=vm[v]
        t.twins[r]=tm[(i+1)%n]*4+(1+rotations[(i+1)%n])%3
        t.twins[(1+r)%3]=tm[(i-1)%n]*4+rotations[(i-1)%n]
        t.twins[(2+r)%3]=0xffffffff
    start=rng.randrange(n); va[vm[0]].edge=tm[start]*4+rotations[start]
    if mode==10:
        broken=rng.randrange(n); ta[tm[broken]].twins[rotations[broken]]=0xffffffff
    if mode==11: va[vm[0]].edge=rng.choice([0xffffffff,n*4,n*4+2,3])
    return ta,va,Mesh(ta,va,None,None,n,n+1,n,n+1),vm[0],mode

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--library',required=True); p.add_argument('--cases',type=int,default=5000); p.add_argument('--seed',type=int,default=1559)
    args=p.parse_args()
    if args.cases<1: p.error('cases must be positive')
    oracle,guard,native=prefix_oracle()
    recovered=C.CDLL(args.library).rcsg_recovered_find_two_surface_candidate
    recovered.argtypes=[C.POINTER(Mesh),C.c_int32,C.POINTER(Candidate)]; recovered.restype=C.c_int32
    rng=random.Random(args.seed); accepted=0; modes=[0]*12
    for case in range(args.cases):
        ta,va,mesh,vertex,mode=fixture(rng,case)
        before=(bytes(ta),bytes(va),bytes(mesh))
        # Stop after this vertex: native starts at vertexCount-1. The prefix
        # reads positions through the unchanged full allocation, not this count.
        native_mesh=Mesh.from_buffer_copy(mesh); native_mesh.vertex_count=vertex+1
        scratch=(C.c_uint32*6)(); context=(NativeVector*6)()
        # Supply live empty-vector storage; only count-clear operations occur.
        for i in range(6): context[i]=NativeVector(C.addressof(scratch)+i*4,C.addressof(scratch)+i*4,C.addressof(scratch)+i*4)
        a,b=Candidate(0x12345678,0xabcdef01),Candidate(0x12345678,0xabcdef01)
        result=native(C.byref(native_mesh),0,None,C.byref(context),C.byref(a))
        actual=recovered(C.byref(mesh),vertex,C.byref(b))
        if actual!=result or bytes(a)!=bytes(b): raise AssertionError((case,mode,result,actual,list(bytes(a)),list(bytes(b))))
        if before!=(bytes(ta),bytes(va),bytes(mesh)): raise AssertionError(('input mutation',case))
        if oracle.errors or oracle.allocations: raise AssertionError((oracle.errors,oracle.allocations))
        accepted+=result; modes[mode]+=result
    print(json.dumps(dict(source_sha256=SHA256,scope='candidate prefix only; two intercepted exits',seed=args.seed,cases=args.cases,accepted=accepted,rejected=args.cases-accepted,accepted_by_mode=modes,mismatches=0),indent=2))
if __name__=='__main__': main()
