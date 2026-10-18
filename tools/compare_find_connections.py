#!/usr/bin/env python3
"""Execute original FindConnections and compare its exact connection semantics.

The inspected macOS function scans the manager node addressed by its brushIndex
argument for every distinct referenced brush ID. This surprising lookup is
preserved and tested, without assuming equivalence to the Windows binary.
"""
import argparse
import ctypes as C
import json
import mmap
import random
import struct

from macho_audit import inspect
from compare_brush_edges import original as base_original, Group
from compare_recovered_geometry import BINARY,SHA256,Vector3
from compare_recovered_triangulation import Mesh,Triangle,Vertex,NativeVector
from compare_recovered_triangle_splitting import SourceList
from compare_fix_gaps import Ref


class Output(C.Structure):
    _fields_ = [('refs',C.POINTER(Ref)),('count',C.c_int32),('capacity',C.c_int32)]


class Node(C.Structure):
    _fields_ = [('groups',C.POINTER(Group)),('count',C.c_int32),('index',C.c_int32),('kind',C.c_int32)]


def original(root=0x35840):
    oracle = base_original(); report,_ = inspect(); data = BINARY.read_bytes()
    functions = {int(f['address'],16):f for f in report['functions']}
    addresses = sorted(functions); ends = dict(zip(addresses,addresses[1:]))
    pending = [root]; ranges = {}; seen = set()
    while pending:
        address = pending.pop()
        if address in seen or address >= 0x3c140: continue
        seen.add(address); f = functions[address]; ranges[address] = ends[address]
        pending.extend(int(c['target_address'],16) for c in f['direct_calls'])
        pending.extend(int(c['target_address'],16) for c in f['tail_jump_candidates'])
    pages = {p for a,b in ranges.items() for p in range(a//4096,(b-1)//4096+1)}
    for p in pages:
        if oracle.libc.mprotect(oracle.address+p*4096,4096,mmap.PROT_READ | mmap.PROT_WRITE): raise OSError('load protection')
    for a,b in ranges.items(): oracle.memory[a:b] = data[a:b]
    for p in pages:
        if oracle.libc.mprotect(oracle.address+p*4096,4096,mmap.PROT_READ | mmap.PROT_EXEC): raise OSError('execute protection')
    oracle.find = C.CFUNCTYPE(None,C.POINTER(Mesh),C.c_int32,C.POINTER(NativeVector))(oracle.address+0x35840)
    return oracle


class Groups:
    def __init__(self,description):
        self.n = len(description)
        self.native = (C.c_byte*(104*self.n))()
        self.refs = [(Ref*len(d[0]))(*[Ref(*r) for r in d[0]]) for d in description]
        self.edges = [(C.c_uint32*len(d[1]))(*d[1]) for d in description]
        self.outputs = [(Ref*128)(*[Ref(*r) for r in d[2]]) for d in description]
        self.view = (Group*self.n)()
        self.sources = (SourceList*self.n)(); self.results = (Output*self.n)()
        for i,(d,refs,edges,out) in enumerate(zip(description,self.refs,self.edges,self.outputs)):
            start = C.addressof(refs)
            struct.pack_into('<QQQ',self.native,104*i,start,start+C.sizeof(refs),start+C.sizeof(refs))
            start = C.addressof(edges)
            struct.pack_into('<QQQ',self.native,104*i+24,start,start+C.sizeof(edges),start+C.sizeof(edges))
            start = C.addressof(out)
            struct.pack_into('<QQQ',self.native,104*i+48,start,start+8*len(d[2]),start+C.sizeof(out))
            struct.pack_into('<ii6f',self.native,104*i+72,*d[3],*d[4],*d[5])
            self.view[i] = Group(refs,len(refs),len(refs),*d[3],Vector3(*d[4]),Vector3(*d[5]))
            self.sources[i] = SourceList(edges,len(edges),len(edges))
            self.results[i] = Output(out,len(d[2]),128)
        self.vector = NativeVector.of(self.native)

    def native_results(self):
        result = []
        for i in range(self.n):
            start,end,_ = struct.unpack_from('<QQQ',self.native,104*i+48)
            result.append([struct.unpack_from('<iI',C.string_at(start,end-start),o) for o in range(0,end-start,8)])
        return result


def native_call(oracle,mesh,own,sets,modes):
    nodes = [(C.c_byte*224)() for _ in sets]
    metadata = (C.c_byte*(4*len(sets)))()
    for i,(node,group,mode) in enumerate(zip(nodes,sets,modes)):
        struct.pack_into('<i',node,24,-1 if mode == 'index' else i)
        C.memmove(C.addressof(node)+160,C.byref(group.vector),24)
        metadata[i*4+3] = 1 if mode == 'kind' else 3
    pointers = (C.c_void_p*len(sets))(*[None if mode == 'absent' else C.addressof(node) for node,mode in zip(nodes,modes)])
    struct.pack_into('<Q',oracle.memory,0x404b0,C.addressof(metadata))
    start = C.addressof(pointers)
    struct.pack_into('<QQQ',oracle.memory,0x404f8,start,start+C.sizeof(pointers),start+C.sizeof(pointers))
    oracle.find(C.byref(mesh),own,C.byref(sets[0].vector))
    if oracle.allocations or oracle.errors: raise AssertionError((oracle.allocations,oracle.errors))
    oracle.freed.clear()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--library'); parser.add_argument('--cases',type=int,default=5000)
    parser.add_argument('--seed',type=int,default=1559); parser.add_argument('--diagnostic',action='store_true')
    args = parser.parse_args(); oracle = original()
    if args.diagnostic:
        own = Groups([([(1,3)],[0],[],(3,4),(1,0,0),(0,0,0))])
        foreign = Groups([([(0,3)],[],[],(8,9),(1,0,0),(0,0,0))])
        triangles = (Triangle*1)(); mesh = Mesh(triangles,None,None,None,1,0,1,0)
        native_call(oracle,mesh,0,[own,foreign],['valid','valid'])
        print(json.dumps({'own_index':0,'surface_refs':[[1,3]],'original_connections':own.native_results()})); return
    if not args.library: parser.error('--library required for comparisons')
    if args.cases < 1: parser.error('cases must be positive')
    lib = C.CDLL(args.library); recovered = lib.rcsg_recovered_find_connections
    recovered.argtypes = [C.POINTER(Mesh),C.c_int32,C.POINTER(Group),C.POINTER(SourceList),C.POINTER(Output),
                         C.c_int32,C.POINTER(Node),C.c_int32]; recovered.restype = C.c_int32
    rng = random.Random(args.seed); calls = appended = cleared = 0
    threshold_bits = struct.unpack('<I',struct.pack('<f',0.9999899864196777))[0]
    scales = [1.,-1.,0.,float('nan')]+[struct.unpack('<f',struct.pack('<I',threshold_bits+k))[0] for k in (-1,0,1)]
    for case in range(args.cases):
        descriptions = []
        for node in range(rng.randrange(1,6)):
            description = []
            for g in range(rng.randrange(0,7)):
                surfaces = (rng.randrange(-1,5),rng.randrange(-1,5))
                refs = [(rng.randrange(-2,7),rng.choice([0,1,2,3,4,0xffffffff])) for _ in range(rng.randrange(7))]
                if refs and rng.randrange(2): refs.append(refs[0])
                handles = [rng.randrange(16)*4+rng.randrange(3) for _ in range(rng.randrange(9))]
                old = [(rng.randrange(8),rng.randrange(6)) for _ in range(rng.randrange(3))] if case%3 == 0 else []
                axis = rng.randrange(3); direction = [rng.choice(scales) if k == axis else 0 for k in range(3)]
                origin = [rng.choice([0.,0.,0.0001,0.0003,0.0004,1.]) for _ in range(3)]
                description.append((refs,handles,old,surfaces,direction,origin))
            descriptions.append(description)
        modes = [rng.choice(['valid']*5+['absent','index','kind']) for _ in descriptions]
        own = rng.randrange(-1,len(descriptions)+1)
        if case%3 == 1:
            own = 0; modes[0] = 'valid'
            descriptions[0] = [
                ([(5,0),(2,1),(5,0),(-1,1)],[0,4],[],(0,1),(1,0,0),(0,0,0)),
                ([(2,0),(5,1),(-1,0)],[8],[],(2,3),(-1,0,0),(1,0,0)),
                ([],[12],[(7,8)],(0,1),(1,0,0),(0,1,0)),
                ([(5,0)],[16],[],(0,1),(float('nan'),0,0),(0,0,0))]
        a,b = [[Groups(d) for d in descriptions] for _ in range(2)]
        before_nonlocal = [bytes(group.native) for group in a[1:]]
        raw = rng.randbytes(16*C.sizeof(Triangle))
        ta,tb = [(Triangle*16).from_buffer_copy(raw) for _ in range(2)]
        ma,mb = [Mesh(t,None,None,None,16,0,16,0) for t in (ta,tb)]
        nodes = (Node*len(b))(*[Node(group.view,group.n,-1 if mode in ('index','absent') else i,
            1 if mode == 'kind' else 3) for i,(group,mode) in enumerate(zip(b,modes))])
        initial = sum(len(d[2]) for d in descriptions[0])
        for repeat in range(2):
            native_call(oracle,ma,own,a,modes)
            rc = recovered(C.byref(mb),own,b[0].view,b[0].sources,b[0].results,b[0].n,nodes,len(nodes))
            if rc: raise AssertionError(('recovered error',case,repeat,rc))
            expected = a[0].native_results()
            actual = [[(out.refs[r].brush,out.refs[r].edge) for r in range(out.count)] for out in b[0].results]
            if expected != actual: raise AssertionError(('connections mismatch',case,repeat,own,modes,expected,actual))
            if bytes(ta) != bytes(tb): raise AssertionError(('mesh mismatch',case,repeat))
            if before_nonlocal != [bytes(group.native) for group in a[1:]]:
                raise AssertionError('Nonlocal native group records mutated')
            # Native mutates only local connection vectors and edgeSource fields.
            for i in range(len(a)):
                if i and a[i].native_results() != [[tuple(r) for r in d[2]] for d in descriptions[i]]:
                    raise AssertionError('Nonlocal native connections mutated')
                if i and [[(out.refs[r].brush,out.refs[r].edge) for r in range(out.count)] for out in b[i].results] != [d[2] for d in descriptions[i]]:
                    raise AssertionError('Nonlocal portable connections mutated')
                for g in range(a[i].n):
                    if bytes(a[i].refs[g]) != bytes(b[i].refs[g]) or bytes(a[i].edges[g]) != bytes(b[i].edges[g]):
                        raise AssertionError('Surface refs/source handles mutated')
                    if C.string_at(C.addressof(a[i].native)+g*104+72,32) != C.string_at(C.addressof(b[i].view[g])+16,32):
                        raise AssertionError('Group geometry mutated')
            calls += 1
        appended += sum(len(g) for g in a[0].native_results())-initial
        cleared += sum(struct.unpack_from('<i',t,60+4*c)[0] == -1 for t in ta for c in range(3))
    print(json.dumps({'original_sha256':SHA256,'seed':args.seed,'cases':args.cases,'original_calls':calls,
        'appended_connections':appended,'final_cleared_edge_sources':cleared,
        'connection_and_mesh_mismatches':0,'manager_lookup':'brushIndex argument, not referenced ID'},indent=2))


if __name__ == '__main__': main()
