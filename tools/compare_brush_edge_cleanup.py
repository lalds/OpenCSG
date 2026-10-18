#!/usr/bin/env python3
"""Compare original CleanUpBrushEdges, optionally including conflicting groups."""
import argparse
import ctypes as C
import json
import random
import struct

from compare_find_connections import original, Groups, Output
from compare_brush_edges import Group
from compare_recovered_geometry import SHA256
from compare_recovered_triangulation import Triangle,Mesh,NativeVector
from compare_recovered_triangle_splitting import SourceList
from compare_fix_gaps import Ref


def prepare_groups(oracle,description):
    a,b = [Groups(description) for _ in range(2)]
    for g,d in enumerate(description):
        for offset,array,count,stride in ((0,a.refs[g],len(d[0]),8),(24,a.edges[g],len(d[1]),4),(48,a.outputs[g],len(d[2]),8)):
            address = oracle.allocate(128*stride)
            if not address: raise MemoryError()
            C.memmove(address,array,count*stride)
            struct.pack_into('<QQQ',a.native,g*104+offset,address,address+count*stride,address+128*stride)
        b.refs[g] = (Ref*128)(*[Ref(*r) for r in d[0]])
        b.view[g].refs = b.refs[g]; b.view[g].capacity = 128
        b.edges[g] = (C.c_uint32*128)(*d[1]); b.sources[g] = SourceList(b.edges[g],len(d[1]),128)
    return a,b


def compare_groups(a,b,count,ta,tb,case):
    expected_count = (a.vector.end-a.vector.begin)//104
    if count != expected_count: raise AssertionError(('group count',case,expected_count,count))
    if bytes(ta) != bytes(tb):
        offset = next(i for i,(x,y) in enumerate(zip(bytes(ta),bytes(tb))) if x != y)
        raise AssertionError(('triangle mismatch',case,offset))
    for g in range(count):
        expected_fields = C.string_at(C.addressof(a.native)+g*104+72,32)
        if expected_fields != C.string_at(C.addressof(b.view[g])+16,32): raise AssertionError(('group fields',case,g))
        for offset,pointer,size,stride in ((0,b.view[g].refs,b.view[g].count,8),
            (24,b.sources[g].edges,b.sources[g].count,4),(48,b.results[g].refs,b.results[g].count,8)):
            start,end,_ = struct.unpack_from('<QQQ',a.native,g*104+offset)
            if end-start != size*stride or C.string_at(start,end-start) != C.string_at(pointer,size*stride):
                raise AssertionError(('group contents',case,g,offset))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--library',required=True); parser.add_argument('--cases',type=int,default=5000)
    parser.add_argument('--seed',type=int,default=1559)
    parser.add_argument('--merges',action='store_true',help='Generate conflicting valid group indices')
    args = parser.parse_args()
    if args.cases < 1: parser.error('cases must be positive')
    oracle = original(0x34f7c)
    canary = C.c_uint64(0x1559155915591559)
    struct.pack_into('<Q',oracle.memory,0x40038,C.addressof(canary))
    clean = C.CFUNCTYPE(None,C.POINTER(Mesh),C.POINTER(NativeVector))(oracle.address+0x34f7c)
    lib = C.CDLL(args.library); recovered = lib.rcsg_recovered_cleanup_brush_edges
    recovered.argtypes = [C.POINTER(Mesh),C.POINTER(Group),C.POINTER(SourceList),C.POINTER(Output),C.POINTER(C.c_int32)]
    recovered.restype = C.c_int32
    rng = random.Random(args.seed); removed = appended = retained_hidden = 0
    for case in range(args.cases):
        gc = rng.randrange(17); tc = 2*rng.randrange(0,17)
        description = []
        for g in range(gc):
            description.append(([(rng.randrange(8),rng.randrange(9)) for _ in range(rng.randrange(6))],
                [rng.randrange(128) for _ in range(rng.randrange(4))],
                [(rng.randrange(8),rng.randrange(9)) for _ in range(rng.randrange(4))],
                (rng.randrange(7),rng.randrange(7)),tuple(rng.uniform(-1,1) for _ in range(3)),
                tuple(rng.uniform(-3,3) for _ in range(3))))
        a,b = prepare_groups(oracle,description)
        ta = (Triangle*tc).from_buffer_copy(rng.randbytes(tc*C.sizeof(Triangle)))
        handles = [p*4+c for p in range(tc) for c in range(3)]; rng.shuffle(handles)
        for i in range(0,len(handles),2):
            e,f = handles[i:i+2]; ta[e>>2].twins[e&3] = f; ta[f>>2].twins[f&3] = e
            tag = rng.randrange(-1,gc) if gc else -1
            if case%7 == 0 and not args.merges: tag = rng.choice([-2,gc,gc+5,-1])
            ta[e>>2].edge_source[e&3] = tag
            ta[f>>2].edge_source[f&3] = rng.randrange(-1,gc) if args.merges and gc else (-1 if rng.randrange(3) == 0 else tag)
        tb = (Triangle*tc).from_buffer_copy(ta)
        ma,mb = [Mesh(t,None,None,None,tc,0,tc,0) for t in (ta,tb)]
        count = C.c_int32(gc)
        try:
            clean(C.byref(ma),C.byref(a.vector))
            if oracle.errors: raise AssertionError(oracle.errors)
            rc = recovered(C.byref(mb),b.view,b.sources,b.results,C.byref(count))
            if rc: raise AssertionError(('recovered error',case,rc))
            compare_groups(a,b,count.value,ta,tb,case)
            removed += gc-count.value
            appended += sum(s.count for s in b.sources[:count.value])
            for p,t in enumerate(tb):
                for c,tag in enumerate(t.edge_source):
                    twin = t.twins[c]
                    retained_hidden += tag >= 0 and not(t.flags&8) and not(tb[twin>>2].flags&8)
        finally:
            for pointer in list(oracle.allocations): oracle.delete(pointer)
            if oracle.errors: raise AssertionError(oracle.errors)
            oracle.freed.clear()
    print(json.dumps({'original_sha256':SHA256,'seed':args.seed,'cases':args.cases,'conflicts_enabled':args.merges,
        'removed_groups':removed,'final_list_entries':appended,'retained_hidden_hidden_tags':retained_hidden,
        'triangle_group_and_list_mismatches':0},indent=2))


if __name__ == '__main__': main()
