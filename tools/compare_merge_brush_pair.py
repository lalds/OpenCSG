#!/usr/bin/env python3
"""Compare MergeBrushIndices, including index-array mutation and remap."""
import argparse
import ctypes as C
import json
import random

from compare_brush_edge_cleanup import prepare_groups,compare_groups
from compare_find_connections import original,Output
from compare_brush_edges import Group
from compare_recovered_geometry import SHA256
from compare_recovered_triangulation import Mesh,Triangle,NativeVector
from compare_recovered_triangle_splitting import SourceList


def native_removal_stays_live(values,group_count):
    """Inspect removal indices without executing accesses to destroyed slots."""
    indices = sorted(values,reverse=True)
    if indices[0] == -1: return True
    active = len(indices)
    while indices[active-1] == -1: active -= 1
    if indices[0] == indices[active-1]: return True
    indices[:active] = reversed(indices[:active])
    i = 0
    while i < active:
        j = i+1
        while j < active:
            if indices[j] == indices[i]:
                active -= 1; indices[j] = indices[active]; j = i+1
            else: j += 1
        i += 1
    i = 1
    while i < active:
        pending = active
        while active > i:
            active -= 1
            if indices[active] != group_count-1: break
            group_count -= 1; pending -= 1
        if pending == i: return True
        if indices[i] >= group_count: return False
        group_count -= 1; active = pending; i += 1
    return True


def main(general=False):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--library',required=True); p.add_argument('--cases',type=int,default=5000)
    p.add_argument('--seed',type=int,default=1559)
    if general: p.add_argument('--max-indices',type=int,default=32)
    args = p.parse_args()
    if args.cases < 1: p.error('cases must be positive')
    if general and args.max_indices < 1: p.error('max-indices must be positive')
    oracle = original(0x3218e)
    merge = C.CFUNCTYPE(C.c_int32,C.POINTER(Mesh),C.POINTER(NativeVector),C.POINTER(C.c_int32),C.c_int32)(oracle.address+0x3218e)
    lib = C.CDLL(args.library)
    recovered = lib.rcsg_recovered_merge_brush_indices if general else lib.rcsg_recovered_merge_brush_pair
    recovered.argtypes = [C.POINTER(Mesh),C.POINTER(Group),C.POINTER(SourceList),C.POINTER(Output),
        C.POINTER(C.c_int32),C.POINTER(C.c_int32)]+([C.c_int32] if general else [])+[C.POINTER(C.c_int32)]
    recovered.restype = C.c_int32
    rng = random.Random(args.seed); actual_merges = tail_merges = rejected = executed = entries = 0
    for case in range(args.cases):
        gc = rng.randrange(17); tc = rng.randrange(17); description = []
        for g in range(gc):
            refs = [(rng.randrange(-1,4),rng.choice([0,1,2,0xffffffff])) for _ in range(rng.randrange(8))]
            if refs and case%3 == 0: refs += [refs[0],refs[0]]
            description.append((refs,[rng.randrange(128) for _ in range(rng.randrange(5))],
                [(rng.randrange(8),rng.randrange(8)) for _ in range(rng.randrange(5))],(g,g+1),
                tuple(rng.uniform(-2,2) for _ in range(3)),tuple(rng.uniform(-3,3) for _ in range(3))))
        a,b = prepare_groups(oracle,description)
        length = rng.randrange(1,args.max_indices+1) if general else 2
        values = [rng.randrange(-1,gc) for _ in range(length)]
        if general and case%4 == 0:
            values = rng.sample(range(-1,gc),min(length,gc+1)); length = len(values)
        ia,ib = [(C.c_int32*length)(*values) for _ in range(2)]
        ta = (Triangle*tc).from_buffer_copy(rng.randbytes(tc*C.sizeof(Triangle)))
        for t in ta:
            for c in range(3): t.edge_source[c] = rng.randrange(-3,gc+3)
        tb = (Triangle*tc).from_buffer_copy(ta)
        ma,mb = [Mesh(t,None,None,None,tc,0,tc,0) for t in (ta,tb)]
        count,result = C.c_int32(gc),C.c_int32(-99)
        try:
            safe = not general or native_removal_stays_live(values,gc)
            expected = merge(C.byref(ma),C.byref(a.vector),ia,length) if safe else None
            if oracle.errors: raise AssertionError(oracle.errors)
            call_args = [C.byref(mb),b.view,b.sources,b.results,C.byref(count),ib]
            if general: call_args.append(length)
            rc = recovered(*call_args,C.byref(result))
            if not safe:
                if rc != -1 or count.value != gc or result.value != -99 or bytes(ib) != bytes(ia):
                    raise AssertionError(('unsafe removal not rejected atomically',case,values,rc))
                compare_groups(a,b,count.value,ta,tb,case); rejected += 1
                continue
            if rc: raise AssertionError(('recovered error',case,rc,values))
            if expected != result.value or bytes(ia) != bytes(ib):
                raise AssertionError(('return/indices',case,values,expected,result.value,list(ia),list(ib)))
            compare_groups(a,b,count.value,ta,tb,case)
            actual_merges += count.value < gc
            tail_merges += count.value < gc and max(values) == gc-1
            executed += 1; entries += length
        finally:
            for pointer in list(oracle.allocations): oracle.delete(pointer)
            if oracle.errors: raise AssertionError(oracle.errors)
            oracle.freed.clear()
    print(json.dumps({'original_sha256':SHA256,'seed':args.seed,'cases':args.cases,'general':general,
        'original_executions':executed,'compared_index_entries':entries,'native_lifetime_order_rejections':rejected,
        'actual_merges':actual_merges,'tail_merges':tail_merges,
        'index_result_triangle_and_group_mismatches':0},indent=2))


if __name__ == '__main__': main()
