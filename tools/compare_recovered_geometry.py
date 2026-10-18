#!/usr/bin/env python3
"""Compare recovered C++ against two isolated original x86_64 routines on Linux.

This explicitly executes the original Alignment/FindVertexSideOnPlane bytes.
It is not a Mach-O loader: only those routines and their three float constants
are copied into anonymous memory. No plugin entry point or macOS import is run.
Use with the exact hash-pinned checked-in binary and valid indexed input only.
Run this tool in its own process (a bad reconstruction can crash that process).
"""

import argparse
import ctypes as C
import hashlib
import json
import math
import mmap
from pathlib import Path
import platform
import random
import struct
import sys


ROOT = Path(__file__).resolve().parents[1]
BINARY = ROOT / "Plugins/Editor/External/RealtimeCSG[1_559].bundle/Contents/MacOS/RealtimeCSG[1_559]"
SHA256 = "26789e84b0c11376582662bf1d53ed21a65c68d57449efb607070b266cb61a7d"


class Vector3(C.Structure):
    _fields_ = [(n, C.c_float) for n in ("x", "y", "z")]


class Plane(C.Structure):
    _fields_ = [("normal", Vector3), ("distance", C.c_float)]


class Polygon(C.Structure):
    _fields_ = [(n, C.c_int32) for n in ("edge_count", "first_edge", "surface_index", "unknown_0c")]
    _fields_ += [("unknown_10", C.c_uint16), ("unknown_12", C.c_uint16)]


class Edge(C.Structure):
    _fields_ = [(n, C.c_int32) for n in ("next_edge", "vertex_index", "twin_edge", "polygon_index")]


class EdgeLink(C.Structure):
    _fields_ = [("previous_edge", C.c_int32), ("unknown_04", C.c_int32)]


class Mesh(C.Structure):
    _fields_ = [("polygons", C.POINTER(Polygon)), ("edges", C.POINTER(Edge)),
                ("edge_links", C.POINTER(EdgeLink)), ("vertices", C.POINTER(Vector3))]


class Surfaces(C.Structure):
    _fields_ = [("unknown", C.c_byte * 32), ("planes", C.POINTER(Plane))]


class VertexState(C.Structure):
    _fields_ = [("position", Vector3), ("distance", C.c_float), ("edge_index", C.c_int32)]


ALIGNMENT = C.CFUNCTYPE(C.c_uint8, C.POINTER(Mesh), C.POINTER(Surfaces), C.c_int32, C.POINTER(Plane))
VERTEX_SIDE = C.CFUNCTYPE(C.c_int32, C.POINTER(VertexState), C.POINTER(Mesh),
                        C.POINTER(C.c_int32), C.POINTER(C.c_int32), *([C.c_float] * 4))


def original_functions(include_splitting=False):
    if sys.platform != "linux" or platform.machine() not in ("x86_64", "AMD64"):
        raise RuntimeError("Original-code comparison requires Linux x86_64")
    data = BINARY.read_bytes()
    if hashlib.sha256(data).hexdigest() != SHA256:
        raise ValueError("Original binary hash changed; re-audit code and addresses before execution")
    # This binary's __TEXT vmaddr and file offset are both zero. Preserve RIP-relative
    # distances, but copy only the two inspected functions and required constants.
    memory = mmap.mmap(-1, 0x40000, prot=mmap.PROT_READ | mmap.PROT_WRITE)
    memory[0x2cc04:0x2d15c] = data[0x2cc04:0x2d15c]
    if include_splitting:
        # Only valid, preallocated paths are used. Set*Capacity / heap routines
        # are intentionally not mapped; exhaustion must never reach those paths.
        for first, last in ((0x60da, 0x6124), (0x6456, 0x6868), (0x2d15c, 0x2dd30)):
            memory[first:last] = data[first:last]
    for offset in (0x3c554, 0x3c56c, 0x3c57c):
        memory[offset:offset + 4] = data[offset:offset + 4]
    address = C.addressof(C.c_char.from_buffer(memory))
    libc = C.CDLL(None, use_errno=True)
    libc.mprotect.argtypes = [C.c_void_p, C.c_size_t, C.c_int]
    libc.mprotect.restype = C.c_int
    protections = [(address, 0x40000, mmap.PROT_READ),
                   (address + 0x2c000, 0x2000, mmap.PROT_READ | mmap.PROT_EXEC)]
    if include_splitting:
        protections.append((address + 0x6000, 0x1000, mmap.PROT_READ | mmap.PROT_EXEC))
    for start, size, protection in protections:
        if libc.mprotect(start, size, protection):
            raise OSError(C.get_errno(), "mprotect failed")
    return memory, ALIGNMENT(address + 0x2cc04), VERTEX_SIDE(address + 0x2cdfe)


def f32(value):
    return C.c_float(value).value


def distance(plane, point):
    xy = f32(f32(point.x * plane.normal.x) + f32(point.y * plane.normal.y))
    xyz = f32(xy + f32(point.z * plane.normal.z))
    return f32(xyz - plane.distance)


def fixture(points, normal, rng, edge_count=None):
    count = len(points)
    vertices = (Vector3 * count)(*[Vector3(*p) for p in points])
    edges, links = (Edge * count)(), (EdgeLink * count)()
    # Deliberately permute edge and vertex storage. The polygon follows links,
    # not array order, and vertexIndex must not be confused with edgeIndex.
    ring = list(range(count))
    rng.shuffle(ring)
    for i, edge in enumerate(ring):
        edges[edge] = Edge(ring[(i + 1) % count], i, 0, 0)
        links[edge] = EdgeLink(ring[(i - 1) % count], 0)
    polygons = (Polygon * 1)(Polygon(count if edge_count is None else edge_count, ring[0], 0, 0, 0, 0))
    planes = (Plane * 1)(Plane(Vector3(*normal), 0))
    surfaces = Surfaces()
    surfaces.planes = planes
    mesh = Mesh(polygons, edges, links, vertices)
    return mesh, surfaces, ring, (vertices, edges, links, polygons, planes)


def generate_case(index, rng):
    epsilon = f32(0.0006)
    boundary_values = [struct.unpack("<f", struct.pack("<I", x))[0]
                       for x in (0x3a1d4951, 0x3a1d4952, 0x3a1d4953)]
    boundary_values += [-x for x in boundary_values] + [0.0, -0.0]
    count = rng.randrange(1, 13)
    mode = index % 6
    if mode == 0:
        # All points exactly coplanar; also exercise zero/one edge records.
        points = [(rng.uniform(-2, 2), rng.uniform(-2, 2), 0.0) for _ in range(count)]
    elif mode == 1:
        points = [(rng.uniform(-2, 2), rng.uniform(-2, 2), rng.choice(boundary_values))
                  for _ in range(count)]
    elif mode == 2:
        # Almost coincident vertices, where the topology-aware resolver matters.
        points = [(rng.uniform(-epsilon, epsilon), rng.uniform(-epsilon, epsilon),
                   rng.uniform(-epsilon, epsilon)) for _ in range(count)]
    elif mode == 3:
        scale = rng.choice((0.0001, 0.01, 1.0, 10000.0))
        points = [tuple(rng.uniform(-scale, scale) for _ in range(3)) for _ in range(count)]
    elif mode == 4:
        points = [(math.cos(i * 2 * math.pi / count), math.sin(i * 2 * math.pi / count),
                   rng.choice(boundary_values)) for i in range(count)]
    else:
        points = [(rng.uniform(-2, 2), rng.uniform(-2, 2), rng.uniform(-2, 2))
                  for _ in range(count)]
        points = [points[0] if rng.random() < 0.5 else p for p in points]
    normal = rng.choice([(0, 0, 1), (0, 0, -1), (1, 0, 0),
                         tuple(rng.uniform(-1, 1) for _ in range(3))])
    plane = Plane(Vector3(*normal), rng.choice((0, 0, 0, epsilon, -epsilon, rng.uniform(-2, 2))))
    surface_normal = rng.choice([(0, 0, 1), (0, 0, -1), (0, 0, 0), normal])
    return points, surface_normal, plane


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--library", type=Path, required=True, help="Compiled recovery shared library")
    parser.add_argument("--cases", type=int, default=10000)
    parser.add_argument("--seed", type=int, default=1559)
    args = parser.parse_args()
    if args.cases < 1:
        parser.error("--cases must be positive")
    memory, original_alignment, original_side = original_functions()
    library = C.CDLL(str(args.library.resolve()))
    recovered_alignment = ALIGNMENT(("rcsg_recovered_alignment", library))
    recovered_side = VERTEX_SIDE(("rcsg_recovered_vertex_side", library))
    rng = random.Random(args.seed)
    alignment_counts = [0] * 6
    helper_checks = 0
    for index in range(args.cases):
        points, normal, plane = generate_case(index, rng)
        mesh, surfaces, ring, owners = fixture(points, normal, rng, edge_count=0 if index % 97 == 0 else None)
        original = original_alignment(C.byref(mesh), C.byref(surfaces), 0, C.byref(plane))
        recovered = recovered_alignment(C.byref(mesh), C.byref(surfaces), 0, C.byref(plane))
        if original != recovered:
            raise AssertionError(json.dumps({"test": "alignment", "seed": args.seed, "case": index,
                                            "original": original, "recovered": recovered,
                                            "points": points,
                                            "plane": [plane.normal.x, plane.normal.y, plane.normal.z, plane.distance]}))
        alignment_counts[original] += 1
        # Compare both the return value AND mutated representative/side caches.
        for cached in (False, True):
            representative_a = C.c_int32(rng.choice(ring) if cached else -1)
            side_a = C.c_int32(rng.choice((-2, -1, 1)))
            representative_b, side_b = C.c_int32(representative_a.value), C.c_int32(side_a.value)
            for edge in ring:
                position = mesh.vertices[mesh.edges[edge].vertex_index]
                state = VertexState(position, distance(plane, position), edge)
                arguments = (plane.normal.x, plane.normal.y, plane.normal.z, plane.distance)
                a = original_side(C.byref(state), C.byref(mesh), C.byref(representative_a), C.byref(side_a), *arguments)
                b = recovered_side(C.byref(state), C.byref(mesh), C.byref(representative_b), C.byref(side_b), *arguments)
                actual = (a, representative_a.value, side_a.value)
                expected = (b, representative_b.value, side_b.value)
                if actual != expected:
                    raise AssertionError(f"vertex-side mismatch seed={args.seed} case={index} edge={edge}: original={actual}, recovered={expected}")
                helper_checks += 1
    # Keep executable mapping and all function/library references alive through comparisons.
    assert memory is not None and owners is not None
    print(json.dumps({"original_sha256": SHA256, "seed": args.seed,
                      "alignment_cases": args.cases, "alignment_result_counts": alignment_counts,
                      "vertex_side_checks": helper_checks, "mismatches": 0}, indent=2))


if __name__ == "__main__":
    main()
