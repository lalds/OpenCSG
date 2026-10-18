#!/usr/bin/env python3
"""Compare recovered topology mutations and SplitByPlane with original code.

Executes isolated, hash-pinned Mach-O routines on Linux x86_64, using preallocated
buffers so no original heap or macOS routine is called. Run as a separate process.
Closed double-sided polygons and rotated boxes exercise local topology, not full CSG.
"""

import argparse
import ctypes as C
import json
import math
from pathlib import Path
import random

from compare_recovered_geometry import (
    Edge, EdgeLink, Plane, Polygon, SHA256, Surfaces, Vector3, distance, f32, original_functions,
)


class Memory(C.Structure):
    _fields_ = [("polygons", C.POINTER(Polygon)), ("edges", C.POINTER(Edge)),
                ("edge_links", C.POINTER(EdgeLink)), ("vertices", C.POINTER(Vector3))]
    _fields_ += [(n, C.c_int32) for n in ("polygon_count", "edge_count", "vertex_count",
                                        "polygon_capacity", "edge_chunk_capacity", "vertex_capacity")]


INSERT = C.CFUNCTYPE(C.c_int32, C.POINTER(Memory), C.c_int32, C.c_int32, C.c_int32)
SPLIT_EDGES = C.CFUNCTYPE(C.c_uint8, C.POINTER(Memory), C.c_int32, C.c_int32,
                         C.c_int32, C.POINTER(C.c_int32))
SPLIT_PLANE = C.CFUNCTYPE(C.c_uint8, C.POINTER(Memory), C.POINTER(Surfaces), C.c_int32,
                         C.POINTER(Plane), C.POINTER(C.c_int32))


class Fixture:
    def __init__(self, points, faces, planes):
        self.polygons = (Polygon * 64)()
        self.edges = (Edge * 512)()
        self.links = (EdgeLink * 512)()
        self.vertices = (Vector3 * 128)()
        self.planes = (Plane * len(planes))(*planes)
        # Poison unused fields identically to detect writes the original does not make.
        for array in (self.polygons, self.edges, self.links, self.vertices):
            C.memset(C.addressof(array), 0xa5, C.sizeof(array))
        for i, point in enumerate(points):
            self.vertices[i] = Vector3(*point)
        ends = {}
        edge_index = 0
        for polygon, face in enumerate(faces):
            first = edge_index
            count = len(face)
            self.polygons[polygon] = Polygon(count, first, polygon, 0, 0x1234, 0xffff)
            for i, vertex in enumerate(face):
                edge = first + i
                previous = first + (i - 1) % count
                self.edges[edge] = Edge(first + (i + 1) % count, vertex, -1, polygon)
                self.links[edge] = EdgeLink(previous, -1)
                ends[(face[i - 1], vertex)] = edge
            edge_index += count
        for (a, b), edge in ends.items():
            self.edges[edge].twin_edge = ends[(b, a)]
        self.memory = Memory(self.polygons, self.edges, self.links, self.vertices,
                             len(faces), edge_index, len(points), 64, 128, 128)
        self.surfaces = Surfaces()
        self.surfaces.planes = self.planes

    def snapshot(self):
        m = self.memory
        return ((m.polygon_count, m.edge_count, m.vertex_count),
                bytes(self.polygons), bytes(self.edges), bytes(self.links), bytes(self.vertices))

    def validate(self):
        m = self.memory
        seen = set()
        for p in range(m.polygon_count):
            polygon = self.polygons[p]
            edge = polygon.first_edge
            first = edge
            for _ in range(polygon.edge_count):
                assert 0 <= edge < m.edge_count
                assert edge not in seen, ("repeated edge", p, edge)
                seen.add(edge)
                record = self.edges[edge]
                assert record.polygon_index == p
                assert 0 <= record.vertex_index < m.vertex_count
                assert self.links[record.next_edge].previous_edge == edge
                assert self.edges[self.links[edge].previous_edge].next_edge == edge
                twin = record.twin_edge
                assert 0 <= twin < m.edge_count
                assert self.edges[twin].twin_edge == edge
                assert self.edges[twin].vertex_index == self.edges[self.links[edge].previous_edge].vertex_index
                edge = record.next_edge
            assert edge == first, ("unclosed polygon", p)
        assert len(seen) == m.edge_count, ("orphan edges", len(seen), m.edge_count)


def polygon_inputs(count, scale, height, angle=0.0):
    points = [(math.cos(angle + i * 2 * math.pi / count) * scale,
               math.sin(angle + i * 2 * math.pi / count) * scale, height) for i in range(count)]
    faces = [list(range(count)), list(reversed(range(count)))]
    planes = [Plane(Vector3(0, 0, 1), height), Plane(Vector3(0, 0, -1), -height)]
    return points, faces, planes


def cube_inputs(rng):
    center = [rng.uniform(-2, 2) for _ in range(3)]
    sizes = [rng.choice((0.01, 1.0, 100.0)) for _ in range(3)]
    a, b = rng.uniform(-math.pi, math.pi), rng.uniform(-math.pi, math.pi)
    ca, sa, cb, sb = math.cos(a), math.sin(a), math.cos(b), math.sin(b)
    axes = [(ca*cb, sa*cb, -sb), (-sa, ca, 0), (ca*sb, sa*sb, cb)]
    signs = [(-1,-1,-1), (1,-1,-1), (1,1,-1), (-1,1,-1),
             (-1,-1,1), (1,-1,1), (1,1,1), (-1,1,1)]
    points = [tuple(center[k] + sum(sign[i] * sizes[i] * axes[i][k] for i in range(3))
                    for k in range(3)) for sign in signs]
    faces = [[3,2,1,0], [4,5,6,7], [0,1,5,4], [1,2,6,5], [2,3,7,6], [3,0,4,7]]
    planes = []
    for axis, sign in [(2,-1), (2,1), (1,-1), (0,1), (1,1), (0,-1)]:
        normal = tuple(sign * x for x in axes[axis])
        planes.append(Plane(Vector3(*normal), sum(normal[k] * center[k] for k in range(3)) + sizes[axis]))
    return points, faces, planes


def compare(operation, index, a, b, original_result, recovered_result):
    if original_result != recovered_result or a.snapshot() != b.snapshot():
        summary = {"operation": operation, "case": index,
                   "original_result": original_result, "recovered_result": recovered_result,
                   "original_counts": a.snapshot()[0], "recovered_counts": b.snapshot()[0]}
        for name, x, y in zip(("counts", "polygons", "edges", "links", "vertices"), a.snapshot(), b.snapshot()):
            if x != y:
                summary[name] = [(i, v, w) for i, (v, w) in enumerate(zip(x, y)) if v != w][:20]
        raise AssertionError(json.dumps(summary))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--library", type=Path, required=True)
    parser.add_argument("--cases", type=int, default=2000)
    parser.add_argument("--seed", type=int, default=1559)
    args = parser.parse_args()
    if args.cases < 1:
        parser.error("--cases must be positive")
    memory, alignment, side = original_functions(include_splitting=True)
    base = C.cast(alignment, C.c_void_p).value - 0x2cc04
    original_insert, original_split_edges = INSERT(base + 0x64b6), SPLIT_EDGES(base + 0x65f8)
    original_split_plane = SPLIT_PLANE(base + 0x2d15c)
    library = C.CDLL(str(args.library.resolve()))
    recovered_insert = INSERT(("rcsg_recovered_insert_vertex", library))
    recovered_split_edges = SPLIT_EDGES(("rcsg_recovered_split_edges", library))
    recovered_split_plane = SPLIT_PLANE(("rcsg_recovered_split_plane", library))
    rng = random.Random(args.seed)
    results = [0] * 6
    cube_checks = 0
    for index in range(args.cases):
        n = rng.randrange(3, 17)
        scale = rng.choice((0.001, 0.01, 1.0, 100.0))
        inputs = polygon_inputs(n, scale, rng.uniform(-2, 2), rng.uniform(-math.pi, math.pi))
        if index % 16 == 0:
            # Two intersections may be closer than the vertex merge threshold.
            # This exercises reuse of an existing vertex and degenerate split loops.
            inputs = ([(x, y * 0.00001, z) for x, y, z in inputs[0]], inputs[1], inputs[2])
        a, b = Fixture(*inputs), Fixture(*inputs)
        previous = rng.randrange(n)
        following = a.edges[previous].next_edge
        position_a, position_b = a.vertices[a.edges[previous].vertex_index], a.vertices[a.edges[following].vertex_index]
        position = Vector3((position_a.x + position_b.x) / 2,
                           (position_a.y + position_b.y) / 2, (position_a.z + position_b.z) / 2)
        for fixture in (a, b):
            fixture.vertices[n] = position
            fixture.memory.vertex_count += 1
        ra = original_insert(C.byref(a.memory), previous, following, n)
        rb = recovered_insert(C.byref(b.memory), previous, following, n)
        compare("insert", index, a, b, ra, rb)
        a.validate()
        b.validate()
        if index % 32 == 0:
            # Exercise real closed polyhedra and repeated mutations, including
            # previously created coplanar faces, not just isolated polygon rings.
            cube = cube_inputs(rng)
            a, b = Fixture(*cube), Fixture(*cube)
            a.validate()
            for step in range(12):
                polygon = rng.randrange(a.memory.polygon_count)
                record = a.polygons[polygon]
                edge = record.first_edge
                center = [0.0, 0.0, 0.0]
                for _ in range(record.edge_count):
                    point = a.vertices[a.edges[edge].vertex_index]
                    for k, value in enumerate((point.x, point.y, point.z)):
                        center[k] += value / record.edge_count
                    edge = a.edges[edge].next_edge
                normal = [rng.uniform(-1, 1) for _ in range(3)]
                length = math.sqrt(sum(x*x for x in normal))
                plane = Plane(Vector3(*(x / length for x in normal)), 0)
                plane.distance = distance(plane, Vector3(*center))
                output_a, output_b = C.c_int32(-10), C.c_int32(-10)
                ra = original_split_plane(C.byref(a.memory), C.byref(a.surfaces), polygon, C.byref(plane), C.byref(output_a))
                rb = recovered_split_plane(C.byref(b.memory), C.byref(b.surfaces), polygon, C.byref(plane), C.byref(output_b))
                compare("cube_split_" + str(step), index, a, b, (ra, output_a.value), (rb, output_b.value))
                a.validate()
                b.validate()
                cube_checks += 1

        a, b = Fixture(*inputs), Fixture(*inputs)
        first = rng.randrange(n)
        second = (first + rng.randrange(1, n)) % n
        if index % 8 == 0:
            # Same vertex ID at both split boundaries takes a distinct no-new-edge
            # branch. Update both faces so the original twin endpoints remain valid.
            keep = a.edges[first].vertex_index
            replace = a.edges[second].vertex_index
            for fixture in (a, b):
                for edge in fixture.edges[:fixture.memory.edge_count]:
                    if edge.vertex_index == replace:
                        edge.vertex_index = keep
        output_a, output_b = C.c_int32(-10), C.c_int32(-10)
        ra = original_split_edges(C.byref(a.memory), 0, first, second, C.byref(output_a))
        rb = recovered_split_edges(C.byref(b.memory), 0, first, second, C.byref(output_b))
        compare("split_edges", index, a, b, (ra, output_a.value), (rb, output_b.value))
        a.validate()
        b.validate()

        a, b = Fixture(*inputs), Fixture(*inputs)
        angle = rng.uniform(-math.pi, math.pi)
        plane = Plane(Vector3(math.cos(angle), math.sin(angle), 0), rng.uniform(-1.2, 1.2) * scale)
        mode = index % 8
        if mode in (1, 5, 6):
            point = a.vertices[rng.randrange(n)]
            plane.distance = distance(plane, point) + plane.distance
            if mode == 5:
                plane.distance = f32(plane.distance + 0.0006)
            elif mode == 6:
                plane.distance = f32(plane.distance - 0.0006)
        elif mode == 2:
            p, q = a.vertices[0], a.vertices[1]
            dx, dy = q.x - p.x, q.y - p.y
            length = math.hypot(dx, dy)
            plane = Plane(Vector3(dy / length, -dx / length, 0), 0)
            plane.distance = distance(plane, p)
        elif mode == 3:
            plane = Plane(Vector3(0, 0, 1), a.planes[0].distance)
        elif mode == 4:
            plane = Plane(Vector3(0, 0, -1), -a.planes[0].distance)
        elif mode == 7:
            x, y, z = (rng.uniform(-1, 1) for _ in range(3))
            length = math.sqrt(x*x + y*y + z*z)
            plane = Plane(Vector3(x / length, y / length, z / length), rng.uniform(-scale, scale))
        output_a, output_b = C.c_int32(-10), C.c_int32(-10)
        ra = original_split_plane(C.byref(a.memory), C.byref(a.surfaces), 0, C.byref(plane), C.byref(output_a))
        rb = recovered_split_plane(C.byref(b.memory), C.byref(b.surfaces), 0, C.byref(plane), C.byref(output_b))
        compare("split_plane", index, a, b, (ra, output_a.value), (rb, output_b.value))
        results[ra] += 1
        a.validate()
        b.validate()
    assert memory is not None and side is not None
    print(json.dumps({"original_sha256": SHA256, "seed": args.seed,
                      "insert_cases": args.cases, "split_edge_cases": args.cases,
                      "split_plane_cases": args.cases, "split_plane_results": results,
                      "repeated_cube_splits": cube_checks,
                      "mismatches": 0}, indent=2))


if __name__ == "__main__":
    main()
