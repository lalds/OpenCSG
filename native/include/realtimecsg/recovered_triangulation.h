#pragma once

#include "realtimecsg/recovered_geometry.h"
#include <vector>

namespace realtimecsg::recovered {

// Packed as (triangle index << 2) | corner. Low bits 3 mean invalid;
// the native initialized InvalidIndex value is 0xffffffff.
inline constexpr std::uint32_t kInvalidTriangleEdge = 0xffffffffu;
struct BrushTriangleEdge {
    std::int32_t brushIndex;
    std::uint32_t edge;
};
static_assert(sizeof(BrushTriangleEdge) == 8);
struct TriangleVertexRecord {
    std::uint32_t edge;
    Vector3 position;
};
struct TriangleRecord {
    std::int32_t vertices[3];
    std::uint32_t twins[3];
    Vector3 cornerNormals[3];
    std::int32_t edgeSource[3];
    BrushTriangleEdge brushEdges[3];
    std::byte unidentified_60[12];
    std::int32_t surfaceIndex;
    std::uint8_t flags;
    std::byte unidentified_71;
    std::uint16_t categorizationState;
};
// Partial original TriangleMesh layout, caller-owned fixed capacity.
struct TriangleMeshView {
    TriangleRecord* triangles;
    TriangleVertexRecord* vertices;
    Vector3* worldPositions;
    std::int32_t* worldVertexLookup;
    std::int32_t triangleCount;
    std::int32_t vertexCount;
    std::int32_t triangleCapacity;
    std::int32_t vertexCapacity;
};
static_assert(sizeof(TriangleRecord) == 116 && offsetof(TriangleRecord, surfaceIndex) == 108);
static_assert(offsetof(TriangleRecord, brushEdges) == 72 && offsetof(TriangleRecord, unidentified_60) == 96);
static_assert(sizeof(TriangleVertexRecord) == 16 && offsetof(TriangleVertexRecord, position) == 4);
static_assert(sizeof(void*) != 8 || sizeof(TriangleMeshView) == 48);

// Recovered from Mach-O 0x2f4d4..0x2f820. Emits a fan without removing
// duplicate/collinear vertices. Each array has cornerCount entries, >=3.
// Input boundary twins refer to already existing triangles or an invalid handle.
// Per-corner normals are copied bitwise, not calculated from vertex positions.
// Writes both sides of adjacency and updates vertex edge handles. Unknown bytes
// remain untouched. Arrays must not alias mutable mesh storage or each other.
// Invalid input / insufficient capacity throws BEFORE mutation; no heap growth.
void triangulateConvexPolygon(TriangleMeshView&, const std::int32_t* vertexIndices,
                             const Vector3* cornerNormals, const std::uint32_t* boundaryTwins,
                             std::uint32_t* boundaryEdges, std::int32_t cornerCount,
                             std::int32_t surfaceIndex, std::uint8_t flags, std::uint16_t state);

// Recovered FlagInvisiblePolygons loop (0x208d4), adapted from brush->mesh to
// an explicit mesh argument. Preserves flag bits 6/7 and derives lower bits from
// the signed 16-bit categorization state. Does not change vertices or winding.
void flagTriangleVisibility(TriangleMeshView&);

// Recovered TriangleMesh::Create (0x30810), with caller-owned output storage.
// Destination must be empty with null world buffers. Copies all input vertices,
// removes <=2-edge polygons and reconnects their twins in the input brush,
// then triangulates remaining polygon loops in source order. Does not apply
// visibility flags, Optimize or gap repair. No geometric convexity test exists
// in the original Create; all surviving loops use the convex fan routine.
// Unknown output bytes and unused capacity stay untouched. Errors are atomic.
// Buffers must be disjoint. Surface count is explicit for checked plane access.
void createTriangleMesh(TriangleMeshView&, BrushMemoryView&, const SurfaceView&,
                        std::int32_t sourcePlaneCount);

struct ConcavePolygonView {
    std::int32_t* vertices;
    Vector3* normals;
    std::uint32_t* twins;
    std::int32_t count;
    std::int32_t capacity;
};
struct TriangleIndexPoolView {
    std::int32_t* indices;
    std::int32_t count;
    std::int32_t capacity;
};
// Recovered TriangulateConcavePolygon (0x2f822). Reuses triangle slots popped
// from the pool; mesh counts/capacities stay fixed. Mutates the three polygon
// arrays by stable erasure, retaining native trailing bytes; count becomes 0
// on completion. For <3 corners this is a no-op. Boundary sources are copied
// unconditionally from referenced triangle records, even for low-bit-3 handles:
// their upper bits MUST still identify an existing record. No 0xffffffff input.
// Pool must contain >=count-2 distinct existing slots, disjoint from boundary
// triangle records. All mutable buffers must be disjoint; errors are atomic.
std::int32_t triangulateConcavePolygon(TriangleMeshView&, const SurfaceView&, std::int32_t planeCount,
    ConcavePolygonView&, TriangleIndexPoolView&, std::int32_t surfaceIndex,
    std::uint8_t flags, std::uint16_t state);

struct TriangulatedBrush {
    std::vector<TriangleRecord> triangles;
    std::vector<TriangleVertexRecord> vertices;
};

// New adapter for the convex research pipeline, using recovered Create on a
// temporary copy. Triangulates all fragments (including hidden) to retain the
// original per-brush adjacency. State 3 is recorded; a renderer/exporter must
// reverse winding and normals when consuming it. Applies recovered visibility
// flags after building triangles. No gap repair or welding.
TriangulatedBrush triangulateConvexBrush(const BrushMemoryView&, const SurfaceView&,
                                        std::int32_t sourcePlaneCount);
} // namespace realtimecsg::recovered

extern "C" {
// Returns number emitted; -1 invalid, -2 capacity, -3 unexpected failure.
std::int32_t rcsg_recovered_triangulate_convex(
    realtimecsg::recovered::TriangleMeshView*, const std::int32_t*,
    const realtimecsg::recovered::Vector3*, const std::uint32_t*, std::uint32_t*,
    std::int32_t, std::int32_t, std::uint8_t, std::uint16_t);
std::int32_t rcsg_recovered_flag_triangle_visibility(realtimecsg::recovered::TriangleMeshView*);
// 0 success; -1 invalid input/topology; -2 capacity; -3 other failure.
std::int32_t rcsg_recovered_create_triangle_mesh(realtimecsg::recovered::TriangleMeshView*,
    realtimecsg::recovered::BrushMemoryView*, const realtimecsg::recovered::SurfaceView*, std::int32_t);
// Returns emitted count (0 for <3 corners); -1 invalid, -2 capacity, -3 other.
std::int32_t rcsg_recovered_triangulate_concave(realtimecsg::recovered::TriangleMeshView*,
    const realtimecsg::recovered::SurfaceView*, std::int32_t planeCount,
    realtimecsg::recovered::ConcavePolygonView*, realtimecsg::recovered::TriangleIndexPoolView*,
    std::int32_t surfaceIndex, std::uint8_t flags, std::uint16_t state);
}
