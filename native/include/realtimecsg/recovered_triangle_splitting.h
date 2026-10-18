#pragma once

#include "realtimecsg/recovered_triangulation.h"

namespace realtimecsg::recovered {

// Portable replacement for BrushEdge's vector<TriangleEdgeIndex> at +24.
// Buffers and counts are explicit; all ownership stays with the caller.
struct BrushEdgeTriangleList {
    std::uint32_t* edges;
    std::int32_t count;
    std::int32_t capacity;
};
struct BrushSpaceTransform { float columnMajor[16]; };

// Recovered SplitTriangleEdgeIndex (0x37196), AddWorldVertex's successful
// append path (0x2f36a), and TransformToBrushSpace (0x4d2c). A closed, reciprocal
// triangle edge pair is required. Adds one vertex and two triangles. World point
// is supplied by the caller; weight is used for native corner-normal operations.
// The native second-side normal formula is preserved, including its use of one
// endpoint twice. This routine does not find T junctions or decide where to cut.
// Existing per-vertex edge handles are not refreshed (some can become stale);
// triangle twins and the new vertex handle are updated, matching the original.
//
// mesh.worldPositions and worldVertexLookup must each have vertexCapacity slots,
// independently caller-owned (native AddWorldVertex reallocates a combined block).
// At least TWO spare vertex slots and two spare triangle slots are required so
// only the reconstructed nongrowing native capacity path is used. VertexCount>0.
// All input/output buffers must not alias. Validation/capacity failure is reported
// before mutation. Returned handle is the second segment on the selected side.
std::uint32_t splitTriangleEdge(TriangleMeshView&, const BrushSpaceTransform&,
                                BrushEdgeTriangleList*, std::int32_t brushEdgeCount,
                                std::uint32_t edge, const Vector3& worldPoint, float weight);

} // namespace realtimecsg::recovered

extern "C" {
// 0 success; -1 invalid input; -2 capacity; -3 unexpected failure.
std::int32_t rcsg_recovered_split_triangle_edge(
    realtimecsg::recovered::TriangleMeshView*, const realtimecsg::recovered::BrushSpaceTransform*,
    realtimecsg::recovered::BrushEdgeTriangleList*, std::int32_t, std::uint32_t,
    const realtimecsg::recovered::Vector3*, float, std::uint32_t*);
}
