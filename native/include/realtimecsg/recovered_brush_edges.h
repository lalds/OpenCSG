#pragma once
#include "realtimecsg/recovered_gap_neighbors.h"

namespace realtimecsg::recovered {
// Original AABB argument has interleaved bounds, not two Vector3 records.
struct BrushEdgeBounds { float minX,maxX,minY,maxY,minZ,maxZ; };
// Portable bounded projection of the fields FindBrushEdges reads/writes.
// Native BrushEdge also has triangle-handle and resolved-connection vectors;
// neither is populated by this function. surfaceRefs means (brush,surface).
struct BrushEdgeGroupView {
    BrushTriangleEdge* surfaceRefs;
    std::int32_t surfaceRefCount, surfaceRefCapacity;
    std::int32_t surfaceA, surfaceB;
    GapLine line;
};
static_assert(sizeof(BrushEdgeBounds) == 24);

// Recovered FindBrushEdges (0x20b53..0x215b5). mesh.edgeLinks[].unidentified_04
// is the source group index, initialized to -1 before the preparation pass.
// localPlane contributes its normal to the line cross product. Foreign planes
// and bounds must already be expressed in local mesh coordinates. treeTransform
// explicitly replaces the manager's local-to-tree matrix; direction uses its
// linear 3x3 part (as native TransformNormalToTreeSpace), then double normalize.
// Every output slot has caller-owned surfaceRef storage, including unused slots.
// Preserves existing groups and deduplicates (foreignBrush,foreignSurface) refs.
// Errors after earlier accepted edges may leave partial mutations. Not triangle
// edge-list generation, connection resolution, brush broad phase or Unity ABI.
void findBrushEdges(BrushMemoryView&, std::int32_t polygonIndex, const Plane& localPlane,
    const Plane* foreignPlanes, std::int32_t foreignPlaneCount, const BrushEdgeBounds&,
    const BrushSpaceTransform& treeTransform, std::int32_t foreignBrush, std::int32_t foreignSurface,
    BrushEdgeGroupView* groups, std::int32_t& groupCount, std::int32_t groupCapacity);
}
extern "C" {
// 0 success; -1 invalid views/index, -2 capacity, -3 other failure.
std::int32_t rcsg_recovered_find_brush_edges(realtimecsg::recovered::BrushMemoryView*, std::int32_t,
    const realtimecsg::recovered::Plane*, const realtimecsg::recovered::Plane*, std::int32_t,
    const realtimecsg::recovered::BrushEdgeBounds*, const realtimecsg::recovered::BrushSpaceTransform*,
    std::int32_t, std::int32_t, realtimecsg::recovered::BrushEdgeGroupView*, std::int32_t*, std::int32_t);
}
