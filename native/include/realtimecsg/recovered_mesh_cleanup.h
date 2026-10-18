#pragma once

#include "realtimecsg/recovered_triangulation.h"

namespace realtimecsg::recovered {

// Recovered CleanUpRemovedVertices (0x32b82). Lists contain unique old indices
// and are sorted in place, as in OptimizationContext. Removed triangles must
// already have surfaceIndex < 0; surviving triangles may not use removed vertices.
// Vertex/triangle buffers keep capacity and unused bytes, except moved records.
// No geometric removal decisions or topology rewiring are performed here.
// Use BEFORE world-position buffers and cross-brush gap references are built.
void cleanUpRemovedVertices(TriangleMeshView&, std::int32_t* removedVertices, std::int32_t vertexRemovalCount,
                           std::int32_t* removedTriangles, std::int32_t triangleRemovalCount);

// New export adapter: copy, remove hidden triangles and unreferenced vertices,
// run recovered compaction. Empty output is handled explicitly. Clears cross-
// brush links and source-group tags: no group lists accompany this export.
// Those references must be rebuilt after renumbering. Does NOT remove slivers,
// weld vertices, or rewrite topology to collapse geometric degeneracies.
TriangulatedBrush compactVisibleTriangles(const TriangulatedBrush&);

// Portable manager substitute. nodeIndex must equal its array position for
// an active entry. sourcePlaneCount belongs to that brush's surface array.
struct GapBrushView {
    TriangleMeshView* mesh;
    std::int32_t nodeIndex;
    std::int32_t sourcePlaneCount;
};

// Recovered ConnectGaps (0x3a24c), with explicit manager instead of global nodes.
// Reads triangle.brushEdges and writes the reciprocal handle in the other mesh.
// It consumes existing correspondences; it does not discover/split/weld gaps.
// Invalid/absent manager nodes are skipped as in native code. Bad edge handles
// on active nodes are rejected before writes rather than dereferenced unchecked.
// For nonempty input, brushIndex must identify this mesh in manager.
void connectGaps(TriangleMeshView&, std::int32_t brushIndex, std::int32_t sourcePlaneCount,
                 const GapBrushView* manager, std::int32_t managerCount);

} // namespace realtimecsg::recovered

extern "C" {
// 0 success; -1 invalid input, -3 allocation/unexpected failure.
std::int32_t rcsg_recovered_cleanup_mesh(realtimecsg::recovered::TriangleMeshView*,
                                        std::int32_t*, std::int32_t, std::int32_t*, std::int32_t);
std::int32_t rcsg_recovered_connect_gaps(realtimecsg::recovered::TriangleMeshView*, std::int32_t,
                                       std::int32_t, const realtimecsg::recovered::GapBrushView*, std::int32_t);
}
