#pragma once
#include "realtimecsg/recovered_gap_neighbors.h"

namespace realtimecsg::recovered {
struct GapConnectionList {
    const BrushTriangleEdge* connections; // brush index, source-group index
    std::int32_t count;
};
// Explicit snapshot of native node metadata, source records and BrushEdges.
// sourceRecordValid models a sourceMeshIndex inside the manager surface array.
// The normal lookup still uses brush regardless of nodeKind, as TriangleNormal
// does; nodeKind == 3 is required only when accepting a connected brush.
struct GapDispatchNode {
    GapNeighborBrushView brush;
    const BrushEdgeTriangleList* sources;
    const GapLine* lines;
    std::int32_t sourceCount;
    std::int32_t nodeKind;
    std::int32_t sourceRecordValid;
};
inline constexpr float kGapDirectionAlignment = 0.9999899864196777f;

// Recovered FixGaps outer traversal (0x380ca..0x384ce, 0x398f3..0x39944),
// composed with fixGapGroup. Resets all local cross-brush links once, visits
// source groups and connections in supplied order, accepts only kind-3 nodes
// with matching nodeIndex, live mesh/source record and nonempty source planes.
// Requires abs(dot(localDirection,foreignDirection)) > kGapDirectionAlignment.
// No brush-edge discovery, welding, optimization or reciprocal ConnectGaps.
// Invalid/absent nodes are skipped. Bad group indices on active nodes are
// rejected safely rather than dereferenced unchecked as in native code.
// Fixed-capacity local buffers; maxSplits is a total budget across all groups.
// Failure can leave the reset and any previously completed cuts/link writes.
std::int32_t fixGaps(TriangleMeshView&, const Plane*, std::int32_t,
    const BrushSpaceTransform&, BrushEdgeTriangleList*, std::int32_t,
    const GapLine*, const GapConnectionList*, const GapDispatchNode*, std::int32_t,
    std::int32_t maxSplits = 1024);
}
extern "C" {
// Split count on success; -1 invalid, -2 capacity/budget, -3 other failure.
std::int32_t rcsg_recovered_fix_gaps(realtimecsg::recovered::TriangleMeshView*,
    const realtimecsg::recovered::Plane*, std::int32_t,
    const realtimecsg::recovered::BrushSpaceTransform*, realtimecsg::recovered::BrushEdgeTriangleList*,
    std::int32_t, const realtimecsg::recovered::GapLine*,
    const realtimecsg::recovered::GapConnectionList*, const realtimecsg::recovered::GapDispatchNode*,
    std::int32_t, std::int32_t);
}
