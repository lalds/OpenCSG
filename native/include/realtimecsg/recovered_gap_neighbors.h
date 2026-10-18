#pragma once

#include "realtimecsg/recovered_gap_intervals.h"

namespace realtimecsg::recovered {
// Explicit substitute for manager nodes and source-surface records. Active
// entries have nodeIndex == array index, a mesh and a nonempty plane array.
struct GapNeighborBrushView {
    TriangleMeshView* mesh;
    const Plane* planes;
    std::int32_t nodeIndex;
    std::int32_t planeCount;
};

// Recovered FixGaps prologue: clear cross-brush references, retain all other bytes.
void resetGapNeighbors(TriangleMeshView&);

// Recovered matched-interval visibility/normal arbitration (0x38b45..0x393cf).
// NEW explicit-group dispatcher: groups must already have matching subdivisions.
// Unmatched intervals are ignored; no splitting, welding or reciprocal writes.
// Local twins are deduplicated; foreign handles are visited individually in
// input order, as in native FixGaps. Existing links participate in arbitration.
// Normals come from source planes, flipped by flags bit 4; visibility is bit 3.
// Greatest dot product wins; equal/unordered comparisons retain the incumbent.
// Invalid manager entries give a zero incumbent normal as in TriangleNormal.
// Invalid handles/surfaces on active entries throw before any changes.
void matchGapNeighbors(TriangleMeshView& local, const Plane* localPlanes, std::int32_t localPlaneCount,
    const GapLine&, const std::uint32_t* localEdges, std::int32_t localCount,
    std::int32_t foreignBrush, const std::uint32_t* foreignEdges, std::int32_t foreignCount,
    const GapNeighborBrushView* manager, std::int32_t managerCount);

// Recovered per-connected-group FixGaps loop, including native temporary
// interval updates after cuts (0x393fb..0x398d0) and neighbor arbitration.
// Caller supplies an already accepted collinear connection; no global manager
// traversal or line-direction filter. Does not reset links: reset once before
// processing all connections. Foreign storage must not alias mutable local
// buffers. Source handles must be unique and owned by sourceIndex.
// New intervals have reverse=0; existing intervals retain their previous bit,
// as in native, even when recollecting geometry would produce another value.
// Returns split count. A later error may leave prior cuts/link writes in place.
std::int32_t fixGapGroup(TriangleMeshView&, const Plane*, std::int32_t,
    const BrushSpaceTransform&, BrushEdgeTriangleList*, std::int32_t, std::int32_t,
    const GapLine&, std::int32_t foreignBrush, const std::uint32_t* foreignEdges,
    std::int32_t foreignCount, const GapNeighborBrushView*, std::int32_t managerCount,
    std::int32_t maxSplits = 1024);
}

extern "C" {
// 0 success, -1 invalid input, -3 unexpected/allocation failure.
std::int32_t rcsg_recovered_reset_gap_neighbors(realtimecsg::recovered::TriangleMeshView*);
std::int32_t rcsg_recovered_match_gap_neighbors(realtimecsg::recovered::TriangleMeshView*,
    const realtimecsg::recovered::Plane*, std::int32_t, const realtimecsg::recovered::GapLine*,
    const std::uint32_t*, std::int32_t, std::int32_t, const std::uint32_t*, std::int32_t,
    const realtimecsg::recovered::GapNeighborBrushView*, std::int32_t);
// Split count on success; -1 invalid, -2 capacity/budget, -3 other failure.
std::int32_t rcsg_recovered_fix_gap_group(realtimecsg::recovered::TriangleMeshView*,
    const realtimecsg::recovered::Plane*, std::int32_t,
    const realtimecsg::recovered::BrushSpaceTransform*, realtimecsg::recovered::BrushEdgeTriangleList*,
    std::int32_t, std::int32_t, const realtimecsg::recovered::GapLine*, std::int32_t,
    const std::uint32_t*, std::int32_t, const realtimecsg::recovered::GapNeighborBrushView*,
    std::int32_t, std::int32_t);
}
