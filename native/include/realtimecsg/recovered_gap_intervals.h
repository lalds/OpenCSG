#pragma once

#include "realtimecsg/recovered_triangle_splitting.h"
#include <vector>

namespace realtimecsg::recovered {

struct GapLine { Vector3 direction; Vector3 origin; };
struct GapInterval {
    float lower, upper;
    std::int32_t lowerVertex, upperVertex;
    std::uint32_t edge;
    std::uint8_t reverse;
    std::byte reserved[3];
};
static_assert(sizeof(GapInterval) == 24 && offsetof(GapInterval, reverse) == 20);
inline constexpr float kGapLineDistanceSquared = 1.0000000116860974e-7f;
inline constexpr float kGapEndpointTolerance = 0.00390625f;

// Recovered interval collection in FixGaps (0x384ce..0x387b3). The unit line
// corresponds to BrushEdge direction/origin at +80/+92. Endpoints use world
// positions. Only accepted twin pairs are deduplicated; repeated same handles
// and zero-length intervals remain, matching native. Hidden edges are included.
// Pure operation: never mutates the mesh. Reserved bytes are initialized here,
// unlike unspecified padding in the native temporary vector.
std::vector<GapInterval> collectGapIntervals(const TriangleMeshView&, const GapLine&,
                                            const std::uint32_t* edges, std::int32_t count);

// NEW bounded adapter joining recovered collection and triangle splitting.
// Caller supplies one existing collinear edge group and a foreign group; this
// is NOT the full FixGaps dispatcher or native visibility/normal arbitration.
// Splits local intervals at foreign endpoints strictly inside by >1/256 on
// both sides, as in native cut tests. Foreign mesh/edges stay unchanged and must
// not alias local buffers. Local source lists are updated by splitTriangleEdge.
// Mutable local group handles must be unique and belong to sourceIndex.
// Each split preflights its buffers; a later failure may leave earlier splits.
// Does not weld positions, connect brushes, remove degenerate geometry, or
// refresh stale per-vertex edge handles. Returns number of new local vertices.
std::int32_t alignGapIntervals(TriangleMeshView& local, const BrushSpaceTransform&,
                               BrushEdgeTriangleList* sources, std::int32_t sourceCount,
                               std::int32_t sourceIndex, const GapLine&,
                               const TriangleMeshView& foreign, const std::uint32_t* foreignEdges,
                               std::int32_t foreignCount, std::int32_t maxSplits = 1024);
} // namespace realtimecsg::recovered

extern "C" {
// Returns required interval count, or -1 invalid, -2 capacity, -3 other error.
// Null output + zero capacity queries size. No partial output on error.
std::int32_t rcsg_recovered_collect_gap_intervals(const realtimecsg::recovered::TriangleMeshView*,
    const realtimecsg::recovered::GapLine*, const std::uint32_t*, std::int32_t,
    realtimecsg::recovered::GapInterval*, std::int32_t);
// New adapter: returns split count, -1 invalid, -2 capacity/budget, -3 other.
// Unlike the pure collector, failures after earlier successful splits can leave
// partial geometry changes. Caller owns all fixed-capacity buffers.
std::int32_t rcsg_recovered_align_gap_intervals(realtimecsg::recovered::TriangleMeshView*,
    const realtimecsg::recovered::BrushSpaceTransform*, realtimecsg::recovered::BrushEdgeTriangleList*,
    std::int32_t, std::int32_t, const realtimecsg::recovered::GapLine*,
    const realtimecsg::recovered::TriangleMeshView*, const std::uint32_t*, std::int32_t, std::int32_t);
}
