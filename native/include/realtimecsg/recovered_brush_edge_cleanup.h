#pragma once
#include "realtimecsg/recovered_connections.h"

namespace realtimecsg::recovered {
// Recovered CleanUpBrushEdges including its two-index MergeBrushIndices calls.
// Each tagged edge must have a reciprocal twin. Conflicting assigned indices
// must identify live groups and are merged with the original remap behavior.
// Propagates one-sided sources, removes visible/visible tags, normalizes invalid
// indices, swap-removes unused groups and remaps surviving sources, then appends
// triangle handles in triangle/corner order. Existing lists are retained.
// Native hidden/hidden branch leaves the tags in place; that behavior is kept.
// Swaps geometry, source and connection views together. Removed slots retain
// caller-owned buffers with counts reset to zero. No allocation/free ownership
// crosses the interface; all required list capacities are preflighted.
// Views and their mutable buffers must be disjoint from each other and mesh.
// Errors leave input unchanged. No triangle/vertex topology or positions change.
void cleanUpBrushEdges(TriangleMeshView&, BrushEdgeGroupView*, BrushEdgeTriangleList*,
    BrushEdgeConnectionList*, std::int32_t& groupCount);
// Recovered MergeBrushIndices restricted to count=2. Indices may be -1 or live
// group indices; the pair is reordered like native. Returns retained index or
// -1 for an empty pair. Root geometry/lists stay; surface refs are appended with
// occurrence-sensitive deduplication. Native clears a merged tail's other tags.
// A convenience wrapper around the general merge with exactly two indices.
std::int32_t mergeBrushIndexPair(TriangleMeshView&, BrushEdgeGroupView*, BrushEdgeTriangleList*,
    BrushEdgeConnectionList*, std::int32_t& groupCount, std::int32_t* twoIndices);
// General nonempty index array, including -1 placeholders and duplicates.
// Preserves native array mutations and group removal order. Some duplicate
// orders in the original can address already-destroyed vector slots; such input
// is rejected atomically rather than dereferencing an object past its lifetime.
// Native remap overwrites are preserved, including tags outside the new count;
// this compatibility helper does not guarantee normalized output source tags.
// Return and error guarantees match mergeBrushIndexPair.
std::int32_t mergeBrushIndices(TriangleMeshView&, BrushEdgeGroupView*, BrushEdgeTriangleList*,
    BrushEdgeConnectionList*, std::int32_t& groupCount, std::int32_t* indices, std::int32_t indexCount);
}
extern "C" {
// 0 success; -1 invalid views/indices; -2 capacity; -3 other failure.
std::int32_t rcsg_recovered_cleanup_brush_edges(realtimecsg::recovered::TriangleMeshView*,
    realtimecsg::recovered::BrushEdgeGroupView*, realtimecsg::recovered::BrushEdgeTriangleList*,
    realtimecsg::recovered::BrushEdgeConnectionList*, std::int32_t* groupCount);
// Same status codes; selectedIndex is written only on success and may be -1.
std::int32_t rcsg_recovered_merge_brush_pair(realtimecsg::recovered::TriangleMeshView*,
    realtimecsg::recovered::BrushEdgeGroupView*, realtimecsg::recovered::BrushEdgeTriangleList*,
    realtimecsg::recovered::BrushEdgeConnectionList*, std::int32_t* groupCount,
    std::int32_t* twoIndices, std::int32_t* selectedIndex);
std::int32_t rcsg_recovered_merge_brush_indices(realtimecsg::recovered::TriangleMeshView*,
    realtimecsg::recovered::BrushEdgeGroupView*, realtimecsg::recovered::BrushEdgeTriangleList*,
    realtimecsg::recovered::BrushEdgeConnectionList*, std::int32_t* groupCount,
    std::int32_t* indices, std::int32_t indexCount, std::int32_t* selectedIndex);
}
