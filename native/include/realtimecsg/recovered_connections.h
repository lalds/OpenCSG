#pragma once
#include "realtimecsg/recovered_brush_edges.h"

namespace realtimecsg::recovered {
struct BrushEdgeConnectionList {
    BrushTriangleEdge* connections;
    std::int32_t count, capacity;
};
struct BrushConnectionNode {
    const BrushEdgeGroupView* groups;
    std::int32_t groupCount, nodeIndex, nodeKind;
};

// Recovered macOS FindConnections (0x35840..0x35de6), explicit manager.
// IMPORTANT observed behavior: for each distinct referenced brush ID, native
// scans manager[brushIndex], not manager[referencedID], and appends
// {brushIndex,candidateGroup}. No automatic correction of that lookup here.
// References are deduplicated/sorted by signed brush ID. Matching uses line
// alignment/proximity and candidate surfaceRefs against the current surface pair.
// Existing connections are retained, new matches appended without deduplication.
// If the final connection list is empty, listed triangle edgeSource fields are
// cleared to -1; the source-handle list itself is retained. Other mesh fields
// stay unchanged. Invalid active handles and capacities are checked safely;
// later failure may leave earlier appends or edgeSource clears.
// This behavior can yield self references; direct use in the independent-mesh
// FixGaps adapter requires further integration work and Windows verification.
void findConnections(TriangleMeshView&, std::int32_t brushIndex,
    const BrushEdgeGroupView*, const BrushEdgeTriangleList*, BrushEdgeConnectionList*,
    std::int32_t groupCount, const BrushConnectionNode*, std::int32_t managerCount);
}
extern "C" {
// 0 success; -1 invalid, -2 capacity, -3 unexpected/allocation error.
std::int32_t rcsg_recovered_find_connections(realtimecsg::recovered::TriangleMeshView*, std::int32_t,
    const realtimecsg::recovered::BrushEdgeGroupView*, const realtimecsg::recovered::BrushEdgeTriangleList*,
    realtimecsg::recovered::BrushEdgeConnectionList*, std::int32_t,
    const realtimecsg::recovered::BrushConnectionNode*, std::int32_t);
}
