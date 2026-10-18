#include "realtimecsg/recovered_gap_dispatch.h"
#include <cmath>
#include <stdexcept>
#include <vector>

namespace realtimecsg::recovered {
std::int32_t fixGaps(TriangleMeshView& mesh, const Plane* planes, int planeCount,
    const BrushSpaceTransform& transform, BrushEdgeTriangleList* sources, int sourceCount,
    const GapLine* lines, const GapConnectionList* connections, const GapDispatchNode* manager,
    int managerCount, int maxSplits) {
    if (sourceCount < 0 || managerCount < 0 || maxSplits < 0 || (!manager && managerCount)
        || (sourceCount && (!sources || !lines || !connections)) || planeCount < 0 || (!planes && planeCount))
        throw std::invalid_argument("Invalid gap dispatcher views");
    for (int i = 0; i < sourceCount; ++i)
        if (connections[i].count < 0 || (!connections[i].connections && connections[i].count))
            throw std::invalid_argument("Invalid gap connections");
    std::vector<GapNeighborBrushView> normals;
    normals.reserve(static_cast<std::size_t>(managerCount));
    for (int i = 0; i < managerCount; ++i) {
        auto view = manager[i].brush;
        if (!manager[i].sourceRecordValid) { view.planes = nullptr; view.planeCount = 0; }
        normals.push_back(view);
    }
    resetGapNeighbors(mesh);
    int splits = 0;
    for (int i = 0; i < sourceCount; ++i) {
        const auto& group = connections[i];
        for (int j = 0; j < group.count; ++j) {
            const auto ref = group.connections[j];
            if (ref.brushIndex < 0 || ref.brushIndex >= managerCount) continue;
            const auto& node = manager[ref.brushIndex];
            if (node.nodeKind != 3 || node.brush.nodeIndex != ref.brushIndex) continue;
            // Native fetches the source group's line before testing its mesh
            // and surface record. Safe API checks that group index explicitly.
            if (node.sourceCount < 0 || ref.edge >= static_cast<unsigned>(node.sourceCount)
                || !node.lines || !node.sources) throw std::invalid_argument("Invalid foreign gap group");
            const auto& a = lines[i].direction;
            const auto& b = node.lines[ref.edge].direction;
            const float alignment = std::abs((a.x*b.x+a.y*b.y)+a.z*b.z);
            // x86 ucomiss + jae skips ordered <= threshold, but admits NaN.
            if (alignment <= kGapDirectionAlignment) continue;
            if (!node.brush.mesh || !node.sourceRecordValid || node.brush.planeCount == 0) continue;
            const auto& foreign = node.sources[ref.edge];
            splits += fixGapGroup(mesh,planes,planeCount,transform,sources,sourceCount,i,lines[i],
                ref.brushIndex,foreign.edges,foreign.count,normals.data(),managerCount,maxSplits-splits);
        }
    }
    return splits;
}
}
extern "C" std::int32_t rcsg_recovered_fix_gaps(realtimecsg::recovered::TriangleMeshView* mesh,
    const realtimecsg::recovered::Plane* planes, std::int32_t pc,
    const realtimecsg::recovered::BrushSpaceTransform* transform, realtimecsg::recovered::BrushEdgeTriangleList* sources,
    std::int32_t sc, const realtimecsg::recovered::GapLine* lines,
    const realtimecsg::recovered::GapConnectionList* connections, const realtimecsg::recovered::GapDispatchNode* manager,
    std::int32_t mc, std::int32_t budget) {
    if (!mesh || !transform) return -1;
    try { return realtimecsg::recovered::fixGaps(*mesh,planes,pc,*transform,sources,sc,lines,connections,manager,mc,budget); }
    catch (const std::invalid_argument&) { return -1; }
    catch (const std::length_error&) { return -2; } catch (...) { return -3; }
}
