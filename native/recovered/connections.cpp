#include "realtimecsg/recovered_connections.h"
#include "realtimecsg/recovered_gap_dispatch.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace realtimecsg::recovered {
namespace {
float dot(Vector3 a, Vector3 b) { return (a.x*b.x+a.y*b.y)+a.z*b.z; }
void validateGroup(const BrushEdgeGroupView& group) {
    if (group.surfaceRefCount < 0 || group.surfaceRefCount > group.surfaceRefCapacity
        || (!group.surfaceRefs && group.surfaceRefCount)) throw std::invalid_argument("Invalid surface references");
}
}
void findConnections(TriangleMeshView& mesh, int brushIndex, const BrushEdgeGroupView* groups,
    const BrushEdgeTriangleList* sources, BrushEdgeConnectionList* connections, int groupCount,
    const BrushConnectionNode* manager, int managerCount) {
    if (groupCount < 0 || managerCount < 0 || (groupCount && (!groups || !sources || !connections))
        || (managerCount && !manager) || mesh.triangleCount < 0 || mesh.triangleCount > mesh.triangleCapacity
        || (!mesh.triangles && mesh.triangleCount)) throw std::invalid_argument("Invalid connection views");
    for (int g = 0; g < groupCount; ++g) {
        const auto& group = groups[g]; auto& output = connections[g]; const auto& source = sources[g];
        validateGroup(group);
        if (output.count < 0 || output.count > output.capacity || (!output.connections && output.capacity)
            || source.count < 0 || source.count > source.capacity || (!source.edges && source.count))
            throw std::invalid_argument("Invalid connection/source storage");
        std::vector<int> referenced;
        referenced.reserve(static_cast<std::size_t>(group.surfaceRefCount));
        for (int r = 0; r < group.surfaceRefCount; ++r) referenced.push_back(group.surfaceRefs[r].brushIndex);
        std::sort(referenced.begin(),referenced.end());
        referenced.erase(std::unique(referenced.begin(),referenced.end()),referenced.end());
        for (const int referencedBrush : referenced) {
            if (brushIndex < 0 || brushIndex >= managerCount) continue;
            const auto& node = manager[brushIndex];
            if (node.nodeKind != 3 || node.nodeIndex != brushIndex) continue;
            if (node.groupCount < 0 || (!node.groups && node.groupCount)) throw std::invalid_argument("Invalid candidate groups");
            for (int candidate = 0; candidate < node.groupCount; ++candidate) {
                const auto& other = node.groups[candidate];
                if (std::abs(dot(other.line.direction,group.line.direction)) <= kGapDirectionAlignment) continue;
                const Vector3 delta{other.line.origin.x-group.line.origin.x,other.line.origin.y-group.line.origin.y,
                                    other.line.origin.z-group.line.origin.z};
                const float parameter = dot(delta,group.line.direction);
                const Vector3 distance{group.line.direction.x*parameter+group.line.origin.x-other.line.origin.x,
                    group.line.direction.y*parameter+group.line.origin.y-other.line.origin.y,
                    group.line.direction.z*parameter+group.line.origin.z-other.line.origin.z};
                if (!(dot(distance,distance) < kGapLineDistanceSquared)) continue;
                validateGroup(other);
                bool match = false;
                for (int r = 0; r < other.surfaceRefCount; ++r) {
                    const auto& ref = other.surfaceRefs[r];
                    if (ref.brushIndex == referencedBrush && (ref.edge == static_cast<unsigned>(group.surfaceA)
                        || ref.edge == static_cast<unsigned>(group.surfaceB))) { match = true; break; }
                }
                if (!match) continue;
                if (output.count == output.capacity) throw std::length_error("Connection capacity exhausted");
                output.connections[output.count++] = {brushIndex,static_cast<unsigned>(candidate)};
            }
        }
        if (output.count) continue;
        for (int i = 0; i < source.count; ++i) {
            const auto e = source.edges[i];
            if ((e&3u) == 3u || e>>2 >= static_cast<unsigned>(mesh.triangleCount))
                throw std::invalid_argument("Invalid unconnected triangle handle");
        }
        for (int i = 0; i < source.count; ++i) {
            const auto e = source.edges[i]; mesh.triangles[e>>2].edgeSource[e&3u] = -1;
        }
    }
}
}
extern "C" std::int32_t rcsg_recovered_find_connections(realtimecsg::recovered::TriangleMeshView* mesh,
    std::int32_t brush, const realtimecsg::recovered::BrushEdgeGroupView* groups,
    const realtimecsg::recovered::BrushEdgeTriangleList* sources,
    realtimecsg::recovered::BrushEdgeConnectionList* connections, std::int32_t count,
    const realtimecsg::recovered::BrushConnectionNode* manager, std::int32_t managerCount) {
    if (!mesh) return -1;
    try { realtimecsg::recovered::findConnections(*mesh,brush,groups,sources,connections,count,manager,managerCount); return 0; }
    catch (const std::invalid_argument&) { return -1; }
    catch (const std::length_error&) { return -2; } catch (...) { return -3; }
}
