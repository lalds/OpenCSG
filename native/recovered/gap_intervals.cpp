#include "realtimecsg/recovered_gap_intervals.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>

namespace realtimecsg::recovered {
namespace {
bool project(const Vector3& p, const GapLine& line, float& parameter) {
    const float x = p.x-line.origin.x, y = p.y-line.origin.y, z = p.z-line.origin.z;
    parameter = (x*line.direction.x+y*line.direction.y)+z*line.direction.z;
    const float dx = (line.direction.x*parameter+line.origin.x)-p.x;
    const float dy = (line.direction.y*parameter+line.origin.y)-p.y;
    const float dz = (line.direction.z*parameter+line.origin.z)-p.z;
    return ((dx*dx+dy*dy)+dz*dz) < kGapLineDistanceSquared;
}
void validate(const TriangleMeshView& mesh, const GapLine& line, const std::uint32_t* edges, int count) {
    if (count < 0 || (!edges && count) || mesh.triangleCount < 0 || mesh.vertexCount < 0
        || mesh.triangleCount > mesh.triangleCapacity || mesh.vertexCount > mesh.vertexCapacity
        || (!mesh.triangles && mesh.triangleCount) || (!mesh.worldPositions && mesh.vertexCount))
        throw std::invalid_argument("Invalid gap interval views");
    const auto& n = line.direction;
    const float lengthSquared = (n.x*n.x+n.y*n.y)+n.z*n.z;
    if (!std::isfinite(lengthSquared) || std::abs(lengthSquared-1.0f) > 1e-5f
        || !std::isfinite(line.origin.x) || !std::isfinite(line.origin.y) || !std::isfinite(line.origin.z))
        throw std::invalid_argument("Gap line must have finite origin and unit direction");
    for (int i = 0; i < count; ++i) {
        const auto edge = edges[i];
        if ((edge & 3u) == 3u || edge>>2 >= static_cast<unsigned>(mesh.triangleCount))
            throw std::invalid_argument("Invalid interval triangle edge");
        const auto& t = mesh.triangles[edge>>2];
        const int start = t.vertices[((edge & 3u)+2)%3], end = t.vertices[edge & 3u];
        if (start < 0 || start >= mesh.vertexCount || end < 0 || end >= mesh.vertexCount)
            throw std::invalid_argument("Invalid interval endpoint");
    }
}
} // namespace

std::vector<GapInterval> collectGapIntervals(const TriangleMeshView& mesh, const GapLine& line,
                                            const std::uint32_t* edges, int count) {
    validate(mesh,line,edges,count);
    std::vector<GapInterval> result;
    result.reserve(static_cast<std::size_t>(count));
    for (int i = 0; i < count; ++i) {
        const auto edge = edges[i];
        const auto& triangle = mesh.triangles[edge>>2];
        const unsigned corner = edge & 3u;
        const auto twin = triangle.twins[corner];
        if (std::any_of(result.begin(),result.end(),[twin](const auto& interval) { return interval.edge == twin; })) continue;
        int a = triangle.vertices[(corner+2)%3], b = triangle.vertices[corner];
        float lower, upper;
        if (!project(mesh.worldPositions[a],line,lower) || !project(mesh.worldPositions[b],line,upper)) continue;
        auto reverse = static_cast<std::uint8_t>((triangle.flags>>4)&1u);
        if (lower > upper) { std::swap(lower,upper); std::swap(a,b); reverse ^= 1u; }
        result.push_back({lower,upper,a,b,edge,reverse,{}});
    }
    return result;
}

std::int32_t alignGapIntervals(TriangleMeshView& local, const BrushSpaceTransform& transform,
                               BrushEdgeTriangleList* sources, int sourceCount, int sourceIndex,
                               const GapLine& line, const TriangleMeshView& foreign,
                               const std::uint32_t* foreignEdges, int foreignCount, int maxSplits) {
    if (sourceCount <= 0 || !sources || sourceIndex < 0 || sourceIndex >= sourceCount || maxSplits < 0
        || &local == &foreign || local.triangles == foreign.triangles || local.worldPositions == foreign.worldPositions)
        throw std::invalid_argument("Invalid independent gap groups");
    auto& group = sources[sourceIndex];
    if (group.count < 0 || group.count > group.capacity || (!group.edges && group.capacity))
        throw std::invalid_argument("Invalid local gap source list");
    const auto candidates = collectGapIntervals(foreign,line,foreignEdges,foreignCount);
    auto intervals = collectGapIntervals(local,line,group.edges,group.count);
    for (int i = 0; i < group.count; ++i) {
        const auto edge = group.edges[i]; // Already index-validated by collection.
        if (local.triangles[edge>>2].edgeSource[edge & 3u] != sourceIndex)
            throw std::invalid_argument("Gap group does not own local triangle edges");
        for (int j = 0; j < i; ++j) if (group.edges[j] == edge)
            throw std::invalid_argument("Mutable gap group contains duplicate handles");
    }
    int splits = 0;
    for (const auto& candidate : candidates) {
        for (int end = 0; end < 2; ++end) {
            const float parameter = end == 0 ? candidate.lower : candidate.upper;
            const int vertex = end == 0 ? candidate.lowerVertex : candidate.upperVertex;
            const Vector3 point = foreign.worldPositions[vertex];
            // Refresh from actual updated source lists after every mutation.
            // This is new orchestration, not the native temporary-vector update.
            for (;;) {
                intervals = collectGapIntervals(local,line,group.edges,group.count);
                const auto found = std::find_if(intervals.begin(),intervals.end(),[parameter](const auto& interval) {
                    return parameter-interval.lower > kGapEndpointTolerance
                        && parameter-interval.upper < -kGapEndpointTolerance;
                });
                if (found == intervals.end()) break;
                if (splits == maxSplits) throw std::length_error("Gap split budget exhausted");
                const float weight = (parameter-found->lower)/(found->upper-found->lower);
                splitTriangleEdge(local,transform,sources,sourceCount,found->edge,point,weight);
                ++splits;
            }
        }
    }
    return splits;
}
} // namespace realtimecsg::recovered

extern "C" std::int32_t rcsg_recovered_collect_gap_intervals(
    const realtimecsg::recovered::TriangleMeshView* mesh, const realtimecsg::recovered::GapLine* line,
    const std::uint32_t* edges, std::int32_t count, realtimecsg::recovered::GapInterval* output, std::int32_t capacity) {
    if (!mesh || !line || capacity < 0 || (!output && capacity)) return -1;
    try {
        const auto intervals = realtimecsg::recovered::collectGapIntervals(*mesh,*line,edges,count);
        if (intervals.size() > static_cast<std::size_t>(std::numeric_limits<int>::max())) return -3;
        const int required = static_cast<int>(intervals.size());
        if (!output && capacity == 0) return required;
        if (required > capacity) return -2;
        if (required) std::memcpy(output,intervals.data(),intervals.size()*sizeof(intervals[0]));
        return required;
    } catch (const std::invalid_argument&) { return -1; }
    catch (...) { return -3; }
}

extern "C" std::int32_t rcsg_recovered_align_gap_intervals(
    realtimecsg::recovered::TriangleMeshView* mesh, const realtimecsg::recovered::BrushSpaceTransform* transform,
    realtimecsg::recovered::BrushEdgeTriangleList* sources, std::int32_t count, std::int32_t index,
    const realtimecsg::recovered::GapLine* line, const realtimecsg::recovered::TriangleMeshView* foreign,
    const std::uint32_t* edges, std::int32_t edgeCount, std::int32_t maxSplits) {
    if (!mesh || !transform || !line || !foreign) return -1;
    try { return realtimecsg::recovered::alignGapIntervals(*mesh,*transform,sources,count,index,*line,*foreign,edges,edgeCount,maxSplits); }
    catch (const std::invalid_argument&) { return -1; }
    catch (const std::length_error&) { return -2; }
    catch (...) { return -3; }
}
