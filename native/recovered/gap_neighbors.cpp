#include "realtimecsg/recovered_gap_neighbors.h"

#include <cmath>
#include <cstring>
#include <stdexcept>

namespace realtimecsg::recovered {
namespace {
void validateMesh(const TriangleMeshView& mesh) {
    if (mesh.triangleCount < 0 || mesh.triangleCount > mesh.triangleCapacity
        || (!mesh.triangles && mesh.triangleCount)) throw std::invalid_argument("Invalid triangle view");
}
const TriangleRecord& triangle(const TriangleMeshView& mesh, std::uint32_t edge) {
    validateMesh(mesh);
    if ((edge&3u) == 3u || edge>>2 >= static_cast<unsigned>(mesh.triangleCount))
        throw std::invalid_argument("Invalid neighbor edge");
    return mesh.triangles[edge>>2];
}
Vector3 normal(const TriangleRecord& t, const Plane* planes, int count) {
    if (!planes || t.surfaceIndex < 0 || t.surfaceIndex >= count)
        throw std::invalid_argument("Invalid neighbor surface");
    Vector3 n = planes[t.surfaceIndex].normal;
    if (t.flags&0x10u) {
        std::uint32_t bits[3];
        std::memcpy(bits,&n,sizeof(bits));
        for (auto& b : bits) b ^= 0x80000000u;
        std::memcpy(&n,bits,sizeof(bits));
    }
    return n;
}
bool active(const GapNeighborBrushView* manager, int count, int index) {
    return index >= 0 && index < count && manager[index].nodeIndex == index
        && manager[index].mesh && manager[index].planes && manager[index].planeCount > 0;
}
float dot(const Vector3& a, const Vector3& b) { return (a.x*b.x+a.y*b.y)+a.z*b.z; }
void selectPair(TriangleMeshView& local, const Plane* planes, int planeCount,
    std::uint32_t localEdge, std::uint32_t foreignEdge, bool same, int foreignBrush,
    const GapNeighborBrushView* manager, int managerCount) {
    const auto& other = manager[foreignBrush];
    const auto& foreign = *other.mesh;
    const auto lt = triangle(local,localEdge).twins[localEdge&3u];
    const auto ft = triangle(foreign,foreignEdge).twins[foreignEdge&3u];
    // Check both pairs before either selection writes a link.
    for (auto e : {localEdge,lt}) {
        const auto& t = triangle(local,e); normal(t,planes,planeCount);
        const auto& ref = t.brushEdges[e&3u];
        if (active(manager,managerCount,ref.brushIndex)) {
            const auto& view = manager[ref.brushIndex];
            normal(triangle(*view.mesh,ref.edge),view.planes,view.planeCount);
        }
    }
    for (auto e : {foreignEdge,ft}) normal(triangle(foreign,e),other.planes,other.planeCount);
    auto choose = [&](std::uint32_t e, std::uint32_t candidate) {
        auto& t = local.triangles[e>>2];
        const auto& f = triangle(foreign,candidate);
        if (!(t.flags&8u) || !(f.flags&8u)) return;
        auto& ref = t.brushEdges[e&3u];
        if (ref.brushIndex >= 0) {
            Vector3 incumbent{};
            if (active(manager,managerCount,ref.brushIndex)) {
                const auto& view = manager[ref.brushIndex];
                incumbent = normal(triangle(*view.mesh,ref.edge),view.planes,view.planeCount);
            }
            const auto n = normal(t,planes,planeCount);
            if (!(dot(n,normal(f,other.planes,other.planeCount)) > dot(incumbent,n))) return;
        }
        ref = {foreignBrush,candidate};
    };
    choose(localEdge,same ? ft : foreignEdge);
    choose(lt,same ? foreignEdge : ft);
}
void updateInterval(GapInterval& interval, const TriangleMeshView& mesh, const GapLine& line, bool endFirst) {
    const auto& t = triangle(mesh,interval.edge); const unsigned c = interval.edge&3u;
    int a = t.vertices[(c+2)%3], b = t.vertices[c];
    if (endFirst) std::swap(a,b);
    auto parameter = [&](int v) {
        const auto& p = mesh.worldPositions[v];
        return ((p.x-line.origin.x)*line.direction.x+(p.y-line.origin.y)*line.direction.y)
            +(p.z-line.origin.z)*line.direction.z;
    };
    float lower = parameter(a), upper = parameter(b);
    if (lower > upper) { std::swap(lower,upper); std::swap(a,b); }
    interval.lower = lower; interval.upper = upper;
    interval.lowerVertex = a; interval.upperVertex = b;
    // Native updates neither the old reverse bit nor the zeroed new one.
}
}

void resetGapNeighbors(TriangleMeshView& mesh) {
    validateMesh(mesh);
    for (int p = 0; p < mesh.triangleCount; ++p)
        for (auto& ref : mesh.triangles[p].brushEdges) ref = {-1,kInvalidTriangleEdge};
}

void matchGapNeighbors(TriangleMeshView& local, const Plane* planes, int planeCount,
    const GapLine& line, const std::uint32_t* edges, int count, int foreignBrush,
    const std::uint32_t* foreignEdges, int foreignCount, const GapNeighborBrushView* manager, int managerCount) {
    if (managerCount < 0 || (!manager && managerCount) || !active(manager,managerCount,foreignBrush)
        || planeCount <= 0 || !planes || foreignCount < 0 || (!foreignEdges && foreignCount))
        throw std::invalid_argument("Invalid gap neighbor views");
    const auto& foreign = *manager[foreignBrush].mesh;
    const auto localIntervals = collectGapIntervals(local,line,edges,count);
    // Foreign collection intentionally does not deduplicate twin handles.
    std::vector<GapInterval> candidates;
    for (int i = 0; i < foreignCount; ++i) {
        const auto one = collectGapIntervals(foreign,line,foreignEdges+i,1);
        candidates.insert(candidates.end(),one.begin(),one.end());
    }
    // Preflight every accessed pair and incumbent, including before visibility
    // rejection; this is a portable safety check, not native unchecked indexing.
    auto validatePair = [&](const TriangleMeshView& mesh, const Plane* ps, int pc, std::uint32_t e, bool incumbent) {
        const auto& a = triangle(mesh,e);
        const auto& b = triangle(mesh,a.twins[e&3u]);
        normal(a,ps,pc); normal(b,ps,pc);
        if (incumbent) for (auto edge : {e,a.twins[e&3u]}) {
            const auto& ref = triangle(mesh,edge).brushEdges[edge&3u];
            if (active(manager,managerCount,ref.brushIndex)) {
                const auto& view = manager[ref.brushIndex];
                normal(triangle(*view.mesh,ref.edge),view.planes,view.planeCount);
            }
        }
    };
    for (const auto& a : localIntervals) validatePair(local,planes,planeCount,a.edge,true);
    const auto& other = manager[foreignBrush];
    for (const auto& b : candidates) validatePair(foreign,other.planes,other.planeCount,b.edge,false);
    for (const auto& b : candidates) for (const auto& a : localIntervals) {
        if (!(std::abs(b.lower-a.lower) < kGapEndpointTolerance
              && std::abs(b.upper-a.upper) < kGapEndpointTolerance)) continue;
        selectPair(local,planes,planeCount,a.edge,b.edge,a.reverse == b.reverse,foreignBrush,manager,managerCount);
    }
}

std::int32_t fixGapGroup(TriangleMeshView& local, const Plane* planes, int planeCount,
    const BrushSpaceTransform& transform, BrushEdgeTriangleList* sources, int sourceCount, int sourceIndex,
    const GapLine& line, int foreignBrush, const std::uint32_t* foreignEdges, int foreignCount,
    const GapNeighborBrushView* manager, int managerCount, int maxSplits) {
    if (managerCount < 0 || (!manager && managerCount) || !active(manager,managerCount,foreignBrush)
        || !planes || planeCount <= 0 || !sources || sourceCount <= 0 || sourceIndex < 0
        || sourceIndex >= sourceCount || foreignCount < 0 || (!foreignEdges && foreignCount) || maxSplits < 0)
        throw std::invalid_argument("Invalid gap group views");
    const auto& foreign = *manager[foreignBrush].mesh;
    if (&local == &foreign || local.triangles == foreign.triangles || local.worldPositions == foreign.worldPositions)
        throw std::invalid_argument("Gap group buffers must be independent");
    const auto& group = sources[sourceIndex];
    if (group.count < 0 || group.count > group.capacity || (!group.edges && group.capacity))
        throw std::invalid_argument("Invalid gap source list");
    auto intervals = collectGapIntervals(local,line,group.edges,group.count);
    // Validate all supplied foreign endpoints before beginning mutation.
    collectGapIntervals(foreign,line,foreignEdges,foreignCount);
    for (int i = 0; i < group.count; ++i) {
        const auto e = group.edges[i];
        if (triangle(local,e).edgeSource[e&3u] != sourceIndex)
            throw std::invalid_argument("Gap group does not own edge");
        for (int j = 0; j < i; ++j) if (e == group.edges[j])
            throw std::invalid_argument("Mutable gap handles must be unique");
    }
    int splits = 0;
    for (int f = 0; f < foreignCount; ++f) {
        const auto candidate = collectGapIntervals(foreign,line,foreignEdges+f,1);
        if (candidate.empty()) continue;
        const auto& b = candidate[0];
        std::size_t i = 0;
        while (i < intervals.size()) {
            const auto a = intervals[i];
            auto interior = [&](float p) {
                return p-a.lower > kGapEndpointTolerance && p-a.upper < -kGapEndpointTolerance;
            };
            const bool lowerCut = interior(b.lower);
            const bool upperCut = !lowerCut && interior(b.upper);
            if (lowerCut || upperCut) {
                if (splits == maxSplits) throw std::length_error("Gap split budget exhausted");
                // Reserve before mutating so a failed vector allocation cannot
                // leave a new triangle without its temporary interval record.
                if (intervals.size() == intervals.capacity()) intervals.reserve(intervals.size()*2+1);
                const float p = lowerCut ? b.lower : b.upper;
                const int vertex = lowerCut ? b.lowerVertex : b.upperVertex;
                const float weight = (p-a.lower)/(a.upper-a.lower);
                const auto added = splitTriangleEdge(local,transform,sources,sourceCount,a.edge,foreign.worldPositions[vertex],weight);
                GapInterval extra{}; extra.edge = added;
                updateInterval(extra,local,line,false);
                intervals.push_back(extra);
                updateInterval(intervals[i],local,line,upperCut);
                ++splits;
                i = 0; // Native retries this same foreign edge from the start.
                continue;
            }
            if (std::abs(b.lower-a.lower) < kGapEndpointTolerance && std::abs(b.upper-a.upper) < kGapEndpointTolerance)
                selectPair(local,planes,planeCount,a.edge,b.edge,a.reverse == b.reverse,foreignBrush,manager,managerCount);
            ++i;
        }
    }
    return splits;
}
}

extern "C" std::int32_t rcsg_recovered_reset_gap_neighbors(realtimecsg::recovered::TriangleMeshView* mesh) {
    if (!mesh) return -1;
    try { realtimecsg::recovered::resetGapNeighbors(*mesh); return 0; }
    catch (const std::invalid_argument&) { return -1; } catch (...) { return -3; }
}
extern "C" std::int32_t rcsg_recovered_match_gap_neighbors(realtimecsg::recovered::TriangleMeshView* mesh,
    const realtimecsg::recovered::Plane* planes, std::int32_t pc, const realtimecsg::recovered::GapLine* line,
    const std::uint32_t* edges, std::int32_t count, std::int32_t brush, const std::uint32_t* otherEdges,
    std::int32_t otherCount, const realtimecsg::recovered::GapNeighborBrushView* manager, std::int32_t managerCount) {
    if (!mesh || !line) return -1;
    try { realtimecsg::recovered::matchGapNeighbors(*mesh,planes,pc,*line,edges,count,brush,otherEdges,otherCount,manager,managerCount); return 0; }
    catch (const std::invalid_argument&) { return -1; } catch (...) { return -3; }
}

extern "C" std::int32_t rcsg_recovered_fix_gap_group(realtimecsg::recovered::TriangleMeshView* mesh,
    const realtimecsg::recovered::Plane* planes, std::int32_t pc,
    const realtimecsg::recovered::BrushSpaceTransform* transform, realtimecsg::recovered::BrushEdgeTriangleList* sources,
    std::int32_t sc, std::int32_t si, const realtimecsg::recovered::GapLine* line, std::int32_t brush,
    const std::uint32_t* edges, std::int32_t ec, const realtimecsg::recovered::GapNeighborBrushView* manager,
    std::int32_t mc, std::int32_t budget) {
    if (!mesh || !transform || !line) return -1;
    try { return realtimecsg::recovered::fixGapGroup(*mesh,planes,pc,*transform,sources,sc,si,*line,brush,edges,ec,manager,mc,budget); }
    catch (const std::invalid_argument&) { return -1; }
    catch (const std::length_error&) { return -2; } catch (...) { return -3; }
}
