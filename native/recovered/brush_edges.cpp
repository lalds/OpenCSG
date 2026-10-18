#include "realtimecsg/recovered_brush_edges.h"
#include "realtimecsg/recovered_gap_dispatch.h"
#include <cmath>
#include <stdexcept>

namespace realtimecsg::recovered {
namespace {
float dot(Vector3 a, Vector3 b) { return (a.x*b.x+a.y*b.y)+a.z*b.z; }
Vector3 transform(const BrushSpaceTransform& matrix, Vector3 v, bool point) {
    const auto& m = matrix.columnMajor;
    Vector3 r{(m[0]*v.x+m[4]*v.y)+m[8]*v.z,(m[1]*v.x+m[5]*v.y)+m[9]*v.z,
              (m[2]*v.x+m[6]*v.y)+m[10]*v.z};
    if (point) { r.x += m[12]; r.y += m[13]; r.z += m[14]; }
    return r;
}
bool nearPlane(Vector3 p, const Plane& plane) {
    const float d = dot(plane.normal,p)-plane.distance;
    return d > -kPlaneDistanceTolerance && d < kPlaneDistanceTolerance;
}
bool inBounds(Vector3 p, const BrushEdgeBounds& b) {
    constexpr float epsilon = 9.999999747378752e-5f;
    return !(b.minX-p.x >= epsilon || b.maxX-p.x < -epsilon
        || b.minY-p.y >= epsilon || b.maxY-p.y < -epsilon
        || b.minZ-p.z >= epsilon || b.maxZ-p.z < -epsilon);
}
}
void findBrushEdges(BrushMemoryView& mesh, int polygonIndex, const Plane& localPlane,
    const Plane* planes, int planeCount, const BrushEdgeBounds& bounds, const BrushSpaceTransform& matrix,
    int foreignBrush, int foreignSurface, BrushEdgeGroupView* groups, int& groupCount, int groupCapacity) {
    if (polygonIndex < 0 || polygonIndex >= mesh.polygonCount || !mesh.polygons || !mesh.edges
        || !mesh.edgeLinks || !mesh.vertices || mesh.edgeCount < 0 || mesh.vertexCount < 0
        || !planes || foreignSurface < 0 || foreignSurface >= planeCount || groupCount < 0
        || groupCount > groupCapacity || (!groups && groupCapacity)) throw std::invalid_argument("Invalid brush-edge views");
    const auto& n = localPlane.normal; const auto& other = planes[foreignSurface]; const auto& o = other.normal;
    Vector3 direction{n.y*o.z-n.z*o.y,n.z*o.x-n.x*o.z,n.x*o.y-n.y*o.x};
    if (dot(direction,direction) < 9.999999747378752e-6f) return;
    direction = transform(matrix,direction,false);
    const double x = direction.x, y = direction.y, z = direction.z;
    const double inverseLength = 1.0/std::sqrt((x*x+y*y)+z*z);
    direction = {static_cast<float>(x*inverseLength),static_cast<float>(y*inverseLength),static_cast<float>(z*inverseLength)};
    const auto& polygon = mesh.polygons[polygonIndex];
    int edge = polygon.firstEdge;
    auto validEdge = [&](int e) {
        if (e < 0 || e >= mesh.edgeCount) throw std::invalid_argument("Invalid brush halfedge");
    };
    auto vertex = [&](int index) {
        if (index < 0 || index >= mesh.vertexCount) throw std::invalid_argument("Invalid brush vertex");
        return mesh.vertices[index];
    };
    for (int i = 0; i < polygon.edgeCount; ++i) {
        validEdge(edge);
        const auto& e = mesh.edges[edge]; const int current = edge;
        edge = e.nextEdge;
        const auto a = vertex(e.vertexIndex);
        if (!nearPlane(a,other)) continue;
        validEdge(e.twinEdge);
        const auto& twin = mesh.edges[e.twinEdge]; const auto b = vertex(twin.vertexIndex);
        if (!nearPlane(b,other) || !inBounds(a,bounds) || !inBounds(b,bounds)) continue;
        bool rejected = false;
        for (int p = 0; p < planeCount; ++p) {
            if (dot(planes[p].normal,a)-planes[p].distance > kPlaneDistanceTolerance
                && dot(planes[p].normal,b)-planes[p].distance > kPlaneDistanceTolerance) {
                rejected = true; break;
            }
        }
        if (rejected) continue;
        const auto origin = transform(matrix,a,true);
        if (twin.polygonIndex < 0 || twin.polygonIndex >= mesh.polygonCount)
            throw std::invalid_argument("Invalid twin polygon");
        const int surfaceA = polygon.surfaceIndex, surfaceB = mesh.polygons[twin.polygonIndex].surfaceIndex;
        const int first = mesh.edgeLinks[current].unidentified_04, second = mesh.edgeLinks[e.twinEdge].unidentified_04;
        int selected = -1;
        if (first == second && first != -1) selected = first;
        else if (first == -1 && second != -1) selected = second;
        else if (second == -1 && first != -1) selected = first;
        else for (int g = 0; g < groupCount; ++g) {
            const auto& group = groups[g];
            if (!((group.surfaceA == surfaceA && group.surfaceB == surfaceB)
                || (group.surfaceA == surfaceB && group.surfaceB == surfaceA))) continue;
            if (std::abs(dot(group.line.direction,direction)) <= kGapDirectionAlignment) continue;
            const Vector3 delta{group.line.origin.x-origin.x,group.line.origin.y-origin.y,group.line.origin.z-origin.z};
            const float parameter = dot(delta,direction);
            const Vector3 distance{direction.x*parameter+origin.x-group.line.origin.x,
                direction.y*parameter+origin.y-group.line.origin.y,direction.z*parameter+origin.z-group.line.origin.z};
            if (dot(distance,distance) < kGapLineDistanceSquared) { selected = g; break; }
        }
        if (selected < -1 || selected >= groupCount) throw std::invalid_argument("Invalid existing brush group");
        if (selected == -1 && groupCount == groupCapacity) throw std::length_error("Brush group capacity exhausted");
        auto& group = groups[selected == -1 ? groupCount : selected];
        const int refCount = selected == -1 ? 0 : group.surfaceRefCount;
        if (refCount < 0 || refCount > group.surfaceRefCapacity || (!group.surfaceRefs && group.surfaceRefCapacity))
            throw std::invalid_argument("Invalid brush surface-reference storage");
        bool exists = false;
        for (int r = 0; r < refCount; ++r)
            if (group.surfaceRefs[r].brushIndex == foreignBrush && group.surfaceRefs[r].edge == static_cast<unsigned>(foreignSurface)) exists = true;
        if (!exists && refCount == group.surfaceRefCapacity) throw std::length_error("Brush surface-reference capacity exhausted");
        if (selected == -1) {
            selected = groupCount++; group.surfaceRefCount = 0;
            group.surfaceA = surfaceA; group.surfaceB = surfaceB; group.line = {direction,origin};
        }
        if (!exists) group.surfaceRefs[group.surfaceRefCount++] = {foreignBrush,static_cast<unsigned>(foreignSurface)};
        mesh.edgeLinks[current].unidentified_04 = selected;
        mesh.edgeLinks[e.twinEdge].unidentified_04 = selected;
    }
}
}
extern "C" std::int32_t rcsg_recovered_find_brush_edges(realtimecsg::recovered::BrushMemoryView* mesh, std::int32_t polygon,
    const realtimecsg::recovered::Plane* local, const realtimecsg::recovered::Plane* planes, std::int32_t count,
    const realtimecsg::recovered::BrushEdgeBounds* bounds, const realtimecsg::recovered::BrushSpaceTransform* matrix,
    std::int32_t brush, std::int32_t surface, realtimecsg::recovered::BrushEdgeGroupView* groups,
    std::int32_t* groupCount, std::int32_t capacity) {
    if (!mesh || !local || !bounds || !matrix || !groupCount) return -1;
    try { realtimecsg::recovered::findBrushEdges(*mesh,polygon,*local,planes,count,*bounds,*matrix,brush,surface,groups,*groupCount,capacity); return 0; }
    catch (const std::invalid_argument&) { return -1; }
    catch (const std::length_error&) { return -2; } catch (...) { return -3; }
}
