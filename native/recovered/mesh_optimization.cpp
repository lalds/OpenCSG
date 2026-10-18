#include "realtimecsg/recovered_mesh_optimization.h"
#include <cstring>
#include <stdexcept>

namespace realtimecsg::recovered {
namespace {
constexpr float tiny = 3.6000002978653356e-7f;
constexpr float areaThreshold = 9.999999439624929e-11f;
constexpr float negativeRatio = -0.9999998807907104f;
constexpr float positiveRatio = 0.9999998807907104f;
constexpr float orientationFactor = 9.999999747378752e-6f;
Vector3 subtract(Vector3 a, Vector3 b) { return {a.x-b.x,a.y-b.y,a.z-b.z}; }
float dot(Vector3 a, Vector3 b) { return (a.x*b.x+a.y*b.y)+a.z*b.z; }
Vector3 cross(Vector3 a, Vector3 b) {
    return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};
}
bool acceptable(Vector3 u, Vector3 v, Vector3 normal) {
    const float a = dot(u,u), b = dot(v,v);
    if (tiny >= a || tiny >= b) return true;
    const auto perpendicular = cross(u,v);
    const float square = dot(perpendicular,perpendicular);
    if (!(square >= areaThreshold)) {
        const float ratio = dot(u,v)/(a*b);
        if (negativeRatio >= ratio) return false;
        if (ratio >= positiveRatio) return true;
    }
    // ucomiss/jb in native also rejects unordered comparisons here.
    return dot(perpendicular,normal) >= square*orientationFactor;
}
}
bool swizzleTriangleEdge(TriangleMeshView& mesh, const SurfaceView& surfaces,
                         int planeCount, std::uint32_t edge) {
    if (mesh.triangleCount < 0 || mesh.triangleCount > mesh.triangleCapacity
        || mesh.vertexCount < 0 || mesh.vertexCount > mesh.vertexCapacity
        || (mesh.triangleCount && !mesh.triangles) || (mesh.vertexCount && !mesh.vertices)
        || planeCount < 0 || (planeCount && !surfaces.planes))
        throw std::invalid_argument("Invalid Swizzle views");
    auto valid = [&](std::uint32_t e) { return (e&3u) != 3u && e>>2 < static_cast<unsigned>(mesh.triangleCount); };
    if (!valid(edge)) throw std::invalid_argument("Invalid Swizzle edge");
    const unsigned ai = edge>>2, ac = edge&3u, an = (ac+1)%3, ap = (ac+2)%3;
    const auto twin = mesh.triangles[ai].twins[ac];
    if (!valid(twin) || twin>>2 == ai || mesh.triangles[twin>>2].twins[twin&3u] != edge)
        throw std::invalid_argument("Swizzle needs distinct reciprocal triangles");
    const unsigned bi = twin>>2, bc = twin&3u, bn = (bc+1)%3, bp = (bc+2)%3;
    const auto oldA = mesh.triangles[ai], oldB = mesh.triangles[bi];
    if (oldA.surfaceIndex < 0 || oldA.surfaceIndex >= planeCount ||
        oldB.surfaceIndex < 0 || oldB.surfaceIndex >= planeCount)
        throw std::invalid_argument("Invalid Swizzle surface");
    const int pi = oldA.vertices[an], qi = oldA.vertices[ap], ri = oldB.vertices[bn], si = oldB.vertices[bp];
    for (int v : {pi,qi,ri,si}) if (v < 0 || v >= mesh.vertexCount)
        throw std::invalid_argument("Invalid Swizzle vertex");
    const std::uint32_t aNext = oldA.twins[an], aPrev = oldA.twins[ap],
                        bNext = oldB.twins[bn], bPrev = oldB.twins[bp];
    for (auto e : {aNext,aPrev,bNext,bPrev})
        if ((e&3u) != 3u && !valid(e)) throw std::invalid_argument("Invalid Swizzle outer twin");
    const auto p = mesh.vertices[pi].position, q = mesh.vertices[qi].position,
               r = mesh.vertices[ri].position, s = mesh.vertices[si].position;
    const auto u = subtract(q,p), v = subtract(r,q);
    const float lengthU = dot(u,u), lengthV = dot(v,v);
    bool useB = false;
    if (!(tiny >= lengthU) && !(tiny >= lengthV)) {
        const auto perpendicular = cross(u,v);
        const float square = dot(perpendicular,perpendicular);
        useB = square >= areaThreshold || !(negativeRatio >= dot(u,v)/(lengthV*lengthU));
    }
    const auto normal = surfaces.planes[useB ? oldB.surfaceIndex : oldA.surfaceIndex].normal;
    if (!acceptable(subtract(s,r),subtract(p,s),normal) || !acceptable(u,v,normal)) return false;

    auto& a = mesh.triangles[ai]; auto& b = mesh.triangles[bi];
    auto copyMetadata = [](TriangleRecord& to, const TriangleRecord& from) {
        to.surfaceIndex = from.surfaceIndex; to.flags = from.flags; to.categorizationState = from.categorizationState;
    };
    if (useB) copyMetadata(a,oldB); else copyMetadata(b,oldA);
    a.vertices[0] = ri; a.vertices[1] = si; a.vertices[2] = pi;
    b.vertices[0] = pi; b.vertices[1] = qi; b.vertices[2] = ri;
    // These are deliberately asymmetric: native writes only A's normals and
    // source tags, leaving B's original arrays in their original slot order.
    std::memcpy(&a.cornerNormals[0],&oldA.cornerNormals[an],sizeof(Vector3));
    std::memcpy(&a.cornerNormals[1],&oldA.cornerNormals[ap],sizeof(Vector3));
    std::memcpy(&a.cornerNormals[2],&oldB.cornerNormals[bn],sizeof(Vector3));
    a.edgeSource[0] = -1; a.edgeSource[1] = oldA.edgeSource[ap]; a.edgeSource[2] = oldB.edgeSource[bn];
    auto outer = [&](unsigned index, unsigned corner, std::uint32_t neighbor) {
        mesh.triangles[index].twins[corner] = neighbor;
        if ((neighbor&3u) != 3u) mesh.triangles[neighbor>>2].twins[neighbor&3u] = index*4+corner;
    };
    outer(ai,1,bPrev); outer(ai,2,aNext); outer(bi,1,aPrev); outer(bi,2,bNext);
    a.twins[0] = bi*4; b.twins[0] = ai*4;
    mesh.vertices[ri].edge = ai*4;
    mesh.vertices[si].edge = ai*4+1;
    mesh.vertices[pi].edge = ai*4+2;
    mesh.vertices[pi].edge = bi*4;
    mesh.vertices[qi].edge = bi*4+1;
    mesh.vertices[ri].edge = bi*4+2;
    return true;
}
}
extern "C" std::int32_t rcsg_recovered_swizzle_triangle_edge(realtimecsg::recovered::TriangleMeshView* mesh,
    const realtimecsg::recovered::SurfaceView* surfaces, std::int32_t planeCount, std::uint32_t edge) {
    if (!mesh || !surfaces) return -1;
    try { return realtimecsg::recovered::swizzleTriangleEdge(*mesh,*surfaces,planeCount,edge) ? 1 : 0; }
    catch (const std::invalid_argument&) { return -1; } catch (...) { return -3; }
}
