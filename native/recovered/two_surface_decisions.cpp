#include "realtimecsg/recovered_mesh_optimization.h"
#include <stdexcept>

namespace realtimecsg::recovered {
namespace {
Vector3 subtract(Vector3 a, Vector3 b) { return {a.x-b.x,a.y-b.y,a.z-b.z}; }
float dot(Vector3 a, Vector3 b) { return (a.x*b.x+a.y*b.y)+a.z*b.z; }
Vector3 cross(Vector3 a, Vector3 b) {
    return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};
}
bool same(const TriangleRecord& a, const TriangleRecord& b) {
    return a.surfaceIndex==b.surfaceIndex && a.categorizationState==b.categorizationState;
}
std::uint32_t next(std::uint32_t h) { return (h&~3u)|(((h&3u)+1)%3); }
}
TwoSurfaceVertexAction planTwoSurfaceVertex(const TriangleMeshView& mesh, const SurfaceView& surfaces,
    int planeCount, int vertex, TwoSurfaceVertexPlan& output) {
    if (planeCount<0 || (planeCount && !surfaces.planes))
        throw std::invalid_argument("Invalid two-surface plane view");
    TwoSurfaceVertexCandidate boundary;
    if (!findTwoSurfaceVertexCandidate(mesh,vertex,boundary)) return TwoSurfaceVertexAction::Rejected;
    const auto ah=boundary.firstBoundary, bh=boundary.secondBoundary;
    const auto& a=mesh.triangles[ah>>2]; const auto& b=mesh.triangles[bh>>2];
    const unsigned ac=ah&3u, bc=bh&3u;
    const auto at=a.twins[ac], bt=b.twins[bc];
    auto checked=[&](std::uint32_t h) -> const TriangleRecord& {
        if ((h&3u)==3u || h>>2>=static_cast<unsigned>(mesh.triangleCount))
            throw std::invalid_argument("Invalid two-surface boundary twin");
        return mesh.triangles[h>>2];
    };
    const auto& acrossA=checked(at); const auto& acrossB=checked(bt);
    if (!same(acrossA,b) || !same(acrossB,a)) return TwoSurfaceVertexAction::Rejected;
    const int center=a.vertices[ac], y=acrossB.vertices[bt&3u], x=a.vertices[(ac+2)%3];
    if (center!=b.vertices[bc] || y!=b.vertices[(bc+2)%3] || x!=acrossA.vertices[at&3u])
        return TwoSurfaceVertexAction::Rejected;
    if (ah>>2==at>>2 || bh>>2==bt>>2) return TwoSurfaceVertexAction::Rejected;
    TwoSurfaceVertexPlan result{boundary,kInvalidTriangleEdge,center};
    auto commit=[&](TwoSurfaceVertexAction action) { output=result; return action; };
    if (at>>2==bh>>2 && bt>>2==ah>>2) return commit(TwoSurfaceVertexAction::RemoveInPlane);

    auto position=[&](int index) {
        if (index<0 || index>=mesh.vertexCount) throw std::invalid_argument("Invalid two-surface endpoint");
        return mesh.vertices[index].position;
    };
    const auto c=position(center), u=subtract(c,position(y)), v=subtract(position(x),c);
    const float lengthU=dot(u,u), lengthV=dot(v,v);
    constexpr float tiny=3.6000002978653356e-7f;
    if (!(tiny>=lengthU) && !(tiny>=lengthV)) {
        // The original reads the plane before even testing the cross product.
        if (a.surfaceIndex<0 || a.surfaceIndex>=planeCount)
            throw std::invalid_argument("Invalid two-surface orientation plane");
        const auto normal=surfaces.planes[a.surfaceIndex].normal;
        const auto perpendicular=cross(u,v); const float square=dot(perpendicular,perpendicular);
        bool skipOrientation=false;
        if (!(square>=9.999999439624929e-11f)) {
            const float ratio=dot(u,v)/(lengthU*lengthV);
            // Unlike Swizzle, both extreme ratios bypass orientation here.
            skipOrientation=-0.9999998807907104f>=ratio || ratio>=0.9999998807907104f;
        }
        if (!skipOrientation && !(dot(perpendicular,normal)>=square*9.999999747378752e-6f))
            return TwoSurfaceVertexAction::Rejected;
    }
    if (ah>>2==bt>>2) {
        if (same(a,b)) return commit(TwoSurfaceVertexAction::RemoveInPlane);
        result.swizzleEdge=next(bt); return commit(TwoSurfaceVertexAction::Swizzle);
    }
    if (at>>2==bh>>2) {
        if (same(a,b)) return commit(TwoSurfaceVertexAction::RemoveInPlane);
        result.swizzleEdge=next(at); return commit(TwoSurfaceVertexAction::Swizzle);
    }
    return commit(TwoSurfaceVertexAction::Split);
}
}
extern "C" std::int32_t rcsg_recovered_plan_two_surface_vertex(
    const realtimecsg::recovered::TriangleMeshView* mesh,
    const realtimecsg::recovered::SurfaceView* surfaces, std::int32_t planeCount,
    std::int32_t vertex, realtimecsg::recovered::TwoSurfaceVertexPlan* output) {
    if (!mesh || !surfaces || !output) return -1;
    try { return static_cast<std::int32_t>(realtimecsg::recovered::planTwoSurfaceVertex(*mesh,*surfaces,planeCount,vertex,*output)); }
    catch (const std::invalid_argument&) { return -1; } catch (...) { return -3; }
}
