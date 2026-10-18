#include "realtimecsg/recovered_mesh_optimization.h"
#include <cmath>
#include <stdexcept>

namespace realtimecsg::recovered {
namespace {
std::uint32_t previous(std::uint32_t h) { return (h&~3u)|(((h&3u)+2)%3); }
Vector3 subtract(Vector3 a, Vector3 b) { return {a.x-b.x,a.y-b.y,a.z-b.z}; }
Vector3 normalize(Vector3 a) {
    const double x=a.x, y=a.y, z=a.z;
    const double reciprocal=1.0/std::sqrt((x*x+y*y)+z*z);
    return {float(x*reciprocal),float(y*reciprocal),float(z*reciprocal)};
}
}
bool findTwoSurfaceVertexCandidate(const TriangleMeshView& mesh, int vertex,
                                  TwoSurfaceVertexCandidate& output) {
    if (mesh.triangleCount < 0 || mesh.triangleCount > mesh.triangleCapacity
        || mesh.triangleCount > (1<<30) || mesh.vertexCount < 0 || mesh.vertexCount > mesh.vertexCapacity
        || (mesh.triangleCount && !mesh.triangles) || !mesh.vertices || vertex < 0 || vertex >= mesh.vertexCount)
        throw std::invalid_argument("Invalid two-surface candidate views");
    const auto start=mesh.vertices[vertex].edge;
    if ((start&3u)==3u || start>>2 >= static_cast<unsigned>(mesh.triangleCount)) return false;
    const auto& initial=mesh.triangles[start>>2];
    if (initial.surfaceIndex < 0) return false;
    auto equal=[](const TriangleRecord& a, const TriangleRecord& b) {
        return a.surfaceIndex==b.surfaceIndex && a.categorizationState==b.categorizationState;
    };
    auto next=[&](std::uint32_t h, std::uint32_t& out) {
        const auto twin=mesh.triangles[h>>2].twins[h&3u];
        if (twin>>2 >= static_cast<unsigned>(mesh.triangleCount)) return false;
        out=previous(twin); return true;
    };
    // The source has no cycle bound. Reject nonclosing malformed topology
    // instead of hanging; retain its ordering on terminating indexed inputs.
    std::size_t steps=0;
    const auto limit=static_cast<std::size_t>(mesh.triangleCount)*9+3;
    auto tick=[&]() { if (++steps>limit) throw std::invalid_argument("Nonclosing two-surface fan"); };
    std::uint32_t a=start, b=0, h=0;
    for (;;) {
        tick(); if (!next(a,h)) return false;
        if (!equal(initial,mesh.triangles[h>>2])) { b=h; break; }
        if (h==start) return false;
        a=h;
    }
    const auto& first=mesh.triangles[a>>2];
    const auto& second=mesh.triangles[b>>2];
    for (;;) {
        tick(); if (!next(b,h)) return false;
        if (!equal(second,mesh.triangles[h>>2])) break;
        if (h==start) return false;
        b=h;
    }
    if (!equal(first,mesh.triangles[h>>2])) return false;
    // Native restarts at b, not h, and accepts a later open end in this
    // third walk. Later stages may still reject the resulting candidate.
    auto current=b;
    for (;;) {
        tick(); if (!next(current,h)) break;
        const auto& record=mesh.triangles[h>>2];
        if (record.surfaceIndex!=first.surfaceIndex || record.categorizationState!=first.categorizationState) return false;
        if (h==start) break;
        current=h;
    }
    auto point=[&](int id) {
        if (id<0 || id>=mesh.vertexCount) throw std::invalid_argument("Invalid seam vertex");
        return mesh.vertices[id].position;
    };
    const auto x=point(first.vertices[((a&3u)+2)%3]);
    const auto center=point(first.vertices[a&3u]);
    const auto y=point(mesh.triangles[b>>2].vertices[((b&3u)+2)%3]);
    const auto u=normalize(subtract(x,center)), v=normalize(subtract(x,y));
    const float dot=(u.x*v.x+u.y*v.y)+u.z*v.z;
    // jb also rejects NaN; this is the exact stored float at 0x3c528.
    if (!(dot>=0.9999998807907104f)) return false;
    output={a,b}; return true;
}
}
extern "C" std::int32_t rcsg_recovered_find_two_surface_candidate(
    const realtimecsg::recovered::TriangleMeshView* mesh, std::int32_t vertex,
    realtimecsg::recovered::TwoSurfaceVertexCandidate* output) {
    if (!mesh || !output) return -1;
    try { return realtimecsg::recovered::findTwoSurfaceVertexCandidate(*mesh,vertex,*output) ? 1 : 0; }
    catch (const std::invalid_argument&) { return -1; } catch (...) { return -3; }
}
