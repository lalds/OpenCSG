#include "realtimecsg/recovered_triangle_splitting.h"

#include <cmath>
#include <cstring>
#include <stdexcept>

namespace realtimecsg::recovered {
namespace {
std::uint32_t handle(int triangle, unsigned corner) { return (static_cast<std::uint32_t>(triangle)<<2) | corner; }
bool valid(std::uint32_t edge) { return (edge & 3u) != 3u; }
void backLink(TriangleMeshView& mesh, std::uint32_t edge, std::uint32_t other) {
    if (valid(edge)) mesh.triangles[edge>>2].twins[edge & 3u] = other;
}
Vector3 normalize(Vector3 v) {
    const double x = v.x, y = v.y, z = v.z;
    const double inverse = 1.0/std::sqrt((x*x+y*y)+z*z);
    return {static_cast<float>(x*inverse),static_cast<float>(y*inverse),static_cast<float>(z*inverse)};
}
Vector3 firstNormal(const Vector3& start, const Vector3& end, float weight) {
    const float other = 1.0f-weight;
    return normalize({end.x*other+start.x*weight,end.y*other+start.y*weight,end.z*other+start.z*weight});
}
Vector3 secondNormal(const Vector3& end, float weight) {
    const float other = 1.0f-weight;
    // Native 0x3759a..0x375d5 uses the same endpoint in BOTH terms.
    return normalize({end.x*weight+end.x*other,end.y*weight+end.y*other,end.z*other+end.z*weight});
}
Vector3 transform(const BrushSpaceTransform& matrix, const Vector3& p) {
    const auto* m = matrix.columnMajor;
    return {((m[0]*p.x+m[4]*p.y)+m[8]*p.z)+m[12],
            ((m[1]*p.x+m[5]*p.y)+m[9]*p.z)+m[13],
            ((m[2]*p.x+m[6]*p.y)+m[10]*p.z)+m[14]};
}
void replace(BrushEdgeTriangleList* lists, int source, std::uint32_t old, std::uint32_t replacement) {
    if (source == -1) return;
    auto& list = lists[source];
    for (int i = 0; i < list.count; ++i) if (list.edges[i] == old) { list.edges[i] = replacement; break; }
}
void append(BrushEdgeTriangleList* lists, int source, std::uint32_t edge) {
    if (source != -1) { auto& list = lists[source]; list.edges[list.count++] = edge; }
}
void copyNormal(Vector3& dst, const Vector3& src) { std::memcpy(&dst,&src,sizeof(Vector3)); }
} // namespace

std::uint32_t splitTriangleEdge(TriangleMeshView& mesh, const BrushSpaceTransform& matrix,
                                BrushEdgeTriangleList* sources, int sourceCount,
                                std::uint32_t edge, const Vector3& point, float weight) {
    if (mesh.triangleCount <= 0 || mesh.vertexCount <= 0 || mesh.triangleCapacity < mesh.triangleCount
        || mesh.vertexCapacity < mesh.vertexCount || !mesh.triangles || !mesh.vertices
        || !mesh.worldPositions || !mesh.worldVertexLookup || sourceCount < 0 || (!sources && sourceCount)
        || !valid(edge) || edge>>2 >= static_cast<unsigned>(mesh.triangleCount))
        throw std::invalid_argument("Invalid split triangle views/edge");
    if (mesh.triangleCapacity-mesh.triangleCount < 2 || mesh.vertexCapacity-mesh.vertexCount < 2
        || static_cast<unsigned>(mesh.triangleCount) > (1u<<30)-2)
        throw std::length_error("Triangle split capacity exhausted");
    const int a = static_cast<int>(edge>>2);
    const unsigned c = edge & 3u, ap = (c+2)%3, an = (c+1)%3;
    const auto twin = mesh.triangles[a].twins[c];
    if (!valid(twin) || twin>>2 >= static_cast<unsigned>(mesh.triangleCount) || twin>>2 == edge>>2)
        throw std::invalid_argument("Split requires two adjacent triangles");
    const int b = static_cast<int>(twin>>2);
    const unsigned d = twin & 3u, bp = (d+2)%3, bn = (d+1)%3;
    const TriangleRecord oldA = mesh.triangles[a], oldB = mesh.triangles[b];
    if (oldB.twins[d] != edge || oldA.vertices[ap] != oldB.vertices[d] || oldA.vertices[c] != oldB.vertices[bp])
        throw std::invalid_argument("Split edge is not reciprocal");
    for (const auto* triangle : {&oldA,&oldB}) {
        for (int v : triangle->vertices) if (v < 0 || v >= mesh.vertexCount)
            throw std::invalid_argument("Triangle vertex out of range");
        for (auto neighbor : triangle->twins) if (valid(neighbor) && neighbor>>2 >= static_cast<unsigned>(mesh.triangleCount))
            throw std::invalid_argument("Triangle neighbor out of range");
        for (int source : triangle->edgeSource) if (source < -1 || source >= sourceCount)
            throw std::invalid_argument("Brush edge source out of range");
    }
    for (int source : {oldA.edgeSource[an],oldB.edgeSource[bp],oldA.edgeSource[c],oldB.edgeSource[d]}) {
        if (source == -1) continue;
        const auto& list = sources[source];
        if (list.count < 0 || list.capacity < list.count || (!list.edges && list.capacity))
            throw std::invalid_argument("Invalid brush edge triangle list");
        const int needed = (oldA.edgeSource[c] == source) + (oldB.edgeSource[d] == source);
        if (list.capacity-list.count < needed) throw std::length_error("Brush edge triangle list exhausted");
    }

    const int addedA = mesh.triangleCount, addedB = addedA+1, vertex = mesh.vertexCount;
    const auto na = firstNormal(oldA.cornerNormals[ap],oldA.cornerNormals[c],weight);
    const auto nb = secondNormal(oldB.cornerNormals[d],weight);
    mesh.triangleCount += 2;
    mesh.vertices[vertex] = {edge,transform(matrix,point)};
    std::memcpy(&mesh.worldPositions[vertex],&point,sizeof(Vector3));
    mesh.worldVertexLookup[vertex] = -1;
    ++mesh.vertexCount;
    auto& A = mesh.triangles[a]; auto& B = mesh.triangles[b];
    auto& C = mesh.triangles[addedA]; auto& D = mesh.triangles[addedB];
    A.vertices[ap] = oldA.vertices[ap]; A.vertices[c] = vertex; A.vertices[an] = oldA.vertices[an];
    B.vertices[bp] = vertex; B.vertices[d] = oldA.vertices[ap]; B.vertices[bn] = oldB.vertices[bn];
    C.vertices[c] = oldA.vertices[c]; C.vertices[an] = oldA.vertices[an]; C.vertices[ap] = vertex;
    D.vertices[d] = oldA.vertices[c]; D.vertices[bn] = vertex; D.vertices[bp] = oldB.vertices[bn];
    copyNormal(A.cornerNormals[ap],oldA.cornerNormals[ap]); A.cornerNormals[c] = na; copyNormal(A.cornerNormals[an],oldA.cornerNormals[an]);
    B.cornerNormals[bp] = nb; copyNormal(B.cornerNormals[d],oldB.cornerNormals[d]); copyNormal(B.cornerNormals[bn],oldB.cornerNormals[bn]);
    copyNormal(C.cornerNormals[c],oldA.cornerNormals[c]); copyNormal(C.cornerNormals[an],oldA.cornerNormals[an]); C.cornerNormals[ap] = na;
    copyNormal(D.cornerNormals[d],oldB.cornerNormals[bp]); D.cornerNormals[bn] = nb; copyNormal(D.cornerNormals[bp],oldB.cornerNormals[bn]);

    // Preserve the native write order, including reciprocal neighbor updates.
    C.twins[an] = oldA.twins[an]; backLink(mesh,oldA.twins[an],handle(addedA,an));
    A.twins[ap] = oldA.twins[ap]; backLink(mesh,oldA.twins[ap],handle(a,ap));
    B.twins[bn] = oldB.twins[bn]; backLink(mesh,oldB.twins[bn],handle(b,bn));
    D.twins[d] = oldB.twins[bp]; backLink(mesh,oldB.twins[bp],handle(addedB,d));
    A.twins[c] = handle(b,d); B.twins[d] = handle(a,c);
    A.twins[an] = handle(addedA,ap); C.twins[ap] = handle(a,an);
    C.twins[c] = handle(addedB,bn); D.twins[bn] = handle(addedA,c);
    D.twins[bp] = handle(b,bp); B.twins[bp] = handle(addedB,bp);
    const auto output = handle(addedA,c);

    replace(sources,oldA.edgeSource[an],handle(a,an),handle(addedA,an));
    replace(sources,oldB.edgeSource[bp],handle(b,bp),handle(addedB,d));
    append(sources,oldA.edgeSource[c],output);
    append(sources,oldB.edgeSource[d],handle(addedB,bn));
    A.edgeSource[ap] = oldA.edgeSource[ap]; A.edgeSource[c] = oldA.edgeSource[c]; A.edgeSource[an] = -1;
    B.edgeSource[bp] = -1; B.edgeSource[d] = oldB.edgeSource[d]; B.edgeSource[bn] = oldB.edgeSource[bn];
    C.edgeSource[c] = oldA.edgeSource[c]; C.edgeSource[an] = oldA.edgeSource[an]; C.edgeSource[ap] = -1;
    D.edgeSource[d] = oldB.edgeSource[bp]; D.edgeSource[bn] = oldB.edgeSource[d]; D.edgeSource[bp] = -1;
    const BrushTriangleEdge invalid{-1,kInvalidTriangleEdge};
    A.brushEdges[ap] = oldA.brushEdges[ap]; A.brushEdges[c] = oldA.brushEdges[c]; A.brushEdges[an] = invalid;
    B.brushEdges[bp] = invalid; B.brushEdges[d] = oldB.brushEdges[d]; B.brushEdges[bn] = oldB.brushEdges[bn];
    C.brushEdges[c] = oldA.brushEdges[c]; C.brushEdges[an] = oldA.brushEdges[an]; C.brushEdges[ap] = invalid;
    D.brushEdges[d] = oldB.brushEdges[bp]; D.brushEdges[bn] = oldB.brushEdges[d]; D.brushEdges[bp] = invalid;
    C.surfaceIndex = A.surfaceIndex; C.flags = A.flags; C.categorizationState = A.categorizationState;
    D.surfaceIndex = B.surfaceIndex; D.flags = B.flags; D.categorizationState = B.categorizationState;
    return output;
}
} // namespace realtimecsg::recovered

extern "C" std::int32_t rcsg_recovered_split_triangle_edge(
    realtimecsg::recovered::TriangleMeshView* mesh, const realtimecsg::recovered::BrushSpaceTransform* transform,
    realtimecsg::recovered::BrushEdgeTriangleList* sources, std::int32_t count, std::uint32_t edge,
    const realtimecsg::recovered::Vector3* point, float weight, std::uint32_t* output) {
    if (!mesh || !transform || !point || !output) return -1;
    try { *output = realtimecsg::recovered::splitTriangleEdge(*mesh,*transform,sources,count,edge,*point,weight); return 0; }
    catch (const std::length_error&) { return -2; }
    catch (const std::invalid_argument&) { return -1; }
    catch (...) { return -3; }
}
