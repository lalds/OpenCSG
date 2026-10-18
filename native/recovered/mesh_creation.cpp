#include "realtimecsg/recovered_triangulation.h"

#include <cstring>
#include <stdexcept>
#include <vector>

namespace realtimecsg::recovered {
void createTriangleMesh(TriangleMeshView& destination, BrushMemoryView& brush,
                        const SurfaceView& surfaces, int planeCount) {
    if (brush.polygonCount < 0 || brush.polygonCount > brush.polygonCapacity
        || brush.edgeCount < 0 || brush.edgeChunkCapacity < 0
        || std::int64_t(brush.edgeCount) > std::int64_t(brush.edgeChunkCapacity)*4
        || brush.vertexCount < 0 || brush.vertexCount > brush.vertexCapacity
        || (brush.polygonCount && !brush.polygons) || (brush.edgeCount && (!brush.edges || !brush.edgeLinks))
        || (brush.vertexCount && !brush.vertices) || planeCount < 0 || (planeCount && !surfaces.planes)
        || destination.triangleCount != 0 || destination.vertexCount != 0
        || destination.worldPositions || destination.worldVertexLookup
        || destination.triangleCapacity < 0 || destination.vertexCapacity < 0
        || (destination.triangleCapacity && !destination.triangles)
        || (destination.vertexCapacity && !destination.vertices))
        throw std::invalid_argument("Invalid Create buffers");
    std::vector<PolygonRecord> polygons;
    std::vector<HalfEdgeRecord> edges;
    if (brush.polygonCount) polygons.assign(brush.polygons,brush.polygons+brush.polygonCount);
    if (brush.edgeCount) edges.assign(brush.edges,brush.edges+brush.edgeCount);
    auto checkedEdge = [&](int e) -> HalfEdgeRecord& {
        if (e < 0 || e >= brush.edgeCount) throw std::invalid_argument("Invalid Create half-edge");
        return edges[e];
    };
    // Native restarts its polygon scan after each removed loop. The recorded
    // count is cleared, but its first edge also has to become -1 to skip it.
    for (int p = 0; p < brush.polygonCount;) {
        auto& polygon = polygons[p];
        if (polygon.edgeCount < 0) throw std::invalid_argument("Negative polygon size");
        if (polygon.edgeCount > 2 || (polygon.edgeCount == 0 && polygon.firstEdge < 0)) { ++p; continue; }
        polygon.edgeCount = 0;
        const int e = polygon.firstEdge;
        if (e < 0) { ++p; continue; }
        const int twinA = checkedEdge(e).twinEdge;
        const int twinB = checkedEdge(checkedEdge(e).nextEdge).twinEdge;
        checkedEdge(twinA); checkedEdge(twinB);
        polygon.firstEdge = -1;
        edges[twinA].twinEdge = twinB; edges[twinB].twinEdge = twinA;
        p = 0;
    }
    std::int64_t count = 0;
    for (const auto& polygon : polygons) {
        if (!polygon.edgeCount) continue;
        if (polygon.surfaceIndex < 0 || polygon.surfaceIndex >= planeCount)
            throw std::invalid_argument("Invalid Create surface");
        count += polygon.edgeCount-2;
    }
    if (count > destination.triangleCapacity || brush.vertexCount > destination.vertexCapacity
        || count >= (std::int64_t(1)<<30)) throw std::length_error("Create output capacity exhausted");
    std::vector<TriangleRecord> triangles;
    if (count) triangles.assign(destination.triangles,destination.triangles+count);
    std::vector<TriangleVertexRecord> vertices(static_cast<std::size_t>(brush.vertexCount));
    for (int v = 0; v < brush.vertexCount; ++v) vertices[v] = {kInvalidTriangleEdge,brush.vertices[v]};
    TriangleMeshView mesh{triangles.data(),vertices.data(),nullptr,nullptr,0,brush.vertexCount,
                          static_cast<int>(count),brush.vertexCount};
    std::vector<std::uint32_t> edgeMap(static_cast<std::size_t>(brush.edgeCount),kInvalidTriangleEdge);
    std::vector<int> indices, sourceEdges;
    std::vector<Vector3> normals;
    std::vector<std::uint32_t> input, output;
    for (const auto& polygon : polygons) {
        if (!polygon.edgeCount) continue;
        const int n = polygon.edgeCount;
        if (n > brush.edgeCount) throw std::invalid_argument("Oversized Create loop");
        indices.clear(); sourceEdges.clear(); input.clear(); output.resize(static_cast<std::size_t>(n));
        normals.assign(static_cast<std::size_t>(n),surfaces.planes[polygon.surfaceIndex].normal);
        int e = polygon.firstEdge;
        for (int i = 0; i < n; ++i) {
            const auto& edge = checkedEdge(e);
            if (edge.twinEdge >= brush.edgeCount) throw std::invalid_argument("Invalid Create twin");
            indices.push_back(edge.vertexIndex); sourceEdges.push_back(e); input.push_back(edgeMap[e]);
            e = edge.nextEdge;
        }
        if (e != polygon.firstEdge) throw std::invalid_argument("Unclosed Create loop");
        triangulateConvexPolygon(mesh,indices.data(),normals.data(),input.data(),output.data(),n,
            polygon.surfaceIndex,static_cast<std::uint8_t>(polygon.unidentified_10),polygon.categorizationState);
        for (int i = 0; i < n; ++i) {
            const int source = sourceEdges[i];
            mesh.triangles[output[i]>>2].edgeSource[output[i]&3u] = brush.edgeLinks[source].unidentified_04;
            const int twin = edges[source].twinEdge;
            if (twin >= 0) edgeMap[twin] = output[i];
        }
    }
    if (!polygons.empty()) std::memcpy(brush.polygons,polygons.data(),polygons.size()*sizeof(PolygonRecord));
    if (!edges.empty()) std::memcpy(brush.edges,edges.data(),edges.size()*sizeof(HalfEdgeRecord));
    if (!triangles.empty()) std::memcpy(destination.triangles,triangles.data(),triangles.size()*sizeof(TriangleRecord));
    if (!vertices.empty()) std::memcpy(destination.vertices,vertices.data(),vertices.size()*sizeof(TriangleVertexRecord));
    destination.triangleCount = mesh.triangleCount; destination.vertexCount = mesh.vertexCount;
}
}
extern "C" std::int32_t rcsg_recovered_create_triangle_mesh(realtimecsg::recovered::TriangleMeshView* mesh,
    realtimecsg::recovered::BrushMemoryView* brush, const realtimecsg::recovered::SurfaceView* surfaces,
    std::int32_t planeCount) {
    if (!mesh || !brush || !surfaces) return -1;
    try { realtimecsg::recovered::createTriangleMesh(*mesh,*brush,*surfaces,planeCount); return 0; }
    catch (const std::invalid_argument&) { return -1; }
    catch (const std::length_error&) { return -2; } catch (...) { return -3; }
}
