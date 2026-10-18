#include "realtimecsg/recovered_triangulation.h"

#include <cstring>
#include <limits>
#include <stdexcept>

namespace realtimecsg::recovered {
namespace {
bool validEdge(std::uint32_t edge) { return (edge & 3u) != 3u; }
std::uint32_t edgeIndex(int triangle, unsigned corner) {
    return (static_cast<std::uint32_t>(triangle) << 2) | corner;
}
void setTwin(TriangleMeshView& mesh, std::uint32_t edge, std::uint32_t twin) {
    if (validEdge(edge)) mesh.triangles[edge >> 2].twins[edge & 3u] = twin;
}
} // namespace

void triangulateConvexPolygon(TriangleMeshView& mesh, const std::int32_t* indices,
                             const Vector3* normals, const std::uint32_t* inputTwins,
                             std::uint32_t* outputEdges, int count, int surface,
                             std::uint8_t flags, std::uint16_t state) {
    if (count < 3 || !indices || !normals || !inputTwins || !outputEdges
        || !mesh.triangles || !mesh.vertices || mesh.triangleCount < 0 || mesh.vertexCount < 0
        || mesh.triangleCapacity < mesh.triangleCount || mesh.vertexCapacity < mesh.vertexCount)
        throw std::invalid_argument("Invalid triangle mesh or polygon buffers");
    if (count-2 > mesh.triangleCapacity-mesh.triangleCount)
        throw std::length_error("Triangle buffer exhausted");
    if (std::uint64_t(mesh.triangleCount)+std::uint64_t(count-2) > (std::uint64_t(1)<<30))
        throw std::length_error("Triangle edge handle limit");
    for (int i = 0; i < count; ++i) {
        if (indices[i] < 0 || indices[i] >= mesh.vertexCount)
            throw std::invalid_argument("Polygon vertex out of range");
        if (validEdge(inputTwins[i]) && inputTwins[i] >> 2 >= static_cast<unsigned>(mesh.triangleCount))
            throw std::invalid_argument("Boundary twin is not an existing triangle edge");
    }

    auto previous = inputTwins[1];
    for (int i = 2; i < count; ++i) {
        const int index = mesh.triangleCount++;
        auto& triangle = mesh.triangles[index];
        triangle.vertices[0] = indices[0];
        triangle.vertices[1] = indices[i-1];
        triangle.vertices[2] = indices[i];
        std::memcpy(&triangle.cornerNormals[0], &normals[0], sizeof(Vector3));
        std::memcpy(&triangle.cornerNormals[1], &normals[i-1], sizeof(Vector3));
        std::memcpy(&triangle.cornerNormals[2], &normals[i], sizeof(Vector3));
        triangle.twins[0] = kInvalidTriangleEdge;
        triangle.twins[1] = previous;
        triangle.twins[2] = inputTwins[i];
        for (unsigned c = 0; c < 3; ++c) {
            triangle.edgeSource[c] = -1;
            mesh.vertices[triangle.vertices[c]].edge = edgeIndex(index,c);
        }
        triangle.surfaceIndex = surface;
        triangle.flags = flags;
        triangle.categorizationState = state;
        setTwin(mesh, previous, edgeIndex(index,1));
        if (i == 2) outputEdges[1] = edgeIndex(index,1);
        setTwin(mesh, inputTwins[i], edgeIndex(index,2));
        outputEdges[i] = edgeIndex(index,2);
        previous = edgeIndex(index,0);
    }
    outputEdges[0] = edgeIndex(mesh.triangleCount-1,0);
    if (validEdge(inputTwins[0])) {
        setTwin(mesh, inputTwins[0],outputEdges[0]);
        mesh.triangles[mesh.triangleCount-1].twins[0] = inputTwins[0];
    }
}

void flagTriangleVisibility(TriangleMeshView& mesh) {
    if (mesh.triangleCount < 0 || mesh.triangleCount > mesh.triangleCapacity
        || (!mesh.triangles && mesh.triangleCount != 0))
        throw std::invalid_argument("Invalid triangle visibility view");
    for (int i = 0; i < mesh.triangleCount; ++i) {
        auto& t = mesh.triangles[i];
        const int state = t.categorizationState <= 32767 ? t.categorizationState : int(t.categorizationState)-65536;
        const unsigned bits = state == 1 || state == 4 ? 0 : state == 2 ? 8 : state == 3 ? 0x18 : 0x28;
        const unsigned low = state > 0 && state <= 4 ? static_cast<unsigned>(state-1) : 1;
        t.flags = static_cast<std::uint8_t>((t.flags & 0xc0u) | bits | low);
    }
}

TriangulatedBrush triangulateConvexBrush(const BrushMemoryView& brush, const SurfaceView& surfaces,
                                        int sourcePlaneCount) {
    if (!surfaces.planes || sourcePlaneCount <= 0 || brush.polygonCount < 0 || brush.vertexCount < 0
        || brush.edgeCount < 0 || !brush.polygons || !brush.edges || !brush.edgeLinks || !brush.vertices)
        throw std::invalid_argument("Invalid convex brush view");
    std::int64_t triangleCount = 0;
    for (int p = 0; p < brush.polygonCount; ++p) {
        const auto& polygon = brush.polygons[p];
        if (polygon.edgeCount < 3 || polygon.surfaceIndex < 0 || polygon.surfaceIndex >= sourcePlaneCount)
            throw std::invalid_argument("Invalid convex source polygon");
        triangleCount += polygon.edgeCount-2;
    }
    if (triangleCount > std::numeric_limits<int>::max() || triangleCount >= (std::int64_t(1)<<30))
        throw std::length_error("Too many triangles");
    TriangulatedBrush result;
    result.triangles.resize(static_cast<std::size_t>(triangleCount));
    result.vertices.resize(static_cast<std::size_t>(brush.vertexCount));
    TriangleMeshView mesh{result.triangles.data(),result.vertices.data(),nullptr,nullptr,0,0,
                          static_cast<int>(triangleCount),brush.vertexCount};
    // Keep the adapter's const input contract while using recovered Create,
    // whose degeneracy pass may mutate polygon records and twin links.
    std::vector<PolygonRecord> polygons(brush.polygons,brush.polygons+brush.polygonCount);
    std::vector<HalfEdgeRecord> edges(brush.edges,brush.edges+brush.edgeCount);
    auto working = brush;
    working.polygons = polygons.data(); working.edges = edges.data();
    createTriangleMesh(mesh,working,surfaces,sourcePlaneCount);
    flagTriangleVisibility(mesh);
    return result;
}
} // namespace realtimecsg::recovered

extern "C" std::int32_t rcsg_recovered_triangulate_convex(
    realtimecsg::recovered::TriangleMeshView* mesh, const std::int32_t* indices,
    const realtimecsg::recovered::Vector3* normals, const std::uint32_t* twins, std::uint32_t* output,
    std::int32_t count, std::int32_t surface, std::uint8_t flags, std::uint16_t state) {
    if (!mesh) return -1;
    try {
        realtimecsg::recovered::triangulateConvexPolygon(*mesh,indices,normals,twins,output,count,surface,flags,state);
        return count-2;
    } catch (const std::length_error&) { return -2; }
      catch (const std::invalid_argument&) { return -1; }
      catch (...) { return -3; }
}

extern "C" std::int32_t rcsg_recovered_flag_triangle_visibility(realtimecsg::recovered::TriangleMeshView* mesh) {
    if (!mesh) return -1;
    try { realtimecsg::recovered::flagTriangleVisibility(*mesh); return 0; }
    catch (...) { return -1; }
}
