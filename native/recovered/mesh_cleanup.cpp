#include "realtimecsg/recovered_mesh_cleanup.h"

#include <algorithm>
#include <cstring>
#include <limits>
#include <numeric>
#include <stdexcept>

namespace realtimecsg::recovered {
namespace {
void validateMesh(const TriangleMeshView& mesh) {
    if (mesh.triangleCount < 0 || mesh.vertexCount < 0 || mesh.triangleCount > mesh.triangleCapacity
        || mesh.vertexCount > mesh.vertexCapacity || (!mesh.triangles && mesh.triangleCount)
        || (!mesh.vertices && mesh.vertexCount)) throw std::invalid_argument("Invalid mesh buffers");
}
std::vector<int> removalMap(int total, const int* removed, int count) {
    if (count < 0 || count > total || (!removed && count)) throw std::invalid_argument("Invalid removal list");
    std::vector<int> map(static_cast<std::size_t>(total));
    std::iota(map.begin(),map.end(),0);
    for (int i = 0; i < count; ++i) {
        if (removed[i] < 0 || removed[i] >= total || map[removed[i]] < 0)
            throw std::invalid_argument("Invalid or duplicate removal index");
        map[removed[i]] = -1;
    }
    return map;
}

// Swap live tail records into sorted holes. Native vertex path skips its count
// subtraction if the tail scan runs out of records; preserve that observed path.
template<class Record>
bool compact(Record* records, std::vector<int>& map, const int* removed, int count) {
    int limit = static_cast<int>(map.size()), work = count;
    for (int i = 0; i < work; ++i) {
        int tail = limit-1;
        while (map[tail] == -1) {
            --tail; --work;
            if (tail < 0) return false;
        }
        if (removed[i] < tail) {
            map[tail] = removed[i];
            std::memcpy(&records[removed[i]],&records[tail],sizeof(Record));
            limit = tail;
        } else {
            limit = tail+1;
        }
    }
    return true;
}
const GapBrushView* activeBrush(const GapBrushView* manager, int count, int index) {
    if (index < 0 || index >= count || !manager[index].mesh || manager[index].nodeIndex != index) return nullptr;
    return &manager[index];
}
} // namespace

void cleanUpRemovedVertices(TriangleMeshView& mesh, int* removedVertices, int vertexRemovalCount,
                           int* removedTriangles, int triangleRemovalCount) {
    validateMesh(mesh);
    auto vertices = removalMap(mesh.vertexCount,removedVertices,vertexRemovalCount);
    auto triangles = removalMap(mesh.triangleCount,removedTriangles,triangleRemovalCount);
    // Validate references before moving anything. Native callers have already
    // invalidated removed polygons and rewired any surviving vertex rings.
    for (int p = 0; p < mesh.triangleCount; ++p) {
        const auto& t = mesh.triangles[p];
        if (triangles[p] < 0 && t.surfaceIndex >= 0)
            throw std::invalid_argument("Removed triangle must already be invalidated");
        for (int c = 0; c < 3; ++c) {
            if (t.surfaceIndex >= 0 && (t.vertices[c] < 0 || t.vertices[c] >= mesh.vertexCount
                || vertices[t.vertices[c]] < 0)) throw std::invalid_argument("Live triangle uses removed/invalid vertex");
            if (triangles[p] >= 0 && t.surfaceIndex < 0)
                throw std::invalid_argument("Invalid triangle is missing from removal list");
        }
    }
    for (int v = 0; v < mesh.vertexCount; ++v) {
        const auto edge = mesh.vertices[v].edge;
        if (edge != kInvalidTriangleEdge && edge >> 2 >= static_cast<unsigned>(mesh.triangleCount))
            throw std::invalid_argument("Invalid surviving vertex edge handle");
    }
    if (vertexRemovalCount) std::sort(removedVertices,removedVertices+vertexRemovalCount);
    if (triangleRemovalCount) std::sort(removedTriangles,removedTriangles+triangleRemovalCount);
    if (compact(mesh.vertices,vertices,removedVertices,vertexRemovalCount)) mesh.vertexCount -= vertexRemovalCount;
    for (int p = 0; p < mesh.triangleCount; ++p) {
        auto& t = mesh.triangles[p];
        if (t.surfaceIndex >= 0) for (int& vertex : t.vertices) vertex = vertices[vertex];
    }
    compact(mesh.triangles,triangles,removedTriangles,triangleRemovalCount);
    mesh.triangleCount -= triangleRemovalCount;
    for (int v = 0; v < mesh.vertexCount; ++v) {
        auto& edge = mesh.vertices[v].edge;
        if (edge != kInvalidTriangleEdge) edge = (static_cast<std::uint32_t>(triangles[edge>>2])<<2) | (edge & 3u);
    }
    for (int p = 0; p < mesh.triangleCount; ++p) {
        auto& t = mesh.triangles[p];
        for (auto& twin : t.twins) {
            const unsigned old = twin>>2;
            if (old < triangles.size() && triangles[old] >= 0 && triangles[old] < mesh.triangleCount)
                twin = (static_cast<std::uint32_t>(triangles[old])<<2) | (twin & 3u);
            else twin = kInvalidTriangleEdge;
        }
        for (unsigned c = 0; c < 3; ++c)
            mesh.vertices[t.vertices[c]].edge = (static_cast<std::uint32_t>(p)<<2) | c;
    }
}

TriangulatedBrush compactVisibleTriangles(const TriangulatedBrush& input) {
    if (input.triangles.size() > static_cast<std::size_t>(std::numeric_limits<int>::max())
        || input.vertices.size() > static_cast<std::size_t>(std::numeric_limits<int>::max()))
        throw std::length_error("Visible mesh exceeds index limit");
    TriangulatedBrush result = input;
    std::vector<int> removedTriangles, removedVertices;
    std::vector<bool> used(result.vertices.size(),false);
    for (std::size_t p = 0; p < result.triangles.size(); ++p) {
        auto& triangle = result.triangles[p];
        if (triangle.categorizationState == 1) {
            triangle.surfaceIndex = -1;
            removedTriangles.push_back(static_cast<int>(p));
        } else {
            if ((triangle.categorizationState != 2 && triangle.categorizationState != 3) || triangle.surfaceIndex < 0)
                throw std::invalid_argument("Visible export requires resolved surface states");
            for (int v : triangle.vertices) {
                if (v < 0 || static_cast<std::size_t>(v) >= used.size()) throw std::invalid_argument("Invalid visible vertex");
                used[v] = true;
            }
        }
        for (auto& link : triangle.brushEdges) link = {-1,kInvalidTriangleEdge};
        // Source lists belong to the complete prepared mesh. This export
        // returns no groups and renumbers triangles, so retaining their IDs
        // would expose an unrelated/stale group namespace to the consumer.
        for (auto& source : triangle.edgeSource) source = -1;
    }
    // A fully empty portable export has no records, handled outside the native
    // tail-scan path that can skip its vertexCount subtraction.
    if (removedTriangles.size() == result.triangles.size()) return {};
    for (std::size_t v = 0; v < used.size(); ++v) if (!used[v]) removedVertices.push_back(static_cast<int>(v));
    TriangleMeshView mesh{result.triangles.data(),result.vertices.data(),nullptr,nullptr,
        static_cast<int>(result.triangles.size()),static_cast<int>(result.vertices.size()),
        static_cast<int>(result.triangles.size()),static_cast<int>(result.vertices.size())};
    cleanUpRemovedVertices(mesh,removedVertices.data(),static_cast<int>(removedVertices.size()),
                          removedTriangles.data(),static_cast<int>(removedTriangles.size()));
    result.triangles.resize(static_cast<std::size_t>(mesh.triangleCount));
    // Native skips its count subtraction when the tail scan reaches the start,
    // including some inputs with a small surviving set moved into low holes.
    // Keep native behavior in the recovered function, but enforce the known
    // dense count at this NEW export-adapter boundary.
    result.vertices.resize(used.size()-removedVertices.size());
    return result;
}

void connectGaps(TriangleMeshView& mesh, int brushIndex, int planeCount,
                 const GapBrushView* manager, int managerCount) {
    validateMesh(mesh);
    if (planeCount < 0 || managerCount < 0 || (!manager && managerCount))
        throw std::invalid_argument("Invalid gap manager or surfaces");
    if (mesh.triangleCount && (brushIndex < 0 || brushIndex >= managerCount
        || manager[brushIndex].mesh != &mesh || manager[brushIndex].nodeIndex != brushIndex))
        throw std::invalid_argument("Source mesh does not match gap manager");
    // Full preflight. The original assumes active node edge handles are valid.
    for (int p = 0; p < mesh.triangleCount; ++p) {
        const auto& t = mesh.triangles[p];
        if (t.surfaceIndex < 0 || t.surfaceIndex >= planeCount) continue;
        for (const auto& link : t.brushEdges) {
            const auto* target = activeBrush(manager,managerCount,link.brushIndex);
            if (!target) continue;
            validateMesh(*target->mesh);
            if ((link.edge & 3u) == 3u || link.edge >> 2 >= static_cast<unsigned>(target->mesh->triangleCount))
                throw std::invalid_argument("Invalid active cross-brush edge handle");
        }
    }
    for (int p = 0; p < mesh.triangleCount; ++p) {
        if (mesh.triangles[p].surfaceIndex < 0 || mesh.triangles[p].surfaceIndex >= planeCount) continue;
        for (unsigned c = 0; c < 3; ++c) {
            // Read as the loop advances: writes can alias this mesh, as native.
            const auto link = mesh.triangles[p].brushEdges[c];
            const auto* target = activeBrush(manager,managerCount,link.brushIndex);
            if (!target) continue;
            target->mesh->triangles[link.edge>>2].brushEdges[link.edge & 3u]
                = {brushIndex,(static_cast<std::uint32_t>(p)<<2) | c};
        }
    }
}
} // namespace realtimecsg::recovered

extern "C" std::int32_t rcsg_recovered_cleanup_mesh(realtimecsg::recovered::TriangleMeshView* mesh,
                                                  std::int32_t* v, std::int32_t vc, std::int32_t* t, std::int32_t tc) {
    if (!mesh) return -1;
    try { realtimecsg::recovered::cleanUpRemovedVertices(*mesh,v,vc,t,tc); return 0; }
    catch (const std::invalid_argument&) { return -1; }
    catch (...) { return -3; }
}
extern "C" std::int32_t rcsg_recovered_connect_gaps(realtimecsg::recovered::TriangleMeshView* mesh,
    std::int32_t brush, std::int32_t planes, const realtimecsg::recovered::GapBrushView* manager, std::int32_t count) {
    if (!mesh) return -1;
    try { realtimecsg::recovered::connectGaps(*mesh,brush,planes,manager,count); return 0; }
    catch (const std::invalid_argument&) { return -1; }
    catch (...) { return -3; }
}
