#include "realtimecsg/recovered_mesh_optimization.h"
#include <algorithm>
#include <cstring>
#include <stdexcept>
#include <vector>

namespace realtimecsg::recovered {
namespace {
template<class T> struct List {
    T* output; int count, capacity;
    std::vector<T> storage;
    List(T* data, int size, int cap): output(data),count(size),capacity(cap) {
        if (size < 0 || size > cap || (cap && !data)) throw std::invalid_argument("Invalid optimization list");
        storage.resize(static_cast<std::size_t>(cap));
        if (cap) std::memcpy(storage.data(),data,static_cast<std::size_t>(cap)*sizeof(T));
    }
    void push(const T& value) {
        if (count == capacity) throw std::length_error("Optimization list capacity exhausted");
        std::memcpy(&storage[count++],&value,sizeof(T));
    }
    bool contains(const T& value) const { return std::find(storage.begin(),storage.begin()+count,value) != storage.begin()+count; }
    void commit() const {
        if (capacity) std::memcpy(output,storage.data(),static_cast<std::size_t>(capacity)*sizeof(T));
    }
};
std::uint32_t previous(std::uint32_t edge) { return (edge&~3u)|((edge%4+2)%3); }
}
bool removeVertexInMiddleOfPlane(TriangleMeshView& mesh, const SurfaceView& surfaces, int planeCount,
    int vertex, PlaneOptimizationContextView& context, BrushEdgeGroupView* groups,
    BrushEdgeTriangleList* sources, BrushEdgeConnectionList* connections, int& groupCount) {
    if (mesh.triangleCount < 0 || mesh.triangleCount > mesh.triangleCapacity || mesh.triangleCount > (1<<30)
        || mesh.vertexCount < 0 || mesh.vertexCount > mesh.vertexCapacity
        || (mesh.triangleCount && !mesh.triangles) || !mesh.vertices || vertex < 0 || vertex >= mesh.vertexCount
        || groupCount < 0) throw std::invalid_argument("Invalid plane-vertex views");
    List<std::uint32_t> boundary(context.boundaryEdges.edges,context.boundaryEdges.count,context.boundaryEdges.capacity);
    List<int> removed(context.removedTriangles.indices,context.removedTriangles.count,context.removedTriangles.capacity);
    List<int> incident(context.incidentTriangles.indices,context.incidentTriangles.count,context.incidentTriangles.capacity);
    List<int> removedVertices(context.removedVertices.indices,context.removedVertices.count,context.removedVertices.capacity);
    List<int> polygon(context.polygonVertices.indices,context.polygonVertices.count,context.polygonVertices.capacity);
    List<Vector3> normals(context.polygonNormals.values,context.polygonNormals.count,context.polygonNormals.capacity);
    std::vector<TriangleRecord> triangles;
    if (mesh.triangleCount) triangles.assign(mesh.triangles,mesh.triangles+mesh.triangleCount);
    std::vector<TriangleVertexRecord> vertices(mesh.vertices,mesh.vertices+mesh.vertexCount);
    auto working = mesh; working.triangles = triangles.data(); working.vertices = vertices.data();
    auto valid = [&](std::uint32_t h) { return (h&3u) != 3u && h>>2 < static_cast<unsigned>(mesh.triangleCount); };
    auto checked = [&](std::uint32_t h) -> TriangleRecord& {
        if (!valid(h)) throw std::invalid_argument("Invalid plane-vertex topology");
        return triangles[h>>2];
    };
    auto commit = [&](bool result) {
        if (!triangles.empty()) std::memcpy(mesh.triangles,triangles.data(),triangles.size()*sizeof(TriangleRecord));
        std::memcpy(mesh.vertices,vertices.data(),vertices.size()*sizeof(TriangleVertexRecord));
        boundary.commit(); removed.commit(); incident.commit(); removedVertices.commit(); polygon.commit(); normals.commit();
        context.boundaryEdges.count = boundary.count; context.removedTriangles.count = removed.count;
        context.incidentTriangles.count = incident.count; context.removedVertices.count = removedVertices.count;
        context.polygonVertices.count = polygon.count; context.polygonNormals.count = normals.count;
        return result;
    };
    const auto start = vertices[vertex].edge;
    if ((start&3u) == 3u) return commit(false);
    boundary.count = 0;
    if (start>>2 >= static_cast<unsigned>(mesh.triangleCount)) return commit(false);
    const auto metadata = triangles[start>>2];
    if (metadata.surfaceIndex < 0) return commit(false);
    incident.count = 0;
    std::vector<unsigned char> visited(static_cast<std::size_t>(mesh.triangleCount)*3,0);
    auto h = start;
    do {
        const auto& current = checked(h);
        if (current.vertices[h&3u] != vertex) throw std::invalid_argument("Fan edge does not end at vertex");
        const auto raw = current.twins[h&3u]; h = previous(raw);
        if (raw>>2 >= static_cast<unsigned>(mesh.triangleCount)) return commit(false);
        const auto& next = checked(h);
        if (next.flags != metadata.flags || next.surfaceIndex != metadata.surfaceIndex
            || next.categorizationState != metadata.categorizationState) return commit(false);
        const auto key = (h>>2)*3+(h&3u);
        if (visited[key]) throw std::invalid_argument("Nonclosing plane-vertex fan");
        visited[key] = 1;
        const int index = static_cast<int>(h>>2);
        if (!incident.contains(index)) incident.push(index);
        boundary.push(h);
    } while (h != start);
    polygon.count = normals.count = 0;
    for (int i = 0; i < boundary.count; ++i) {
        const auto opposite = previous(boundary.storage[i]);
        const auto outside = checked(opposite).twins[opposite&3u];
        if (!valid(outside)) return commit(false);
        // Native's second membership test still uses the first outside index;
        // replacing the edge on the alternate path cannot make it admissible.
        if (incident.contains(static_cast<int>(outside>>2))) return commit(false);
        const auto reverse = checked(outside).twins[outside&3u];
        const auto corner = previous(reverse);
        const auto& record = checked(corner);
        const int id = record.vertices[corner&3u];
        if (id < 0 || id >= mesh.vertexCount) throw std::invalid_argument("Invalid fan boundary vertex");
        if (polygon.contains(id)) return commit(false);
        boundary.storage[i] = outside;
        polygon.push(id); normals.push(record.cornerNormals[corner&3u]);
    }
    for (int i = 0; i < incident.count; ++i) {
        const int id = incident.storage[i]; removed.push(id); triangles[id].surfaceIndex = -1;
    }
    vertices[vertex].edge = kInvalidTriangleEdge; removedVertices.push(vertex);
    if (polygon.count != 2) {
        ConcavePolygonView contour{polygon.storage.data(),normals.storage.data(),boundary.storage.data(),polygon.count,
            std::min({polygon.capacity,normals.capacity,boundary.capacity})};
        TriangleIndexPoolView pool{removed.storage.data(),removed.count,removed.capacity};
        triangulateConcavePolygon(working,surfaces,planeCount,contour,pool,metadata.surfaceIndex,metadata.flags,metadata.categorizationState);
        polygon.count = normals.count = boundary.count = contour.count; removed.count = pool.count;
        return commit(true);
    }
    auto deleteA = boundary.storage[0], deleteB = boundary.storage[1];
    auto keepA = checked(deleteA).twins[deleteA&3u], keepB = checked(deleteB).twins[deleteB&3u];
    if ((keepA^keepB) >= 4u && (deleteA^deleteB) >= 4u) {
        std::swap(deleteA,keepA); std::swap(deleteB,keepB);
    }
    for (auto edge : {deleteA,deleteB}) {
        auto& record = checked(edge); const int id = static_cast<int>(edge>>2);
        if (!removed.contains(id)) removed.push(id);
        record.surfaceIndex = -1;
    }
    auto& a = checked(keepA); auto& b = checked(keepB);
    a.twins[keepA&3u] = keepB; b.twins[keepB&3u] = keepA;
    const int va = a.vertices[keepA&3u], vb = b.vertices[keepB&3u];
    if (va < 0 || va >= mesh.vertexCount || vb < 0 || vb >= mesh.vertexCount)
        throw std::invalid_argument("Invalid retained-edge vertex");
    vertices[va].edge = keepA; vertices[vb].edge = keepB;
    int pair[] = {a.edgeSource[keepA&3u],b.edgeSource[keepB&3u]};
    a.edgeSource[keepA&3u] = b.edgeSource[keepB&3u] = -1;
    // All other potentially failing work is complete. The recovered merge
    // preflights group capacities and is atomic; after success only nonthrowing
    // field writes and buffer commits remain, including caller groupCount.
    const int selected = mergeBrushIndexPair(working,groups,sources,connections,groupCount,pair);
    a.edgeSource[keepA&3u] = b.edgeSource[keepB&3u] = selected;
    return commit(false);
}
}
extern "C" std::int32_t rcsg_recovered_remove_plane_vertex(realtimecsg::recovered::TriangleMeshView* mesh,
    const realtimecsg::recovered::SurfaceView* surfaces, std::int32_t planeCount, std::int32_t vertex,
    realtimecsg::recovered::PlaneOptimizationContextView* context, realtimecsg::recovered::BrushEdgeGroupView* groups,
    realtimecsg::recovered::BrushEdgeTriangleList* sources, realtimecsg::recovered::BrushEdgeConnectionList* connections,
    std::int32_t* count) {
    if (!mesh || !surfaces || !context || !count) return -1;
    try { return realtimecsg::recovered::removeVertexInMiddleOfPlane(*mesh,*surfaces,planeCount,vertex,*context,groups,sources,connections,*count) ? 1 : 0; }
    catch (const std::invalid_argument&) { return -1; }
    catch (const std::length_error&) { return -2; } catch (...) { return -3; }
}
