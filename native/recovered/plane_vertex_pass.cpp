#include "realtimecsg/recovered_mesh_optimization.h"
#include "realtimecsg/recovered_mesh_cleanup.h"
#include <algorithm>
#include <cstring>
#include <stdexcept>
#include <vector>

namespace realtimecsg::recovered {
namespace detail {
void removeTwoSurfaceVerticesWorking(TriangleMeshView&, const SurfaceView&, int, PlaneOptimizationContextView&,
    BrushEdgeGroupView*, BrushEdgeTriangleList*, BrushEdgeConnectionList*, int&);
}
namespace {
template<class T> std::vector<T> copyStorage(const T* data, int count, int capacity) {
    if (count < 0 || count > capacity || (capacity && !data)) throw std::invalid_argument("Invalid optimization storage");
    // Keep unique backing addresses even for empty groups, so ownership can
    // be translated after the recovered merge swaps or removes descriptors.
    std::vector<T> result(static_cast<std::size_t>(std::max(capacity,1)));
    if (capacity) std::memcpy(result.data(),data,static_cast<std::size_t>(capacity)*sizeof(T));
    return result;
}
template<class T> void writeStorage(T* data, const std::vector<T>& values, int capacity) {
    if (capacity) std::memcpy(data,values.data(),static_cast<std::size_t>(capacity)*sizeof(T));
}
struct ContextCopy {
    PlaneOptimizationContextView original, view;
    std::vector<std::uint32_t> boundary;
    std::vector<int> removed, incident, removedVertices, polygon;
    std::vector<Vector3> normals;
    explicit ContextCopy(const PlaneOptimizationContextView& c): original(c),view(c),
        boundary(copyStorage(c.boundaryEdges.edges,c.boundaryEdges.count,c.boundaryEdges.capacity)),
        removed(copyStorage(c.removedTriangles.indices,c.removedTriangles.count,c.removedTriangles.capacity)),
        incident(copyStorage(c.incidentTriangles.indices,c.incidentTriangles.count,c.incidentTriangles.capacity)),
        removedVertices(copyStorage(c.removedVertices.indices,c.removedVertices.count,c.removedVertices.capacity)),
        polygon(copyStorage(c.polygonVertices.indices,c.polygonVertices.count,c.polygonVertices.capacity)),
        normals(copyStorage(c.polygonNormals.values,c.polygonNormals.count,c.polygonNormals.capacity)) {
        view.boundaryEdges.edges = boundary.data(); view.removedTriangles.indices = removed.data();
        view.incidentTriangles.indices = incident.data(); view.removedVertices.indices = removedVertices.data();
        view.polygonVertices.indices = polygon.data(); view.polygonNormals.values = normals.data();
    }
    void commit(PlaneOptimizationContextView& out) const {
        writeStorage(original.boundaryEdges.edges,boundary,original.boundaryEdges.capacity);
        writeStorage(original.removedTriangles.indices,removed,original.removedTriangles.capacity);
        writeStorage(original.incidentTriangles.indices,incident,original.incidentTriangles.capacity);
        writeStorage(original.removedVertices.indices,removedVertices,original.removedVertices.capacity);
        writeStorage(original.polygonVertices.indices,polygon,original.polygonVertices.capacity);
        writeStorage(original.polygonNormals.values,normals,original.polygonNormals.capacity);
        out.boundaryEdges.count = view.boundaryEdges.count; out.removedTriangles.count = view.removedTriangles.count;
        out.incidentTriangles.count = view.incidentTriangles.count; out.removedVertices.count = view.removedVertices.count;
        out.polygonVertices.count = view.polygonVertices.count; out.polygonNormals.count = view.polygonNormals.count;
    }
};
struct GroupCopy {
    BrushEdgeGroupView group;
    BrushEdgeTriangleList source;
    BrushEdgeConnectionList connection;
    std::vector<BrushTriangleEdge> refs, links;
    std::vector<std::uint32_t> edges;
    GroupCopy(BrushEdgeGroupView g, BrushEdgeTriangleList s, BrushEdgeConnectionList c): group(g),source(s),connection(c),
        refs(copyStorage(g.surfaceRefs,g.surfaceRefCount,g.surfaceRefCapacity)),
        links(copyStorage(c.connections,c.count,c.capacity)),edges(copyStorage(s.edges,s.count,s.capacity)) {}
};
}
static void runVertexPass(TriangleMeshView& mesh, const SurfaceView& surfaces, int planeCount,
    PlaneOptimizationContextView& context, BrushEdgeGroupView* groups, BrushEdgeTriangleList* sources,
    BrushEdgeConnectionList* connections, int& groupCount, int passes) {
    if (mesh.triangleCount < 0 || mesh.triangleCount > mesh.triangleCapacity || mesh.triangleCount > (1<<30)
        || mesh.vertexCount < 0 || mesh.vertexCount > mesh.vertexCapacity
        || (mesh.triangleCount && !mesh.triangles) || (mesh.vertexCount && !mesh.vertices)
        || groupCount < 0 || (groupCount && (!groups || !sources || !connections)))
        throw std::invalid_argument("Invalid plane optimization views");
    const int triangleStorage=passes ? mesh.triangleCapacity : mesh.triangleCount;
    const int vertexStorage=passes ? mesh.vertexCapacity : mesh.vertexCount;
    auto triangles = copyStorage(mesh.triangles,mesh.triangleCount,triangleStorage);
    auto vertices = copyStorage(mesh.vertices,mesh.vertexCount,vertexStorage);
    auto working = mesh; working.triangles = triangles.data(); working.vertices = vertices.data();
    ContextCopy copiedContext(context);
    std::vector<GroupCopy> owned; owned.reserve(static_cast<std::size_t>(groupCount));
    std::vector<BrushEdgeGroupView> copiedGroups;
    std::vector<BrushEdgeTriangleList> copiedSources;
    std::vector<BrushEdgeConnectionList> copiedConnections;
    for (int i = 0; i < groupCount; ++i) {
        owned.emplace_back(groups[i],sources[i],connections[i]); auto& owner = owned.back();
        auto g = groups[i]; g.surfaceRefs = owner.refs.data(); copiedGroups.push_back(g);
        auto s = sources[i]; s.edges = owner.edges.data(); copiedSources.push_back(s);
        auto c = connections[i]; c.connections = owner.links.data(); copiedConnections.push_back(c);
    }
    int count = groupCount;
    copiedContext.view.removedTriangles.count = copiedContext.view.removedVertices.count = 0;
    auto compact=[&]() {
        cleanUpRemovedVertices(working,copiedContext.view.removedVertices.indices,copiedContext.view.removedVertices.count,
            copiedContext.view.removedTriangles.indices,copiedContext.view.removedTriangles.count);
    };
    if (passes!=1) {
        for (int vertex = 0; vertex < working.vertexCount; ++vertex)
            removeVertexInMiddleOfPlane(working,surfaces,planeCount,vertex,copiedContext.view,
                copiedGroups.data(),copiedSources.data(),copiedConnections.data(),count);
        compact();
    }
    if (passes) {
        copiedContext.view.removedTriangles.count = copiedContext.view.removedVertices.count = 0;
        detail::removeTwoSurfaceVerticesWorking(working,surfaces,planeCount,copiedContext.view,
            copiedGroups.data(),copiedSources.data(),copiedConnections.data(),count);
        compact();
    }

    // Translate all active and retired descriptors before touching caller
    // buffers. A merge carries all three views with their original owner.
    for (int i = 0; i < groupCount; ++i) {
        const auto found = std::find_if(owned.begin(),owned.end(),[&](const GroupCopy& owner) {
            return owner.refs.data() == copiedGroups[i].surfaceRefs;
        });
        if (found == owned.end() || found->edges.data() != copiedSources[i].edges
            || found->links.data() != copiedConnections[i].connections)
            throw std::logic_error("Optimization lost group buffer ownership");
        copiedGroups[i].surfaceRefs = found->group.surfaceRefs;
        copiedSources[i].edges = found->source.edges; copiedConnections[i].connections = found->connection.connections;
    }
    for (const auto& owner : owned) {
        writeStorage(owner.group.surfaceRefs,owner.refs,owner.group.surfaceRefCapacity);
        writeStorage(owner.source.edges,owner.edges,owner.source.capacity);
        writeStorage(owner.connection.connections,owner.links,owner.connection.capacity);
    }
    for (int i = 0; i < groupCount; ++i) {
        groups[i] = copiedGroups[i]; sources[i] = copiedSources[i]; connections[i] = copiedConnections[i];
    }
    if (triangleStorage) std::memcpy(mesh.triangles,triangles.data(),static_cast<std::size_t>(triangleStorage)*sizeof(TriangleRecord));
    if (vertexStorage) std::memcpy(mesh.vertices,vertices.data(),static_cast<std::size_t>(vertexStorage)*sizeof(TriangleVertexRecord));
    mesh.triangleCount = working.triangleCount; mesh.vertexCount = working.vertexCount;
    copiedContext.commit(context); groupCount = count;
}
void removeVerticesInMiddleOfPlane(TriangleMeshView& mesh, const SurfaceView& surfaces, int planeCount,
    PlaneOptimizationContextView& context, BrushEdgeGroupView* groups, BrushEdgeTriangleList* sources,
    BrushEdgeConnectionList* connections, int& groupCount) {
    runVertexPass(mesh,surfaces,planeCount,context,groups,sources,connections,groupCount,0);
}
void removeVertexOnEdgeBetween2Surfaces(TriangleMeshView& mesh, const SurfaceView& surfaces, int planeCount,
    PlaneOptimizationContextView& context, BrushEdgeGroupView* groups, BrushEdgeTriangleList* sources,
    BrushEdgeConnectionList* connections, int& groupCount) {
    runVertexPass(mesh,surfaces,planeCount,context,groups,sources,connections,groupCount,1);
}
void optimizeTriangleMesh(TriangleMeshView& mesh, const SurfaceView& surfaces, int planeCount,
    PlaneOptimizationContextView& context, BrushEdgeGroupView* groups, BrushEdgeTriangleList* sources,
    BrushEdgeConnectionList* connections, int& groupCount) {
    runVertexPass(mesh,surfaces,planeCount,context,groups,sources,connections,groupCount,2);
}
}
extern "C" std::int32_t rcsg_recovered_remove_plane_vertices(realtimecsg::recovered::TriangleMeshView* mesh,
    const realtimecsg::recovered::SurfaceView* surfaces, std::int32_t planeCount,
    realtimecsg::recovered::PlaneOptimizationContextView* context, realtimecsg::recovered::BrushEdgeGroupView* groups,
    realtimecsg::recovered::BrushEdgeTriangleList* sources, realtimecsg::recovered::BrushEdgeConnectionList* connections,
    std::int32_t* count) {
    if (!mesh || !surfaces || !context || !count) return -1;
    try { realtimecsg::recovered::removeVerticesInMiddleOfPlane(*mesh,*surfaces,planeCount,*context,groups,sources,connections,*count); return 0; }
    catch (const std::invalid_argument&) { return -1; }
    catch (const std::length_error&) { return -2; } catch (...) { return -3; }
}
extern "C" std::int32_t rcsg_recovered_remove_two_surface_vertices(realtimecsg::recovered::TriangleMeshView* mesh,
    const realtimecsg::recovered::SurfaceView* surfaces, std::int32_t planeCount,
    realtimecsg::recovered::PlaneOptimizationContextView* context, realtimecsg::recovered::BrushEdgeGroupView* groups,
    realtimecsg::recovered::BrushEdgeTriangleList* sources, realtimecsg::recovered::BrushEdgeConnectionList* connections,
    std::int32_t* count) {
    if (!mesh || !surfaces || !context || !count) return -1;
    try { realtimecsg::recovered::removeVertexOnEdgeBetween2Surfaces(*mesh,*surfaces,planeCount,*context,groups,sources,connections,*count); return 0; }
    catch (const std::invalid_argument&) { return -1; }
    catch (const std::length_error&) { return -2; } catch (...) { return -3; }
}
extern "C" std::int32_t rcsg_recovered_optimize_mesh(realtimecsg::recovered::TriangleMeshView* mesh,
    const realtimecsg::recovered::SurfaceView* surfaces, std::int32_t planeCount,
    realtimecsg::recovered::PlaneOptimizationContextView* context, realtimecsg::recovered::BrushEdgeGroupView* groups,
    realtimecsg::recovered::BrushEdgeTriangleList* sources, realtimecsg::recovered::BrushEdgeConnectionList* connections,
    std::int32_t* count) {
    if (!mesh || !surfaces || !context || !count) return -1;
    try { realtimecsg::recovered::optimizeTriangleMesh(*mesh,*surfaces,planeCount,*context,groups,sources,connections,*count); return 0; }
    catch (const std::invalid_argument&) { return -1; }
    catch (const std::length_error&) { return -2; } catch (...) { return -3; }
}
