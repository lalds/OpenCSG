#include "realtimecsg/recovered_mesh_optimization.h"
#include "realtimecsg/recovered_mesh_cleanup.h"
#include <array>
#include <cmath>
#include <cstring>
#include <iostream>
#include <stdexcept>

using namespace realtimecsg::recovered;
namespace {
void require(bool condition, const char* text) { if (!condition) throw std::runtime_error(text); }
struct Fixture {
    std::array<TriangleRecord,8> triangles{};
    std::array<TriangleVertexRecord,9> vertices{};
    std::array<std::uint32_t,32> boundary{};
    std::array<int,32> removed{},incident{},removedVertices{},polygon{};
    std::array<Vector3,32> normals{};
    Plane plane{{0,0,1},0}; SurfaceView surfaces{};
    TriangleMeshView mesh;
    PlaneOptimizationContextView context{{boundary.data(),0,32},{removed.data(),0,32},
        {incident.data(),0,32},{removedVertices.data(),0,32},{polygon.data(),0,32},{normals.data(),0,32}};
    int groups = 0;
    explicit Fixture(int n = 4): mesh{triangles.data(),vertices.data(),nullptr,nullptr,2*n,2*n+1,8,9} {
        surfaces.planes = &plane;
        for (auto& t : triangles) {
            for (auto& twin : t.twins) twin = kInvalidTriangleEdge;
            for (auto& source : t.edgeSource) source = -1;
            for (auto& link : t.brushEdges) link = {-1,kInvalidTriangleEdge};
            t.surfaceIndex = 0; t.flags = 8; t.categorizationState = 2;
            for (auto& normal : t.cornerNormals) normal = {0,0,1};
        }
        for (int i = 0; i < n; ++i) {
            const double angle = 2*3.141592653589793*i/n;
            vertices[i+1].position = {float(std::cos(angle)),float(std::sin(angle)),0};
            const double outer = angle+3.141592653589793/n;
            vertices[n+i+1].position = {float(3*std::cos(outer)),float(3*std::sin(outer)),0};
            auto& t = triangles[i];
            t.vertices[0] = 0; t.vertices[1] = i+1; t.vertices[2] = (i+1)%n+1;
            t.twins[0] = ((i+1)%n)*4+1; t.twins[1] = ((i+n-1)%n)*4; t.twins[2] = (n+i)*4;
            auto& outside = triangles[n+i];
            outside.vertices[0] = i+1; outside.vertices[1] = n+i+1; outside.vertices[2] = (i+1)%n+1;
            outside.twins[0] = i*4+2; outside.edgeSource[0] = -1;
        }
        for (int i = 0; i < 2*n; ++i) for (int c = 0; c < 3; ++c) vertices[triangles[i].vertices[c]].edge = i*4+c;
        vertices[0].edge = 0;
    }
    int run() { return rcsg_recovered_remove_plane_vertex(&mesh,&surfaces,1,0,&context,nullptr,nullptr,nullptr,&groups); }
};
float area(const TriangleMeshView& mesh) {
    float result = 0;
    for (int i = 0; i < mesh.triangleCount; ++i) {
        const auto& t = mesh.triangles[i]; if (t.surfaceIndex < 0) continue;
        const auto a = mesh.vertices[t.vertices[0]].position, b = mesh.vertices[t.vertices[1]].position,
                   c = mesh.vertices[t.vertices[2]].position;
        result += ((b.x-a.x)*(c.y-a.y)-(b.y-a.y)*(c.x-a.x))*.5f;
    }
    return result;
}
void removeAndCompact() {
    Fixture f; const float before = area(f.mesh);
    require(f.run() == 1,"Planar center removal failed");
    require(f.mesh.triangleCount == 8 && f.mesh.vertexCount == 9,"Removal compacted prematurely");
    require(f.context.removedTriangles.count == 2 && f.removed[0] == 1 && f.removed[1] == 2 &&
        f.context.removedVertices.count == 1 && f.removedVertices[0] == 0,"Removed record order differs");
    require(f.context.incidentTriangles.count == 4 && f.incident[0] == 1 && f.incident[3] == 0,
        "Fan rotation order differs");
    require(f.context.boundaryEdges.count == 0 && f.context.polygonVertices.count == 0 &&
        f.context.polygonNormals.count == 0 && f.vertices[0].edge == kInvalidTriangleEdge,"Retringulation scratch counts differ");
    for (int i = 0; i < f.mesh.triangleCount; ++i) if (f.triangles[i].surfaceIndex >= 0)
        for (int vertex : f.triangles[i].vertices) require(vertex != 0,"Live triangle still uses removed center");
    require(std::abs(area(f.mesh)-before) < 1e-5f,"Planar removal changed area");
    cleanUpRemovedVertices(f.mesh,f.removedVertices.data(),1,f.removed.data(),2);
    require(f.mesh.triangleCount == 6 && f.mesh.vertexCount == 8,"Removal/compaction integration counts differ");
    require(std::abs(area(f.mesh)-before) < 1e-5f,"Compaction changed patch area");
}
void twoCornersAndGroups() {
    Fixture f(2);
    std::array<std::array<BrushTriangleEdge,4>,3> refs{},connections{};
    std::array<std::array<std::uint32_t,4>,3> handles{};
    std::array<BrushEdgeGroupView,3> groups{}; std::array<BrushEdgeTriangleList,3> sources{};
    std::array<BrushEdgeConnectionList,3> outputs{};
    for (int i = 0; i < 3; ++i) {
        refs[i][0] = {i,static_cast<unsigned>(i)};
        groups[i] = {refs[i].data(),1,4,i,i+1,{{1,0,0},{0,0,0}}};
        sources[i] = {handles[i].data(),0,4}; outputs[i] = {connections[i].data(),0,4};
    }
    f.groups = 3; f.triangles[2].edgeSource[0] = 0; f.triangles[3].edgeSource[0] = 2;
    const auto savedTriangles = f.triangles; const auto savedVertices = f.vertices; const auto savedRefs = refs;
    const auto savedContext = f.context;
    groups[0].surfaceRefCapacity = 1;
    require(rcsg_recovered_remove_plane_vertex(&f.mesh,&f.surfaces,1,0,&f.context,
        groups.data(),sources.data(),outputs.data(),&f.groups) == -2 && f.groups == 3 &&
        std::memcmp(savedTriangles.data(),f.triangles.data(),sizeof(savedTriangles)) == 0 &&
        std::memcmp(savedVertices.data(),f.vertices.data(),sizeof(savedVertices)) == 0 &&
        std::memcmp(savedRefs.data(),refs.data(),sizeof(savedRefs)) == 0 &&
        std::memcmp(&savedContext,&f.context,sizeof(savedContext)) == 0 && groups[0].surfaceRefCount == 1,
        "Group capacity failure partially committed vertex removal");
    groups[0].surfaceRefCapacity = 4;
    const int result = rcsg_recovered_remove_plane_vertex(&f.mesh,&f.surfaces,1,0,&f.context,
        groups.data(),sources.data(),outputs.data(),&f.groups);
    require(result == 0 && f.vertices[0].edge == kInvalidTriangleEdge && f.context.removedVertices.count == 1,
        "Native false-with-mutation branch not preserved");
    require(f.triangles[0].surfaceIndex == -1 && f.triangles[1].surfaceIndex == -1 &&
        f.triangles[2].twins[0] == 12 && f.triangles[3].twins[0] == 8,"Two-corner rewiring differs");
    require(f.groups == 2 && groups[0].surfaceRefCount == 2 &&
        f.triangles[2].edgeSource[0] == 0 && f.triangles[3].edgeSource[0] == 0,"Two-corner group merge differs");
    require(f.context.boundaryEdges.count == 2 && f.context.polygonVertices.count == 2,
        "Two-corner scratch was cleared unexpectedly");
}
void rejectionAndAtomicErrors() {
    Fixture f; f.triangles[2].flags ^= 1;
    f.context.polygonVertices.count = f.context.polygonNormals.count = 2;
    const auto old = f.triangles; const auto vertices = f.vertices;
    require(f.run() == 0 && f.context.boundaryEdges.count == 1 && f.boundary[0] == 4 &&
        f.context.incidentTriangles.count == 1 && f.incident[0] == 1,"Metadata rejection scratch differs");
    require(f.context.polygonVertices.count == 2 && f.context.polygonNormals.count == 2 &&
        std::memcmp(old.data(),f.triangles.data(),sizeof(old)) == 0 &&
        std::memcmp(vertices.data(),f.vertices.data(),sizeof(vertices)) == 0,"Metadata rejection changed mesh or later scratch");
    Fixture small; small.context.removedTriangles.capacity = 1;
    const auto original = small.triangles; const auto originalVertices = small.vertices;
    const auto context = small.context;
    require(small.run() == -2 && std::memcmp(&context,&small.context,sizeof(context)) == 0 &&
        std::memcmp(original.data(),small.triangles.data(),sizeof(original)) == 0 &&
        std::memcmp(originalVertices.data(),small.vertices.data(),sizeof(originalVertices)) == 0 &&
        small.boundary[0] == 0 && small.removed[0] == 0,"Capacity error partially changed state");
    Fixture invalid; invalid.vertices[0].edge = kInvalidTriangleEdge;
    invalid.context.boundaryEdges.count = 2; invalid.context.incidentTriangles.count = 2;
    require(invalid.run() == 0 && invalid.context.boundaryEdges.count == 2 &&
        invalid.context.incidentTriangles.count == 2,"Invalid starting handle cleared scratch");
    Fixture singleton(1);
    require(singleton.run() == 1 && singleton.context.removedTriangles.count == 1 &&
        singleton.context.removedVertices.count == 1 && singleton.context.boundaryEdges.count == 1 &&
        singleton.context.polygonVertices.count == 1,"Synthetic one-corner path differs");
}
}
int main() {
    try { removeAndCompact(); twoCornersAndGroups(); rejectionAndAtomicErrors(); }
    catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
    std::cout << "Planar removal, compaction integration, two-corner merge and context errors passed\n";
}
