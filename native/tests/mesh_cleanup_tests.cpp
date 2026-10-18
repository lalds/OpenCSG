#include "realtimecsg/recovered_mesh_cleanup.h"

#include <array>
#include <cstring>
#include <iostream>
#include <stdexcept>

using namespace realtimecsg::recovered;
namespace {
void require(bool value, const char* message) { if (!value) throw std::runtime_error(message); }
void compaction() {
    std::array<TriangleRecord,8> triangles{};
    std::array<TriangleVertexRecord,8> vertices{};
    for (int v = 0; v < 6; ++v) vertices[v] = {kInvalidTriangleEdge,{float(v),float(v+10),float(v+20)}};
    const int ids[4][3] = {{0,2,3},{1,1,1},{2,4,3},{0,5,2}};
    for (int p = 0; p < 4; ++p) {
        std::memcpy(triangles[p].vertices,ids[p],sizeof(ids[p]));
        for (auto& edge : triangles[p].twins) edge = kInvalidTriangleEdge;
        triangles[p].surfaceIndex = p;
        triangles[p].categorizationState = 2;
        triangles[p].unidentified_60[0] = std::byte{static_cast<unsigned char>(p+1)};
    }
    triangles[1].surfaceIndex = -1;
    triangles[0].twins[0] = 13; triangles[3].twins[1] = 0;
    triangles[0].twins[2] = 8; triangles[2].twins[0] = 2;
    triangles[2].twins[1] = 4;
    TriangleMeshView mesh{triangles.data(),vertices.data(),nullptr,nullptr,4,6,8,8};
    int removedV[] = {1}, removedT[] = {1};
    cleanUpRemovedVertices(mesh,removedV,1,removedT,1);
    require(mesh.vertexCount == 5 && mesh.triangleCount == 3,"Wrong compacted counts");
    require(vertices[1].position.x == 5 && triangles[1].surfaceIndex == 3,"Tail records not moved into holes");
    require(triangles[1].vertices[1] == 1 && triangles[1].unidentified_60[0] == std::byte{4},"Moved triangle metadata/index changed");
    require(triangles[0].twins[0] == 5 && triangles[1].twins[1] == 0,"Moved triangle twins not remapped");
    require(triangles[2].twins[1] == kInvalidTriangleEdge,"Reference to removed triangle retained");
    const std::uint32_t expectedEdges[] = {4,5,8,10,9};
    for (int v = 0; v < 5; ++v) require(vertices[v].edge == expectedEdges[v],"Vertex edge not rebuilt");
    const auto savedTriangles = triangles;
    const auto savedVertices = vertices;
    int duplicate[] = {0,0};
    require(rcsg_recovered_cleanup_mesh(&mesh,duplicate,2,nullptr,0) == -1,"Duplicate removal accepted");
    require(std::memcmp(triangles.data(),savedTriangles.data(),sizeof(triangles)) == 0
        && std::memcmp(vertices.data(),savedVertices.data(),sizeof(vertices)) == 0,"Invalid input partially mutated mesh");
    int liveVertex[] = {0};
    require(rcsg_recovered_cleanup_mesh(&mesh,liveVertex,1,nullptr,0) == -1,"Removing a live triangle vertex accepted");
    // Preserve observed native all-removed-vertex path; the new export adapter
    // handles fully empty results outside this routine.
    for (auto& v : vertices) v.edge = kInvalidTriangleEdge;
    mesh.triangleCount = 0; mesh.vertexCount = 2;
    int all[] = {1,0};
    cleanUpRemovedVertices(mesh,all,2,nullptr,0);
    require(mesh.vertexCount == 2 && all[0] == 0 && all[1] == 1,"Native all-removed path changed");
}

void crossBrushLinks() {
    std::array<TriangleRecord,2> a{},b{};
    for (auto* records : {&a,&b}) for (auto& t : *records) {
        t.surfaceIndex = 0;
        for (auto& link : t.brushEdges) link = {-1,kInvalidTriangleEdge};
    }
    TriangleMeshView ma{a.data(),nullptr,nullptr,nullptr,2,0,2,0};
    TriangleMeshView mb{b.data(),nullptr,nullptr,nullptr,2,0,2,0};
    GapBrushView manager[] = {{&ma,0,1},{&mb,1,1}};
    a[0].brushEdges[2] = {1,5};
    connectGaps(ma,0,1,manager,2);
    require(b[1].brushEdges[1].brushIndex == 0 && b[1].brushEdges[1].edge == 2,"Cross-brush reverse link missing");
    // An invalid source surface is skipped, including otherwise bad handles.
    a[1].surfaceIndex = 4;
    a[1].brushEdges[0] = {1,kInvalidTriangleEdge};
    connectGaps(ma,0,1,manager,2);
    // Stale manager nodes are skipped; active nodes with bad handles rejected.
    manager[1].nodeIndex = 99;
    a[0].brushEdges[0] = {1,kInvalidTriangleEdge};
    connectGaps(ma,0,1,manager,2);
    manager[1].nodeIndex = 1;
    const auto saved = b;
    require(rcsg_recovered_connect_gaps(&ma,0,1,manager,2) == -1,"Bad active gap handle accepted");
    require(std::memcmp(b.data(),saved.data(),sizeof(b)) == 0,"Bad gap input wrote partial reverse links");
}

void visibleExport() {
    TriangulatedBrush mesh;
    mesh.triangles.resize(3);
    mesh.vertices.resize(5);
    for (auto& v : mesh.vertices) v.edge = kInvalidTriangleEdge;
    for (auto& t : mesh.triangles) for (auto& edge : t.twins) edge = kInvalidTriangleEdge;
    mesh.triangles[0].vertices[0] = 0; mesh.triangles[0].vertices[1] = 2; mesh.triangles[0].vertices[2] = 4;
    mesh.triangles[0].surfaceIndex = 7; mesh.triangles[0].categorizationState = 3;
    mesh.triangles[1].categorizationState = mesh.triangles[2].categorizationState = 1;
    const auto output = compactVisibleTriangles(mesh);
    require(output.triangles.size() == 1 && output.vertices.size() == 3,"Visible export was not compacted");
    require(output.triangles[0].surfaceIndex == 7 && output.triangles[0].categorizationState == 3,"Export lost reversed surface provenance");
    require(mesh.triangles.size() == 3 && mesh.vertices.size() == 5 && mesh.triangles[1].surfaceIndex == 0,"Export mutated input");
    for (const auto& link : output.triangles[0].brushEdges)
        require(link.brushIndex == -1 && link.edge == kInvalidTriangleEdge,"Stale cross-brush link retained");
    mesh.triangles[0].categorizationState = 1;
    const auto empty = compactVisibleTriangles(mesh);
    require(empty.triangles.empty() && empty.vertices.empty(),"Empty export retains old vertices");
    require(compactVisibleTriangles({}).vertices.empty(),"Empty input failed");
    mesh.vertices.resize(8);
    for (auto& v : mesh.vertices) v.edge = kInvalidTriangleEdge;
    mesh.triangles[0].categorizationState = 2;
    mesh.triangles[0].vertices[0] = 5; mesh.triangles[0].vertices[1] = 6; mesh.triangles[0].vertices[2] = 7;
    const auto moved = compactVisibleTriangles(mesh);
    require(moved.vertices.size() == 3,"Native tail-scan count leaked into dense export");
    for (int v : moved.triangles[0].vertices) require(v >= 0 && v < 3,"Dense export remapping failed");
}
} // namespace

int main() {
    try {
        compaction(); crossBrushLinks(); visibleExport();
        std::cout << "Recovered mesh cleanup checks passed\n";
    } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
