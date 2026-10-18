#include "realtimecsg/recovered_triangulation.h"
#include <array>
#include <cstring>
#include <iostream>
#include <stdexcept>

using namespace realtimecsg::recovered;
namespace {
void require(bool condition, const char* text) { if (!condition) throw std::runtime_error(text); }
struct Fixture {
    std::array<PolygonRecord,3> polygons{{{3,0,0,19,0xa582,3},{3,3,1,20,0x4231,2},{2,6,0,21,0x1234,1}}};
    std::array<HalfEdgeRecord,8> edges{{{1,0,6,0},{2,1,5,0},{0,2,4,0},
        {4,2,7,1},{5,1,2,1},{3,0,1,1},{7,2,0,2},{6,0,3,2}}};
    std::array<EdgeLinkRecord,8> links{{{2,10},{0,11},{1,12},{5,13},{3,14},{4,15},{7,16},{6,17}}};
    std::array<Vector3,4> points{{{0,0,0},{1,0,0},{0,1,0},{7,8,9}}};
    std::array<Plane,2> planes{{{{0,0,2},0},{{0,0,-3},0}}};
    BrushMemoryView brush{polygons.data(),edges.data(),links.data(),points.data(),3,8,4,3,2,4};
    SurfaceView surfaces{};
    std::array<TriangleRecord,8> triangles;
    std::array<TriangleVertexRecord,8> vertices;
    TriangleMeshView mesh{triangles.data(),vertices.data(),nullptr,nullptr,0,0,8,8};
    Fixture() {
        surfaces.planes = planes.data();
        std::memset(triangles.data(),0xa5,sizeof(triangles));
        std::memset(vertices.data(),0x5a,sizeof(vertices));
    }
    int run() { return rcsg_recovered_create_triangle_mesh(&mesh,&brush,&surfaces,2); }
};
void creation() {
    Fixture f; const auto savedLinks = f.links; const auto savedPoints = f.points;
    require(f.run() == 0 && f.mesh.triangleCount == 2 && f.mesh.vertexCount == 4,"Create counts differ");
    require(f.polygons[2].edgeCount == 0 && f.polygons[2].firstEdge == -1 &&
        f.edges[0].twinEdge == 3 && f.edges[3].twinEdge == 0,"Digon was not removed/reconnected");
    const int expectedVertices[2][3] = {{0,1,2},{2,1,0}};
    for (int p = 0; p < 2; ++p) {
        const auto& t = f.triangles[p];
        require(std::memcmp(t.vertices,expectedVertices[p],sizeof(t.vertices)) == 0,"Create fan differs");
        require(t.surfaceIndex == p && t.flags == (p == 0 ? 0x82 : 0x31) &&
            t.categorizationState == (p == 0 ? 3 : 2),"Create modified flags/state");
        for (int c = 0; c < 3; ++c) {
            const auto twin = t.twins[c];
            require((twin&3) < 3 && twin>>2 < 2 &&
                f.triangles[twin>>2].twins[twin&3] == static_cast<unsigned>(p*4+c),"Create twins not reciprocal");
            require(t.edgeSource[c] == 10+p*3+c,"Create boundary source differs");
            require(std::memcmp(&t.cornerNormals[c],&f.planes[p].normal,sizeof(Vector3)) == 0,"Create normal changed");
        }
        const auto* unknown = reinterpret_cast<const unsigned char*>(t.brushEdges);
        for (int b = 0; b < 36; ++b) require(unknown[b] == 0xa5,"Create initialized unrelated fields");
        require(t.unidentified_71 == std::byte{0xa5},"Create changed unknown padding");
    }
    require(f.vertices[3].edge == kInvalidTriangleEdge && f.vertices[3].position.x == 7,
        "Unused input vertex was not retained");
    require(std::memcmp(f.links.data(),savedLinks.data(),sizeof(savedLinks)) == 0 &&
        std::memcmp(f.points.data(),savedPoints.data(),sizeof(savedPoints)) == 0,"Create changed links/positions");
    const auto* spare = reinterpret_cast<const unsigned char*>(&f.triangles[2]);
    for (std::size_t i = 0; i < sizeof(TriangleRecord)*6; ++i) require(spare[i] == 0xa5,"Unused capacity changed");
}
void atomicErrorsAndEmpty() {
    Fixture f;
    const auto p = f.polygons; const auto e = f.edges; const auto t = f.triangles; const auto v = f.vertices;
    auto unchanged = [&]() {
        return f.mesh.triangleCount == 0 && f.mesh.vertexCount == 0 &&
            std::memcmp(p.data(),f.polygons.data(),sizeof(p)) == 0 &&
            std::memcmp(e.data(),f.edges.data(),sizeof(e)) == 0 &&
            std::memcmp(t.data(),f.triangles.data(),sizeof(t)) == 0 &&
            std::memcmp(v.data(),f.vertices.data(),sizeof(v)) == 0;
    };
    f.mesh.triangleCapacity = 1;
    require(f.run() == -2 && unchanged(),"Create capacity failure changed input/output");
    f.mesh.triangleCapacity = 8; f.edges[1].vertexIndex = 4;
    require(f.run() == -1,"Create accepted invalid vertex after degeneracy pass");
    f.edges[1].vertexIndex = e[1].vertexIndex;
    require(unchanged(),"Late invalid input partially changed Create state");
    f.edges[6].twinEdge = -1;
    require(f.run() == -1,"Create accepted invalid removed-loop twin");
    f.edges[6].twinEdge = e[6].twinEdge;
    require(unchanged(),"Bad removed loop changed Create state");
    TriangleMeshView empty{}; BrushMemoryView brush{}; SurfaceView surfaces{};
    require(rcsg_recovered_create_triangle_mesh(&empty,&brush,&surfaces,0) == 0,"Empty Create failed");
}
}
int main() {
    try { creation(); atomicErrorsAndEmpty(); }
    catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
    std::cout << "Create topology, source attributes, unused vertices and atomic errors passed\n";
}
