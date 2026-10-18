#include "realtimecsg/recovered_triangulation.h"
#include <array>
#include <cmath>
#include <cstring>
#include <iostream>
#include <stdexcept>

using namespace realtimecsg::recovered;
namespace {
void require(bool condition, const char* text) { if (!condition) throw std::runtime_error(text); }
struct Fixture {
    std::array<TriangleRecord,12> triangles;
    std::array<TriangleVertexRecord,7> vertices;
    std::array<Plane,2> planes{{{{0,0,1},0},{{0,0,1},0}}};
    std::array<int,8> indices{{0,1,2,3,4,5,77,88}}, slots{{6,7,8,9,10,11,77,88}};
    std::array<Vector3,8> normals{};
    std::array<std::uint32_t,8> twins{{0,4,8,12,16,20,77,88}};
    SurfaceView surfaces{};
    TriangleMeshView mesh{triangles.data(),vertices.data(),nullptr,nullptr,12,7,12,7};
    ConcavePolygonView polygon{indices.data(),normals.data(),twins.data(),6,8};
    TriangleIndexPoolView pool{slots.data(),6,8};
    Fixture() {
        surfaces.planes = planes.data();
        std::memset(triangles.data(),0xa5,sizeof(triangles)); std::memset(vertices.data(),0x5a,sizeof(vertices));
        const Vector3 points[] = {{0,0,0},{2,0,0},{2,1,0},{1,1,0},{1,2,0},{0,2,0}};
        for (int i = 0; i < 6; ++i) { vertices[i].position = points[i]; triangles[i].edgeSource[0] = 10+i; }
        for (int i = 0; i < 8; ++i) normals[i] = {float(i),float(i+1),float(i+2)};
    }
    int run() { return rcsg_recovered_triangulate_concave(&mesh,&surfaces,2,&polygon,&pool,1,0x92,7); }
};
void concaveSolidContour() {
    Fixture f; const auto old = f.triangles; const auto vertices = f.vertices; const auto slots = f.slots;
    require(f.run() == 4 && f.polygon.count == 0 && f.pool.count == 2,"Concave counts/pool differ");
    const int expectedVertices[4][3] = {{2,3,0},{4,0,3},{1,2,0},{5,0,4}};
    const std::uint32_t expectedTwins[4][3] = {{42,8,38},{12,46,34},{0,4,32},{16,20,37}};
    const int expectedSources[4][3] = {{-1,12,-1},{13,-1,-1},{10,11,-1},{14,15,-1}};
    float area = 0;
    for (int i = 0; i < 4; ++i) {
        const auto& t = f.triangles[8+i];
        require(std::memcmp(t.vertices,expectedVertices[i],sizeof(t.vertices)) == 0,"Concave ear order differs");
        require(std::memcmp(t.twins,expectedTwins[i],sizeof(t.twins)) == 0,"Concave adjacency differs");
        require(std::memcmp(t.edgeSource,expectedSources[i],sizeof(t.edgeSource)) == 0,"Concave boundary sources differ");
        require(t.surfaceIndex == 1 && t.flags == 0x92 && t.categorizationState == 7,"Concave metadata differs");
        for (int c = 0; c < 3; ++c) {
            const auto neighbor = t.twins[c];
            require(f.triangles[neighbor>>2].twins[neighbor&3] == static_cast<unsigned>((8+i)*4+c),"Concave reverse twin lost");
            require(t.cornerNormals[c].x == t.vertices[c] && t.cornerNormals[c].y == t.vertices[c]+1,
                "Corner normals did not follow vertices");
        }
        const auto a = f.vertices[t.vertices[0]].position, b = f.vertices[t.vertices[1]].position,
                   c = f.vertices[t.vertices[2]].position;
        const float cross = (b.x-a.x)*(c.y-a.y)-(b.y-a.y)*(c.x-a.x);
        require(cross > 0,"Concave winding/degeneracy differs"); area += cross*.5f;
        require(std::memcmp(old[8+i].brushEdges,t.brushEdges,36) == 0 &&
            old[8+i].unidentified_71 == t.unidentified_71,"Concave touched unrelated bytes");
    }
    require(std::abs(area-3.f) < 1e-6f,"Concave contour area differs");
    const std::array<int,8> expectedIndices{{0,2,3,4,4,5,77,88}};
    const std::array<std::uint32_t,8> expectedBoundary{{42,8,38,46,46,20,77,88}};
    require(f.indices == expectedIndices && f.twins == expectedBoundary && f.slots == slots,
        "Stable erasure trailing bytes/pool entries differ");
    require(std::memcmp(&vertices[6],&f.vertices[6],sizeof(TriangleVertexRecord)) == 0,"Unused vertex changed");
    require(f.mesh.triangleCount == 12 && f.mesh.vertexCount == 7,"Concave changed mesh counts");
}
void errorsAndNoop() {
    Fixture f; const auto triangles = f.triangles; const auto vertices = f.vertices;
    const auto indices = f.indices; const auto normals = f.normals; const auto twins = f.twins;
    auto unchanged = [&]() { return f.polygon.count == 6 &&
        std::memcmp(triangles.data(),f.triangles.data(),sizeof(triangles)) == 0 &&
        std::memcmp(vertices.data(),f.vertices.data(),sizeof(vertices)) == 0 && f.indices == indices &&
        std::memcmp(normals.data(),f.normals.data(),sizeof(normals)) == 0 && f.twins == twins; };
    f.pool.count = 3;
    require(f.run() == -2 && f.pool.count == 3 && unchanged(),"Pool exhaustion partially changed state");
    f.pool.count = 6; f.twins[0] = kInvalidTriangleEdge;
    require(f.run() == -1,"Unbounded invalid source handle accepted");
    f.twins[0] = twins[0]; require(unchanged() && f.pool.count == 6,"Bad boundary changed buffers");
    f.slots[5] = f.slots[4]; require(f.run() == -1 && unchanged(),"Duplicate pool slot accepted");
    TriangleMeshView empty{}; SurfaceView surfaces{}; ConcavePolygonView polygon{}; TriangleIndexPoolView pool{};
    require(rcsg_recovered_triangulate_concave(&empty,&surfaces,0,&polygon,&pool,-1,0,0) == 0,
        "Empty concave contour did not return before surface access");
}
void boundedInvalidAndDegenerate() {
    Fixture f; f.polygon.count = 3;
    f.twins[0] = 3; f.triangles[0].brushEdges[0].brushIndex = 9876;
    require(f.run() == 1 && f.triangles[11].edgeSource[0] == 9876 && f.triangles[11].twins[0] == 3,
        "Native low-bit-3 source read was not preserved");
    Fixture collapsed;
    for (auto& vertex : collapsed.vertices) vertex.position = {0,0,0};
    require(collapsed.run() == 4 && collapsed.polygon.count == 0,"Collapsed fallback failed to terminate");
}
}
int main() {
    try { concaveSolidContour(); errorsAndNoop(); boundedInvalidAndDegenerate(); }
    catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
    std::cout << "Concave ear order, area, reuse, boundary sources and atomic errors passed\n";
}
