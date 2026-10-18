#include "realtimecsg/recovered_mesh_optimization.h"
#include <array>
#include <cstring>
#include <iostream>
#include <stdexcept>

using namespace realtimecsg::recovered;
namespace {
void require(bool condition, const char* text) { if (!condition) throw std::runtime_error(text); }
struct Fixture {
    std::array<TriangleRecord,6> triangles;
    std::array<TriangleVertexRecord,4> vertices{};
    std::array<Plane,2> planes{{{{0,0,-1},0},{{0,0,-1},0}}};
    SurfaceView surfaces{};
    TriangleMeshView mesh{triangles.data(),vertices.data(),nullptr,nullptr,6,4,6,4};
    Fixture() {
        surfaces.planes = planes.data(); std::memset(triangles.data(),0xa5,sizeof(triangles));
        vertices[0] = {99,{1,0,0}}; vertices[1] = {98,{0,0,0}};
        vertices[2] = {97,{0,1,0}}; vertices[3] = {96,{1,1,0}};
        const int a[] = {3,0,1}, b[] = {1,2,3};
        std::memcpy(triangles[0].vertices,a,sizeof(a)); std::memcpy(triangles[1].vertices,b,sizeof(b));
        const std::uint32_t at[] = {4,8,13}, bt[] = {0,18,20};
        std::memcpy(triangles[0].twins,at,sizeof(at)); std::memcpy(triangles[1].twins,bt,sizeof(bt));
        triangles[2].twins[0] = 1; triangles[3].twins[1] = 2;
        triangles[4].twins[2] = 5; triangles[5].twins[0] = 6;
        for (int p = 0; p < 2; ++p) {
            triangles[p].surfaceIndex = p; triangles[p].flags = static_cast<std::uint8_t>(0x40+p);
            triangles[p].categorizationState = static_cast<std::uint16_t>(3+p);
            for (int c = 0; c < 3; ++c) {
                triangles[p].edgeSource[c] = 10*p+c;
                triangles[p].cornerNormals[c] = {float(10*p+c),float(c+1),float(c+2)};
            }
        }
    }
    int run() { return rcsg_recovered_swizzle_triangle_edge(&mesh,&surfaces,2,0); }
};
void acceptedFlip() {
    Fixture f; const auto old = f.triangles;
    require(f.run() == 1,"Regular diagonal replacement rejected");
    const int av[] = {2,3,0}, bv[] = {0,1,2};
    const std::uint32_t at[] = {4,20,8}, bt[] = {0,13,18};
    require(std::memcmp(f.triangles[0].vertices,av,sizeof(av)) == 0 &&
        std::memcmp(f.triangles[1].vertices,bv,sizeof(bv)) == 0,"Swizzle vertex order differs");
    require(std::memcmp(f.triangles[0].twins,at,sizeof(at)) == 0 &&
        std::memcmp(f.triangles[1].twins,bt,sizeof(bt)) == 0,"Swizzle twin order differs");
    for (int p = 0; p < 2; ++p) for (unsigned c = 0; c < 3; ++c) {
        const auto twin = f.triangles[p].twins[c];
        require(f.triangles[twin>>2].twins[twin&3] == unsigned(p*4)+c,"Outer twin not linked back");
    }
    require(f.triangles[0].surfaceIndex == 1 && f.triangles[0].flags == old[1].flags &&
        f.triangles[0].categorizationState == old[1].categorizationState,"Native metadata selection differs");
    const Vector3 normals[] = {old[0].cornerNormals[1],old[0].cornerNormals[2],old[1].cornerNormals[1]};
    require(std::memcmp(normals,f.triangles[0].cornerNormals,sizeof(normals)) == 0 &&
        std::memcmp(old[1].cornerNormals,f.triangles[1].cornerNormals,sizeof(normals)) == 0,"Asymmetric normal writes differ");
    require(f.triangles[0].edgeSource[0] == -1 && f.triangles[0].edgeSource[1] == 2 &&
        f.triangles[0].edgeSource[2] == 11 &&
        std::memcmp(old[1].edgeSource,f.triangles[1].edgeSource,sizeof(old[1].edgeSource)) == 0,
        "Asymmetric source writes differ");
    const unsigned handles[] = {4,5,6,1};
    for (int v = 0; v < 4; ++v) require(f.vertices[v].edge == handles[v],"Final vertex handle differs");
    for (int p = 0; p < 2; ++p) {
        require(std::memcmp(old[p].brushEdges,f.triangles[p].brushEdges,36) == 0 &&
            f.triangles[p].unidentified_71 == old[p].unidentified_71,"Swizzle touched unknown/cross-brush fields");
    }
    require(f.mesh.triangleCount == 6 && f.mesh.vertexCount == 4,"Swizzle changed mesh counts");
}
void degeneracyAndRejection() {
    Fixture collapsed;
    for (auto& v : collapsed.vertices) v.position = {0,0,0};
    require(collapsed.run() == 1 && collapsed.triangles[1].surfaceIndex == 0 &&
        collapsed.triangles[1].flags == 0x40,"Collapsed branch must choose A metadata");
    Fixture rejected; rejected.planes[1].normal = {0,0,1};
    const auto before = rejected.triangles; const auto vertices = rejected.vertices;
    auto unchanged = [&]() { return std::memcmp(before.data(),rejected.triangles.data(),sizeof(before)) == 0 &&
        std::memcmp(vertices.data(),rejected.vertices.data(),sizeof(vertices)) == 0; };
    require(rejected.run() == 0 && unchanged(),"Geometric rejection changed buffers");
    rejected.triangles[0].twins[1] = 100;
    require(rejected.run() == -1,"Bad outer twin accepted");
    rejected.triangles[0].twins[1] = before[0].twins[1];
    require(unchanged(),"Invalid view partially mutated buffers");
    require(rcsg_recovered_swizzle_triangle_edge(&rejected.mesh,&rejected.surfaces,2,3) == -1 && unchanged(),
        "Invalid shared edge accepted or changed buffers");
}
}
int main() {
    try { acceptedFlip(); degeneracyAndRejection(); }
    catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
    std::cout << "Swizzle topology, asymmetric metadata, degeneracy and atomic rejection passed\n";
}
