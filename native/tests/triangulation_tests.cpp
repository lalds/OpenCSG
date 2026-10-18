#include "realtimecsg/recovered_triangulation.h"

#include <array>
#include <cstring>
#include <iostream>
#include <stdexcept>

using namespace realtimecsg::recovered;
namespace {
void require(bool value, const char* message) {
    if (!value) throw std::runtime_error(message);
}
} // namespace

int main() {
    try {
        std::array<TriangleRecord,16> triangles;
        std::array<TriangleVertexRecord,8> vertices;
        std::memset(triangles.data(),0xa5,sizeof(triangles));
        std::memset(vertices.data(),0x5a,sizeof(vertices));
        const int indices[] = {0,1,2,3,4};
        // The first fan triangle is collinear and must remain present, as in
        // the original routine. Geometry cleanup is a later pipeline stage.
        vertices[0].position = {0,0,0}; vertices[1].position = {1,0,0};
        vertices[2].position = {2,0,0}; vertices[3].position = {2,1,0};
        vertices[4].position = {0,1,0};
        const Vector3 normals[] = {{1,2,3},{4,5,6},{7,8,9},{10,11,12},{13,14,15}};
        const std::uint32_t twins[] = {0,5,10,12,17};
        std::array<std::uint32_t,5> output{};
        TriangleMeshView mesh{triangles.data(),vertices.data(),nullptr,nullptr,5,8,16,8};
        require(rcsg_recovered_triangulate_convex(&mesh,indices,normals,twins,output.data(),5,23,0x92,0xaabb) == 3,
                "Convex triangulation failed");
        require(mesh.triangleCount == 8,"Wrong triangle count");
        const std::uint32_t expectedOutput[] = {28,21,22,26,30};
        require(std::memcmp(output.data(),expectedOutput,sizeof(expectedOutput)) == 0,"Wrong polygon boundary handles");
        const int expectedVertices[3][3] = {{0,1,2},{0,2,3},{0,3,4}};
        const std::uint32_t expectedTwins[3][3] = {{25,5,10},{29,20,12},{0,24,17}};
        const std::uint32_t expectedVertexEdges[] = {28,21,25,29,30};
        for (int i = 0; i < 3; ++i) {
            const auto& t = triangles[5+i];
            require(std::memcmp(t.vertices,expectedVertices[i],sizeof(t.vertices)) == 0,"Wrong fan vertices");
            require(std::memcmp(t.twins,expectedTwins[i],sizeof(t.twins)) == 0,"Wrong triangle adjacency");
            require(t.surfaceIndex == 23 && t.flags == 0x92 && t.categorizationState == 0xaabb,"Surface metadata changed");
            require(t.unidentified_71 == std::byte{0xa5},"Unknown padding overwritten");
            const auto* unchangedBytes = reinterpret_cast<const unsigned char*>(&t.brushEdges[0]);
            for (int b = 0; b < 36; ++b) require(unchangedBytes[b] == 0xa5,"Later-stage record field overwritten");
            for (int c = 0; c < 3; ++c) {
                require(t.edgeSource[c] == -1,"Source edge must initially be unassigned");
                require(std::memcmp(&t.cornerNormals[c],&normals[t.vertices[c]],sizeof(Vector3)) == 0,"Corner normal changed");
                const auto twin = t.twins[c];
                require(triangles[twin>>2].twins[twin&3] == static_cast<unsigned>((5+i)*4+c),"Existing neighbor not linked back");
            }
        }
        for (int v = 0; v < 5; ++v) require(vertices[v].edge == expectedVertexEdges[v],"Wrong per-vertex edge handle");
        const auto savedTriangles = triangles;
        const auto savedVertices = vertices;
        const auto savedOutput = output;
        const int savedCount = mesh.triangleCount;
        auto unchanged = [&]() {
            return mesh.triangleCount == savedCount
                && std::memcmp(triangles.data(),savedTriangles.data(),sizeof(triangles)) == 0
                && std::memcmp(vertices.data(),savedVertices.data(),sizeof(vertices)) == 0 && output == savedOutput;
        };
        mesh.triangleCapacity = mesh.triangleCount;
        require(rcsg_recovered_triangulate_convex(&mesh,indices,normals,twins,output.data(),5,0,0,0) == -2,
                "Capacity exhaustion not reported");
        require(unchanged(),"Capacity error partially mutated output");
        mesh.triangleCapacity = 16;
        const int badIndices[] = {0,1,2,3,8};
        require(rcsg_recovered_triangulate_convex(&mesh,badIndices,normals,twins,output.data(),5,0,0,0) == -1,
                "Invalid vertex index accepted");
        require(unchanged(),"Invalid vertex partially mutated output");
        const std::uint32_t badTwins[] = {0,5,10,12,80};
        require(rcsg_recovered_triangulate_convex(&mesh,indices,normals,badTwins,output.data(),5,0,0,0) == -1,
                "Invalid neighbor accepted");
        require(unchanged(),"Invalid neighbor partially mutated output");
        require(rcsg_recovered_triangulate_convex(&mesh,indices,normals,twins,output.data(),2,0,0,0) == -1,
                "Less than three corners accepted");
        require(unchanged(),"Short polygon mutated output");
        std::array<TriangleRecord,8> visibility{};
        const std::uint16_t states[] = {0,1,2,3,4,5,0xffff,0x8000};
        const unsigned expectedFlags[] = {0xe9,0xc0,0xc9,0xda,0xc3,0xe9,0xe9,0xe9};
        for (int i = 0; i < 8; ++i) { visibility[i].flags = 0xff; visibility[i].categorizationState = states[i]; }
        TriangleMeshView flagMesh{visibility.data(),nullptr,nullptr,nullptr,8,0,8,0};
        flagTriangleVisibility(flagMesh);
        for (int i = 0; i < 8; ++i) require(visibility[i].flags == expectedFlags[i],"Signed state visibility flags changed");
        std::cout << "Recovered triangulation checks passed\n";
    } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
