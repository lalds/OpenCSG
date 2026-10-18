#include "realtimecsg/recovered_triangle_splitting.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <iostream>
#include <map>
#include <random>
#include <stdexcept>
#include <utility>

using namespace realtimecsg::recovered;
namespace {
void require(bool value, const char* message) { if (!value) throw std::runtime_error(message); }
struct Tetrahedron {
    std::array<TriangleRecord,512> triangles{};
    std::array<TriangleVertexRecord,256> vertices{};
    std::array<Vector3,256> world{};
    std::array<int,256> lookup{};
    std::array<std::array<std::uint32_t,512>,6> edgeStorage{};
    std::array<BrushEdgeTriangleList,6> sources{};
    TriangleMeshView mesh{triangles.data(),vertices.data(),world.data(),lookup.data(),4,4,512,256};
    BrushSpaceTransform matrix{{1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1}};

    Tetrahedron() {
        world[0] = {0,0,0}; world[1] = {2,0,0}; world[2] = {0,2,0}; world[3] = {0,0,2};
        const int faces[4][3] = {{0,2,1},{0,1,3},{0,3,2},{1,2,3}};
        const float n = 1.0f/std::sqrt(3.0f);
        const Vector3 normals[] = {{0,0,-1},{0,-1,0},{-1,0,0},{n,n,n}};
        for (int v = 0; v < 4; ++v) vertices[v] = {kInvalidTriangleEdge,world[v]};
        for (int i = 0; i < 6; ++i) sources[i] = {edgeStorage[i].data(),0,512};
        std::map<std::pair<int,int>,int> physicalEdges;
        for (int p = 0; p < 4; ++p) {
            triangles[p].surfaceIndex = p; triangles[p].flags = 9; triangles[p].categorizationState = 2;
            for (int c = 0; c < 3; ++c) triangles[p].vertices[c] = faces[p][c];
            for (unsigned c = 0; c < 3; ++c) {
                auto& t = triangles[p];
                const int start = t.vertices[(c+2)%3], end = t.vertices[c];
                const auto key = std::minmax(start,end);
                auto found = physicalEdges.find(key);
                const int source = found == physicalEdges.end() ? static_cast<int>(physicalEdges.size()) : found->second;
                physicalEdges[key] = source;
                t.edgeSource[c] = source;
                sources[source].edges[sources[source].count++] = static_cast<unsigned>(p)*4+c;
                t.cornerNormals[c] = normals[p];
                t.brushEdges[c] = {-1,kInvalidTriangleEdge};
                vertices[end].edge = static_cast<unsigned>(p)*4+c;
                for (int q = 0; q < 4; ++q) for (unsigned d = 0; d < 3; ++d)
                    if (faces[q][d] == start && faces[q][(d+2)%3] == end) t.twins[c] = static_cast<unsigned>(q)*4+d;
            }
        }
    }
    void validate(bool checkVertexHandles = false) const {
        for (int p = 0; p < mesh.triangleCount; ++p) for (unsigned c = 0; c < 3; ++c) {
            const auto& t = triangles[p];
            const auto twin = t.twins[c];
            require((twin & 3u) < 3 && twin>>2 < static_cast<unsigned>(mesh.triangleCount),"Missing closed triangle twin");
            const auto& other = triangles[twin>>2];
            require(other.twins[twin & 3u] == (static_cast<unsigned>(p)*4+c),"Asymmetric twin after split");
            require(t.vertices[c] == other.vertices[((twin&3u)+2)%3]
                && t.vertices[(c+2)%3] == other.vertices[twin&3u],"Split changed twin endpoints");
            require(t.surfaceIndex >= 0 && t.surfaceIndex < 4 && t.flags == 9 && t.categorizationState == 2,"Lost surface metadata");
        }
        for (int v = 0; checkVertexHandles && v < mesh.vertexCount; ++v) {
            const auto edge = vertices[v].edge;
            require((edge & 3u) < 3 && edge>>2 < static_cast<unsigned>(mesh.triangleCount),"Vertex edge invalid");
            require(triangles[edge>>2].vertices[edge&3u] == v,"Vertex edge points to another vertex");
        }
        for (int s = 0; s < 6; ++s) for (int i = 0; i < sources[s].count; ++i) {
            const auto edge = sources[s].edges[i];
            require((edge & 3u) < 3 && edge>>2 < static_cast<unsigned>(mesh.triangleCount),"Source triangle edge invalid");
            require(triangles[edge>>2].edgeSource[edge&3u] == s,"Source list did not follow moved edge");
        }
        require(mesh.vertexCount-mesh.triangleCount/2 == 2,"Split changed closed mesh Euler characteristic");
    }
    std::array<double,2> moments() const {
        double volume = 0, area = 0;
        for (int p = 0; p < mesh.triangleCount; ++p) {
            const auto& t = triangles[p];
            const auto& a = vertices[t.vertices[0]].position;
            const auto& b = vertices[t.vertices[1]].position;
            const auto& c = vertices[t.vertices[2]].position;
            const double x = (double(b.y)-a.y)*(double(c.z)-a.z)-(double(b.z)-a.z)*(double(c.y)-a.y);
            const double y = (double(b.z)-a.z)*(double(c.x)-a.x)-(double(b.x)-a.x)*(double(c.z)-a.z);
            const double z = (double(b.x)-a.x)*(double(c.y)-a.y)-(double(b.y)-a.y)*(double(c.x)-a.x);
            volume += (a.x*x+a.y*y+a.z*z)/6;
            area += std::sqrt(x*x+y*y+z*z)/2;
        }
        return {volume,area};
    }
};

void repeatedSplits() {
    Tetrahedron body;
    body.validate(true);
    const auto initial = body.moments();
    require(std::abs(initial[0]-4.0/3) < 1e-7,"Invalid tetrahedron volume fixture");
    std::mt19937 rng(1559);
    for (int i = 0; i < 100; ++i) {
        const auto edge = (rng()%static_cast<unsigned>(body.mesh.triangleCount))*4+rng()%3;
        const auto& t = body.triangles[edge>>2];
        const auto& start = body.world[t.vertices[((edge&3u)+2)%3]];
        const auto& end = body.world[t.vertices[edge&3u]];
        const float weight = float(1+rng()%3)/4;
        const Vector3 point{start.x*weight+end.x*(1-weight),start.y*weight+end.y*(1-weight),start.z*weight+end.z*(1-weight)};
        const int vertex = body.mesh.vertexCount, triangles = body.mesh.triangleCount;
        const auto output = splitTriangleEdge(body.mesh,body.matrix,body.sources.data(),6,edge,point,weight);
        require(output == (static_cast<unsigned>(triangles)*4+(edge&3u)),"Wrong returned second segment");
        require(body.lookup[vertex] == -1,"New world vertex lookup not invalidated");
        require(body.vertices[vertex].edge == edge && body.triangles[edge>>2].vertices[edge&3u] == vertex,
                "New vertex handle does not point to split vertex");
        require(body.mesh.vertexCount == vertex+1 && body.mesh.triangleCount == triangles+2,"Wrong split counts");
        body.validate();
        const auto now = body.moments();
        require(std::abs(now[0]-initial[0]) < 1e-5 && std::abs(now[1]-initial[1]) < 1e-5,"Splitting changed volume or area");
    }
}

void transformAndErrors() {
    Tetrahedron body;
    body.matrix = {{2,0,0,0,0,3,0,0,0,0,4,0,-5,7,-9,1}};
    const Vector3 point{0.5f,0.25f,0};
    const int vertex = body.mesh.vertexCount;
    splitTriangleEdge(body.mesh,body.matrix,body.sources.data(),6,0,point,0.25f);
    require(body.vertices[vertex].position.x == -4 && body.vertices[vertex].position.y == 7.75f
        && body.vertices[vertex].position.z == -9,"World-to-local affine transform changed");
    require(std::memcmp(&body.world[vertex],&point,sizeof(point)) == 0,"World point not retained");

    // Native only initializes the new vertex's handle; an old endpoint's handle
    // can still reference the changed corner. Preserve this until a later pass.
    Tetrahedron handles;
    const int oldEnd = handles.triangles[0].vertices[0];
    handles.vertices[oldEnd].edge = 0;
    splitTriangleEdge(handles.mesh,handles.matrix,handles.sources.data(),6,0,point,0.5f);
    require(handles.vertices[oldEnd].edge == 0 && handles.triangles[0].vertices[0] != oldEnd,
            "Native old endpoint handle behavior changed");

    Tetrahedron untouched;
    const auto triangles = untouched.triangles;
    const auto vertices = untouched.vertices;
    const auto storage = untouched.edgeStorage;
    const auto world = untouched.world;
    const auto lookup = untouched.lookup;
    const auto sourceLists = untouched.sources;
    std::uint32_t output = 0xabcdef01;
    auto call = [&](std::uint32_t edge) { return rcsg_recovered_split_triangle_edge(&untouched.mesh,&untouched.matrix,
            untouched.sources.data(),6,edge,&point,0.5f,&output); };
    auto unchanged = [&]() {
        for (int s = 0; s < 6; ++s) if (untouched.sources[s].count != sourceLists[s].count) return false;
        return std::memcmp(untouched.triangles.data(),triangles.data(),sizeof(triangles)) == 0
            && std::memcmp(untouched.vertices.data(),vertices.data(),sizeof(vertices)) == 0
            && untouched.edgeStorage == storage && std::memcmp(untouched.world.data(),world.data(),sizeof(world)) == 0
            && untouched.lookup == lookup && untouched.mesh.vertexCount == 4 && untouched.mesh.triangleCount == 4
            && output == 0xabcdef01;
    };
    untouched.mesh.triangleCapacity = 5;
    require(call(0) == -2 && unchanged(),"Triangle exhaustion partially mutated output");
    untouched.mesh.triangleCapacity = 512;
    untouched.mesh.vertexCapacity = 5;
    require(call(0) == -2 && unchanged(),"Native vertex slack requirement ignored");
    untouched.mesh.vertexCapacity = 256;
    require(call(kInvalidTriangleEdge) == -1 && unchanged(),"Invalid selected edge accepted");
    const int source = untouched.triangles[0].edgeSource[0];
    const int oldCapacity = untouched.sources[source].capacity;
    untouched.sources[source].capacity = untouched.sources[source].count+1;
    require(call(0) == -2 && unchanged(),"Shared brush edge append capacity not checked twice");
    untouched.sources[source].capacity = oldCapacity;
}

void normalOperations() {
    Tetrahedron body;
    const auto twin = body.triangles[0].twins[0];
    const int b = static_cast<int>(twin>>2);
    const unsigned d = twin&3u, bp = (d+2)%3;
    body.triangles[0].cornerNormals[2] = {1,0,0};
    body.triangles[0].cornerNormals[0] = {0,1,0};
    body.triangles[b].cornerNormals[d] = {0,0,2};
    body.triangles[b].cornerNormals[bp] = {2,0,0};
    const auto output = splitTriangleEdge(body.mesh,body.matrix,body.sources.data(),6,0,{0.5f,0,0},0.25f);
    const auto& first = body.triangles[0].cornerNormals[0];
    require(std::abs(first.x-1.0/std::sqrt(10.0)) < 1e-7 && std::abs(first.y-3.0/std::sqrt(10.0)) < 1e-7
        && first.z == 0,"First-side weighted normalization changed");
    const auto& second = body.triangles[b].cornerNormals[bp];
    require(second.x == 0 && second.y == 0 && second.z == 1,"Native second-side endpoint formula changed");
    const auto& copied = body.triangles[output>>2].cornerNormals[0];
    require(copied.x == 0 && copied.y == 1 && copied.z == 0,"Original endpoint normal was not preserved");
}
} // namespace

int main() {
    try {
        repeatedSplits(); transformAndErrors(); normalOperations();
        std::cout << "Recovered triangle splitting checks passed\n";
    } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
