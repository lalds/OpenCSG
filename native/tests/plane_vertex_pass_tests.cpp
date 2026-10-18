#include "realtimecsg/recovered_mesh_optimization.h"
#include <array>
#include <cmath>
#include <cstring>
#include <iostream>
#include <map>
#include <stdexcept>
#include <vector>

using namespace realtimecsg::recovered;
namespace {
void require(bool condition, const char* text) { if (!condition) throw std::runtime_error(text); }
struct Fixture {
    std::vector<TriangleRecord> triangles;
    std::vector<TriangleVertexRecord> vertices;
    std::array<std::uint32_t,128> boundary{};
    std::array<int,128> removed{},incident{},removedVertices{},polygon{};
    std::array<Vector3,128> normals{};
    std::vector<Plane> planes{{{0,0,1},0},{{0,0,1},0}};
    SurfaceView surfaces{}; TriangleMeshView mesh{};
    PlaneOptimizationContextView context{{boundary.data(),0,128},{removed.data(),0,128},
        {incident.data(),0,128},{removedVertices.data(),0,128},{polygon.data(),0,128},{normals.data(),0,128}};
    int groupCount = 0;
    Fixture() { surfaces.planes = planes.data(); }
    int point(float x, float y) {
        vertices.push_back({kInvalidTriangleEdge,{x,y,0}}); return static_cast<int>(vertices.size())-1;
    }
    int triangle(int a, int b, int c, int surface = 0) {
        TriangleRecord t{}; t.vertices[0] = a; t.vertices[1] = b; t.vertices[2] = c;
        t.surfaceIndex = surface; t.flags = 8; t.categorizationState = 2;
        for (int corner = 0; corner < 3; ++corner) {
            t.twins[corner] = kInvalidTriangleEdge; t.edgeSource[corner] = -1;
            t.cornerNormals[corner] = {0,0,1}; t.brushEdges[corner] = {-1,kInvalidTriangleEdge};
        }
        triangles.push_back(t); return static_cast<int>(triangles.size())-1;
    }
    void grid() {
        for (int y = 0; y < 4; ++y) for (int x = 0; x < 4; ++x) point(float(x),float(y));
        for (int y = 0; y < 3; ++y) for (int x = 0; x < 3; ++x) {
            const int a = y*4+x,b = a+1,c = b+4,d = a+4;
            triangle(a,b,c); triangle(a,c,d);
        }
        const int apex = point(1.5f,1.5f); vertices[apex].position.z = -1;
        const int perimeter[] = {0,1,2,3,7,11,15,14,13,12,8,4};
        for (int i = 0; i < 12; ++i) {
            const int current = perimeter[i], next = perimeter[(i+1)%12];
            const auto a = vertices[next].position,b = vertices[current].position,p = vertices[apex].position;
            const Vector3 u{a.x-p.x,a.y-p.y,a.z-p.z},v{b.x-p.x,b.y-p.y,b.z-p.z};
            Vector3 normal{u.y*v.z-u.z*v.y,u.z*v.x-u.x*v.z,u.x*v.y-u.y*v.x};
            const float length = std::sqrt(normal.x*normal.x+normal.y*normal.y+normal.z*normal.z);
            normal = {normal.x/length,normal.y/length,normal.z/length};
            triangle(apex,next,current,static_cast<int>(planes.size()));
            planes.push_back({normal,(normal.x*p.x+normal.y*p.y)+normal.z*p.z});
        }
        std::map<std::pair<int,int>,std::uint32_t> lookup;
        for (int p = 0; p < static_cast<int>(triangles.size()); ++p) for (int c = 0; c < 3; ++c)
            lookup[{triangles[p].vertices[(c+2)%3],triangles[p].vertices[c]}] = p*4+c;
        for (const auto& entry : lookup) {
            const auto found = lookup.find({entry.first.second,entry.first.first});
            if (found != lookup.end()) triangles[entry.second>>2].twins[entry.second&3] = found->second;
        }
    }
    void fan(int n, float offset) {
        const int center = point(offset,0); const int ring = static_cast<int>(vertices.size());
        for (int i = 0; i < n; ++i) {
            const double a = i*2*3.141592653589793/n;
            point(offset+float(std::cos(a)),float(std::sin(a)));
        }
        const int outside = static_cast<int>(vertices.size());
        for (int i = 0; i < n; ++i) {
            const double a = (i+.5)*2*3.141592653589793/n;
            point(offset+float(3*std::cos(a)),float(3*std::sin(a)));
        }
        const int first = static_cast<int>(triangles.size());
        for (int i = 0; i < n; ++i) triangle(center,ring+i,ring+(i+1)%n);
        for (int i = 0; i < n; ++i) triangle(ring+i,outside+i,ring+(i+1)%n,1);
        for (int i = 0; i < n; ++i) {
            auto& t = triangles[first+i];
            t.twins[0] = (first+(i+1)%n)*4+1; t.twins[1] = (first+(i+n-1)%n)*4;
            t.twins[2] = (first+n+i)*4; triangles[first+n+i].twins[0] = (first+i)*4+2;
        }
    }
    void finish() {
        surfaces.planes = planes.data();
        for (int p = 0; p < static_cast<int>(triangles.size()); ++p) for (int c = 0; c < 3; ++c)
            vertices[triangles[p].vertices[c]].edge = p*4+c;
        mesh = {triangles.data(),vertices.data(),nullptr,nullptr,static_cast<int>(triangles.size()),
            static_cast<int>(vertices.size()),static_cast<int>(triangles.size()),static_cast<int>(vertices.size())};
    }
    int run(BrushEdgeGroupView* groups = nullptr, BrushEdgeTriangleList* sources = nullptr,
            BrushEdgeConnectionList* connections = nullptr) {
        return rcsg_recovered_remove_plane_vertices(&mesh,&surfaces,static_cast<int>(planes.size()),&context,groups,sources,connections,&groupCount);
    }
};
float volume(const TriangleMeshView& mesh) {
    float sum = 0;
    for (int p = 0; p < mesh.triangleCount; ++p) {
        const auto& t = mesh.triangles[p];
        const auto a = mesh.vertices[t.vertices[0]].position,b = mesh.vertices[t.vertices[1]].position,c = mesh.vertices[t.vertices[2]].position;
        sum += (a.x*(b.y*c.z-b.z*c.y)+a.y*(b.z*c.x-b.x*c.z))+a.z*(b.x*c.y-b.y*c.x);
    }
    return sum/6;
}
void sequentialGrid() {
    Fixture f; f.grid(); f.finish();
    f.removed[0] = 999; f.removedVertices[0] = 999;
    f.context.removedTriangles.count = f.context.removedVertices.count = 1;
    require(f.run() == 0,"Full planar pass rejected a grid");
    require(f.mesh.triangleCount == 22 && f.mesh.vertexCount == 13 && f.context.removedVertices.count == 4,
        "Sequential planar removal counts differ");
    require(std::abs(volume(f.mesh)-3.f) < 1e-5f,"Sequential removals changed solid volume");
    for (int p = 0; p < f.mesh.triangleCount; ++p) for (unsigned c = 0; c < 3; ++c) {
        const auto& t = f.triangles[p]; const auto twin = t.twins[c];
        require((twin&3) != 3,"Planar pass opened a closed solid");
        require(t.vertices[c] >= 0 && t.vertices[c] < f.mesh.vertexCount,"Compacted grid has invalid vertex");
        if ((twin&3) != 3) require(twin>>2 < unsigned(f.mesh.triangleCount) &&
            f.triangles[twin>>2].twins[twin&3] == unsigned(p*4)+c,"Compacted grid lost reciprocal twin");
    }
}
void transactionAndOwnership() {
    Fixture f; f.fan(2,0); f.fan(4,10); f.finish(); f.groupCount = 3;
    f.triangles[2].edgeSource[0] = 0; f.triangles[3].edgeSource[0] = 1;
    std::array<std::array<BrushTriangleEdge,8>,3> refs{},links{};
    std::array<std::array<std::uint32_t,8>,3> edges{};
    std::array<BrushEdgeGroupView,3> groups{}; std::array<BrushEdgeTriangleList,3> sources{};
    std::array<BrushEdgeConnectionList,3> connections{};
    for (int i = 0; i < 3; ++i) {
        refs[i][0] = {i,unsigned(i)};
        groups[i] = {refs[i].data(),1,8,i,i+1,{{1,0,0},{float(i),0,0}}};
        sources[i] = {edges[i].data(),0,8}; connections[i] = {links[i].data(),0,8};
    }
    f.context.polygonVertices.capacity = 2;
    const auto oldTriangles = f.triangles; const auto oldVertices = f.vertices;
    const auto oldContext = f.context; const auto oldGroups = groups; const auto oldRefs = refs;
    require(f.run(groups.data(),sources.data(),connections.data()) == -2,"Expected late context exhaustion");
    require(f.groupCount == 3 && std::memcmp(groups.data(),oldGroups.data(),sizeof(groups)) == 0 &&
        std::memcmp(refs.data(),oldRefs.data(),sizeof(refs)) == 0 &&
        std::memcmp(f.triangles.data(),oldTriangles.data(),f.triangles.size()*sizeof(TriangleRecord)) == 0 &&
        std::memcmp(f.vertices.data(),oldVertices.data(),f.vertices.size()*sizeof(TriangleVertexRecord)) == 0 &&
        std::memcmp(&f.context,&oldContext,sizeof(oldContext)) == 0 && f.removed[0] == 0,
        "Late error failed to roll back an earlier group merge");
    f.context.polygonVertices.capacity = 128;
    require(f.run(groups.data(),sources.data(),connections.data()) == 0 && f.groupCount == 2,
        "Combined planar/degenerate pass failed");
    require(groups[0].surfaceRefs == refs[0].data() && groups[0].surfaceRefCount == 2 &&
        groups[1].surfaceA == 2 && groups[1].surfaceRefs == refs[2].data() &&
        sources[1].edges == edges[2].data() && connections[1].connections == links[2].data() &&
        groups[2].surfaceRefs == refs[1].data() && groups[2].surfaceRefCount == 0,
        "Transaction did not restore original group buffer ownership");
    require(f.mesh.triangleCount == 8 && f.mesh.vertexCount == 12,"Combined pass counts differ");
}
void emptyStorage() {
    Fixture f; f.finish(); f.groupCount = 1;
    BrushEdgeGroupView group{}; BrushEdgeTriangleList source{}; BrushEdgeConnectionList connection{};
    f.context.removedTriangles.count = f.context.removedVertices.count = 2;
    require(f.run(&group,&source,&connection) == 0 && f.mesh.triangleCount == 0 && f.mesh.vertexCount == 0 &&
        f.context.removedTriangles.count == 0 && f.context.removedVertices.count == 0 &&
        group.surfaceRefs == nullptr && source.edges == nullptr && connection.connections == nullptr,
        "Empty pass changed buffer ownership or did not reset removals");
}
}
int main() {
    try { sequentialGrid(); transactionAndOwnership(); emptyStorage(); }
    catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
    std::cout << "Full planar pass, volume, sequential removals, transaction rollback and ownership passed\n";
}
