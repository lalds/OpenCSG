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
            triangle(apex,next,current,2+i/3);
            if (i%3==0) planes.push_back({normal,(normal.x*p.x+normal.y*p.y)+normal.z*p.z});
        }
        std::map<std::pair<int,int>,std::uint32_t> lookup;
        for (int p = 0; p < static_cast<int>(triangles.size()); ++p) for (int c = 0; c < 3; ++c)
            lookup[{triangles[p].vertices[(c+2)%3],triangles[p].vertices[c]}] = p*4+c;
        for (const auto& entry : lookup) {
            const auto found = lookup.find({entry.first.second,entry.first.first});
            if (found != lookup.end()) triangles[entry.second>>2].twins[entry.second&3] = found->second;
        }
    }
    void finish() {
        surfaces.planes = planes.data();
        for (int p = 0; p < static_cast<int>(triangles.size()); ++p) for (int c = 0; c < 3; ++c)
            vertices[triangles[p].vertices[c]].edge = p*4+c;
        const int tc=static_cast<int>(triangles.size()), vc=static_cast<int>(vertices.size());
        triangles.resize(128); vertices.resize(64);
        mesh = {triangles.data(),vertices.data(),nullptr,nullptr,tc,vc,128,64};
    }
    int run(BrushEdgeGroupView* groups = nullptr, BrushEdgeTriangleList* sources = nullptr,
            BrushEdgeConnectionList* connections = nullptr) {
        return rcsg_recovered_remove_two_surface_vertices(&mesh,&surfaces,static_cast<int>(planes.size()),&context,groups,sources,connections,&groupCount);
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
void closedGeometry(const Fixture& f) {
    require(std::abs(volume(f.mesh)-3.f)<1e-5f,"Seam optimization changed volume");
    for (int i=0;i<f.mesh.triangleCount;++i) for (unsigned c=0;c<3;++c) {
        const auto& t=f.triangles[i]; const auto h=t.twins[c];
        require((h&3u)!=3u && h>>2<static_cast<unsigned>(f.mesh.triangleCount),"Seam optimization opened solid");
        require(f.triangles[h>>2].twins[h&3u]==static_cast<unsigned>(i)*4+c,"Seam backlink lost");
        require(t.vertices[c]>=0 && t.vertices[c]<f.mesh.vertexCount,"Seam vertex out of range");
    }
}
void seamAndBothPasses() {
    Fixture f; f.grid(); f.finish();
    require(f.run()==0,"Second pass failed");
    require(f.mesh.triangleCount==14 && f.mesh.vertexCount==9,"Second pass did not remove eight seam vertices");
    closedGeometry(f);
    Fixture both; both.grid(); both.finish();
    require(rcsg_recovered_remove_plane_vertices(&both.mesh,&both.surfaces,static_cast<int>(both.planes.size()),
        &both.context,nullptr,nullptr,nullptr,&both.groupCount)==0,"First pass failed");
    require(both.run()==0 && both.mesh.triangleCount==6 && both.mesh.vertexCount==5,"Two-pass pyramid should have five vertices and six triangles");
    closedGeometry(both);
    Fixture combined; combined.grid(); combined.finish();
    require(rcsg_recovered_optimize_mesh(&combined.mesh,&combined.surfaces,static_cast<int>(combined.planes.size()),
        &combined.context,nullptr,nullptr,nullptr,&combined.groupCount)==0 &&
        combined.mesh.triangleCount==6 && combined.mesh.vertexCount==5,"Combined Optimize API failed");
    closedGeometry(combined);
    Fixture limited; limited.grid(); limited.finish(); limited.mesh.triangleCapacity=limited.mesh.triangleCount;
    const auto oldT=limited.triangles; const auto oldV=limited.vertices; const auto oldMesh=limited.mesh;
    require(rcsg_recovered_optimize_mesh(&limited.mesh,&limited.surfaces,static_cast<int>(limited.planes.size()),
        &limited.context,nullptr,nullptr,nullptr,&limited.groupCount)==-2,"Expected second-pass capacity failure after planar removals");
    require(std::memcmp(&limited.mesh,&oldMesh,sizeof(oldMesh))==0 &&
        std::memcmp(limited.triangles.data(),oldT.data(),oldT.size()*sizeof(TriangleRecord))==0 &&
        std::memcmp(limited.vertices.data(),oldV.data(),oldV.size()*sizeof(TriangleVertexRecord))==0,
        "Failure in second pass retained first-pass mutations");
}
void rollbackAndGroups() {
    Fixture f; f.grid(); f.finish(); f.groupCount=2;
    std::array<std::array<BrushTriangleEdge,8>,2> refs{},links{};
    std::array<std::array<std::uint32_t,8>,2> edges{};
    std::array<BrushEdgeGroupView,2> groups{}; std::array<BrushEdgeTriangleList,2> sources{};
    std::array<BrushEdgeConnectionList,2> connections{};
    for (int i=0;i<2;++i) {
        refs[i][0]={i,static_cast<unsigned>(i)};
        groups[i]={refs[i].data(),1,8,i,i+1,{{1,0,0},{float(i),0,0}}};
        sources[i]={edges[i].data(),0,8}; connections[i]={links[i].data(),0,8};
    }
    for (int i=0;i<f.mesh.triangleCount;++i) for (int c=0;c<3;++c) f.triangles[i].edgeSource[c]=(i+c)%2;
    f.context.polygonVertices.capacity=0;
    const auto oldT=f.triangles; const auto oldV=f.vertices;
    const auto oldGroups=groups; const auto oldRefs=refs; const auto oldContext=f.context;
    require(f.run(groups.data(),sources.data(),connections.data())==-2,"Expected context exhaustion after split/merge");
    require(f.groupCount==2 && std::memcmp(groups.data(),oldGroups.data(),sizeof(groups))==0 &&
        std::memcmp(refs.data(),oldRefs.data(),sizeof(refs))==0 && std::memcmp(&f.context,&oldContext,sizeof(oldContext))==0 &&
        std::memcmp(f.triangles.data(),oldT.data(),oldT.size()*sizeof(TriangleRecord))==0 &&
        std::memcmp(f.vertices.data(),oldV.data(),oldV.size()*sizeof(TriangleVertexRecord))==0,
        "Split/group merge did not roll back on late failure");
    f.context.polygonVertices.capacity=128;
    require(f.run(groups.data(),sources.data(),connections.data())==0 && f.groupCount==1,"Split sources did not merge groups");
    require(groups[0].surfaceRefs==refs[0].data() && groups[1].surfaceRefs==refs[1].data() &&
        groups[1].surfaceRefCount==0 && sources[1].edges==edges[1].data() &&
        connections[1].connections==links[1].data(),"Retired group lost caller ownership");
    closedGeometry(f);
    Fixture shortStorage; shortStorage.grid(); shortStorage.finish();
    shortStorage.mesh.triangleCapacity=shortStorage.mesh.triangleCount;
    const auto untouched=shortStorage.triangles;
    require(shortStorage.run()==-2 && std::memcmp(shortStorage.triangles.data(),untouched.data(),untouched.size()*sizeof(TriangleRecord))==0,
        "Missing temporary triangle storage must fail atomically");
}
void decisionBranches() {
    std::array<TriangleRecord,4> t{}; std::array<TriangleVertexRecord,5> v{};
    std::array<Plane,2> p{{{{0,0,1},0},{{0,0,1},0}}}; SurfaceView s{}; s.planes=p.data();
    for (int n=2;n<=4;++n) {
        v[0]={0,{0,0,0}}; v[1].position={-1,0,0}; v[2].position={1,0,0};
        v[3].position={0,-1,0}; v[4].position={0,1,0};
        for (int i=0;i<n;++i) {
            t[i].vertices[0]=0; t[i].vertices[1]=i+1; t[i].vertices[2]=(i+1)%n+1;
            t[i].twins[0]=((i+1)%n)*4+1; t[i].twins[1]=((i+n-1)%n)*4; t[i].twins[2]=kInvalidTriangleEdge;
            t[i].surfaceIndex=i==0 ? 0:1; t[i].categorizationState=2;
        }
        TriangleMeshView m{t.data(),v.data(),nullptr,nullptr,n,n+1,4,5}; TwoSurfaceVertexPlan plan;
        const int action=rcsg_recovered_plan_two_surface_vertex(&m,&s,2,0,&plan);
        require(action==(n==2 ? 1:2),"Two-triangle removal or single-region Swizzle decision differs");
        if (n>2) require(plan.swizzleEdge==2,"Wrong Swizzle handle selected");
    }
}
}
int main() {
    try { seamAndBothPasses(); rollbackAndGroups(); decisionBranches(); }
    catch (const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
    std::cout<<"Two-surface decisions, full pass, two-pass volume, twins and atomic rollback passed\n";
}
