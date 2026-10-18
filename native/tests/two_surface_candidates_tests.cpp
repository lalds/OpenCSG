#include "realtimecsg/recovered_mesh_optimization.h"
#include <array>
#include <cstring>
#include <iostream>
#include <limits>
#include <stdexcept>
using namespace realtimecsg::recovered;
namespace {
void require(bool value, const char* message) { if (!value) throw std::runtime_error(message); }
struct Fixture {
    std::array<TriangleRecord,4> triangles{};
    std::array<TriangleVertexRecord,5> vertices{};
    TriangleMeshView mesh{triangles.data(),vertices.data(),nullptr,nullptr,4,5,4,5};
    Fixture() {
        vertices[0]={0,{0,0,0}};
        vertices[1]={kInvalidTriangleEdge,{-1,0,0}};
        vertices[2]={kInvalidTriangleEdge,{0,1,1}};
        vertices[3]={kInvalidTriangleEdge,{1,0,0}};
        vertices[4]={kInvalidTriangleEdge,{0,-1,1}};
        for (int i=0;i<4;++i) {
            auto& t=triangles[i]; t.vertices[0]=0; t.vertices[1]=i+1; t.vertices[2]=(i+1)%4+1;
            t.twins[0]=((i+1)%4)*4+1; t.twins[1]=((i+3)%4)*4; t.twins[2]=kInvalidTriangleEdge;
            t.surfaceIndex=i/2; t.categorizationState=2; t.flags=static_cast<unsigned char>(1<<i);
        }
    }
    int run(TwoSurfaceVertexCandidate& out) { return rcsg_recovered_find_two_surface_candidate(&mesh,0,&out); }
};
void regionsAndDirection() {
    Fixture f; TwoSurfaceVertexCandidate out{99,98};
    const auto triangles=f.triangles; const auto vertices=f.vertices;
    require(f.run(out)==1 && out.firstBoundary==4 && out.secondBoundary==12,"Two contiguous regions not found");
    require(std::memcmp(f.triangles.data(),triangles.data(),sizeof(triangles))==0 &&
        std::memcmp(f.vertices.data(),vertices.data(),sizeof(vertices))==0,"Candidate lookup mutated mesh");
    // Surface ID alone is insufficient: state separates two regions too.
    for (int i=0;i<4;++i) { f.triangles[i].surfaceIndex=0; f.triangles[i].categorizationState=i/2; }
    require(f.run(out)==1,"Distinct states on the same surface were ignored");
    for (auto& t:f.triangles) t.categorizationState=0;
    out={99,98}; require(f.run(out)==0 && out.firstBoundary==99 && out.secondBoundary==98,"Uniform fan changed output");
    for (int i=0;i<4;++i) f.triangles[i].surfaceIndex=i%2;
    require(f.run(out)==0,"Disconnected alternating regions accepted");
    Fixture bent; bent.vertices[3].position.y=.1f;
    require(bent.run(out)==0,"Bent seam accepted");
}
void degenerateDirection() {
    Fixture f; TwoSurfaceVertexCandidate out{99,98};
    // Native normalizes X-center and X-Y; a coincident X-center rejects,
    // but the same geometry approached from the opposite region can pass.
    f.vertices[3].position={0,0,0};
    require(f.run(out)==0,"Zero direction not rejected");
    f.vertices[0].edge=8;
    require(f.run(out)==1 && out.firstBoundary==12 && out.secondBoundary==4,"Native degenerate direction asymmetry lost");
    f.vertices[1].position.x=std::numeric_limits<float>::quiet_NaN();
    out={99,98}; require(f.run(out)==0 && out.firstBoundary==99,"NaN accepted or rejection changed output");
}
void invalidAndOpenFans() {
    Fixture f; TwoSurfaceVertexCandidate out{99,98};
    f.triangles[0].twins[0]=kInvalidTriangleEdge;
    require(f.run(out)==0,"Open first region accepted");
    f.vertices[0].edge=kInvalidTriangleEdge;
    require(f.run(out)==0,"Invalid start should reject normally");
    f.vertices[0].edge=0;
    f.triangles[0].twins[0]=1; // previous(twin) returns this same start.
    f.triangles[0].surfaceIndex=0;
    require(f.run(out)==0,"Uniform one-triangle loop should reject normally");
    f.triangles[0].twins[0]=5;
    f.triangles[1].twins[0]=5; // Unclosed cycle cannot reach start or region B.
    require(f.run(out)==-1 && out.firstBoundary==99 && out.secondBoundary==98,"Nonclosing cycle not bounded atomically");
    require(rcsg_recovered_find_two_surface_candidate(&f.mesh,-1,&out)==-1,"Negative vertex accepted");
    require(rcsg_recovered_find_two_surface_candidate(nullptr,0,&out)==-1,"Null mesh accepted");
}
}
int main() {
    try { regionsAndDirection(); degenerateDirection(); invalidAndOpenFans(); std::cout<<"Two-surface candidate checks passed\n"; }
    catch (const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
}
