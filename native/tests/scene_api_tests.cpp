#include "scene_fixture.h"
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <atomic>
using namespace realtimecsg::recovered;
namespace {
void require(bool value,const char* message) { if (!value) throw std::runtime_error(message); }
void lifetimeAndBuffers() {
    scene_test::Box box; const CategorizationNode nodes[]={{0,1,CSGOperation::Additive,0,0,0},{0,0,CSGOperation::Additive,1,0,0}};
    const int children[]={1}; const auto brush=box.input(1,41);
    SceneBuildInput input{1,0,nodes,2,children,1,&brush,1,128,2048,4096,512};
    std::uint64_t handle=12345; require(rcsg_recovered_scene_abi_version()==1,"ABI version differs");
    require(rcsg_recovered_scene_build(&input,&handle)==0 && handle!=12345,"Native build failed");
    SceneCounts counts{}; require(rcsg_recovered_scene_counts(handle,&counts)==0 && counts.vertexCount==36 && counts.triangleCount==12 && counts.submeshCount==2,"Native counts differ");
    std::vector<SceneVertex> vertices(static_cast<std::size_t>(counts.vertexCount)); std::vector<int> indices(static_cast<std::size_t>(counts.indexCount),123);
    std::vector<SceneSubmesh> submeshes(static_cast<std::size_t>(counts.submeshCount)); std::vector<SceneTriangleOrigin> triangles(static_cast<std::size_t>(counts.triangleCount));
    SceneOutputBuffers output{vertices.data(),counts.vertexCount,indices.data(),counts.indexCount,submeshes.data(),counts.submeshCount,triangles.data(),counts.triangleCount-1};
    require(rcsg_recovered_scene_copy(handle,&output)==-2 && indices[0]==123 && vertices[0].position.x==0,"Capacity failure wrote partial output");
    require(std::strlen(rcsg_recovered_scene_last_error())>0,"Missing native error diagnostic");
    output.triangleCapacity=counts.triangleCount; auto aliased=output; aliased.indices=reinterpret_cast<int*>(vertices.data());
    require(rcsg_recovered_scene_copy(handle,&aliased)==-1 && indices[0]==123,"Overlapping arrays accepted");
    // Input lifetime ends after Build: later source edits cannot alter snapshot.
    box.vertices[0]={999,999,999};
    require(rcsg_recovered_scene_copy(handle,&output)==0 && vertices[0].position.x<=1,"Snapshot aliases source arrays");
    for (int i=0;i<counts.indexCount;++i) require(indices[i]==i,"Native index copy differs");
    for (const auto& origin:triangles) require(origin.brushIndex==41 && origin.surfaceIndex>=0 && origin.surfaceIndex<6,"Origin copy differs");
    std::atomic<bool> failed{false};
    std::thread reader([&]() { for (int i=0;i<1000;++i) { SceneCounts c{}; const auto status=rcsg_recovered_scene_counts(handle,&c); if (status!=0 && status!=-1) failed=true; } });
    require(rcsg_recovered_scene_release(handle)==0,"Release failed"); reader.join(); require(!failed,"Concurrent release/count failed");
    counts.vertexCount=999; require(rcsg_recovered_scene_counts(handle,&counts)==-1 && counts.vertexCount==999,"Released handle wrote counts");
    require(rcsg_recovered_scene_release(handle)==-1,"Double release accepted");
    input.abiVersion=99; handle=12345;
    require(rcsg_recovered_scene_build(&input,&handle)==-1 && handle==12345,"Unsupported version published a handle");
    require(rcsg_recovered_scene_copy(0,&output)==-1,"Invalid zero handle accepted");
}
void invalidTopologyAndCapacity() {
    scene_test::Box a,b({-.5f,-.75f,-.25f},{1.5f,1.25f,1.75f});
    const CategorizationNode nodes[]={{0,2,CSGOperation::Additive,0,0,0},{0,0,CSGOperation::Additive,1,0,0},{0,0,CSGOperation::Additive,1,0,0}};
    const int children[]={1,2}; SceneBrushInput brushes[]={a.input(1,7),b.input(2,19)};
    SceneBuildInput input{1,0,nodes,3,children,2,brushes,2,128,8,24,6}; std::uint64_t token=919;
    require(rcsg_recovered_scene_build(&input,&token)==-2 && token==919,"Clip workspace exhaustion was not atomic");
    input.maxBrushVertices=2048; input.maxBrushEdges=4096; input.maxBrushPolygons=512;
    a.edges[0].twinIndex=-1;
    require(rcsg_recovered_scene_build(&input,&token)==-1 && token==919,"Bad source topology published a handle");
}
}
int main() {
    try { lifetimeAndBuffers(); invalidTopologyAndCapacity(); }
    catch (const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
    std::cout<<"Scene C ABI layouts, counts/copy, atomic buffers, snapshot lifetime, release and input errors passed\n";
}
