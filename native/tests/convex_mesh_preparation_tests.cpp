#include "realtimecsg/recovered_convex_csg.h"
#include "realtimecsg/recovered_mesh_cleanup.h"
#include <array>
#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>
#include <stdexcept>
using namespace realtimecsg::recovered;
namespace {
void require(bool value, const char* message) { if (!value) throw std::runtime_error(message); }
struct Box {
    std::array<PolygonRecord,6> polygons{};
    std::array<HalfEdgeRecord,24> edges{};
    std::array<EdgeLinkRecord,24> links{};
    std::array<Vector3,8> vertices{{{-1,-1,-1},{1,-1,-1},{1,1,-1},{-1,1,-1},
                                  {-1,-1,1},{1,-1,1},{1,1,1},{-1,1,1}}};
    std::array<Plane,6> planes{{{{0,0,-1},1},{{0,0,1},1},{{0,-1,0},1},
                              {{1,0,0},1},{{0,1,0},1},{{-1,0,0},1}}};
    BrushMemoryView memory{polygons.data(),edges.data(),links.data(),vertices.data(),6,24,8,6,6,8};
    SurfaceView surfaces{};
    Box() {
        surfaces.planes=planes.data();
        const int faces[6][4]={{3,2,1,0},{4,5,6,7},{0,1,5,4},{1,2,6,5},{2,3,7,6},{3,0,4,7}};
        for (int p=0;p<6;++p) {
            polygons[p]={4,p*4,p,0,0,2};
            for (int c=0;c<4;++c) {
                const int e=p*4+c;
                edges[e]={p*4+(c+1)%4,faces[p][c],-1,p};
                links[e]={p*4+(c+3)%4,777}; // deliberately stale input group ID
            }
        }
        for (int a=0;a<24;++a) for (int b=0;b<24;++b)
            if (edges[a].vertexIndex==edges[links[b].previousEdge].vertexIndex &&
                edges[b].vertexIndex==edges[links[a].previousEdge].vertexIndex) edges[a].twinEdge=b;
    }
    ConvexMeshInput input(int id) const { return {&memory,&surfaces,6,id}; }
    std::vector<std::byte> snapshot() const {
        std::vector<std::byte> bytes;
        auto append=[&](const auto& value) {
            const auto* start=reinterpret_cast<const std::byte*>(&value);
            bytes.insert(bytes.end(),start,start+sizeof(value));
        };
        append(polygons); append(edges); append(links); append(vertices); append(planes); append(memory); append(surfaces);
        return bytes;
    }
};
void closed(const PreparedConvexBrush& prepared) {
    const auto& mesh=prepared.mesh;
    for (std::size_t p=0;p<mesh.triangles.size();++p) for (unsigned c=0;c<3;++c) {
        const auto& t=mesh.triangles[p]; const auto h=t.twins[c];
        require((h&3u)!=3u && h>>2<mesh.triangles.size(),"Prepared cube opened");
        require(mesh.triangles[h>>2].twins[h&3u]==p*4+c,"Prepared cube twin not reciprocal");
        require(t.vertices[c]>=0 && static_cast<std::size_t>(t.vertices[c])<mesh.vertices.size(),"Prepared cube vertex invalid");
    }
}
void isolatedAndOwnership() {
    Box box; const auto before=box.snapshot(); const auto input=box.input(37);
    auto result=prepareConvexMeshes(&input,1);
    require(result.size()==1 && result[0].brushIndex==37 && result[0].mesh.triangles.size()==12 &&
        result[0].mesh.vertices.size()==8 && result[0].edgeGroups.empty(),"Isolated cube preparation differs");
    for (const auto& t:result[0].mesh.triangles) for (int c=0;c<3;++c) {
        require(t.edgeSource[c]==-1,"Stale input group escaped into isolated output");
        require(t.brushEdges[c].brushIndex==-1 && t.brushEdges[c].edge==kInvalidTriangleEdge,"Gap references were not initialized");
    }
    closed(result[0]);
    require(box.snapshot()==before,"Preparation mutated source cube");
    const auto old=result[0].mesh.vertices[0].position;
    box.vertices[0]={100,100,100};
    const auto now=result[0].mesh.vertices[0].position;
    require(old.x==now.x && old.y==now.y && old.z==now.z,"Prepared vertex aliases source memory");
    require(prepareConvexMeshes(nullptr,0).empty(),"Empty input did not return empty output");
}
void groupsAndHidden() {
    Box a,b;
    for (int p=0;p<6;++p) a.polygons[p].categorizationState=b.polygons[p].categorizationState=p==0 ? 2:1;
    const auto beforeA=a.snapshot(),beforeB=b.snapshot();
    const ConvexMeshInput inputs[]={a.input(11),b.input(89)};
    const auto result=prepareConvexMeshes(inputs,2);
    for (std::size_t i=0;i<result.size();++i) {
        closed(result[i]); require(!result[i].edgeGroups.empty(),"Hidden/visible source groups not retained");
        std::vector<int> seen(result[i].mesh.triangles.size()*3,0);
        for (std::size_t g=0;g<result[i].edgeGroups.size();++g) {
            const auto& group=result[i].edgeGroups[g];
            require(!group.surfaceRefs.empty() && !group.triangleEdges.empty(),"Prepared group lost refs/handles");
            for (const auto& ref:group.surfaceRefs)
                require(ref.brushIndex==inputs[1-i].brushIndex && ref.edge<6,"Sparse brush IDs lost");
            for (auto h:group.triangleEdges) {
                require((h&3u)<3 && h>>2<result[i].mesh.triangles.size(),"Stale group handle");
                require(result[i].mesh.triangles[h>>2].edgeSource[h&3u]==static_cast<int>(g),"Group namespace mismatch");
                require(++seen[(h>>2)*3+(h&3u)]==1,"Duplicated group handle");
            }
        }
        const auto visible=compactVisibleTriangles(result[i].mesh);
        require(visible.triangles.size()==2 && visible.vertices.size()==4,"Visible export from preparation differs");
        for (const auto& triangle:visible.triangles) for (int source:triangle.edgeSource)
            require(source==-1,"Visible export retained complete-mesh group IDs");
    }
    require(a.snapshot()==beforeA && b.snapshot()==beforeB,"Multi-brush preparation changed inputs");
    for (auto& p:a.polygons) p.categorizationState=1;
    const auto hiddenInput=a.input(7);
    const auto hidden=prepareConvexMeshes(&hiddenInput,1);
    require(compactVisibleTriangles(hidden[0].mesh).triangles.empty(),"Hidden brush did not export empty");
}
void invalidInputs() {
    Box first,last;
    auto expectFailure=[&](const ConvexMeshInput* inputs,int count) {
        const auto a=first.snapshot(),b=last.snapshot(); bool rejected=false;
        try { (void)prepareConvexMeshes(inputs,count); } catch (const std::invalid_argument&) { rejected=true; }
        require(rejected,"Invalid preparation input accepted");
        require(first.snapshot()==a && last.snapshot()==b,"Failed preparation changed source input");
    };
    ConvexMeshInput inputs[]={first.input(1),last.input(1)}; expectFailure(inputs,2);
    inputs[1].brushIndex=20; last.polygons[0].categorizationState=0; expectFailure(inputs,2);
    last.polygons[0].categorizationState=2; last.planes[0].normal.z=-2; expectFailure(inputs,2);
    last.planes[0].normal.z=-1; last.planes[0].distance=std::numeric_limits<float>::infinity(); expectFailure(inputs,2);
    last.planes[0].distance=1;
    // All metadata passes initial validation. The first brush can finish its
    // local pipeline before the second brush fails while copying its edges.
    last.memory.edges=nullptr; expectFailure(inputs,2);
    expectFailure(nullptr,1); expectFailure(nullptr,-1);
}
}
int main() {
    try { isolatedAndOwnership(); groupsAndHidden(); invalidInputs(); }
    catch (const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
    std::cout<<"Convex preparation, owned results, sparse IDs, groups, hidden export and unchanged inputs passed\n";
}
