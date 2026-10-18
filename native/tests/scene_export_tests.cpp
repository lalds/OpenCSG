#include "scene_fixture.h"
#include <cmath>
#include <iostream>
#include <stdexcept>
using namespace realtimecsg::recovered;
namespace {
void require(bool value,const char* message) { if (!value) throw std::runtime_error(message); }
void attributesAndReversal() {
    PreparedConvexBrush brush{81,{}, {}};
    brush.mesh.vertices={ {0,{0,0,0}},{0,{2,0,0}},{0,{0,3,0}} };
    brush.mesh.triangles.resize(3);
    for (int p=0;p<3;++p) {
        auto& t=brush.mesh.triangles[p]; t.surfaceIndex=p; t.categorizationState=static_cast<std::uint16_t>(p==2 ? 1:p+2); t.flags=p==1 ? 26:9;
        for (int c=0;c<3;++c) { t.vertices[c]=c; t.cornerNormals[c]={0,0,1}; }
    }
    SceneSurface surfaces[]={{{{0,0,1},0},{2,0,0,.25f},{0,3,0,-.5f},20},
        {{{0,0,1},0},{2,0,0,.25f},{0,3,0,-.5f},10},{{{0,0,1},0},{},{},99}};
    const SceneExportInput input{&brush,surfaces,3}; const auto result=exportPreparedScene(&input,1);
    require(result.vertices.size()==6 && result.indices.size()==6 && result.submeshes.size()==2,"Wrong visible corner count");
    require(result.submeshes[0].materialId==10 && result.submeshes[1].materialId==20,"Material order not stable/sorted");
    require(result.triangles[0].brushIndex==81 && result.triangles[0].surfaceIndex==1 && result.triangles[0].state==3,"Reverse provenance lost");
    require(result.vertices[1].position.y==3 && result.vertices[2].position.x==2,"Reverse winding not emitted");
    for (std::size_t i=0;i<result.vertices.size();++i) {
        const auto& v=result.vertices[i];
        require(v.normal.z==(i<3 ? -1.f:1.f) && v.tangent.x==1 && v.tangent.y==0 && v.tangent.z==0 &&
            v.tangent.w==(i<3 ? -1.f:1.f),"Reverse normal/tangent handedness differs");
        require(v.uv.x==2*v.position.x+.25f && v.uv.y==3*v.position.y-.5f,"UV row projection differs");
    }
    require(result.counts.boundsCenter.x==1 && result.counts.boundsCenter.y==1.5f && result.counts.boundsSize.x==2 && result.counts.boundsSize.y==3,"Scene bounds differ");
    surfaces[0].uvU={-2,0,0,0};
    require(exportPreparedScene(&input,1).vertices[3].tangent.x==-1 && exportPreparedScene(&input,1).vertices[3].tangent.w==-1,"Mirrored UV tangent differs");
    surfaces[0].uvU=surfaces[0].uvV={};
    const auto fallback=exportPreparedScene(&input,1).vertices[3].tangent;
    require(std::isfinite(fallback.x) && std::abs(fallback.x*fallback.x+fallback.y*fallback.y+fallback.z*fallback.z-1)<1e-6,"Degenerate UV tangent invalid");
    for (auto& t:brush.mesh.triangles) t.categorizationState=1;
    const auto empty=exportPreparedScene(&input,1);
    require(empty.vertices.empty() && empty.submeshes.empty() && empty.counts.boundsSize.x==0,"Hidden export not empty");
}
void fullBuild() {
    scene_test::Box a,b({0,-1,-1},{2,1,1},30);
    CategorizationNode nodes[]={{0,2,CSGOperation::Additive,0,0,0},{0,0,CSGOperation::Additive,1,0,0},{0,0,CSGOperation::Additive,1,0,0}};
    const int children[]={1,2}; const SceneBrushInput brushes[]={a.input(1,11),b.input(2,89)};
    SceneBuildInput input{1,0,nodes,3,children,2,brushes,2,128,2048,4096,512};
    const double volumes[]={12,4,4};
    for (int operation=0;operation<3;++operation) {
        nodes[2].operation=static_cast<CSGOperation>(operation); const auto result=buildConvexScene(input);
        require(std::abs(scene_test::volume(result)-volumes[operation])<1e-5,"Full build Boolean volume differs");
        for (const auto& v:result.vertices) require(std::isfinite(v.uv.x) && std::isfinite(v.tangent.x),"Nonfinite output attributes");
        for (const auto& origin:result.triangles) require(origin.brushIndex==11||origin.brushIndex==89,"Real brush provenance lost");
    }
    scene_test::Box cavity({-.5f,-.5f,-.5f},{.5f,.5f,.5f},30);
    SceneBrushInput cavityBrushes[]={a.input(1,11),cavity.input(2,89)}; input.brushes=cavityBrushes; nodes[2].operation=CSGOperation::Subtractive;
    const auto result=buildConvexScene(input); require(std::abs(scene_test::volume(result)-7)<1e-5,"Cavity volume/winding differs");
    bool reverse=false; for (const auto& t:result.triangles) if (t.brushIndex==89) { reverse=true; require(t.state==3,"Cutter face not reversed"); }
    require(reverse,"Cavity omitted cutter provenance");
}
void roundedSliver() {
    PreparedConvexBrush brush{81,{}, {}};
    brush.mesh.vertices={{0,{-2.9049964f,.6855081f,1.09092247f}},
        {0,{-2.718748f,.9938984f,1.04522753f}},{0,{-2.20990849f,1.83643723f,.9203867f}}};
    TriangleRecord triangle{}; triangle.categorizationState=2;
    for (int c=0;c<3;++c) { triangle.vertices[c]=c; triangle.cornerNormals[c]={0,0,1}; }
    brush.mesh.triangles.push_back(triangle);
    SceneSurface surface{{{0,0,1},0},{1,0,0,0},{0,1,0,0},7};
    SceneExportInput input{&brush,&surface,1};
    require(exportPreparedScene(&input,1).triangles.empty(),"Rounded sliver reached scene output");
    brush.mesh.vertices[2].position.z+=.01f;
    require(exportPreparedScene(&input,1).triangles.size()==1,"Resolvable thin triangle was removed");
    // The area policy must scale with coordinates rather than use fixed world units.
    for (float scale:{.001f,1000.f}) {
        auto scaled=brush;
        for (auto& v:scaled.mesh.vertices) { v.position.x*=scale; v.position.y*=scale; v.position.z*=scale; }
        SceneExportInput test{&scaled,&surface,1};
        require(exportPreparedScene(&test,1).triangles.size()==1,"Area policy changed under uniform scaling");
    }
    brush.mesh.triangles[0].vertices[2]=999;
    bool rejected=false;
    try { exportPreparedScene(&input,1); } catch (const std::invalid_argument&) { rejected=true; }
    require(rejected,"Area filter hid invalid topology");
}
}
int main() {
    try { attributesAndReversal(); fullBuild(); roundedSliver(); }
    catch (const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
    std::cout<<"Scene export attributes, material ranges, reversed winding, UV, tangents, bounds and full Boolean build passed\n";
}
