#pragma once
#include "realtimecsg/recovered_scene.h"
#include <array>
namespace scene_test {
using namespace realtimecsg::recovered;
struct Box {
    std::array<Vector3,8> vertices;
    std::array<SceneHalfEdge,24> edges{};
    std::array<ScenePolygon,6> polygons{};
    std::array<SceneSurface,6> surfaces{};
    Box(Vector3 lo={-1,-1,-1},Vector3 hi={1,1,1},int material=10) {
        vertices={{{lo.x,lo.y,lo.z},{hi.x,lo.y,lo.z},{hi.x,hi.y,lo.z},{lo.x,hi.y,lo.z},
                   {lo.x,lo.y,hi.z},{hi.x,lo.y,hi.z},{hi.x,hi.y,hi.z},{lo.x,hi.y,hi.z}}};
        const Plane planes[]={{{0,0,-1},-lo.z},{{0,0,1},hi.z},{{0,-1,0},-lo.y},{{1,0,0},hi.x},{{0,1,0},hi.y},{{-1,0,0},-lo.x}};
        const int faces[6][4]={{3,2,1,0},{4,5,6,7},{0,1,5,4},{1,2,6,5},{2,3,7,6},{3,0,4,7}};
        for (int p=0;p<6;++p) {
            polygons[p]={4*p,4,p};
            const SceneFloat4 u=p==3||p==5 ? SceneFloat4{0,1,0,.25f}:SceneFloat4{1,0,0,.25f};
            const SceneFloat4 v=p<2 ? SceneFloat4{0,1,0,-.5f}:SceneFloat4{0,0,1,-.5f};
            surfaces[p]={planes[p],u,v,material+p%2};
            for (int c=0;c<4;++c) {
                edges[4*p+c]={faces[p][c],-1};
                for (int q=0;q<6;++q) for (int d=0;d<4;++d)
                    if (faces[p][c]==faces[q][(d+3)%4] && faces[p][(c+3)%4]==faces[q][d]) edges[4*p+c].twinIndex=4*q+d;
            }
        }
    }
    SceneBrushInput input(int node,int id) const { return {node,id,vertices.data(),8,edges.data(),24,polygons.data(),6,surfaces.data(),6}; }
};
inline double volume(const ExportedScene& scene) {
    double value=0;
    for (std::size_t i=0;i<scene.indices.size();i+=3) {
        const auto a=scene.vertices[scene.indices[i]].position,b=scene.vertices[scene.indices[i+1]].position,c=scene.vertices[scene.indices[i+2]].position;
        value+=double(a.x)*(double(b.y)*c.z-double(b.z)*c.y)+double(a.y)*(double(b.z)*c.x-double(b.x)*c.z)+double(a.z)*(double(b.x)*c.y-double(b.y)*c.x);
    }
    return value/6;
}
}
