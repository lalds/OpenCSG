#include "realtimecsg/recovered_gap_neighbors.h"
#include "realtimecsg/recovered_mesh_cleanup.h"

#include <array>
#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>
#include <stdexcept>

using namespace realtimecsg::recovered;
namespace {
void require(bool condition, const char* text) { if (!condition) throw std::runtime_error(text); }
struct Body {
    std::array<TriangleRecord,2> triangles{};
    std::array<TriangleVertexRecord,3> vertices{};
    std::array<Vector3,3> world{{{0,0,0},{2,0,0},{0,1,0}}};
    std::array<Plane,2> planes{{{{1,0,0},0},{{-1,0,0},0}}};
    TriangleMeshView mesh{triangles.data(),vertices.data(),world.data(),nullptr,2,3,2,3};
    Body() {
        const int faces[2][3] = {{1,2,0},{0,2,1}};
        for (int i = 0; i < 2; ++i) {
            auto& t = triangles[i]; t.flags = 8; t.surfaceIndex = i;
            for (int c = 0; c < 3; ++c) {
                t.vertices[c] = faces[i][c]; t.twins[c] = kInvalidTriangleEdge;
                t.brushEdges[c] = {-1,kInvalidTriangleEdge};
                t.cornerNormals[c] = {7,8,9};
            }
            t.twins[0] = static_cast<unsigned>((1-i)*4);
        }
    }
    GapNeighborBrushView view(int i) { return {&mesh,planes.data(),i,2}; }
};
const GapLine line{{1,0,0},{0,0,0}};
const std::uint32_t edge = 0;
void match(Body& local, int foreign, const GapNeighborBrushView* views, int count) {
    matchGapNeighbors(local.mesh,local.planes.data(),2,line,&edge,1,foreign,&edge,1,views,count);
}
void visibilityOrientation() {
    for (int mask = 0; mask < 256; ++mask) {
        Body local,foreign;
        for (int i = 0; i < 2; ++i) {
            local.triangles[i].flags = static_cast<std::uint8_t>(((mask>>(2*i))&3)*8);
            foreign.triangles[i].flags = static_cast<std::uint8_t>(((mask>>(4+2*i))&3)*8);
        }
        GapNeighborBrushView views[] = {local.view(0),foreign.view(1)};
        match(local,1,views,2);
        const bool same = (local.triangles[0].flags&16) == (foreign.triangles[0].flags&16);
        for (int i = 0; i < 2; ++i) {
            const int candidate = same ? 1-i : i;
            const auto& ref = local.triangles[i].brushEdges[0];
            const bool visible = (local.triangles[i].flags&8) && (foreign.triangles[candidate].flags&8);
            require(ref.brushIndex == (visible ? 1 : -1),"Wrong visibility selection");
            require(ref.edge == (visible ? static_cast<unsigned>(candidate*4) : kInvalidTriangleEdge),"Wrong orientation pairing");
        }
    }
}
void arbitration() {
    Body local,first,better;
    first.planes[1].normal = {0.25f,0,0}; first.planes[0].normal = {-0.25f,0,0};
    better.planes[1].normal = {0.75f,0,0}; better.planes[0].normal = {-0.75f,0,0};
    GapNeighborBrushView views[] = {local.view(0),first.view(1),better.view(2)};
    match(local,1,views,3); match(local,2,views,3);
    require(local.triangles[0].brushEdges[0].brushIndex == 2 && local.triangles[1].brushEdges[0].brushIndex == 2,"Greater dot did not win");
    match(local,1,views,3);
    require(local.triangles[0].brushEdges[0].brushIndex == 2,"Worse normal replaced incumbent");
    first.planes = better.planes;
    match(local,1,views,3);
    require(local.triangles[0].brushEdges[0].brushIndex == 2,"Equal dot did not retain incumbent");
    first.planes[1].normal.x = std::numeric_limits<float>::quiet_NaN();
    match(local,1,views,3);
    require(local.triangles[0].brushEdges[0].brushIndex == 2,"NaN comparison replaced incumbent");
    better.planes[1].normal.x = std::numeric_limits<float>::quiet_NaN();
    first.planes[1].normal.x = 1;
    match(local,1,views,3);
    require(local.triangles[0].brushEdges[0].brushIndex == 2,"NaN incumbent was replaced");
    local.triangles[0].brushEdges[0] = {99,kInvalidTriangleEdge};
    match(local,1,views,3);
    require(local.triangles[0].brushEdges[0].brushIndex == 1,"Absent manager normal did not fall back to zero");
    local.triangles[0].brushEdges[0] = {99,kInvalidTriangleEdge};
    first.planes[1].normal.x = -1;
    match(local,1,views,3);
    require(local.triangles[0].brushEdges[0].brushIndex == 99,"Negative dot replaced zero fallback");
    // Sign-bit reversal comes from flags, independent of corner normals.
    first.triangles[1].flags = 24;
    match(local,1,views,3);
    require(local.triangles[0].brushEdges[0].brushIndex == 1,"Reversed source normal was ignored");
}
void endpointsAndValidation() {
    Body local,foreign;
    GapNeighborBrushView views[] = {local.view(0),foreign.view(1)};
    foreign.world[0].x += kGapEndpointTolerance; foreign.world[1].x += kGapEndpointTolerance;
    match(local,1,views,2);
    require(local.triangles[0].brushEdges[0].brushIndex == -1,"Exact tolerance matched");
    foreign.world[0].x = 0; foreign.world[1].x = 2;
    foreign.triangles[1].surfaceIndex = 9;
    const auto before = local.triangles;
    require(rcsg_recovered_match_gap_neighbors(&local.mesh,local.planes.data(),2,&line,&edge,1,1,&edge,1,views,2) == -1,"Bad surface accepted");
    require(std::memcmp(before.data(),local.triangles.data(),sizeof(before)) == 0,"Invalid surface left partial links");
    foreign.triangles[1].surfaceIndex = 1;
    local.triangles[1].twins[0] = kInvalidTriangleEdge;
    // This edge is the twin of the group edge; its own twin is not read by
    // selection. A bad directly accessed twin must still be rejected.
    local.triangles[0].twins[0] = kInvalidTriangleEdge;
    require(rcsg_recovered_match_gap_neighbors(&local.mesh,local.planes.data(),2,&line,&edge,1,1,&edge,1,views,2) == -1,"Bad twin accepted");
}
void reciprocalConnections() {
    Body local,foreign;
    GapNeighborBrushView views[] = {local.view(0),foreign.view(1)};
    const auto before = local.triangles;
    match(local,1,views,2);
    const GapBrushView connectionViews[] = {{&local.mesh,0,2},{&foreign.mesh,1,2}};
    connectGaps(local.mesh,0,2,connectionViews,2);
    require(foreign.triangles[1].brushEdges[0].brushIndex == 0 && foreign.triangles[1].brushEdges[0].edge == 0,"Missing reciprocal first link");
    require(foreign.triangles[0].brushEdges[0].brushIndex == 0 && foreign.triangles[0].brushEdges[0].edge == 4,"Missing reciprocal twin link");
    resetGapNeighbors(local.mesh);
    require(std::memcmp(before.data(),local.triangles.data(),sizeof(before)) == 0,"Reset changed non-link bytes");
}
}
int main() {
    try { visibilityOrientation(); arbitration(); endpointsAndValidation(); reciprocalConnections(); }
    catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
    std::cout << "Gap neighbor visibility, arbitration, validation and reciprocal connections passed\n";
}
