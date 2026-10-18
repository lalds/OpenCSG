#include "realtimecsg/recovered_gap_dispatch.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

using namespace realtimecsg::recovered;
namespace {
void require(bool value, const char* text) { if (!value) throw std::runtime_error(text); }
struct Body {
    std::array<TriangleRecord,64> triangles{};
    std::array<TriangleVertexRecord,64> vertices{};
    std::array<Vector3,64> world{};
    std::array<int,64> lookup{};
    std::array<std::array<std::uint32_t,64>,3> edges{};
    std::array<BrushEdgeTriangleList,3> sources{};
    std::array<GapLine,3> lines{{{{1,0,0},{0,0,0}},{{0,1,0},{0,0,0}},{{0,0,1},{0,0,0}}}};
    std::array<Plane,4> planes{{{{0,0,-1},0},{{0,-1,0},0},{{-1,0,0},0},{{1,1,1},2}}};
    TriangleMeshView mesh{triangles.data(),vertices.data(),world.data(),lookup.data(),4,4,64,64};
    BrushSpaceTransform transform{{1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1}};
    Body() {
        world[0] = {0,0,0}; world[1] = {2,0,0}; world[2] = {0,2,0}; world[3] = {0,0,2};
        for (int g = 0; g < 3; ++g) sources[g] = {edges[g].data(),0,64};
        const int faces[4][3] = {{0,2,1},{0,1,3},{0,3,2},{1,2,3}};
        for (int v = 0; v < 4; ++v) vertices[v] = {0,world[v]};
        for (int p = 0; p < 4; ++p) {
            auto& t = triangles[p]; t.surfaceIndex = p; t.flags = 9; t.categorizationState = 2;
            for (unsigned c = 0; c < 3; ++c) {
                t.vertices[c] = faces[p][c]; t.cornerNormals[c] = {0,0,1};
                t.brushEdges[c] = {99,0x12345678};
                const int a = faces[p][(c+2)%3], b = faces[p][c];
                const int group = std::min(a,b) == 0 ? std::max(a,b)-1 : -1;
                t.edgeSource[c] = group;
                if (group >= 0) sources[group].edges[sources[group].count++] = static_cast<unsigned>(p)*4+c;
                for (int q = 0; q < 4; ++q) for (unsigned d = 0; d < 3; ++d)
                    if (faces[q][d] == a && faces[q][(d+2)%3] == b) t.twins[c] = static_cast<unsigned>(q)*4+d;
            }
        }
    }
    float coordinate(int vertex, int axis) const {
        const auto& p = world[vertex]; return axis == 0 ? p.x : axis == 1 ? p.y : p.z;
    }
    void cut(int axis, float value) {
        const auto& group = sources[axis];
        for (int i = 0; i < group.count; ++i) {
            const auto e = group.edges[i]; const auto& t = triangles[e>>2]; const unsigned c = e&3u;
            if (coordinate(t.vertices[(c+2)%3],axis) < value && value < coordinate(t.vertices[c],axis)) {
                const Vector3 point{axis == 0 ? value : 0,axis == 1 ? value : 0,axis == 2 ? value : 0};
                splitTriangleEdge(mesh,transform,sources.data(),3,e,point,0.5f); return;
            }
        }
        throw std::runtime_error("Fixture cut edge missing");
    }
    GapDispatchNode node(int index) {
        return {{&mesh,planes.data(),index,4},sources.data(),lines.data(),3,3,1};
    }
    void validate() const {
        for (int p = 0; p < mesh.triangleCount; ++p) for (unsigned c = 0; c < 3; ++c) {
            const auto& t = triangles[p]; const auto twin = t.twins[c];
            require((twin&3u) < 3 && twin>>2 < static_cast<unsigned>(mesh.triangleCount),"Missing dispatcher twin");
            const auto& other = triangles[twin>>2]; const unsigned d = twin&3u;
            require(other.twins[d] == static_cast<unsigned>(p)*4+c,"Dispatcher broke reciprocal twins");
            require(t.vertices[c] == other.vertices[(d+2)%3] && t.vertices[(c+2)%3] == other.vertices[d],"Dispatcher changed twin endpoints");
        }
    }
};
struct Dispatch {
    Body local,foreign;
    std::array<BrushTriangleEdge,3> refs{{{1,0},{1,1},{1,2}}};
    std::array<GapConnectionList,3> connections{{{&refs[0],1},{&refs[1],1},{&refs[2],1}}};
    std::array<GapDispatchNode,2> nodes{{local.node(0),foreign.node(1)}};
    int run(int budget = 64) {
        return rcsg_recovered_fix_gaps(&local.mesh,local.planes.data(),4,&local.transform,local.sources.data(),3,
            local.lines.data(),connections.data(),nodes.data(),2,budget);
    }
};
void severalGroups() {
    Dispatch d;
    d.foreign.cut(0,0.5f); d.foreign.cut(1,1.5f); d.foreign.cut(2,1.0f);
    require(d.run() == 3,"Dispatcher did not process three groups");
    require(d.local.mesh.vertexCount == 7 && d.local.mesh.triangleCount == 10,"Wrong dispatcher mesh counts");
    d.local.validate(); d.foreign.validate();
    for (int g = 0; g < 3; ++g) {
        require(d.local.sources[g].count == 4,"Source lists were not updated across groups");
        for (int i = 0; i < d.local.sources[g].count; ++i) {
            const auto e = d.local.sources[g].edges[i];
            const auto& ref = d.local.triangles[e>>2].brushEdges[e&3u];
            require(ref.brushIndex == 1 && ref.edge>>2 < static_cast<unsigned>(d.foreign.mesh.triangleCount),"Group did not get foreign neighbor");
        }
    }
    Dispatch limited;
    limited.foreign.cut(0,0.5f); limited.foreign.cut(1,1.5f);
    require(limited.run(1) == -2,"Dispatcher budget was not shared between groups");
    require(limited.local.mesh.vertexCount == 5 && limited.local.mesh.triangleCount == 6,"Dispatcher partial execution changed");
    limited.local.validate();
}
void directionFilter() {
    const float threshold = kGapDirectionAlignment;
    const float above = std::nextafter(threshold,1.0f);
    const float values[] = {0,threshold,std::nextafter(threshold,0.0f),above,-above,std::numeric_limits<float>::quiet_NaN()};
    for (unsigned i = 0; i < 6; ++i) {
        Dispatch d; d.foreign.cut(0,0.5f);
        d.foreign.lines[0].direction.x = values[i];
        require(d.run() == (i < 3 ? 0 : 1),"Direction filter threshold/unordered behavior differs");
    }
}
void invalidConnections() {
    for (int mode = 0; mode < 8; ++mode) {
        Dispatch d; d.foreign.cut(0,0.5f);
        auto& node = d.nodes[1];
        if (mode == 0) for (auto& r : d.refs) r.brushIndex = -1;
        if (mode == 1) for (auto& r : d.refs) r.brushIndex = 5;
        if (mode == 2) { node.nodeKind = 1; node.sources = nullptr; node.lines = nullptr; }
        if (mode == 3) node.brush.nodeIndex = -1;
        if (mode == 4) node.brush.mesh = nullptr;
        if (mode == 5) node.sourceRecordValid = 0;
        if (mode == 6) node.brush.planeCount = 0;
        if (mode == 7) { for (auto& c : d.connections) c.count = 0; }
        require(d.run() == 0,"Inactive connection was not skipped");
        for (int p = 0; p < 4; ++p) for (const auto& ref : d.local.triangles[p].brushEdges)
            require(ref.brushIndex == -1 && ref.edge == kInvalidTriangleEdge,"Dispatcher did not reset links");
    }
    Dispatch bad;
    bad.refs[0].edge = 3;
    require(bad.run() == -1,"Active node bad source index was accepted");
}
}
int main() {
    try { severalGroups(); directionFilter(); invalidConnections(); }
    catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
    std::cout << "Gap dispatcher traversal, filters, source updates and budget checks passed\n";
}
