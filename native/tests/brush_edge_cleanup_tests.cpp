#include "realtimecsg/recovered_brush_edge_cleanup.h"
#include <array>
#include <cstring>
#include <iostream>
#include <stdexcept>

using namespace realtimecsg::recovered;
namespace {
void require(bool value, const char* text) { if (!value) throw std::runtime_error(text); }
struct Fixture {
    std::array<TriangleRecord,2> triangles{};
    TriangleMeshView mesh{triangles.data(),nullptr,nullptr,nullptr,2,0,2,0};
    std::array<std::array<BrushTriangleEdge,2>,4> refs{},links{};
    std::array<std::array<std::uint32_t,16>,4> handles{};
    std::array<BrushEdgeGroupView,4> groups{};
    std::array<BrushEdgeTriangleList,4> sources{};
    std::array<BrushEdgeConnectionList,4> connections{};
    int count = 4;
    Fixture() {
        for (int g = 0; g < 4; ++g) {
            refs[g][0] = {g+10,static_cast<unsigned>(g+20)};
            links[g][0] = {g+30,static_cast<unsigned>(g+40)};
            groups[g] = {refs[g].data(),1,2,g,g+1,{{1,0,0},{static_cast<float>(g),0,0}}};
            sources[g] = {handles[g].data(),0,16};
            connections[g] = {links[g].data(),1,2};
        }
        for (int p = 0; p < 2; ++p) for (unsigned c = 0; c < 3; ++c) {
            triangles[p].twins[c] = static_cast<unsigned>((1-p)*4)+c;
            triangles[p].edgeSource[c] = -1;
            triangles[p].brushEdges[c] = {77,88};
        }
    }
    int run() { return rcsg_recovered_cleanup_brush_edges(&mesh,groups.data(),sources.data(),connections.data(),&count); }
};
void compactionAndAppend() {
    Fixture f;
    f.triangles[0].flags = 8; f.triangles[1].flags = 0;
    f.triangles[0].edgeSource[0] = 3; // Propagate to the other side.
    f.triangles[0].edgeSource[1] = f.triangles[1].edgeSource[1] = 1;
    const auto before = f.triangles;
    f.sources[3].edges[f.sources[3].count++] = 123; // Native retains old entries.
    require(f.run() == 0 && f.count == 2,"Unused source groups not compacted");
    require(f.groups[0].surfaceA == 3 && f.groups[1].surfaceA == 1,"Native swap-tail group order changed");
    require(f.groups[0].surfaceRefs[0].brushIndex == 13 && f.connections[0].connections[0].brushIndex == 33,"Group vectors not moved together");
    require(f.triangles[0].edgeSource[0] == 0 && f.triangles[1].edgeSource[0] == 0,"One-sided source propagation/remapping failed");
    require(f.sources[0].count == 3 && f.sources[0].edges[0] == 123 && f.sources[0].edges[1] == 0 && f.sources[0].edges[2] == 4,"Source handle append/order changed");
    require(f.sources[1].count == 2 && f.sources[1].edges[0] == 1 && f.sources[1].edges[1] == 5,"Wrong second group handles");
    auto restored = f.triangles;
    for (int p = 0; p < 2; ++p) for (int c = 0; c < 3; ++c) restored[p].edgeSource[c] = before[p].edgeSource[c];
    require(std::memcmp(restored.data(),before.data(),sizeof(before)) == 0,"Cleanup changed non-source triangle bytes");
    require(f.run() == 0 && f.sources[0].count == 5 && f.sources[1].count == 4,"Repeated cleanup did not append again");
}
void visibilityAndErrors() {
    Fixture visible;
    visible.triangles[0].flags = visible.triangles[1].flags = 8;
    visible.triangles[0].edgeSource[0] = visible.triangles[1].edgeSource[0] = 2;
    require(visible.run() == 0 && visible.count == 0 && visible.triangles[0].edgeSource[0] == -1,"Visible/visible tag retained");
    Fixture hidden;
    hidden.triangles[0].edgeSource[0] = hidden.triangles[1].edgeSource[0] = 2;
    require(hidden.run() == 0 && hidden.count == 1 && hidden.sources[0].count == 2,"Native hidden/hidden tags lost");
    Fixture invalid;
    invalid.triangles[0].edgeSource[0] = invalid.triangles[1].edgeSource[0] = 10;
    require(invalid.run() == 0 && invalid.count == 0,"Invalid source IDs not normalized");
    Fixture conflict;
    conflict.triangles[0].edgeSource[0] = 1; conflict.triangles[1].edgeSource[0] = 2;
    require(conflict.run() == 0 && conflict.count == 1,"Conflicting groups were not merged");
    require(conflict.groups[0].surfaceRefCount == 2 && conflict.sources[0].count == 2,
        "Merged group lost surface refs or source handles");
    require(conflict.triangles[0].edgeSource[0] == 0 && conflict.triangles[1].edgeSource[0] == 0,
        "Conflicting twin pair not restored after remap");
    Fixture small;
    small.triangles[0].edgeSource[0] = small.triangles[1].edgeSource[0] = 2;
    small.sources[2].capacity = 1;
    const auto old = small.triangles;
    require(small.run() == -2 && small.count == 4,"Source list capacity not preflighted");
    require(std::memcmp(old.data(),small.triangles.data(),sizeof(old)) == 0 && small.groups[0].surfaceA == 0,"Capacity error partially compacted groups");
}
void preparationThroughTriangulation() {
    std::array<PolygonRecord,2> polygons{{{5,0,0,0,0,2},{5,5,1,0,0,1}}};
    std::array<HalfEdgeRecord,10> edges{}; std::array<EdgeLinkRecord,10> edgeLinks{};
    std::array<Vector3,5> vertices{{{0,0,0},{1,0,0},{2,0,0},{2,1,0},{0,1,0}}};
    const int faces[2][5] = {{0,1,2,3,4},{4,3,2,1,0}};
    for (int p = 0; p < 2; ++p) for (int c = 0; c < 5; ++c) {
        const int e = p*5+c, a = faces[p][(c+4)%5], b = faces[p][c];
        edges[e] = {p*5+(c+1)%5,b,-1,p}; edgeLinks[e] = {p*5+(c+4)%5,-1};
        for (int q = 0; q < 2; ++q) for (int d = 0; d < 5; ++d)
            if (faces[q][d] == a && faces[q][(d+4)%5] == b) edges[e].twinEdge = q*5+d;
    }
    BrushMemoryView brush{polygons.data(),edges.data(),edgeLinks.data(),vertices.data(),2,10,5,2,3,5};
    Fixture f; f.count = 0;
    const Plane sourcePlanes[] = {{{0,0,1},0},{{0,0,-1},0}};
    const Plane foreign[] = {{{0,-1,0},0},{{0,1,0},1}};
    const BrushSpaceTransform matrix{{1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1}};
    const BrushEdgeBounds bounds{-3,3,-3,3,-3,3};
    for (int surface = 0; surface < 2; ++surface)
        findBrushEdges(brush,0,sourcePlanes[0],foreign,2,bounds,matrix,7,surface,f.groups.data(),f.count,4);
    require(f.count == 2,"Preparation did not create the two boundary lines");
    SurfaceView surfaces{}; surfaces.planes = sourcePlanes;
    auto triangulated = triangulateConvexBrush(brush,surfaces,2);
    TriangleMeshView mesh{triangulated.triangles.data(),triangulated.vertices.data(),nullptr,nullptr,
        static_cast<int>(triangulated.triangles.size()),static_cast<int>(triangulated.vertices.size()),
        static_cast<int>(triangulated.triangles.size()),static_cast<int>(triangulated.vertices.size())};
    cleanUpBrushEdges(mesh,f.groups.data(),f.sources.data(),f.connections.data(),f.count);
    require(f.count == 2 && f.sources[0].count == 4 && f.sources[1].count == 2,"Polygon groups did not become triangle lists");
    for (int g = 0; g < f.count; ++g) for (int i = 0; i < f.sources[g].count; ++i) {
        const auto e = f.sources[g].edges[i];
        require(mesh.triangles[e>>2].edgeSource[e&3u] == g,"Generated source list disagrees with triangle tag");
        const auto twin = mesh.triangles[e>>2].twins[e&3u];
        require((twin&3u) < 3 && mesh.triangles[twin>>2].twins[twin&3u] == e,"Preparation broke triangle twins");
    }
}
}
int main() {
    try { compactionAndAppend(); visibilityAndErrors(); preparationThroughTriangulation(); }
    catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
    std::cout << "Brush source cleanup, compaction, list append and polygon-to-triangle bridge passed\n";
}
