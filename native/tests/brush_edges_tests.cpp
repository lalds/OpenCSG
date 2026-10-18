#include "realtimecsg/recovered_brush_edges.h"
#include <array>
#include <cmath>
#include <cstring>
#include <iostream>
#include <stdexcept>

using namespace realtimecsg::recovered;
namespace {
void require(bool value, const char* text) { if (!value) throw std::runtime_error(text); }
struct Fixture {
    std::array<PolygonRecord,2> polygons{{{5,0,0,7,8,9},{5,5,1,7,8,9}}};
    std::array<HalfEdgeRecord,10> edges{};
    std::array<EdgeLinkRecord,10> links{};
    std::array<Vector3,5> vertices{{{0,0,0},{1,0,0},{2,0,0},{2,1,0},{0,1,0}}};
    BrushMemoryView mesh{polygons.data(),edges.data(),links.data(),vertices.data(),2,10,5,2,3,5};
    std::array<std::array<BrushTriangleEdge,8>,8> refs{};
    std::array<BrushEdgeGroupView,8> groups{};
    int count = 0;
    BrushSpaceTransform matrix{{1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1}};
    Plane local{{0,0,1},0};
    std::array<Plane,2> foreign{{{{0,-1,0},0},{{0,1,0},1}}};
    BrushEdgeBounds bounds{-3,3,-3,3,-3,3};
    Fixture() {
        const int faces[2][5] = {{0,1,2,3,4},{4,3,2,1,0}};
        for (int p = 0; p < 2; ++p) for (int c = 0; c < 5; ++c) {
            const int e = p*5+c, a = faces[p][(c+4)%5], b = faces[p][c];
            edges[e] = {p*5+(c+1)%5,b,-1,p}; links[e] = {p*5+(c+4)%5,-1};
            for (int q = 0; q < 2; ++q) for (int d = 0; d < 5; ++d)
                if (faces[q][d] == a && faces[q][(d+4)%5] == b) edges[e].twinEdge = q*5+d;
        }
        for (int g = 0; g < 8; ++g) groups[g] = {refs[g].data(),0,8,0,0,{{0,0,0},{0,0,0}}};
    }
    int run(int brush = 4, int surface = 0, int capacity = 8) {
        return rcsg_recovered_find_brush_edges(&mesh,0,&local,foreign.data(),2,&bounds,&matrix,
            brush,surface,groups.data(),&count,capacity);
    }
};
void groupingAndReuse() {
    Fixture f;
    const auto oldEdges = f.edges;
    const auto oldPolygons = f.polygons;
    require(f.run() == 0 && f.count == 1,"Collinear halves were not grouped");
    const auto& g = f.groups[0];
    require(g.surfaceA == 0 && g.surfaceB == 1 && g.line.direction.x == 1 && g.line.direction.y == 0,
            "Surface pair or cross-product direction changed");
    require(g.line.origin.x == 1 && g.line.origin.y == 0 && g.surfaceRefCount == 1,"Wrong first origin/reference");
    for (int e : {1,2}) require(f.links[e].unidentified_04 == 0 && f.links[f.edges[e].twinEdge].unidentified_04 == 0,"Source index not written on both halves");
    require(f.run() == 0 && g.surfaceRefCount == 1,"Repeated surface reference was duplicated");
    require(f.run(7) == 0 && g.surfaceRefCount == 2 && g.surfaceRefs[1].brushIndex == 7,"New brush reference missing");
    require(f.run(4,1) == 0 && f.count == 2 && f.groups[1].line.origin.y == 1,"Parallel separate line reused old group");
    f.links[1].unidentified_04 = -1;
    require(f.run() == 0 && f.links[1].unidentified_04 == 0,"Single existing twin source was ignored");
    f.links[1].unidentified_04 = 1; f.links[f.edges[1].twinEdge].unidentified_04 = 0;
    require(f.run() == 0 && f.links[1].unidentified_04 == 0,"Different existing sources did not search geometry");
    require(std::memcmp(oldEdges.data(),f.edges.data(),sizeof(oldEdges)) == 0
        && std::memcmp(oldPolygons.data(),f.polygons.data(),sizeof(oldPolygons)) == 0,"FindBrushEdges changed topology");
}
void boundsAndPlanes() {
    Fixture near;
    near.foreign[0].distance = std::nextafter(kPlaneDistanceTolerance,0.0f);
    require(near.run() == 0 && near.count == 1,"Near plane endpoint rejected");
    Fixture threshold;
    threshold.foreign[0].distance = kPlaneDistanceTolerance;
    require(threshold.run() == 0 && threshold.count == 0,"Exact plane tolerance accepted");
    Fixture bounds;
    bounds.bounds.minX = 1.5f;
    require(bounds.run() == 0 && bounds.count == 0,"Bounds should require both endpoints inside");
    Fixture clipped;
    clipped.foreign[1] = {{1,0,0},-1};
    require(clipped.run() == 0 && clipped.count == 0,"Edge outside another plane accepted");
    Fixture partial;
    partial.foreign[1] = {{1,0,0},0.5f};
    require(partial.run() == 0 && partial.count == 1 && partial.links[2].unidentified_04 == -1,
        "Both-endpoint outside-plane rejection changed");
    Fixture parallel;
    parallel.local.normal = {0,1,0};
    require(parallel.run() == 0 && parallel.count == 0,"Parallel planes made a line");
}
void transformAndCapacity() {
    Fixture transformed;
    transformed.matrix = {{2,0,0,0,0,3,0,0,0,0,4,0,-5,7,-9,1}};
    require(transformed.run() == 0,"Tree transform failed");
    const auto& line = transformed.groups[0].line;
    require(line.direction.x == 1 && line.direction.y == 0 && line.direction.z == 0,"Transformed direction not normalized");
    require(line.origin.x == -3 && line.origin.y == 7 && line.origin.z == -9,"Tree-space origin incorrect");
    Fixture capacity;
    const auto before = capacity.links;
    require(capacity.run(4,0,0) == -2 && capacity.count == 0,"Group capacity not enforced");
    require(std::memcmp(before.data(),capacity.links.data(),sizeof(before)) == 0,"Capacity failure wrote halfedge sources");
    capacity.groups[0].surfaceRefCapacity = 0;
    require(capacity.run() == -2 && capacity.count == 0,"Reference capacity not enforced");
    Fixture invalid;
    invalid.links[1].unidentified_04 = 100;
    require(invalid.run() == -1,"Bad existing group index accepted");
}
}
int main() {
    try { groupingAndReuse(); boundsAndPlanes(); transformAndCapacity(); }
    catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
    std::cout << "Brush edge grouping, surface references, transforms and boundary checks passed\n";
}
