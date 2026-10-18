#include "realtimecsg/recovered_connections.h"
#include "realtimecsg/recovered_gap_dispatch.h"
#include <array>
#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>
#include <stdexcept>

using namespace realtimecsg::recovered;
namespace {
void require(bool value, const char* text) { if (!value) throw std::runtime_error(text); }
struct Fixture {
    std::array<TriangleRecord,8> triangles{};
    TriangleMeshView mesh{triangles.data(),nullptr,nullptr,nullptr,8,0,8,0};
    std::array<std::array<BrushTriangleEdge,8>,4> refs{};
    std::array<std::array<BrushTriangleEdge,32>,4> output{};
    std::array<std::array<std::uint32_t,2>,4> handles{{{{0,4}},{{8,0}},{{12,0}},{{16,0}}}};
    std::array<BrushEdgeGroupView,4> groups{};
    std::array<BrushEdgeTriangleList,4> sources{};
    std::array<BrushEdgeConnectionList,4> connections{};
    std::array<BrushConnectionNode,2> nodes{};
    Fixture() {
        for (auto& t : triangles) for (auto& source : t.edgeSource) source = 42;
        refs[0][0] = {5,0}; refs[0][1] = {2,1}; refs[0][2] = {5,0}; refs[0][3] = {-1,1};
        refs[1][0] = {2,0}; refs[1][1] = {5,1}; refs[1][2] = {-1,0};
        refs[3][0] = {5,0};
        for (int g = 0; g < 4; ++g) {
            groups[g] = {refs[g].data(),0,8,0,1,{{1,0,0},{0,0,0}}};
            sources[g] = {handles[g].data(),g == 0 ? 2 : 1,2};
            connections[g] = {output[g].data(),0,32};
        }
        groups[0].surfaceRefCount = 4;
        groups[1].surfaceRefCount = 3; groups[1].surfaceA = 2; groups[1].surfaceB = 3;
        groups[1].line = {{-1,0,0},{1,0,0}};
        groups[2].line.origin.y = 1; connections[2].count = 1; output[2][0] = {7,8};
        groups[3].surfaceRefCount = 1; groups[3].line.direction.x = std::numeric_limits<float>::quiet_NaN();
        nodes[0] = {groups.data(),4,0,3}; nodes[1] = {nullptr,0,1,3};
    }
    int run(int own = 0) {
        return rcsg_recovered_find_connections(&mesh,own,groups.data(),sources.data(),connections.data(),4,nodes.data(),2);
    }
};
void nativeLookupAndOrdering() {
    Fixture f;
    const auto before = f.triangles;
    require(f.run() == 0,"FindConnections failed");
    // Golden sequence from the original: sorted referenced IDs -1,2,5,
    // candidate groups in index order, manager lookup by argument own=0.
    const unsigned expected[] = {0,1,0,1,0,1,3};
    require(f.connections[0].count == 7,"Wrong match count/deduplication");
    for (int i = 0; i < 7; ++i) require(f.output[0][i].brushIndex == 0 && f.output[0][i].edge == expected[i],"Native lookup/order changed");
    require(f.triangles[2].edgeSource[0] == -1 && f.triangles[4].edgeSource[0] == -1,"Unconnected source not cleared");
    require(f.triangles[0].edgeSource[0] == 42 && f.triangles[1].edgeSource[0] == 42,"Connected source was cleared");
    require(f.triangles[3].edgeSource[0] == 42 && f.connections[2].count == 1,"Existing connection was discarded");
    require(f.sources[1].count == 1 && f.sources[1].edges[0] == 8,"Source handle list was cleared");
    auto restored = f.triangles;
    restored[2].edgeSource[0] = 42; restored[4].edgeSource[0] = 42;
    require(std::memcmp(before.data(),restored.data(),sizeof(before)) == 0,"Unrelated triangle bytes changed");
    require(f.run() == 0 && f.connections[0].count == 14,"Repeated call did not append native duplicates");
}
void filteredNodesAndGeometry() {
    for (int mode = 0; mode < 4; ++mode) {
        Fixture f;
        if (mode == 0) f.nodes[0].nodeKind = 1;
        if (mode == 1) f.nodes[0].nodeIndex = -1;
        require(f.run(mode == 2 ? -1 : mode == 3 ? 2 : 0) == 0,"Invalid own node not skipped");
        require(f.connections[0].count == 0 && f.triangles[0].edgeSource[0] == -1,"Skipped node still produced connections");
    }
    const float threshold = kGapDirectionAlignment;
    for (int test = 0; test < 3; ++test) {
        Fixture f;
        // All candidate references must match surface pair 0,1, but only
        // candidate 1 is eligible geometrically in this isolated check.
        f.nodes[0].groups = &f.groups[1]; f.nodes[0].groupCount = 1;
        f.groups[1].line.origin = {1,test == 0 ? 0.0003f : test == 1 ? 0.0004f : 0,0};
        f.groups[1].line.direction = {test == 2 ? threshold : -1,0,0};
        require(f.run() == 0,"Geometry filter call failed");
        require(f.connections[0].count == (test == 0 ? 3 : 0),"Proximity or exact direction threshold changed");
    }
}
void errorPaths() {
    Fixture limited; limited.connections[0].capacity = 6;
    require(limited.run() == -2 && limited.connections[0].count == 6,"Connection capacity/partial append changed");
    Fixture invalid; invalid.handles[1][0] = kInvalidTriangleEdge;
    require(invalid.run() == -1,"Invalid unconnected source handle accepted");
    Fixture bad; bad.connections[0].count = -1;
    require(bad.run() == -1,"Invalid output count accepted");
}
}
int main() {
    try { nativeLookupAndOrdering(); filteredNodesAndGeometry(); errorPaths(); }
    catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
    std::cout << "Connection lookup, sorted references, append semantics and orphan sources passed\n";
}
