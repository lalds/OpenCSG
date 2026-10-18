#include "realtimecsg/recovered_brush_edge_cleanup.h"
#include <array>
#include <cstring>
#include <iostream>
#include <stdexcept>

using namespace realtimecsg::recovered;
namespace {
void require(bool condition, const char* text) { if (!condition) throw std::runtime_error(text); }
struct Fixture {
    std::array<TriangleRecord,2> triangles{};
    TriangleMeshView mesh{triangles.data(),nullptr,nullptr,nullptr,2,0,2,0};
    std::array<std::array<BrushTriangleEdge,16>,4> refs{},links{};
    std::array<std::array<std::uint32_t,16>,4> edges{};
    std::array<BrushEdgeGroupView,4> groups{};
    std::array<BrushEdgeTriangleList,4> sources{};
    std::array<BrushEdgeConnectionList,4> connections{};
    int count = 4;
    Fixture() {
        for (int g = 0; g < 4; ++g) {
            refs[g][0] = {g+10,static_cast<unsigned>(g)};
            edges[g][0] = static_cast<unsigned>(g*4);
            links[g][0] = {g+20,static_cast<unsigned>(g)};
            groups[g] = {refs[g].data(),1,16,g,g+1,{{1,0,0},{static_cast<float>(g),0,0}}};
            sources[g] = {edges[g].data(),1,16}; connections[g] = {links[g].data(),1,16};
        }
        const int tags[2][3] = {{0,1,2},{3,-1,10}};
        for (int p = 0; p < 2; ++p) for (int c = 0; c < 3; ++c) {
            triangles[p].edgeSource[c] = tags[p][c]; triangles[p].brushEdges[c] = {99,88};
        }
    }
    int run(std::array<int,2>& pair, int& selected) {
        return rcsg_recovered_merge_brush_pair(&mesh,groups.data(),sources.data(),connections.data(),&count,pair.data(),&selected);
    }
};
void nonTailAndTail() {
    Fixture moved;
    const auto before = moved.triangles;
    std::array<int,2> pair{{2,0}}; int selected = -99;
    require(moved.run(pair,selected) == 0 && selected == 0 && moved.count == 3,"Non-tail merge failed");
    require(pair[0] == 0 && pair[1] == 2,"Native pair sorting not reproduced");
    require(moved.groups[2].surfaceA == 3 && moved.groups[0].surfaceA == 0,"Wrong moved/root geometry");
    require(moved.triangles[0].edgeSource[2] == 0 && moved.triangles[1].edgeSource[0] == 2,"Non-tail remap incorrect");
    require(moved.triangles[1].edgeSource[2] == -1,"Out-of-range tag not normalized");
    require(moved.sources[0].count == 1 && moved.sources[0].edges[0] == 0 && moved.connections[0].count == 1,
        "Root source/connection lists were merged unexpectedly");
    auto restored = moved.triangles;
    for (int p = 0; p < 2; ++p) for (int c = 0; c < 3; ++c) restored[p].edgeSource[c] = before[p].edgeSource[c];
    require(std::memcmp(before.data(),restored.data(),sizeof(before)) == 0,"Merge changed non-source triangle bytes");
    Fixture tail;
    pair = {{3,1}};
    require(tail.run(pair,selected) == 0 && selected == 1 && tail.count == 3,"Tail merge failed");
    require(tail.triangles[1].edgeSource[0] == -1 && tail.triangles[0].edgeSource[1] == 1,
        "Native tail deletion must clear the removed tail's other tags");
}
void occurrenceSensitiveRefs() {
    Fixture f;
    f.refs[0][0] = {5,7}; f.refs[0][1] = {5,7}; f.groups[0].surfaceRefCount = 2;
    f.refs[1][0] = {5,7}; f.refs[1][1] = {5,7}; f.refs[1][2] = {5,7};
    f.refs[1][3] = {-1,3}; f.refs[1][4] = {8,9}; f.groups[1].surfaceRefCount = 5;
    std::array<int,2> pair{{0,1}}; int selected = -99;
    require(f.run(pair,selected) == 0 && f.groups[0].surfaceRefCount == 4,"Occurrence-sensitive refs count differs");
    for (int i = 0; i < 3; ++i) require(f.groups[0].surfaceRefs[i].brushIndex == 5 && f.groups[0].surfaceRefs[i].edge == 7,
        "Native duplicate occurrences were removed");
    require(f.groups[0].surfaceRefs[3].brushIndex == 8,"Negative or new reference handling differs");
}
void emptyEqualAndErrors() {
    Fixture f; std::array<int,2> pair{{-1,2}}; int selected = -99;
    const auto before = f.triangles;
    require(f.run(pair,selected) == 0 && selected == 2 && pair[0] == 2 && pair[1] == -1 && f.count == 4,
        "One unassigned input changed groups");
    require(std::memcmp(before.data(),f.triangles.data(),sizeof(before)) == 0,"Early return remapped triangle tags");
    pair = {{1,1}};
    require(f.run(pair,selected) == 0 && selected == 1 && f.count == 4,"Equal pair changed groups");
    pair = {{-1,-1}};
    require(f.run(pair,selected) == 0 && selected == -1 && f.count == 4,"Empty pair not handled");
    pair = {{1,7}}; selected = 123;
    require(f.run(pair,selected) == -1 && selected == 123 && pair[0] == 1 && pair[1] == 7,"Invalid pair changed outputs");
    Fixture small;
    small.groups[0].surfaceRefCapacity = 1; pair = {{0,2}}; selected = 123;
    const auto old = small.triangles;
    require(small.run(pair,selected) == -2 && small.count == 4 && selected == 123,"Merge capacity failure not atomic");
    require(std::memcmp(old.data(),small.triangles.data(),sizeof(old)) == 0 && pair[0] == 0 && pair[1] == 2,
        "Capacity failure partially merged state");
}
void generalArrays() {
    Fixture f;
    int indices[] = {0,0,1,2}; int selected = -99;
    auto run = [&](Fixture& value, int* input, int length) {
        return rcsg_recovered_merge_brush_indices(&value.mesh,value.groups.data(),value.sources.data(),
            value.connections.data(),&value.count,input,length,&selected);
    };
    require(run(f,indices,4) == 0 && selected == 0 && f.count == 2,"General duplicate merge failed");
    const int expected[] = {0,2,1,2};
    require(std::memcmp(indices,expected,sizeof(expected)) == 0,"Unstable duplicate removal/tail bytes differ");
    require(f.groups[0].surfaceRefCount == 3 && f.groups[0].surfaceRefs[0].brushIndex == 10 &&
        f.groups[0].surfaceRefs[1].brushIndex == 12 && f.groups[0].surfaceRefs[2].brushIndex == 11,
        "Duplicate removal changed reference append order");
    // Original writes the remap once per swap, without composing mappings:
    // the second removal overwrites mapping[2], and mapping[3] remains 2.
    require(f.groups[1].surfaceA == 3 && f.triangles[1].edgeSource[0] == 2 &&
        f.triangles[0].edgeSource[0] == 0 && f.triangles[0].edgeSource[1] == 0 &&
        f.triangles[0].edgeSource[2] == 1,"Native non-composed remap differs");
    require(f.sources[0].count == 1 && f.connections[0].count == 1,"Root auxiliary lists changed");

    Fixture tail; int three[] = {3,2,1};
    require(run(tail,three,3) == 0 && selected == 1 && tail.count == 2,"Multiple tail removal failed");
    require(tail.triangles[0].edgeSource[2] == -1 && tail.triangles[1].edgeSource[0] == -1,
        "Multiple tail tags were not cleared");

    Fixture early; const auto unchanged = early.triangles; int equal[] = {2,-1,2};
    require(run(early,equal,3) == 0 && selected == 2 && early.count == 4 &&
        equal[0] == 2 && equal[1] == 2 && equal[2] == -1,"General early return differs");
    require(std::memcmp(unchanged.data(),early.triangles.data(),sizeof(unchanged)) == 0,"Early return changed tags");

    Fixture invalid; int unsafe[] = {0,1,1,2,3}; const auto before = invalid.triangles;
    const auto groups = invalid.groups; const auto refs = invalid.refs;
    selected = 123;
    require(run(invalid,unsafe,5) == -1 && selected == 123 && invalid.count == 4,
        "Removal through destroyed slot was not rejected");
    const int original[] = {0,1,1,2,3};
    require(std::memcmp(unsafe,original,sizeof(original)) == 0 &&
        std::memcmp(before.data(),invalid.triangles.data(),sizeof(before)) == 0 &&
        std::memcmp(groups.data(),invalid.groups.data(),sizeof(groups)) == 0 &&
        std::memcmp(refs.data(),invalid.refs.data(),sizeof(refs)) == 0,
        "Rejected general merge mutated caller buffers");
    require(run(invalid,unsafe,0) == -1 && selected == 123,"Empty array accepted");
}
}
int main() {
    try { nonTailAndTail(); occurrenceSensitiveRefs(); emptyEqualAndErrors(); generalArrays(); }
    catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
    std::cout << "Pair/general merges, duplicate order, tail remap and atomic errors passed\n";
}
