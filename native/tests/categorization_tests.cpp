#include "realtimecsg/recovered_categorization.h"

#include <array>
#include <cstdlib>
#include <iostream>
#include <stdexcept>

using namespace realtimecsg::recovered;

namespace {
void require(bool condition, const char* message) {
    if (!condition) { std::cerr << message << '\n'; std::exit(1); }
}
SurfaceState run(const CategorizationProgram& program, const std::vector<std::uint8_t>& categories) {
    return evaluateCategorization(program.rows.data(), static_cast<int>(program.rows.size()),
                                    program.initialState, categories.data(), static_cast<int>(categories.size()));
}

void splitAndCategorize() {
    // Integration of recovered splitting with recovered category semantics.
    // A double-sided square is cut by B's half-space; A-B keeps the outside half.
    std::array<PolygonRecord, 8> polygons{};
    polygons[0] = {4,0,0,0,0,0};
    polygons[1] = {4,4,1,0,0,0};
    std::array<HalfEdgeRecord, 32> edges{};
    std::array<EdgeLinkRecord, 32> links{};
    std::array<Vector3, 16> points{};
    points[0] = {-1,-1,0}; points[1] = {1,-1,0};
    points[2] = {1,1,0}; points[3] = {-1,1,0};
    const int twins[] = {4,7,6,5,0,3,2,1};
    for (int i = 0; i < 4; ++i) {
        edges[i] = {(i+1)%4,i,twins[i],0};
        links[i] = {(i+3)%4,-1};
        edges[i+4] = {4+(i+1)%4,3-i,twins[i+4],1};
        links[i+4] = {4+(i+3)%4,-1};
    }
    const Plane planes[] = {{{0,0,1},0}, {{0,0,-1},0}};
    SurfaceView surfaces{};
    surfaces.planes = planes;
    BrushMemoryView memory{polygons.data(), edges.data(), links.data(), points.data(),
                            2,8,4,8,8,16};
    int outside = -1;
    require(splitByPlane(memory, surfaces, 0, {{1,0,0},0}, outside) == PolygonSplitResult::Split,
            "Integration split failed");
    CategorizationNode tree[] = {{0,2,CSGOperation::Additive,0,0,0},
        {0,0,CSGOperation::Additive,1,0,0}, {0,0,CSGOperation::Subtractive,1,0,0}};
    const int children[] = {1,2};
    const auto program = compileCategorization(tree, 3, children, 2, 1);
    polygons[0].categorizationState = static_cast<std::uint16_t>(run(program, {0,1,0}));
    polygons[outside].categorizationState = static_cast<std::uint16_t>(run(program, {0,1,3}));
    require(polygons[0].categorizationState == 1 && polygons[outside].categorizationState == 2,
            "A-B did not hide the inside fragment and retain the outside fragment");
    for (int p : {0, outside}) {
        int edge = polygons[p].firstEdge;
        for (int i = 0; i < polygons[p].edgeCount; ++i) {
            const float x = points[edges[edge].vertexIndex].x;
            require(p == 0 ? x <= 0 : x >= 0, "Fragment labels do not match cut geometry");
            edge = edges[edge].nextEdge;
        }
    }
}
}

int main() {
    // Independently extracted packed matrices, not a second implementation of bitwise formulas.
    const std::uint32_t packed[] = {0xe4a04400u, 0xffbb5f1bu, 0xffeef5e4u};
    for (unsigned op = 0; op < 3; ++op)
        for (unsigned a = 0; a < 4; ++a)
            for (unsigned b = 0; b < 4; ++b)
                require(static_cast<unsigned>(combineCategories(static_cast<CSGOperation>(op),
                        static_cast<CategoryIndex>(a), static_cast<CategoryIndex>(b)))
                        == ((packed[op] >> (a*8 + b*2)) & 3), "Operation matrix changed");

    CategorizationNode nodes[] = {{0,2,CSGOperation::Additive,0,0,0},
                                   {0,0,CSGOperation::Additive,1,0,0},
                                   {0,0,CSGOperation::Additive,1,0,0}};
    const int children[] = {1,2};
    auto first = compileCategorization(nodes, 3, children, 2, 1);
    auto second = compileCategorization(nodes, 3, children, 2, 2);
    const int unionFirst[] = {1,1,1,2}, unionSecond[] = {1,2,1,2};
    for (int category = 0; category < 4; ++category) {
        require(static_cast<int>(run(first, {0,1,static_cast<std::uint8_t>(category)})) == unionFirst[category],
                "Union classification/ownership for earlier brush changed");
        require(static_cast<int>(run(second, {0,static_cast<std::uint8_t>(category),1})) == unionSecond[category],
                "Union classification/ownership for later brush changed");
    }

    nodes[2].operation = CSGOperation::Subtractive;
    first = compileCategorization(nodes, 3, children, 2, 1);
    second = compileCategorization(nodes, 3, children, 2, 2);
    const int subtractionSecond[] = {3,1,3,1};
    for (int category = 0; category < 4; ++category) {
        require(static_cast<int>(run(first, {0,1,static_cast<std::uint8_t>(category)})) == unionFirst[category],
                "Subtraction classification for minuend changed");
        require(static_cast<int>(run(second, {0,static_cast<std::uint8_t>(category),1})) == subtractionSecond[category],
                "Subtraction must reverse surviving cutter surfaces");
    }
    nodes[2].operation = CSGOperation::Intersecting;
    first = compileCategorization(nodes, 3, children, 2, 1);
    second = compileCategorization(nodes, 3, children, 2, 2);
    const int intersectionFirst[] = {2,2,2,1}, intersectionSecond[] = {2,1,2,1};
    for (int category = 0; category < 4; ++category) {
        require(static_cast<int>(run(first, {0,1,static_cast<std::uint8_t>(category)})) == intersectionFirst[category],
                "Intersection classification for earlier brush changed");
        require(static_cast<int>(run(second, {0,static_cast<std::uint8_t>(category),1})) == intersectionSecond[category],
                "Intersection classification for later brush changed");
    }

    // A - (B intersect C): a nested group must retain its parentheses.
    CategorizationNode nested[] = {{0,2,CSGOperation::Additive,0,0,0},
        {0,0,CSGOperation::Additive,1,0,0}, {0,0,CSGOperation::Additive,1,0,0},
        {0,0,CSGOperation::Intersecting,1,0,0}, {2,2,CSGOperation::Subtractive,0,0,0}};
    const int nestedChildren[] = {1,4,2,3};
    auto grouped = compileCategorization(nested, 5, nestedChildren, 4, 1);
    require(run(grouped, {0,1,0,3,0}) == SurfaceState::Aligned, "Empty group intersection removed A");
    require(run(grouped, {0,1,0,0,0}) == SurfaceState::Hidden, "Nested subtraction did not remove A");
    grouped = compileCategorization(nested, 5, nestedChildren, 4, 2);
    require(run(grouped, {0,0,1,0,0}) == SurfaceState::ReverseAligned, "Nested cutter orientation lost");

    // Resolved leaf hints eliminate a geometric dependency.
    nodes[2].operation = CSGOperation::Additive;
    std::uint8_t hints[] = {1,1,2};
    first = compileCategorization(nodes, 3, children, 2, 1, hints);
    require(first.rows.empty() && run(first, {0,1,3}) == SurfaceState::Hidden, "Inside hint was not folded");
    hints[2] = 0;
    first = compileCategorization(nodes, 3, children, 2, 1, hints);
    require(first.rows.empty() && run(first, {0,1,0}) == SurfaceState::Aligned, "Outside hint was not folded");

    // Infinite special case observed in the original generator, including non-additive op.
    nodes[2].infinite = 1;
    nodes[2].operation = CSGOperation::Subtractive;
    first = compileCategorization(nodes, 3, children, 2, 1);
    require(run(first, {0,1,3}) == SurfaceState::Hidden, "Native Infinite accumulator override changed");
    nodes[2].infinite = 0;
    nodes[1].operation = CSGOperation::Subtractive;
    nodes[2].operation = CSGOperation::Additive;
    first = compileCategorization(nodes, 3, children, 2, 1);
    require(run(first, {0,1,3}) == SurfaceState::Hidden, "Leading subtractive brush must not start a group");

    // A group containing only cutters has no initial solid.
    nodes[2].operation = CSGOperation::Subtractive;
    first = compileCategorization(nodes, 3, children, 2, 1);
    require(run(first, {0,1,0}) == SurfaceState::Hidden, "All-cutter group must be empty");

    // Empty groups retain their Boolean operation: A intersect empty is empty,
    // whereas A union/subtract empty preserves A. They must not be pruned.
    CategorizationNode emptyGroup[] = {{0,2,CSGOperation::Additive,0,0,0},
        {0,0,CSGOperation::Additive,1,0,0}, {2,0,CSGOperation::Additive,0,0,0}};
    for (int op = 0; op < 3; ++op) {
        emptyGroup[2].operation = static_cast<CSGOperation>(op);
        auto program = compileCategorization(emptyGroup, 3, children, 2, 1);
        require(run(program, {0,1,0}) == (op == 2 ? SurfaceState::Hidden : SurfaceState::Aligned),
                "Empty group Boolean semantics differ");
    }
    emptyGroup[2].childCount = -1;
    bool rejected = false;
    try { compileCategorization(emptyGroup, 3, children, 2, 1); }
    catch (const std::invalid_argument&) { rejected = true; }
    require(rejected, "Negative branch span accepted");
    nodes[1].operation = CSGOperation::Additive;
    const int duplicate[] = {1,1};
    rejected = false;
    try { compileCategorization(nodes, 3, duplicate, 2, 1); }
    catch (const std::invalid_argument&) { rejected = true; }
    require(rejected, "Shared child accepted as tree");
    require(rcsg_recovered_combine_categories(3, 0, 0) == -1, "Invalid public operation accepted");
    splitAndCategorize();
    std::cout << "Recovered categorization checks passed\n";
}
