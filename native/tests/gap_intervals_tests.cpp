#include "realtimecsg/recovered_gap_intervals.h"
#include "realtimecsg/recovered_gap_neighbors.h"
#include "realtimecsg/recovered_mesh_cleanup.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <iostream>
#include <stdexcept>

using namespace realtimecsg::recovered;
namespace {
void require(bool value, const char* message) { if (!value) throw std::runtime_error(message); }

struct Body {
    std::array<TriangleRecord,64> triangles{};
    std::array<TriangleVertexRecord,64> vertices{};
    std::array<Vector3,64> world{};
    std::array<int,64> lookup{};
    std::array<std::uint32_t,64> edges{};
    BrushEdgeTriangleList source{edges.data(),0,64};
    TriangleMeshView mesh{triangles.data(),vertices.data(),world.data(),lookup.data(),4,4,64,64};
    BrushSpaceTransform transform{{1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1}};
    GapLine line{{1,0,0},{0,0,0}};
    Body() {
        world[0] = {0,0,0}; world[1] = {2,0,0}; world[2] = {0,2,0}; world[3] = {0,0,2};
        const int faces[4][3] = {{0,2,1},{0,1,3},{0,3,2},{1,2,3}};
        for (int v = 0; v < 4; ++v) vertices[v] = {kInvalidTriangleEdge,world[v]};
        for (int p = 0; p < 4; ++p) {
            auto& t = triangles[p];
            t.surfaceIndex = p; t.flags = 9; t.categorizationState = 2;
            for (unsigned c = 0; c < 3; ++c) {
                t.vertices[c] = faces[p][c]; t.cornerNormals[c] = {0,0,1};
                t.brushEdges[c] = {-1,kInvalidTriangleEdge};
                const int start = faces[p][(c+2)%3], end = faces[p][c];
                t.edgeSource[c] = std::min(start,end) == 0 && std::max(start,end) == 1 ? 0 : -1;
                if (t.edgeSource[c] == 0) source.edges[source.count++] = static_cast<unsigned>(p)*4+c;
                for (int q = 0; q < 4; ++q) for (unsigned d = 0; d < 3; ++d)
                    if (faces[q][d] == start && faces[q][(d+2)%3] == end) t.twins[c] = static_cast<unsigned>(q)*4+d;
            }
        }
    }
    void validate() const {
        for (int p = 0; p < mesh.triangleCount; ++p) for (unsigned c = 0; c < 3; ++c) {
            const auto& t = triangles[p]; const auto twin = t.twins[c];
            require((twin&3u) < 3 && twin>>2 < static_cast<unsigned>(mesh.triangleCount),"Missing twin after gap split");
            const auto& other = triangles[twin>>2];
            require(other.twins[twin&3u] == static_cast<unsigned>(p)*4+c,"Asymmetric gap twin");
            require(t.vertices[c] == other.vertices[((twin&3u)+2)%3]
                && t.vertices[(c+2)%3] == other.vertices[twin&3u],"Gap twin endpoints changed");
        }
    }
    std::array<double,2> moments() const {
        double volume = 0, area = 0;
        for (int p = 0; p < mesh.triangleCount; ++p) {
            const auto& t = triangles[p];
            const auto& a = vertices[t.vertices[0]].position;
            const auto& b = vertices[t.vertices[1]].position;
            const auto& c = vertices[t.vertices[2]].position;
            const double x = (double(b.y)-a.y)*(double(c.z)-a.z)-(double(b.z)-a.z)*(double(c.y)-a.y);
            const double y = (double(b.z)-a.z)*(double(c.x)-a.x)-(double(b.x)-a.x)*(double(c.z)-a.z);
            const double z = (double(b.x)-a.x)*(double(c.y)-a.y)-(double(b.y)-a.y)*(double(c.x)-a.x);
            volume += (a.x*x+a.y*y+a.z*z)/6;
            area += std::sqrt(x*x+y*y+z*z)/2;
        }
        return {volume,area};
    }
};

void collectorRules() {
    Body body;
    auto intervals = collectGapIntervals(body.mesh,body.line,body.source.edges,body.source.count);
    require(intervals.size() == 1 && intervals[0].lower == 0 && intervals[0].upper == 2,"Twin pair not deduplicated");
    require(intervals[0].lowerVertex == 0 && intervals[0].upperVertex == 1 && intervals[0].reverse == 1,"Interval order/orientation changed");
    const std::uint32_t duplicate[] = {0,0};
    require(collectGapIntervals(body.mesh,body.line,duplicate,2).size() == 2,"Native same-handle duplicates removed");
    body.triangles[0].flags = 0; // Hidden edges are still collected.
    require(collectGapIntervals(body.mesh,body.line,duplicate,1).size() == 1,"Hidden edge rejected");
    body.triangles[0].flags = 0x10;
    require(collectGapIntervals(body.mesh,body.line,duplicate,1)[0].reverse == 0,"Reverse flag not combined with direction");
    body.world[0].y = 0.0003f;
    require(collectGapIntervals(body.mesh,body.line,duplicate,1).size() == 1,"Near-line endpoint rejected");
    body.world[0].y = 0.0004f;
    require(collectGapIntervals(body.mesh,body.line,duplicate,1).empty(),"Off-line endpoint accepted");
    body.world[0] = body.world[1];
    intervals = collectGapIntervals(body.mesh,body.line,duplicate,1);
    require(intervals.size() == 1 && intervals[0].lower == intervals[0].upper,"Native zero-length interval changed");
    std::array<GapInterval,1> output{};
    std::memset(output.data(),0xa5,sizeof(output)); const auto before = output;
    require(rcsg_recovered_collect_gap_intervals(&body.mesh,&body.line,duplicate,2,output.data(),1) == -2,"Small output buffer accepted");
    require(std::memcmp(output.data(),before.data(),sizeof(output)) == 0,"Interval capacity error partially wrote output");
    const std::uint32_t bad[] = {kInvalidTriangleEdge};
    require(rcsg_recovered_collect_gap_intervals(&body.mesh,&body.line,bad,1,output.data(),1) == -1,"Invalid interval edge accepted");
    require(rcsg_recovered_collect_gap_intervals(&body.mesh,&body.line,duplicate,2,nullptr,0) == 2,"Interval size query failed");
}

void alignTjunctions() {
    Body local,foreign;
    const auto moments = local.moments();
    const auto first = splitTriangleEdge(foreign.mesh,foreign.transform,&foreign.source,1,5,{0.5f,0,0},0.75f);
    splitTriangleEdge(foreign.mesh,foreign.transform,&foreign.source,1,first,{1.5f,0,0},1.0f/3);
    const auto foreignTriangles = foreign.triangles;
    const auto foreignVertices = foreign.vertices;
    const auto foreignEdges = foreign.edges;
    require(collectGapIntervals(local.mesh,local.line,local.source.edges,local.source.count).size() == 1,"Wrong initial local segment count");
    require(alignGapIntervals(local.mesh,local.transform,&local.source,1,0,local.line,foreign.mesh,foreign.edges.data(),foreign.source.count) == 2,
            "T-junction endpoints were not split");
    local.validate(); foreign.validate();
    auto a = collectGapIntervals(local.mesh,local.line,local.source.edges,local.source.count);
    auto b = collectGapIntervals(foreign.mesh,foreign.line,foreign.source.edges,foreign.source.count);
    auto order = [](const auto& x,const auto& y) { return x.lower < y.lower; };
    std::sort(a.begin(),a.end(),order); std::sort(b.begin(),b.end(),order);
    require(a.size() == 3 && a.size() == b.size(),"Aligned segment count wrong");
    for (std::size_t i = 0; i < a.size(); ++i)
        require(a[i].lower == b[i].lower && a[i].upper == b[i].upper,"Different subdivisions remain");
    const auto after = local.moments();
    require(std::abs(after[0]-moments[0]) < 1e-7 && std::abs(after[1]-moments[1]) < 1e-7,"Gap alignment changed volume/area");
    require(std::memcmp(foreignTriangles.data(),foreign.triangles.data(),sizeof(foreignTriangles)) == 0
        && std::memcmp(foreignVertices.data(),foreign.vertices.data(),sizeof(foreignVertices)) == 0 && foreignEdges == foreign.edges,
        "Foreign mesh was mutated");
    require(alignGapIntervals(local.mesh,local.transform,&local.source,1,0,local.line,foreign.mesh,foreign.edges.data(),foreign.source.count) == 0,
            "Gap alignment is not idempotent");
    require(alignGapIntervals(foreign.mesh,foreign.transform,&foreign.source,1,0,foreign.line,local.mesh,local.edges.data(),local.source.count) == 0,
            "Reverse direction found another T junction");

    // Exercise the full scoped chain on the three newly aligned segments.
    const Plane planes[] = {{{0,0,-1},0},{{0,-1,0},0},{{-1,0,0},0},{{1,1,1},2}};
    const GapNeighborBrushView neighborViews[] = {{&local.mesh,planes,0,4},{&foreign.mesh,planes,1,4}};
    matchGapNeighbors(local.mesh,planes,4,local.line,local.source.edges,local.source.count,
                      1,foreign.source.edges,foreign.source.count,neighborViews,2);
    const GapBrushView connectionViews[] = {{&local.mesh,0,4},{&foreign.mesh,1,4}};
    connectGaps(local.mesh,0,4,connectionViews,2);
    for (int i = 0; i < local.source.count; ++i) {
        const auto e = local.source.edges[i];
        const auto& t = local.triangles[e>>2]; const unsigned c = e&3u;
        const auto& ref = t.brushEdges[c];
        require(ref.brushIndex == 1 && (ref.edge&3u) < 3 && ref.edge>>2 < static_cast<unsigned>(foreign.mesh.triangleCount),"Aligned segment did not get a neighbor");
        const auto& f = foreign.triangles[ref.edge>>2]; const unsigned d = ref.edge&3u;
        require(f.brushEdges[d].brushIndex == 0 && f.brushEdges[d].edge == e,"Aligned segment has no reciprocal link");
        require(local.world[t.vertices[c]].x == foreign.world[f.vertices[(d+2)%3]].x
            && local.world[t.vertices[(c+2)%3]].x == foreign.world[f.vertices[d]].x,"Connected segments have different endpoints");
    }

    Body transformed;
    transformed.transform = {{2,0,0,0,0,3,0,0,0,0,4,0,-5,7,-9,1}};
    for (int v = 0; v < 4; ++v) {
        const auto& p = transformed.world[v];
        transformed.vertices[v].position = {2*p.x-5,3*p.y+7,4*p.z-9};
    }
    const auto transformedBefore = transformed.moments();
    require(alignGapIntervals(transformed.mesh,transformed.transform,&transformed.source,1,0,transformed.line,
                             foreign.mesh,foreign.edges.data(),foreign.source.count) == 2,
            "Different local coordinate system changed gap subdivision");
    transformed.validate();
    for (int v = 4; v < transformed.mesh.vertexCount; ++v) {
        const auto& p = transformed.world[v]; const auto& q = transformed.vertices[v].position;
        require(q.x == 2*p.x-5 && q.y == 3*p.y+7 && q.z == 4*p.z-9,"Foreign world endpoint was not transformed locally");
    }
    const auto transformedAfter = transformed.moments();
    require(std::abs(transformedBefore[0]-transformedAfter[0]) < 1e-7
        && std::abs(transformedBefore[1]-transformedAfter[1]) < 1e-7,"Transformed gap alignment changed volume/area");

    Body thresholdLocal,thresholdOther;
    splitTriangleEdge(thresholdOther.mesh,thresholdOther.transform,&thresholdOther.source,1,5,{kGapEndpointTolerance,0,0},0.5f);
    require(alignGapIntervals(thresholdLocal.mesh,thresholdLocal.transform,&thresholdLocal.source,1,0,thresholdLocal.line,
                             thresholdOther.mesh,thresholdOther.edges.data(),thresholdOther.source.count) == 0,
            "Endpoint tolerance should be strict");
    Body limited;
    bool rejected = false;
    try { alignGapIntervals(limited.mesh,limited.transform,&limited.source,1,0,limited.line,foreign.mesh,foreign.edges.data(),foreign.source.count,0); }
    catch (const std::length_error&) { rejected = true; }
    require(rejected && limited.mesh.vertexCount == 4 && limited.mesh.triangleCount == 4,"Zero split budget mutated mesh");
    Body partial;
    rejected = false;
    try { alignGapIntervals(partial.mesh,partial.transform,&partial.source,1,0,partial.line,foreign.mesh,foreign.edges.data(),foreign.source.count,1); }
    catch (const std::length_error&) { rejected = true; }
    require(rejected && partial.mesh.vertexCount == 5 && partial.mesh.triangleCount == 6,"Partial split failure contract changed");
    partial.validate();
    Body duplicate;
    duplicate.source.edges[2] = duplicate.source.edges[0]; duplicate.source.count = 3;
    rejected = false;
    try { alignGapIntervals(duplicate.mesh,duplicate.transform,&duplicate.source,1,0,duplicate.line,foreign.mesh,foreign.edges.data(),foreign.source.count); }
    catch (const std::invalid_argument&) { rejected = true; }
    require(rejected && duplicate.mesh.vertexCount == 4 && duplicate.mesh.triangleCount == 4,"Duplicate mutable group was accepted or changed mesh");
}
void nativeGroupLoop() {
    Body local,foreign;
    const auto added = splitTriangleEdge(foreign.mesh,foreign.transform,&foreign.source,1,5,{0.5f,0,0},0.75f);
    splitTriangleEdge(foreign.mesh,foreign.transform,&foreign.source,1,added,{1.5f,0,0},1.0f/3);
    const Plane planes[] = {{{0,0,-1},0},{{0,-1,0},0},{{-1,0,0},0},{{1,1,1},2}};
    const GapNeighborBrushView views[] = {{&local.mesh,planes,0,4},{&foreign.mesh,planes,1,4}};
    const auto before = local.moments();
    require(fixGapGroup(local.mesh,planes,4,local.transform,&local.source,1,0,local.line,
        1,foreign.source.edges,foreign.source.count,views,2) == 2,"Recovered group loop did not split twice");
    local.validate(); const auto after = local.moments();
    require(std::abs(before[0]-after[0]) < 1e-7 && std::abs(before[1]-after[1]) < 1e-7,"Group loop changed volume/area");
    // Golden handles from a full original FixGaps call on this exact geometry.
    // Recollecting intervals after each split gives different link orientation.
    const std::uint32_t expectedEdges[] = {0,5,16,22,24,30};
    const std::uint32_t expectedNeighbors[] = {25,30,0,5,21,17};
    require(local.source.count == 6,"Wrong group source count");
    for (int i = 0; i < 6; ++i) {
        const auto e = local.source.edges[i];
        require(e == expectedEdges[i],"Native group source order changed");
        const auto& ref = local.triangles[e>>2].brushEdges[e&3u];
        require(ref.brushIndex == 1 && ref.edge == expectedNeighbors[i],"Native temporary-interval orientation changed");
    }
    Body limited;
    const GapNeighborBrushView limitedViews[] = {{&limited.mesh,planes,0,4},views[1]};
    require(rcsg_recovered_fix_gap_group(&limited.mesh,planes,4,&limited.transform,&limited.source,1,0,&limited.line,
        1,foreign.source.edges,foreign.source.count,limitedViews,2,0) == -2,"Zero group split budget accepted");
    require(limited.mesh.vertexCount == 4 && limited.mesh.triangleCount == 4,"Zero group budget changed geometry");
    require(rcsg_recovered_fix_gap_group(&limited.mesh,planes,4,&limited.transform,&limited.source,1,0,&limited.line,
        1,foreign.source.edges,foreign.source.count,limitedViews,2,1) == -2,"Partial group budget accepted");
    require(limited.mesh.vertexCount == 5 && limited.mesh.triangleCount == 6,"Partial group failure contract changed");
    limited.validate();
}
} // namespace

int main() {
    try { collectorRules(); alignTjunctions(); nativeGroupLoop(); std::cout << "Gap interval checks passed\n"; }
    catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
