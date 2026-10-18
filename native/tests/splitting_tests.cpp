#include "realtimecsg/recovered_geometry.h"

#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>

using namespace realtimecsg::recovered;

namespace {
void require(bool condition, const char* message) {
    if (!condition) {
        std::cerr << message << '\n';
        std::exit(1);
    }
}

struct Cube {
    std::array<PolygonRecord, 32> polygons{};
    std::array<HalfEdgeRecord, 256> edges{};
    std::array<EdgeLinkRecord, 256> links{};
    std::array<Vector3, 64> vertices{};
    std::array<Plane, 6> planes{{{{0, 0, -1}, 1}, {{0, 0, 1}, 1},
                                {{0, -1, 0}, 1}, {{1, 0, 0}, 1},
                                {{0, 1, 0}, 1}, {{-1, 0, 0}, 1}}};
    BrushMemoryView memory{polygons.data(), edges.data(), links.data(), vertices.data(),
                           6, 24, 8, 32, 64, 64};
    SurfaceView surfaces{};

    Cube() {
        surfaces.planes = planes.data();
        vertices[0] = {-1, -1, -1}; vertices[1] = {1, -1, -1};
        vertices[2] = {1, 1, -1}; vertices[3] = {-1, 1, -1};
        vertices[4] = {-1, -1, 1}; vertices[5] = {1, -1, 1};
        vertices[6] = {1, 1, 1}; vertices[7] = {-1, 1, 1};
        const int faces[6][4] = {{3,2,1,0}, {4,5,6,7}, {0,1,5,4},
                                 {1,2,6,5}, {2,3,7,6}, {3,0,4,7}};
        for (int polygon = 0; polygon < 6; ++polygon) {
            polygons[polygon] = {4, polygon * 4, polygon, 0, 0x1234, 0xffff};
            for (int i = 0; i < 4; ++i) {
                const int edge = polygon * 4 + i;
                edges[edge] = {polygon * 4 + (i+1)%4, faces[polygon][i], -1, polygon};
                links[edge] = {polygon * 4 + (i+3)%4, -1};
            }
        }
        for (int edge = 0; edge < 24; ++edge) {
            const int start = edges[links[edge].previousEdge].vertexIndex;
            const int end = edges[edge].vertexIndex;
            for (int twin = 0; twin < 24; ++twin) {
                if (edges[twin].vertexIndex == start
                    && edges[links[twin].previousEdge].vertexIndex == end)
                    edges[edge].twinEdge = twin;
            }
        }
    }

    void validate() const {
        std::array<bool, 256> seen{};
        int visited = 0;
        for (int polygon = 0; polygon < memory.polygonCount; ++polygon) {
            int edge = polygons[polygon].firstEdge;
            for (int i = 0; i < polygons[polygon].edgeCount; ++i) {
                require(edge >= 0 && edge < memory.edgeCount, "Edge index out of range");
                require(!seen[edge], "Edge belongs to more than one polygon loop");
                seen[edge] = true;
                ++visited;
                const auto& e = edges[edge];
                require(e.polygonIndex == polygon, "Wrong polygon owner");
                require(e.vertexIndex >= 0 && e.vertexIndex < memory.vertexCount, "Vertex out of range");
                require(links[e.nextEdge].previousEdge == edge, "Broken next/previous link");
                require(e.twinEdge >= 0 && e.twinEdge < memory.edgeCount, "Twin out of range");
                require(edges[e.twinEdge].twinEdge == edge, "Asymmetric twin");
                require(edges[e.twinEdge].vertexIndex == edges[links[edge].previousEdge].vertexIndex,
                        "Twin endpoint changed");
                edge = e.nextEdge;
            }
            require(edge == polygons[polygon].firstEdge, "Polygon loop not closed");
        }
        require(visited == memory.edgeCount, "Orphaned edges");
    }

    double totalArea() const {
        double total = 0;
        for (int p = 0; p < memory.polygonCount; ++p) {
            int edge = polygons[p].firstEdge;
            double x = 0, y = 0, z = 0;
            for (int i = 0; i < polygons[p].edgeCount; ++i) {
                const auto& a = vertices[edges[edge].vertexIndex];
                edge = edges[edge].nextEdge;
                const auto& b = vertices[edges[edge].vertexIndex];
                x += double(a.y)*b.z - double(a.z)*b.y;
                y += double(a.z)*b.x - double(a.x)*b.z;
                z += double(a.x)*b.y - double(a.y)*b.x;
            }
            total += 0.5 * std::sqrt(x*x + y*y + z*z);
        }
        return total;
    }
};
} // namespace

int main() {
    Cube cube;
    cube.validate();
    int outside = -10;
    auto result = splitByPlane(cube.memory, cube.surfaces, 0, {{1, 0, 0}, 0}, outside);
    require(result == PolygonSplitResult::Split && outside == 6, "Expected two polygon pieces");
    require(cube.memory.polygonCount == 7 && cube.memory.edgeCount == 30 && cube.memory.vertexCount == 10,
            "Wrong counts after first cut");
    require(cube.polygons[6].surfaceIndex == 0, "Surface provenance lost");
    require(cube.polygons[6].unidentified_10 == 0x34, "Original copies only the low flag byte");
    cube.validate();
    require(std::abs(cube.totalArea() - 24.0) < 1e-6, "Cut changed cube surface area");

    result = splitByPlane(cube.memory, cube.surfaces, 0, {{0, 1, 0}, 0}, outside);
    require(result == PolygonSplitResult::Split && outside == 7, "Repeated cut failed");
    require(cube.memory.polygonCount == 8 && cube.memory.edgeCount == 36 && cube.memory.vertexCount == 12,
            "Wrong counts after repeated cut");
    cube.validate();
    require(std::abs(cube.totalArea() - 24.0) < 1e-6, "Repeated cut changed surface area");

    Cube untouched;
    result = splitByPlane(untouched.memory, untouched.surfaces, 0, {{0, 0, 1}, -2}, outside);
    require(result == PolygonSplitResult::CompletelyOutside && outside == 0, "Wrong outside polygon ID");
    require(untouched.memory.polygonCount == 6 && untouched.memory.edgeCount == 24
            && untouched.memory.vertexCount == 8, "Unsplit cube changed size");
    untouched.validate();

    // Research API must report exhaustion instead of reallocating caller-owned buffers.
    untouched.memory.edgeChunkCapacity = 0;
    require(rcsg_recovered_insert_vertex(&untouched.memory, 0, 1, 0) == -2,
            "Capacity exhaustion was not reported");
    require(untouched.memory.edgeCount == 24, "Insert capacity preflight mutated counts");
    std::cout << "Recovered splitting checks passed\n";
}
