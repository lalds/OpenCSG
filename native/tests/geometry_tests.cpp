#include "realtimecsg/recovered_geometry.h"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>

using namespace realtimecsg::recovered;

namespace {
void require(bool condition, const char* message) {
    if (!condition) {
        std::cerr << message << '\n';
        std::exit(1);
    }
}
}

int main() {
    // Expected values were also observed by calling the original Mach-O routines.
    // No original code is loaded by this test; it runs on the rebuilt C++ only.
    Vector3 points[] = {{-1, -1, 0}, {1, -1, 0}, {1, 1, 0}, {-1, 1, 0}};
    HalfEdgeRecord edges[] = {{1, 0, 0, 0}, {2, 1, 0, 0}, {3, 2, 0, 0}, {0, 3, 0, 0}};
    EdgeLinkRecord links[] = {{3, 0}, {0, 0}, {1, 0}, {2, 0}};
    PolygonRecord polygon{4, 0, 0, 0, 0, 0};
    Plane surface{{0, 0, 1}, 0};
    PolygonMeshView mesh{&polygon, edges, links, points};
    SurfaceView surfaces{};
    surfaces.planes = &surface;

    require(alignment(mesh, surfaces, 0, {{0, 0, 1}, 1}) == PolygonSplitResult::CompletelyInside,
            "Plane distance must be subtracted, and negative distance must classify inside");
    require(alignment(mesh, surfaces, 0, {{0, 0, 1}, -1}) == PolygonSplitResult::CompletelyOutside,
            "Positive distance must classify outside");
    require(alignment(mesh, surfaces, 0, {{1, 0, 0}, 0}) == PolygonSplitResult::Split,
            "Polygon crossing the plane must classify split");

    // Do not replace the shared topology-aware cache with a stateless epsilon test.
    // The original returns Outside for this particular all-coplanar four-edge input.
    require(alignment(mesh, surfaces, 0, surface) == PolygonSplitResult::CompletelyOutside,
            "Preserve observed coplanar cache behavior until higher-level invariants are recovered");

    polygon.edgeCount = 0;
    require(alignment(mesh, surfaces, 0, surface) == PolygonSplitResult::CompletelyInside,
            "Zero edges returns Inside in original Alignment, not Collapsed");

    polygon.edgeCount = 1;
    edges[0].nextEdge = links[0].previousEdge = 0;
    require(alignment(mesh, surfaces, 0, surface) == PolygonSplitResult::PlaneAligned,
            "Single on-plane vertex with matching normal must return PlaneAligned");
    require(alignment(mesh, surfaces, 0, {{0, 0, -1}, 0}) == PolygonSplitResult::PlaneOppositeAligned,
            "Opposite surface normal must return PlaneOppositeAligned");
    require(alignment(mesh, surfaces, 0, {{0, 0, 1}, kPlaneDistanceTolerance}) == PolygonSplitResult::PlaneAligned,
            "The tolerance boundary is inclusive");
    const float beyond = std::nextafter(kPlaneDistanceTolerance, std::numeric_limits<float>::infinity());
    require(alignment(mesh, surfaces, 0, {{0, 0, 1}, beyond}) == PolygonSplitResult::CompletelyInside,
            "One float step beyond tolerance must classify by sign");

    // A near-coincident pair chooses the smaller signed distance as representative.
    polygon.edgeCount = 2;
    edges[0].nextEdge = 1;
    edges[1].nextEdge = 0;
    links[0].previousEdge = 1;
    links[1].previousEdge = 0;
    points[0] = {0, 0, 0.0001f};
    points[1] = {0, 0, -0.0001f};
    std::int32_t representative = -1, cachedSide = -2;
    const auto side = findVertexSideOnPlane({points[0], 0.0001f, 0}, mesh,
                                          representative, cachedSide, surface);
    require(side == 1 && representative == 1 && cachedSide == 1,
            "Near-coincident representative selection and signed fallback changed");
    std::cout << "Recovered geometry checks passed\n";
}
