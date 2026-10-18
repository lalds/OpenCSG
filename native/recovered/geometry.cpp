#include "realtimecsg/recovered_geometry.h"

namespace realtimecsg::recovered {
namespace {
// Preserve the original ordered/unordered comparisons for non-finite input too.
bool outsideTolerance(float distance) {
    return !(distance <= kPlaneDistanceTolerance && distance >= -kPlaneDistanceTolerance);
}
std::int32_t sideOf(float distance) { return distance < 0.0f ? -1 : 1; }
float squaredDistance(const Vector3& a, const Vector3& b) {
    const float x = a.x - b.x, y = a.y - b.y, z = a.z - b.z;
    return (x * x + y * y) + z * z;
}
const Vector3& edgeVertex(const PolygonMeshView& mesh, std::int32_t edge) {
    return mesh.vertices[mesh.edges[edge].vertexIndex];
}
} // namespace

float signedDistance(const Plane& plane, const Vector3& point) {
    return ((point.x * plane.normal.x + point.y * plane.normal.y)
            + point.z * plane.normal.z) - plane.distance;
}

// Reconstructed from Mach-O 0x2cdfe..0x2d15b. See docs/recovery/plane-classification.ru.md.
std::int32_t findVertexSideOnPlane(const VertexState& vertex, const PolygonMeshView& mesh,
                                std::int32_t& representativeEdge, std::int32_t& cachedSide,
                                const Plane& plane) {
    if (representativeEdge < 0) {
        float bestDistance = vertex.distance;
        representativeEdge = vertex.edgeIndex;
        std::int32_t edge = mesh.edgeLinks[vertex.edgeIndex].previousEdge;
        int iterations = 0;
        while (edge != vertex.edgeIndex) {
            const auto& candidate = edgeVertex(mesh, edge);
            const float distance = signedDistance(plane, candidate);
            if (outsideTolerance(distance)) {
                cachedSide = sideOf(distance);
                break;
            }
            if (squaredDistance(vertex.position, candidate) > kCoincidentVertexDistanceSquared)
                break;
            if (bestDistance > distance) {
                representativeEdge = edge;
                bestDistance = distance;
            }
            if (++iterations > 10000) break;
            edge = mesh.edgeLinks[edge].previousEdge;
        }

        edge = mesh.edges[vertex.edgeIndex].nextEdge;
        iterations = 0;
        while (edge != vertex.edgeIndex) {
            const auto& candidate = edgeVertex(mesh, edge);
            const float distance = signedDistance(plane, candidate);
            if (outsideTolerance(distance)
                || squaredDistance(vertex.position, candidate) > kCoincidentVertexDistanceSquared)
                break;
            if (bestDistance > distance) {
                representativeEdge = edge;
                bestDistance = distance;
            }
            if (++iterations > 10000) break;
            edge = mesh.edges[edge].nextEdge;
        }
    }

    if (representativeEdge == vertex.edgeIndex) {
        auto edge = mesh.edges[representativeEdge].nextEdge;
        int iterations = 0;
        while (edge != representativeEdge) {
            const float distance = signedDistance(plane, edgeVertex(mesh, edge));
            if (outsideTolerance(distance)) {
                cachedSide = sideOf(distance);
                break;
            }
            if (++iterations > 10000) break;
            edge = mesh.edges[edge].nextEdge;
        }
        return 0;
    }

    if (cachedSide != -2) return cachedSide;
    auto edge = mesh.edgeLinks[vertex.edgeIndex].previousEdge;
    int iterations = 0;
    while (edge != vertex.edgeIndex) {
        const float distance = signedDistance(plane, edgeVertex(mesh, edge));
        if (outsideTolerance(distance)) {
            cachedSide = sideOf(distance);
            return cachedSide;
        }
        if (++iterations > 10000) break;
        edge = mesh.edgeLinks[edge].previousEdge;
    }
    cachedSide = sideOf(vertex.distance);
    return cachedSide;
}

// Reconstructed from Mach-O 0x2cc04..0x2cdfc, including the shared side cache.
PolygonSplitResult alignment(const PolygonMeshView& mesh, const SurfaceView& surfaces,
                             std::int32_t polygonIndex, const Plane& plane) {
    const auto& polygon = mesh.polygons[polygonIndex];
    const auto& surfaceNormal = surfaces.planes[polygon.surfaceIndex].normal;
    std::int32_t representativeEdge = -1, cachedSide = -2;
    int positive = 0, negative = 0, onPlane = 0;
    auto edge = polygon.firstEdge;
    for (std::int32_t i = 0; i < polygon.edgeCount; ++i) {
        const auto& position = edgeVertex(mesh, edge);
        const float distance = signedDistance(plane, position);
        std::int32_t side;
        if (!outsideTolerance(distance)) {
            side = findVertexSideOnPlane({position, distance, edge}, mesh,
                                         representativeEdge, cachedSide, plane);
        } else {
            representativeEdge = -1;
            cachedSide = side = sideOf(distance);
        }
        if (side > 0) {
            if (negative > 0) return PolygonSplitResult::Split;
            ++positive;
        } else if (side < 0) {
            if (positive > 0) return PolygonSplitResult::Split;
            ++negative;
        } else {
            ++onPlane;
        }
        edge = mesh.edges[edge].nextEdge;
    }
    if (onPlane > 0 && positive == 0 && negative == 0) {
        const float dot = (surfaceNormal.x * plane.normal.x + surfaceNormal.y * plane.normal.y)
                          + surfaceNormal.z * plane.normal.z;
        return dot > 0.0f ? PolygonSplitResult::PlaneAligned
                          : PolygonSplitResult::PlaneOppositeAligned;
    }
    return positive > 0 ? PolygonSplitResult::CompletelyOutside
                        : PolygonSplitResult::CompletelyInside;
}
} // namespace realtimecsg::recovered

extern "C" std::uint8_t rcsg_recovered_alignment(
    const realtimecsg::recovered::PolygonMeshView* mesh,
    const realtimecsg::recovered::SurfaceView* surfaces, std::int32_t polygon,
    const realtimecsg::recovered::Plane* plane) {
    return static_cast<std::uint8_t>(realtimecsg::recovered::alignment(*mesh, *surfaces, polygon, *plane));
}

extern "C" std::int32_t rcsg_recovered_vertex_side(
    const realtimecsg::recovered::VertexState* vertex,
    const realtimecsg::recovered::PolygonMeshView* mesh,
    std::int32_t* representative, std::int32_t* side, float x, float y, float z, float distance) {
    return realtimecsg::recovered::findVertexSideOnPlane(*vertex, *mesh, *representative, *side,
                                                       {{x, y, z}, distance});
}
