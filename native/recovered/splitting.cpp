#include "realtimecsg/recovered_geometry.h"

#include <stdexcept>

namespace realtimecsg::recovered {
namespace {
void requireEdges(const BrushMemoryView& memory, int additional) {
    if ((memory.edgeCount + additional + 4) / 4 > memory.edgeChunkCapacity)
        throw std::length_error("Preallocated edge chunks exhausted");
}
int createEdge(BrushMemoryView& memory) {
    requireEdges(memory, 1);
    return memory.edgeCount++;
}
int createVertex(BrushMemoryView& memory, const Vector3& point) {
    if (memory.vertexCount >= memory.vertexCapacity)
        throw std::length_error("Preallocated vertices exhausted");
    const int index = memory.vertexCount++;
    memory.vertices[index] = point;
    return index;
}
const Vector3& edgeVertex(const BrushMemoryView& memory, int edge) {
    return memory.vertices[memory.edges[edge].vertexIndex];
}
float squaredDistance(const Vector3& a, const Vector3& b) {
    const float x = a.x - b.x, y = a.y - b.y, z = a.z - b.z;
    return (x * x + y * y) + z * z;
}
Vector3 projectToPlane(Vector3 point, const Plane& plane) {
    // Original computes dot(point - normal*distance, normal), not a fused
    // dot(point, normal)-distance expression. Preserve its float rounding.
    const auto& n = plane.normal;
    const float offset = ((point.x - n.x * plane.distance) * n.x
                         + (point.y - n.y * plane.distance) * n.y)
                         + (point.z - n.z * plane.distance) * n.z;
    point.x -= n.x * offset;
    point.y -= n.y * offset;
    point.z -= n.z * offset;
    return point;
}
PolygonSplitResult unsplitResult(int side, int polygon, const Plane& surface,
                                 const Plane& cutting, int& outsidePolygon) {
    outsidePolygon = -1;
    if (side < 0) return PolygonSplitResult::CompletelyInside;
    if (side > 0) {
        outsidePolygon = polygon;
        return PolygonSplitResult::CompletelyOutside;
    }
    const auto& a = surface.normal;
    const auto& b = cutting.normal;
    return (a.x * b.x + a.y * b.y) + a.z * b.z > 0.0f
               ? PolygonSplitResult::PlaneAligned : PolygonSplitResult::PlaneOppositeAligned;
}
} // namespace

// Mach-O 0x64b6..0x65f6. Each inserted vertex also subdivides the twin face.
std::int32_t insertVertexBetweenEdges(BrushMemoryView& memory, int previous,
                                      int next, int vertex) {
    requireEdges(memory, 2);
    const int polygon = memory.edges[next].polygonIndex;
    const int twin = memory.edges[next].twinEdge;
    const int twinPolygon = memory.edges[twin].polygonIndex;
    const int twinPrevious = memory.edgeLinks[twin].previousEdge;
    const int extra = memory.edgeLinks[next].unidentified_04;
    const int twinExtra = memory.edgeLinks[twin].unidentified_04;

    const int inserted = createEdge(memory);
    memory.edges[inserted] = {next, vertex, twin, polygon};
    memory.edges[previous].nextEdge = inserted;
    memory.edgeLinks[inserted] = {previous, extra};
    memory.edgeLinks[next].previousEdge = inserted;

    const int insertedTwin = createEdge(memory);
    memory.edges[insertedTwin] = {twin, vertex, next, twinPolygon};
    memory.edges[twinPrevious].nextEdge = insertedTwin;
    memory.edgeLinks[insertedTwin] = {twinPrevious, twinExtra};
    memory.edgeLinks[twin].previousEdge = insertedTwin;
    ++memory.polygons[polygon].edgeCount;
    ++memory.polygons[twinPolygon].edgeCount;
    memory.edges[next].twinEdge = insertedTwin;
    memory.edges[twin].twinEdge = inserted;
    return inserted;
}

// Mach-O 0x65f8..0x6863. The existing polygon retains the inside loop.
bool splitAtEdges(BrushMemoryView& memory, int polygon, int edgeA,
                  int edgeB, int& outsidePolygon) {
    if (polygon < 0 || polygon >= memory.polygonCount || memory.polygonCount <= 0)
        return false;
    if (memory.polygonCount >= memory.polygonCapacity)
        throw std::length_error("Preallocated polygons exhausted");
    const int newPolygon = memory.polygonCount++;
    int remainingCount = memory.polygons[polygon].edgeCount - 1;
    outsidePolygon = newPolygon;
    const int nextA = memory.edges[edgeA].nextEdge;
    const int nextB = memory.edges[edgeB].nextEdge;
    const int vertexA = memory.edges[edgeA].vertexIndex;
    const int vertexB = memory.edges[edgeB].vertexIndex;

    int newCount = 2;
    int edge = nextB;
    for (;;) {
        if (edge < 0 || edge >= memory.edgeCount) return false;
        memory.edges[edge].polygonIndex = newPolygon;
        if (edge == edgeA) break;
        if (newCount - 2 > 999) return false;
        edge = memory.edges[edge].nextEdge;
        ++newCount;
        --remainingCount;
        if (edge == nextB) break;
    }

    memory.edges[edgeA].nextEdge = nextB;
    memory.edgeLinks[nextB].previousEdge = edgeA;
    if (vertexA == vertexB) {
        --newCount;
        memory.edges[edgeB].nextEdge = nextA;
        memory.edgeLinks[nextA].previousEdge = edgeB;
    } else {
        requireEdges(memory, 2);
        ++remainingCount;
        const int closingNew = createEdge(memory);
        memory.edges[closingNew] = {nextB, vertexB, -1, newPolygon};
        memory.edges[edgeA].nextEdge = closingNew;
        memory.edgeLinks[closingNew] = {edgeA, -1};
        memory.edgeLinks[nextB].previousEdge = closingNew;
        const int closingOld = createEdge(memory);
        memory.edges[closingOld] = {nextA, vertexA, closingNew, polygon};
        memory.edges[edgeB].nextEdge = closingOld;
        memory.edgeLinks[closingOld] = {edgeB, -1};
        memory.edgeLinks[nextA].previousEdge = closingOld;
        memory.edges[closingNew].twinEdge = closingOld;
    }

    memory.polygons[newPolygon].surfaceIndex = memory.polygons[polygon].surfaceIndex;
    // Only the low flag byte is copied. Preserve all other originally untouched bytes.
    auto& flags = memory.polygons[newPolygon].unidentified_10;
    flags = static_cast<std::uint16_t>((flags & 0xff00u)
                                    | (memory.polygons[polygon].unidentified_10 & 0xffu));
    memory.polygons[polygon].edgeCount = remainingCount;
    memory.polygons[polygon].firstEdge = edgeB;
    memory.polygons[newPolygon].edgeCount = newCount;
    memory.polygons[newPolygon].firstEdge = edgeA;
    return true;
}

// Reconstructed from 0x2d15c..0x2dd2b. Checked against original instructions;
// higher-level brush categorization and memory growth are separate work.
PolygonSplitResult splitByPlane(BrushMemoryView& memory, const SurfaceView& surfaces,
                                int polygonIndex, const Plane& plane, int& outsidePolygon) {
    const Plane surface = surfaces.planes[memory.polygons[polygonIndex].surfaceIndex];
    const int originalEdgeCount = memory.polygons[polygonIndex].edgeCount;
    outsidePolygon = -1;
    int representative = -1, cachedSide = -2;
    struct State { Vector3 position; float distance; int edge; int side; };
    const auto readState = [&](int edge) {
        State state{edgeVertex(memory, edge), 0.0f, edge, 0};
        state.distance = signedDistance(plane, state.position);
        if (state.distance <= kPlaneDistanceTolerance && state.distance >= -kPlaneDistanceTolerance) {
            state.side = findVertexSideOnPlane({state.position, state.distance, edge}, memory.view(),
                                               representative, cachedSide, plane);
        } else {
            representative = -1;
            cachedSide = state.side = state.distance < 0.0f ? -1 : 1;
        }
        return state;
    };
    State current = readState(memory.polygons[polygonIndex].firstEdge);
    State next = readState(memory.edges[current.edge].nextEdge);
    int entering = -1, leaving = -1;
    for (int i = 0; i < originalEdgeCount; ++i) {
        State ahead = readState(memory.edges[next.edge].nextEdge);
        if (current.side != next.side) {
            if (next.side == 0) {
                if (current.side == ahead.side)
                    return unsplitResult(current.side, polygonIndex, surface, plane, outsidePolygon);
                if (current.side < 0) leaving = next.edge;
                else entering = next.edge;
            } else if (current.side != 0) {
                // Both branches interpolate from the negative-side endpoint, so twins
                // use the same arithmetic order rather than opposite lerp directions.
                const State& inside = current.side < 0 ? current : next;
                const State& outside = current.side < 0 ? next : current;
                const float factor = inside.distance / (inside.distance - outside.distance);
                Vector3 point{inside.position.x - (inside.position.x - outside.position.x) * factor,
                              inside.position.y - (inside.position.y - outside.position.y) * factor,
                              inside.position.z - (inside.position.z - outside.position.z) * factor};
                point = projectToPlane(projectToPlane(point, plane), surface);
                int vertex = -1;
                if (entering >= 0 && squaredDistance(edgeVertex(memory, entering), point)
                                     <= kCoincidentVertexDistanceSquared)
                    vertex = memory.edges[entering].vertexIndex;
                if (leaving >= 0 && squaredDistance(edgeVertex(memory, leaving), point)
                                    <= kCoincidentVertexDistanceSquared)
                    vertex = memory.edges[leaving].vertexIndex;
                if (vertex == -1) vertex = createVertex(memory, point);
                const int inserted = insertVertexBetweenEdges(memory, current.edge, next.edge, vertex);
                if (current.side > 0) entering = inserted;
                else leaving = inserted;
            }
            if (entering >= 0 && leaving >= 0) {
                const bool success = splitAtEdges(memory, polygonIndex, entering, leaving, outsidePolygon);
                if (success && outsidePolygon < 0) return PolygonSplitResult::CompletelyInside;
                return PolygonSplitResult::Split;
            }
        }
        current = next;
        next = ahead;
    }
    return unsplitResult(current.side, polygonIndex, surface, plane, outsidePolygon);
}
} // namespace realtimecsg::recovered

extern "C" std::int32_t rcsg_recovered_insert_vertex(
    realtimecsg::recovered::BrushMemoryView* memory, int previous, int next, int vertex) {
    try { return realtimecsg::recovered::insertVertexBetweenEdges(*memory, previous, next, vertex); }
    catch (const std::length_error&) { return -2; }
}
extern "C" std::int32_t rcsg_recovered_split_edges(
    realtimecsg::recovered::BrushMemoryView* memory, int polygon, int a, int b, int* outside) {
    try { return realtimecsg::recovered::splitAtEdges(*memory, polygon, a, b, *outside) ? 1 : 0; }
    catch (const std::length_error&) { return -2; }
}
extern "C" std::uint8_t rcsg_recovered_split_plane(
    realtimecsg::recovered::BrushMemoryView* memory, const realtimecsg::recovered::SurfaceView* surfaces,
    int polygon, const realtimecsg::recovered::Plane* plane, int* outside) {
    try { return static_cast<std::uint8_t>(realtimecsg::recovered::splitByPlane(*memory, *surfaces, polygon, *plane, *outside)); }
    catch (const std::length_error&) { return 255; }
}
