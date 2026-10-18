#pragma once

#include <cstddef>
#include <cstdint>

namespace realtimecsg::recovered {

// Names and values recovered from ResultToString / CategoryIndexToString.
enum class PolygonSplitResult : std::uint8_t {
    CompletelyInside = 0,
    CompletelyOutside = 1,
    Split = 2,
    PlaneAligned = 3,
    PlaneOppositeAligned = 4,
    Collapsed = 5
};

enum class CategoryIndex : std::uint8_t {
    Inside = 0, Aligned = 1, RevAligned = 2, Outside = 3
};

inline constexpr float kPlaneDistanceTolerance = 0.0006000000284984708f;
inline constexpr float kCoincidentVertexDistanceSquared = 3.6000002978653356e-7f;

struct Vector3 { float x, y, z; };
// Native plane equation is dot(normal, point) - distance, not Unity's public Plane convention.
struct Plane { Vector3 normal; float distance; };

// Partial internal records for research. Unknown fields are intentionally unnamed.
// These are NOT complete definitions of the original classes or the Unity C ABI.
struct PolygonRecord {
    std::int32_t edgeCount;
    std::int32_t firstEdge;
    std::int32_t surfaceIndex;
    std::int32_t unidentified_0c;
    std::uint16_t unidentified_10;
    std::uint16_t categorizationState;
};
struct HalfEdgeRecord {
    std::int32_t nextEdge;
    std::int32_t vertexIndex;
    std::int32_t twinEdge;
    std::int32_t polygonIndex;
};
struct EdgeLinkRecord {
    std::int32_t previousEdge;
    // Confirmed by FindBrushEdges: source BrushEdge group index (-1 unassigned).
    // Historical field name retained to avoid changing existing research APIs.
    std::int32_t unidentified_04;
};
struct PolygonMeshView {
    const PolygonRecord* polygons;
    const HalfEdgeRecord* edges;
    const EdgeLinkRecord* edgeLinks;
    const Vector3* vertices;
};
struct SurfaceView {
    std::byte unidentified_00[32];
    const Plane* planes;
};
struct VertexState {
    Vector3 position;
    float distance;
    std::int32_t edgeIndex;
};

// Caller-owned, preallocated research buffers. Capacity of the two edge arrays
// is edgeChunkCapacity * 4. No implicit ownership or allocation across the C ABI.
struct BrushMemoryView {
    PolygonRecord* polygons;
    HalfEdgeRecord* edges;
    EdgeLinkRecord* edgeLinks;
    Vector3* vertices;
    std::int32_t polygonCount;
    std::int32_t edgeCount;
    std::int32_t vertexCount;
    std::int32_t polygonCapacity;
    std::int32_t edgeChunkCapacity;
    std::int32_t vertexCapacity;

    PolygonMeshView view() const { return {polygons, edges, edgeLinks, vertices}; }
};

static_assert(sizeof(Vector3) == 12 && sizeof(Plane) == 16);
static_assert(sizeof(PolygonRecord) == 20 && sizeof(HalfEdgeRecord) == 16);
static_assert(sizeof(EdgeLinkRecord) == 8 && offsetof(VertexState, edgeIndex) == 16);
static_assert(offsetof(SurfaceView, planes) == 32);
static_assert(sizeof(void*) != 8 || sizeof(BrushMemoryView) == 56);

float signedDistance(const Plane& plane, const Vector3& point);

// Valid indexed topology is a precondition, matching the original internal routines.
std::int32_t findVertexSideOnPlane(const VertexState& vertex, const PolygonMeshView& mesh,
                                std::int32_t& representativeEdge, std::int32_t& cachedSide,
                                const Plane& plane);
PolygonSplitResult alignment(const PolygonMeshView& mesh, const SurfaceView& surfaces,
                             std::int32_t polygonIndex, const Plane& plane);
// Valid topology and sufficient preallocated capacity are required. The C++
// functions throw length_error on exhaustion rather than reallocating caller memory.
std::int32_t insertVertexBetweenEdges(BrushMemoryView&, std::int32_t previousEdge,
                                      std::int32_t nextEdge, std::int32_t vertexIndex);
bool splitAtEdges(BrushMemoryView&, std::int32_t polygon, std::int32_t edgeA,
                  std::int32_t edgeB, std::int32_t& outsidePolygon);
PolygonSplitResult splitByPlane(BrushMemoryView&, const SurfaceView&, std::int32_t polygon,
                                const Plane&, std::int32_t& outsidePolygon);

} // namespace realtimecsg::recovered

// Research entry points for differential testing; not exports of the original plugin.
extern "C" {
std::uint8_t rcsg_recovered_alignment(
    const realtimecsg::recovered::PolygonMeshView*, const realtimecsg::recovered::SurfaceView*,
    std::int32_t, const realtimecsg::recovered::Plane*);
std::int32_t rcsg_recovered_vertex_side(
    const realtimecsg::recovered::VertexState*, const realtimecsg::recovered::PolygonMeshView*,
    std::int32_t*, std::int32_t*, float, float, float, float);
// Capacity exhaustion is reported as -2 / 255, not an exception through ctypes.
std::int32_t rcsg_recovered_insert_vertex(realtimecsg::recovered::BrushMemoryView*,
                                         std::int32_t, std::int32_t, std::int32_t);
std::int32_t rcsg_recovered_split_edges(realtimecsg::recovered::BrushMemoryView*,
                                       std::int32_t, std::int32_t, std::int32_t, std::int32_t*);
std::uint8_t rcsg_recovered_split_plane(realtimecsg::recovered::BrushMemoryView*,
                                       const realtimecsg::recovered::SurfaceView*, std::int32_t,
                                       const realtimecsg::recovered::Plane*, std::int32_t*);
}
