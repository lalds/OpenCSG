#pragma once
#include "realtimecsg/recovered_brush_edge_cleanup.h"

namespace realtimecsg::recovered {
struct TwoSurfaceVertexCandidate {
    std::uint32_t firstBoundary;
    std::uint32_t secondBoundary;
};
// Read-only extraction of 0x33ac3..0x34079 from the second Optimize pass.
// Finds two consecutive (surface,state) regions and tests the seam direction.
// Flags are deliberately ignored. True is only a candidate, not permission to
// remove: the original performs further topology/orientation tests afterwards.
// Output must not alias mesh storage and changes only on success.
// Bounded indexed fans are required; malformed
// nonclosing cycles throw. No mesh/context/groups are changed.
bool findTwoSurfaceVertexCandidate(const TriangleMeshView&, std::int32_t vertex,
                                  TwoSurfaceVertexCandidate&);
enum class TwoSurfaceVertexAction : std::int32_t {
    Rejected = 0, RemoveInPlane = 1, Swizzle = 2, Split = 3
};
struct TwoSurfaceVertexPlan {
    TwoSurfaceVertexCandidate boundary;
    std::uint32_t swizzleEdge;
    std::int32_t vertex;
};
// Read-only decisions of 0x339fe through the first mutating action. Includes
// candidate traversal, neighbor/endpoint checks and orientation tests, but
// omits native capacity growth. A selected Swizzle can still reject inside
// Swizzle; Split is only a plan, not a completed split or vertex removal.
// Requires indexed boundary twins with corners 0..2. Output must not alias
// mesh/surfaces and stays unchanged on rejection or error. Plane access is
// validated only when the native path reads that plane.
TwoSurfaceVertexAction planTwoSurfaceVertex(const TriangleMeshView&, const SurfaceView&,
    std::int32_t planeCount, std::int32_t vertex, TwoSurfaceVertexPlan&);
// Recovered Swizzle (Mach-O 0x331ae): replace a shared diagonal after native
// degeneracy/orientation tests. Not the full Optimize pass. Indexed input must
// have two distinct reciprocal adjacent triangles; invalid outer handles with
// low bits 3 are retained. Updates native fields only, including its asymmetric
// corner-normal/source writes and last-write-wins vertex handles. No allocation.
// False means geometric rejection, with no mutation. Bad views throw before
// mutation. Mutable mesh buffers must be disjoint from surfaces.
bool swizzleTriangleEdge(TriangleMeshView&, const SurfaceView&, std::int32_t planeCount,
                         std::uint32_t edge);

struct OptimizationNormalList {
    Vector3* values;
    std::int32_t count;
    std::int32_t capacity;
};
// Portable fixed-capacity version of the six native OptimizationContext vectors.
// Scratch list mutations on a normal native rejection are preserved. Removed
// triangle/vertex lists accumulate across calls; the other lists are scratch.
struct PlaneOptimizationContextView {
    BrushEdgeTriangleList boundaryEdges;
    TriangleIndexPoolView removedTriangles;
    TriangleIndexPoolView incidentTriangles;
    TriangleIndexPoolView removedVertices;
    TriangleIndexPoolView polygonVertices;
    OptimizationNormalList polygonNormals;
};
// Recovered RemoveVertexInMiddleOfPlane (0x316a2). Requires a bounded closed
// vertex fan with valid indexed boundary backlinks. Metadata mismatches and
// native early rejections return false and may change context scratch lists.
// IMPORTANT: the two-corner path mutates mesh/groups but ALSO returns false.
// Mesh counts stay fixed; removed records are tracked for later compaction.
// Buffers must be disjoint. Invalid input or exhausted capacity is atomic.
bool removeVertexInMiddleOfPlane(TriangleMeshView&, const SurfaceView&, std::int32_t planeCount,
    std::int32_t vertex, PlaneOptimizationContextView&, BrushEdgeGroupView*, BrushEdgeTriangleList*,
    BrushEdgeConnectionList*, std::int32_t& groupCount);
// Recovered first Optimize pass (0x33112). Clears the two removal-list counts,
// visits vertices in ascending original order (ignoring the helper's bool),
// then runs recovered compaction once. Before world/gap references are built.
// Mesh, context and all group storage form one transaction: errors are atomic
// even if earlier vertices have already caused group merges in the working copy.
void removeVerticesInMiddleOfPlane(TriangleMeshView&, const SurfaceView&, std::int32_t planeCount,
    PlaneOptimizationContextView&, BrushEdgeGroupView*, BrushEdgeTriangleList*,
    BrushEdgeConnectionList*, std::int32_t& groupCount);
// Recovered second Optimize pass (0x339fe): descending original vertex order,
// repeated Swizzle, temporary split, plane removals, final compaction. Same
// atomic transaction guarantee as the first pass. Fixed caller capacity:
// requires space for two extra triangles and one vertex at each actual split;
// native's speculative capacity growth is not reproduced. World/gap buffers
// must not have been built yet. Unknown bytes in new slots are retained.
void removeVertexOnEdgeBetween2Surfaces(TriangleMeshView&, const SurfaceView&, std::int32_t planeCount,
    PlaneOptimizationContextView&, BrushEdgeGroupView*, BrushEdgeTriangleList*,
    BrushEdgeConnectionList*, std::int32_t& groupCount);
// Both passes in native Optimize order, each followed by compaction. Caller
// supplies fixed context/mesh capacities instead of native temporary vectors.
// One transaction spans both passes, including failure in the second pass.
void optimizeTriangleMesh(TriangleMeshView&, const SurfaceView&, std::int32_t planeCount,
    PlaneOptimizationContextView&, BrushEdgeGroupView*, BrushEdgeTriangleList*,
    BrushEdgeConnectionList*, std::int32_t& groupCount);
}
extern "C" {
// 1 candidate; 0 native prefix rejection; -1 invalid input; -3 other failure.
std::int32_t rcsg_recovered_find_two_surface_candidate(const realtimecsg::recovered::TriangleMeshView*,
    std::int32_t vertex, realtimecsg::recovered::TwoSurfaceVertexCandidate*);
// 0 rejected; 1 RemoveInPlane; 2 Swizzle; 3 Split; -1 invalid; -3 other error.
std::int32_t rcsg_recovered_plan_two_surface_vertex(const realtimecsg::recovered::TriangleMeshView*,
    const realtimecsg::recovered::SurfaceView*, std::int32_t planeCount, std::int32_t vertex,
    realtimecsg::recovered::TwoSurfaceVertexPlan*);
// 1 changed; 0 geometric rejection; -1 invalid input; -3 other failure.
std::int32_t rcsg_recovered_swizzle_triangle_edge(realtimecsg::recovered::TriangleMeshView*,
    const realtimecsg::recovered::SurfaceView*, std::int32_t planeCount, std::uint32_t edge);
// Native result 0/1; -1 invalid input; -2 capacity; -3 other failure.
// Result 0 does NOT imply absence of mesh/group/context changes.
std::int32_t rcsg_recovered_remove_plane_vertex(realtimecsg::recovered::TriangleMeshView*,
    const realtimecsg::recovered::SurfaceView*, std::int32_t planeCount, std::int32_t vertex,
    realtimecsg::recovered::PlaneOptimizationContextView*, realtimecsg::recovered::BrushEdgeGroupView*,
    realtimecsg::recovered::BrushEdgeTriangleList*, realtimecsg::recovered::BrushEdgeConnectionList*,
    std::int32_t* groupCount);
// 0 success; -1 invalid input; -2 capacity; -3 other failure.
std::int32_t rcsg_recovered_remove_plane_vertices(realtimecsg::recovered::TriangleMeshView*,
    const realtimecsg::recovered::SurfaceView*, std::int32_t planeCount,
    realtimecsg::recovered::PlaneOptimizationContextView*, realtimecsg::recovered::BrushEdgeGroupView*,
    realtimecsg::recovered::BrushEdgeTriangleList*, realtimecsg::recovered::BrushEdgeConnectionList*,
    std::int32_t* groupCount);
// 0 success; -1 invalid views/topology; -2 capacity; -3 other failure.
std::int32_t rcsg_recovered_remove_two_surface_vertices(realtimecsg::recovered::TriangleMeshView*,
    const realtimecsg::recovered::SurfaceView*, std::int32_t planeCount,
    realtimecsg::recovered::PlaneOptimizationContextView*, realtimecsg::recovered::BrushEdgeGroupView*,
    realtimecsg::recovered::BrushEdgeTriangleList*, realtimecsg::recovered::BrushEdgeConnectionList*,
    std::int32_t* groupCount);
// Same status codes and atomic error guarantee as the individual passes.
std::int32_t rcsg_recovered_optimize_mesh(realtimecsg::recovered::TriangleMeshView*,
    const realtimecsg::recovered::SurfaceView*, std::int32_t planeCount,
    realtimecsg::recovered::PlaneOptimizationContextView*, realtimecsg::recovered::BrushEdgeGroupView*,
    realtimecsg::recovered::BrushEdgeTriangleList*, realtimecsg::recovered::BrushEdgeConnectionList*,
    std::int32_t* groupCount);
}
