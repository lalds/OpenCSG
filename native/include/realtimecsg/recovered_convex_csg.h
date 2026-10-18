#pragma once

#include "realtimecsg/recovered_categorization.h"
#include "realtimecsg/recovered_brush_edges.h"

namespace realtimecsg::recovered {

struct ConvexBrushView {
    const Plane* planes;
    std::int32_t planeCount;
};

enum class PlaneRelation { Separate, Aligned, OppositeAligned };

// Common-coordinate specialization of the coplanar prepass in native Categorize.
// Planes must have unit normals. Separate also includes parallel distinct planes.
PlaneRelation coplanarRelation(const Plane&, const Plane&);

// New orchestration of recovered routines, NOT a reconstruction of Categorize.
// Input is a valid closed, convex half-edge mesh; every brush and source plane is
// expressed in the same coordinate system with unit, outward-facing normals.
// sourcePlaneCount bounds polygon.surfaceIndex; clippers are indexed by tree node.
// program must come from compileCategorization. It is applied to every polygon;
// output state 1 hides, 2 retains, 3 retains with reversed winding. Surface indices
// survive splitting. This does not merge meshes, stitch gaps, or triangulate.
// Buffers remain caller-owned. Exhaustion throws and may leave partial mutations.
void categorizeConvexFragments(BrushMemoryView&, const SurfaceView&, std::int32_t sourcePlaneCount,
                              const CategorizationProgram&, const ConvexBrushView* clippers,
                              std::int32_t clipperCount);

struct CategorizedBrushView {
    BrushMemoryView* memory;
    const SurfaceView* surfaces;
};

// New, quadratic boundary cleanup for the research pipeline. Hides pairs of
// opposite coincident convex fragments with equal footprints (collinear extra
// vertices allowed). Removes zero-thickness faces at contact. Does not resolve
// partially overlapping footprints, weld vertices, or reconstruct native Optimize.
std::int32_t removeOppositeCoincidentFragments(const CategorizedBrushView*, std::int32_t count);

struct ConvexMeshInput {
    const BrushMemoryView* brush;
    const SurfaceView* surfaces;
    std::int32_t planeCount;
    std::int32_t brushIndex;
};
struct PreparedBrushEdgeGroup {
    std::vector<BrushTriangleEdge> surfaceRefs; // (foreign brush ID, surface ID)
    std::vector<std::uint32_t> triangleEdges;
    std::int32_t surfaceA, surfaceB;
    GapLine line;
    std::vector<BrushTriangleEdge> connections; // (real foreign brush ID, group ID)
};
struct PreparedConvexBrush {
    std::int32_t brushIndex;
    TriangulatedBrush mesh; // includes hidden faces and per-brush twins
    std::vector<PreparedBrushEdgeGroup> edgeGroups;
};
// New common-space orchestration, not recovered native Categorize/manager:
// fresh FindBrushEdges -> Create/visibility -> Optimize -> CleanUpBrushEdges.
// Takes already categorized closed convex source meshes, unit outward planes
// and unique nonnegative brush IDs. All inputs share one coordinate system.
// Rebuilds group tags on private copies (input tags are ignored), owns scratch
// and temporary mesh capacity, preserves all inputs on failure and success.
// Returns complete per-brush meshes; compactVisibleTriangles is a separate
// final export which clears source tags and invalidates these group handles. Does not run
// FindConnections/FixGaps, weld between brushes, or build world/gap buffers.
std::vector<PreparedConvexBrush> prepareConvexMeshes(const ConvexMeshInput*, std::int32_t count);

struct ConvexGapInput {
    const PreparedConvexBrush* brush;
    const SurfaceView* surfaces;
    std::int32_t planeCount;
};
struct ConvexGapRepairResult {
    std::vector<PreparedConvexBrush> brushes;
    std::int32_t splitCount;
};
// New common-space scene adapter. Resolves real foreign IDs with reciprocal
// surface refs and native line tolerances; it does NOT use the macOS
// FindConnections self-lookup. Runs recovered FixGaps until a no-cut sweep,
// then ConnectGaps, and refreshes vertex handles. Each brush's total cut budget
// spans all sweeps. Input meshes/groups/surfaces are unchanged on all paths.
// Complete prepared meshes, unique nonnegative IDs and finite unit planes/lines/corner normals
// are required. All referenced brushes must be supplied. Old connections and
// gap links are rebuilt. Outputs retain per-brush topology and real brush IDs;
// no cross-brush vertex welding, triangle renumbering or Unity ABI is provided.
ConvexGapRepairResult repairConvexGaps(const ConvexGapInput*, std::int32_t count,
    std::int32_t maxSplitsPerBrush = 128);

} // namespace realtimecsg::recovered
