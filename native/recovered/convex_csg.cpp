#include "realtimecsg/recovered_convex_csg.h"

#include <cmath>
#include <stdexcept>

namespace realtimecsg::recovered {
namespace {
// Mach-O Categorize: normal comparison at 0x3c574, distance at 0x3c564.
constexpr float normalToleranceSquared = 1.0000000116860974e-7f;
constexpr float distanceTolerance = 4.999999873689376e-6f;
bool closePlane(float x, float y, float z, float distance) {
    return ((x*x + y*y) + z*z) < normalToleranceSquared
        && std::abs(distance) < distanceTolerance;
}

CategoryIndex clipFragment(BrushMemoryView& memory, const SurfaceView& surfaces,
                          int polygon, const ConvexBrushView& clipper,
                          std::uint16_t outsideState) {
    CategoryIndex category = CategoryIndex::Inside;
    for (int p = 0; p < clipper.planeCount; ++p) {
        const auto& plane = clipper.planes[p];
        const auto relation = coplanarRelation(surfaces.planes[memory.polygons[polygon].surfaceIndex], plane);
        if (relation != PlaneRelation::Separate) {
            category = relation == PlaneRelation::Aligned ? CategoryIndex::Aligned : CategoryIndex::RevAligned;
            continue;
        }
        const auto result = alignment(memory.view(), surfaces, polygon, plane);
        if (result == PolygonSplitResult::CompletelyOutside) return CategoryIndex::Outside;
        if (result == PolygonSplitResult::Split) {
            int outside = -1;
            const auto split = splitByPlane(memory, surfaces, polygon, plane, outside);
            if (split == PolygonSplitResult::CompletelyOutside) return CategoryIndex::Outside;
            if (split == PolygonSplitResult::Split) {
                memory.polygons[outside].categorizationState = outsideState;
            } else if (split != PolygonSplitResult::CompletelyInside) {
                throw std::runtime_error("Unsupported degenerate convex split");
            }
        } else if (result == PolygonSplitResult::PlaneAligned) {
            category = CategoryIndex::Aligned;
        } else if (result == PolygonSplitResult::PlaneOppositeAligned) {
            category = CategoryIndex::RevAligned;
        } else if (result != PolygonSplitResult::CompletelyInside) {
            throw std::runtime_error("Unsupported degenerate convex alignment");
        }
    }
    return category;
}

bool containsPoint(const BrushMemoryView& memory, int polygon, const Vector3& normal, const Vector3& point) {
    const auto& poly = memory.polygons[polygon];
    int edge = poly.firstEdge;
    for (int i = 0; i < poly.edgeCount; ++i) {
        const auto& a = memory.vertices[memory.edges[edge].vertexIndex];
        edge = memory.edges[edge].nextEdge;
        const auto& b = memory.vertices[memory.edges[edge].vertexIndex];
        // Double precision here belongs to new cleanup, not recovered float code.
        const double x = double(b.x)-a.x, y = double(b.y)-a.y, z = double(b.z)-a.z;
        const double px = double(point.x)-a.x, py = double(point.y)-a.y, pz = double(point.z)-a.z;
        const double length = std::sqrt(x*x+y*y+z*z);
        const double side = (y*pz-z*py)*normal.x + (z*px-x*pz)*normal.y + (x*py-y*px)*normal.z;
        if (side < -1e-6*length) return false;
    }
    return true;
}
bool containsPolygon(const CategorizedBrushView& outer, int a, const CategorizedBrushView& inner, int b) {
    const auto& poly = inner.memory->polygons[b];
    const auto& normal = outer.surfaces->planes[outer.memory->polygons[a].surfaceIndex].normal;
    int edge = poly.firstEdge;
    for (int i = 0; i < poly.edgeCount; ++i) {
        if (!containsPoint(*outer.memory, a, normal, inner.memory->vertices[inner.memory->edges[edge].vertexIndex]))
            return false;
        edge = inner.memory->edges[edge].nextEdge;
    }
    return true;
}
Plane orientedPlane(const CategorizedBrushView& brush, int p) {
    const auto& poly = brush.memory->polygons[p];
    auto plane = brush.surfaces->planes[poly.surfaceIndex];
    if (poly.categorizationState == 3) {
        plane.normal = {-plane.normal.x,-plane.normal.y,-plane.normal.z};
        plane.distance = -plane.distance;
    }
    return plane;
}
} // namespace

PlaneRelation coplanarRelation(const Plane& a, const Plane& b) {
    // Original checks opposite orientation first (0x20140..0x2034b).
    if (closePlane(a.normal.x+b.normal.x, a.normal.y+b.normal.y, a.normal.z+b.normal.z,
                   a.distance+b.distance)) return PlaneRelation::OppositeAligned;
    if (closePlane(a.normal.x-b.normal.x, a.normal.y-b.normal.y, a.normal.z-b.normal.z,
                   a.distance-b.distance)) return PlaneRelation::Aligned;
    return PlaneRelation::Separate;
}

void categorizeConvexFragments(BrushMemoryView& memory, const SurfaceView& surfaces, int sourcePlaneCount,
                              const CategorizationProgram& program, const ConvexBrushView* clippers,
                              int clipperCount) {
    if (!surfaces.planes || sourcePlaneCount <= 0 || clipperCount < 0
        || (!clippers && clipperCount != 0)) throw std::invalid_argument("Invalid convex plane views");
    for (int p = 0; p < memory.polygonCount; ++p)
        if (memory.polygons[p].surfaceIndex < 0 || memory.polygons[p].surfaceIndex >= sourcePlaneCount)
            throw std::invalid_argument("Invalid source surface index");
    for (const auto& row : program.rows)
        if (row.brushIndex < 0 || row.brushIndex >= clipperCount
            || !clippers[row.brushIndex].planes || clippers[row.brushIndex].planeCount <= 0)
            throw std::invalid_argument("Missing convex clipper for program dependency");

    for (int p = 0; p < memory.polygonCount; ++p)
        memory.polygons[p].categorizationState = program.initialState;
    std::size_t first = 0;
    while (first < program.rows.size()) {
        std::size_t end = first + 1;
        const int brush = program.rows[first].brushIndex;
        while (end < program.rows.size() && program.rows[end].brushIndex == brush) ++end;
        // New outside fragments already have this stage's successor. Do not run
        // them through this brush again. Twin-edge insertions preserve other faces.
        const int stagePolygonCount = memory.polygonCount;
        for (int p = 0; p < stagePolygonCount; ++p) {
            const auto state = memory.polygons[p].categorizationState;
            for (std::size_t r = first; r < end; ++r) {
                const auto& row = program.rows[r];
                if (row.state != state) continue;
                const auto category = clipFragment(memory, surfaces, p, clippers[brush], row.next[3]);
                memory.polygons[p].categorizationState = row.next[static_cast<unsigned>(category)];
                break;
            }
        }
        first = end;
    }
    for (int p = 0; p < memory.polygonCount; ++p)
        if (memory.polygons[p].categorizationState < 1 || memory.polygons[p].categorizationState > 3)
            throw std::runtime_error("Unresolved convex fragment state");
}

std::int32_t removeOppositeCoincidentFragments(const CategorizedBrushView* brushes, int count) {
    if (count < 0 || (!brushes && count != 0)) throw std::invalid_argument("Invalid categorized brush views");
    for (int b = 0; b < count; ++b)
        if (!brushes[b].memory || !brushes[b].surfaces || !brushes[b].surfaces->planes)
            throw std::invalid_argument("Missing categorized mesh or planes");
    int removed = 0;
    for (int a = 0; a < count; ++a) {
        for (int p = 0; p < brushes[a].memory->polygonCount; ++p) {
            auto& first = brushes[a].memory->polygons[p];
            if (first.categorizationState == 1) continue;
            bool matched = false;
            for (int b = a+1; b < count && !matched; ++b) {
                for (int q = 0; q < brushes[b].memory->polygonCount; ++q) {
                    auto& second = brushes[b].memory->polygons[q];
                    if (second.categorizationState == 1) continue;
                    if (coplanarRelation(orientedPlane(brushes[a],p),orientedPlane(brushes[b],q))
                            != PlaneRelation::OppositeAligned) continue;
                    if (!containsPolygon(brushes[a],p,brushes[b],q)
                        || !containsPolygon(brushes[b],q,brushes[a],p)) continue;
                    first.categorizationState = second.categorizationState = 1;
                    removed += 2;
                    matched = true;
                    break;
                }
            }
        }
    }
    return removed;
}
} // namespace realtimecsg::recovered
