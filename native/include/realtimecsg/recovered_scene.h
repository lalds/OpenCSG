#pragma once
#include "realtimecsg/recovered_convex_csg.h"

namespace realtimecsg::recovered {
// Versioned research bridge, NOT the original RealtimeCSG Unity ABI.
inline constexpr std::uint32_t kSceneAbiVersion=1;
#pragma pack(push,4)
struct SceneFloat2 { float x,y; };
struct SceneFloat4 { float x,y,z,w; };
struct SceneSurface { Plane plane; SceneFloat4 uvU,uvV; std::int32_t materialId; };
struct SceneHalfEdge { std::int32_t vertexIndex,twinIndex; };
struct ScenePolygon { std::int32_t firstEdge,edgeCount,surfaceIndex; };
struct SceneBrushInput {
    std::int32_t nodeIndex,brushIndex;
    const Vector3* vertices; std::int32_t vertexCount;
    const SceneHalfEdge* edges; std::int32_t edgeCount;
    const ScenePolygon* polygons; std::int32_t polygonCount;
    const SceneSurface* surfaces; std::int32_t surfaceCount;
};
struct SceneBuildInput {
    std::uint32_t abiVersion;
    std::int32_t rootNode;
    const CategorizationNode* nodes; std::int32_t nodeCount;
    const std::int32_t* children; std::int32_t childCount;
    const SceneBrushInput* brushes; std::int32_t brushCount;
    std::int32_t maxGapSplitsPerBrush,maxBrushVertices,maxBrushEdges,maxBrushPolygons;
};
struct SceneVertex { Vector3 position,normal; SceneFloat4 tangent; SceneFloat2 uv; };
struct SceneSubmesh { std::int32_t materialId,firstIndex,indexCount; };
struct SceneTriangleOrigin { std::int32_t brushIndex,surfaceIndex,materialId; std::uint16_t state,flags; };
struct SceneCounts {
    std::int32_t vertexCount,indexCount,submeshCount,triangleCount,brushCount;
    Vector3 boundsCenter,boundsSize;
};
struct SceneOutputBuffers {
    SceneVertex* vertices; std::int32_t vertexCapacity;
    std::int32_t* indices; std::int32_t indexCapacity;
    SceneSubmesh* submeshes; std::int32_t submeshCapacity;
    SceneTriangleOrigin* triangles; std::int32_t triangleCapacity;
};
#pragma pack(pop)
static_assert(sizeof(SceneVertex)==48 && sizeof(SceneSurface)==52 && sizeof(SceneCounts)==44);
static_assert(sizeof(SceneTriangleOrigin)==16 && sizeof(SceneSubmesh)==12 && sizeof(ScenePolygon)==12);
static_assert(sizeof(void*)!=8 || (sizeof(SceneBrushInput)==56 && sizeof(SceneBuildInput)==60 && sizeof(SceneOutputBuffers)==48));
struct SceneExportInput { const PreparedConvexBrush* brush; const SceneSurface* surfaces; std::int32_t surfaceCount; };
struct ExportedScene {
    std::vector<SceneVertex> vertices;
    std::vector<std::int32_t> indices;
    std::vector<SceneSubmesh> submeshes;
    std::vector<SceneTriangleOrigin> triangles;
    SceneCounts counts{};
};
// New attribute/export adapter: visible states only, reverse state 3 winding
// and normals, UV=dot(row.xyz,commonPosition)+row.w, material-sorted submeshes.
// One vertex per triangle corner retains normal/UV seams; no welding/smoothing.
// Tangents come from the UV gradients; degenerate UV uses an orthogonal fallback.
ExportedScene exportPreparedScene(const SceneExportInput*,std::int32_t count);
// New complete common-space build on owned scratch, with explicit per-brush
// capacities. Closed convex input, contiguous polygon edge spans and one input
// per brush tree node. Input data is borrowed only during this call.
ExportedScene buildConvexScene(const SceneBuildInput&);
}
extern "C" {
std::uint32_t rcsg_recovered_scene_abi_version();
// All calls except version/error return 0 success, -1 invalid/unsupported,
// -2 capacity/budget, -3 other failure. No C++ exceptions cross the C boundary.
// Build publishes an immutable native snapshot; output handle changes only on
// success. Handles are process-local integers, never raw pointers. Release once.
std::int32_t rcsg_recovered_scene_build(const realtimecsg::recovered::SceneBuildInput*,std::uint64_t* handle);
std::int32_t rcsg_recovered_scene_counts(std::uint64_t,realtimecsg::recovered::SceneCounts*);
// Full preflight before writing caller arrays. All four buffers must have
// enough capacity and must not overlap. Null+zero allowed for empty channels.
std::int32_t rcsg_recovered_scene_copy(std::uint64_t,const realtimecsg::recovered::SceneOutputBuffers*);
std::int32_t rcsg_recovered_scene_release(std::uint64_t);
// Thread-local UTF-8 diagnostic, valid until the next bridge operation on this thread.
const char* rcsg_recovered_scene_last_error();
}
