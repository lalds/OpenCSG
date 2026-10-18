#pragma once

#include <cstddef>
#include <cstdint>

#if defined(_WIN32) || defined(__CYGWIN__)
#if defined(REALTIMECSG_EXPORT_DLL)
#define RCSG_EXPORT __declspec(dllexport)
#else
#define RCSG_EXPORT __declspec(dllimport)
#endif
#else
#if defined(__GNUC__) && __GNUC__ >= 4
#define RCSG_EXPORT __attribute__((visibility("default")))
#else
#define RCSG_EXPORT
#endif
#endif

#pragma pack(push, 4)

enum class CSGNodeType : uint8_t { None = 0, Tree = 1, Branch = 2, Brush = 3 };

enum class CSGOperationType : uint8_t {
  Additive = 0,
  Subtractive = 1,
  Intersecting = 2
};

enum CSGTreeBrushFlags : int32_t {
  CSGTreeBrushFlags_Default = 0,
  CSGTreeBrushFlags_Infinite = 1
};

enum SurfaceFlags : int32_t {
  SurfaceFlags_None = 0,
  SurfaceFlags_TextureIsInWorldSpace = 1
};

enum LayerUsageFlags : int32_t {
  LayerUsageFlags_None = 0,
  LayerUsageFlags_Renderable = 1 << 0,
  LayerUsageFlags_CastShadows = 1 << 1,
  LayerUsageFlags_ReceiveShadows = 1 << 2,
  LayerUsageFlags_Collidable = 1 << 3,
  LayerUsageFlags_RenderCastShadows = LayerUsageFlags_Renderable |
      LayerUsageFlags_CastShadows,
  LayerUsageFlags_RenderReceiveShadows = LayerUsageFlags_Renderable |
      LayerUsageFlags_ReceiveShadows,
  LayerUsageFlags_RenderReceiveCastShadows = LayerUsageFlags_Renderable |
      LayerUsageFlags_CastShadows | LayerUsageFlags_ReceiveShadows,
  LayerUsageFlags_Culled = 1 << 23
};

enum VertexChannelFlags : uint8_t {
  VertexChannelFlags_None = 0,
  VertexChannelFlags_Position = 0,
  VertexChannelFlags_Normal = 1 << 2,
  VertexChannelFlags_Tangent = 1 << 1,
  VertexChannelFlags_UV0 = 1 << 3,
  VertexChannelFlags_All = VertexChannelFlags_Position |
      VertexChannelFlags_Normal | VertexChannelFlags_Tangent |
      VertexChannelFlags_UV0
};

struct Vector2 {
  float x;
  float y;
};

struct Vector3 {
  float x;
  float y;
  float z;
};

struct Vector4 {
  float x;
  float y;
  float z;
  float w;
};

struct Matrix4x4 {
  float values[16];
};

struct Plane {
  Vector3 normal;
  float distance;
};

struct AABB {
  float minX, maxX;
  float minY, maxY;
  float minZ, maxZ;
};

struct UVMatrix {
  Vector4 U;
  Vector4 V;
};

struct SurfaceDescription {
  uint32_t smoothingGroup;
  SurfaceFlags surfaceFlags;
  UVMatrix UV0;
};

struct SurfaceLayers {
  LayerUsageFlags layerUsage;
  int32_t layerParameter1;
  int32_t layerParameter2;
  int32_t layerParameter3;
};

struct BrushMeshHalfEdge {
  int32_t vertexIndex;
  int32_t twinIndex;
};

struct BrushMeshPolygon {
  int32_t firstEdge;
  int32_t edgeCount;
  int32_t polygonID;
  SurfaceDescription surface;
  SurfaceLayers layers;
};

struct MeshQuery {
  uint32_t layers;
  uint32_t maskAndChannels;
};

struct CSGSurfaceIntersection {
  Plane localPlane;
  Plane modelPlane;
  Plane worldPlane;
  Vector3 worldIntersection;
  Vector2 surfaceIntersection;
  float distance;
};

struct CSGTreeBrushIntersection {
  int32_t treeNodeID;
  int32_t brushNodeID;
  int32_t surfaceID;
  int32_t brushUserID;
  CSGSurfaceIntersection intersection;
};

typedef void (*StringLogFn)(const char *text, int32_t uniqueObjectID);
typedef const char *(*ReturnStringMethodFn)(int32_t uniqueObjectID);

struct UnityMethods {
  StringLogFn debugLog;
  StringLogFn debugLogError;
  StringLogFn debugLogWarning;
  ReturnStringMethodFn nameForUserID;
};

#pragma pack(pop)

#pragma pack(push, 8)
struct GeneratedMeshDescription {
  MeshQuery meshQuery;
  int32_t surfaceParameter;
  int32_t meshQueryIndex;
  int32_t subMeshQueryIndex;
  int64_t geometryHashValue;
  int64_t surfaceHashValue;
  int32_t vertexCount;
  int32_t indexCount;
};
#pragma pack(pop)

extern "C" {

// Diagnostics and registration
RCSG_EXPORT int32_t HasBeenCompiledInDebugMode();
RCSG_EXPORT void RegisterMethods(const UnityMethods *unityMethods);
RCSG_EXPORT void ClearMethods();
RCSG_EXPORT void ClearAllNodes();
RCSG_EXPORT void RebuildAll();
RCSG_EXPORT void LogDiagnostics();

// Tree / Branch / Brush generation and deletion
RCSG_EXPORT int32_t GenerateTree(int32_t userID, int32_t *generatedTreeNodeID);
RCSG_EXPORT int32_t GenerateBranch(int32_t userID,
                                   int32_t *generatedOperationNodeID);
RCSG_EXPORT int32_t GenerateBrush(int32_t userID, int32_t *generatedNodeID);
RCSG_EXPORT int32_t DestroyNode(int32_t nodeID);
RCSG_EXPORT int32_t DestroyNodes(int32_t nodeCount, const int32_t *nodeIDs);
RCSG_EXPORT int32_t DestroyAllNodesWithUserID(int32_t userID);
RCSG_EXPORT int32_t IsNodeIDValid(int32_t nodeID);
RCSG_EXPORT CSGNodeType GetTypeOfNode(int32_t nodeID);
RCSG_EXPORT int32_t GetUserIDOfNode(int32_t nodeID);
RCSG_EXPORT int32_t GetNodeCount();
RCSG_EXPORT int32_t GetTreeCount();
RCSG_EXPORT int32_t GetBranchCount();
RCSG_EXPORT int32_t GetBrushCount();
RCSG_EXPORT int32_t GetAllTreeNodes(int32_t nodeCount, int32_t *allNodeIDs);
RCSG_EXPORT int32_t GetAllTrees(int32_t nodeCount, int32_t *allNodeIDs);
RCSG_EXPORT int32_t GetParentOfNode(int32_t nodeID);
RCSG_EXPORT int32_t GetTreeOfNode(int32_t nodeID);
RCSG_EXPORT int32_t FindTreeByUserID(int32_t userID);

// Hierarchy / children management
RCSG_EXPORT int32_t AddChildNode(int32_t nodeID, int32_t childNodeID);
RCSG_EXPORT int32_t InsertChildNode(int32_t nodeID, int32_t index,
                                    int32_t childNodeID);
RCSG_EXPORT int32_t InsertChildNodeRange(int32_t nodeID, int32_t index,
                                         int32_t count, const int32_t *nodes);
RCSG_EXPORT int32_t RemoveChildNode(int32_t nodeID, int32_t childNodeID);
RCSG_EXPORT int32_t RemoveChildNodeAt(int32_t nodeID, int32_t index);
RCSG_EXPORT int32_t RemoveChildNodeRange(int32_t nodeID, int32_t index,
                                         int32_t count);
RCSG_EXPORT int32_t ClearChildNodes(int32_t nodeID);
RCSG_EXPORT int32_t GetChildNodeCount(int32_t nodeID);
RCSG_EXPORT int32_t GetChildNodeAtIndex(int32_t nodeID, int32_t index);
RCSG_EXPORT int32_t IndexOfChildNode(int32_t nodeID, int32_t childNodeID);
RCSG_EXPORT int32_t SetChildNodes(int32_t nodeID, int32_t childCount,
                                  const int32_t *childrenNodeIDs);
RCSG_EXPORT int32_t GetChildNodes(int32_t nodeID, int32_t childCount,
                                  int32_t *children, int32_t arrayIndex);
RCSG_EXPORT int32_t GetNumberOfBrushesInTree(int32_t nodeID);
RCSG_EXPORT int32_t DoesTreeContainBrush(int32_t nodeID, int32_t brushID);

// Flags, Operations, Dirty
RCSG_EXPORT int32_t IsNodeDirty(int32_t nodeID);
RCSG_EXPORT int32_t SetDirty(int32_t nodeID);
RCSG_EXPORT int32_t ClearDirty(int32_t nodeID);
RCSG_EXPORT int32_t GetBranchOperationType(int32_t branchNodeID);
RCSG_EXPORT int32_t SetBranchOperationType(int32_t branchNodeID,
                                           CSGOperationType operation);
RCSG_EXPORT int32_t GetBrushOperationType(int32_t brushNodeID);
RCSG_EXPORT int32_t SetBrushOperationType(int32_t brushNodeID,
                                          CSGOperationType operation);
RCSG_EXPORT CSGTreeBrushFlags GetBrushFlags(int32_t brushNodeID);
RCSG_EXPORT int32_t SetBrushFlags(int32_t brushNodeID, CSGTreeBrushFlags flags);
RCSG_EXPORT int32_t SetTreeEnabled(int32_t modelNodeID, int32_t isEnabled);
RCSG_EXPORT int32_t GetTreeEnabled(int32_t modelNodeID);

// Transformations
RCSG_EXPORT int32_t SetNodeLocalTransformation(int32_t brushNodeID,
                                               const float *brushToTreeSpace);
RCSG_EXPORT int32_t GetNodeLocalTransformation(int32_t brushNodeID,
                                               float *brushToTreeSpace);
RCSG_EXPORT int32_t GetInvNodeLocalTransformation(int32_t brushNodeID,
                                                  float *invTransformation);
RCSG_EXPORT int32_t GetNodeToTreeSpaceMatrix(int32_t nodeID, float *matrix);
RCSG_EXPORT int32_t GetTreeToNodeSpaceMatrix(int32_t nodeID, float *matrix);
RCSG_EXPORT int32_t GetBrushBounds(int32_t brushNodeID, AABB *bounds);

// BrushMesh storage
RCSG_EXPORT int32_t CreateBrushMesh(int32_t userID, int32_t vertexCount,
                                    const Vector3 *vertices,
                                    int32_t halfEdgeCount,
                                    const BrushMeshHalfEdge *halfEdges,
                                    int32_t polygonCount,
                                    const BrushMeshPolygon *polygons);
RCSG_EXPORT int32_t UpdateBrushMesh(int32_t brushMeshIndex, int32_t vertexCount,
                                    const Vector3 *vertices,
                                    int32_t halfEdgeCount,
                                    const BrushMeshHalfEdge *halfEdges,
                                    int32_t polygonCount,
                                    const BrushMeshPolygon *polygons);
RCSG_EXPORT int32_t DestroyBrushMesh(int32_t brushMeshIndex);
RCSG_EXPORT int32_t IsBrushMeshIDValid(int32_t brushMeshIndex);
RCSG_EXPORT int32_t GetBrushMeshCount();
RCSG_EXPORT int32_t GetBrushMeshUserID(int32_t brushMeshIndex);
RCSG_EXPORT int32_t GetAllBrushMeshInstances(int32_t nodeCount,
                                             int32_t *allNodeIDs);
RCSG_EXPORT int32_t SetBrushMeshID(int32_t brushNodeID, int32_t brushMeshID);
RCSG_EXPORT int32_t GetBrushMeshID(int32_t brushNodeID);

// Mesh Generation
RCSG_EXPORT int32_t UpdateAllTreeMeshes();
RCSG_EXPORT int32_t GenerateMeshDescriptions(
    int32_t treeNodeID, int32_t meshTypeCount, const MeshQuery *meshTypes,
    VertexChannelFlags vertexChannelMask, int32_t *meshDescriptionCount);
RCSG_EXPORT int32_t
GetMeshDescriptions(int32_t treeNodeID, int32_t meshDescriptionCount,
                    GeneratedMeshDescription *meshDescriptions);
RCSG_EXPORT int32_t GetGeneratedMesh(int32_t treeNodeID, int32_t meshIndex,
                                     int32_t subMeshIndex, int32_t indexCount,
                                     int32_t *indices, int32_t vertexCount,
                                     Vector3 *positions, Vector4 *tangents,
                                     Vector3 *normals, Vector2 *uvs,
                                     Vector3 *boundsCenter,
                                     Vector3 *boundsSize);

// Picking / Raycast / Frustum
RCSG_EXPORT int32_t RayCastIntoBrush(
    int32_t brushNodeID, const Vector3 *rayStart, const Vector3 *rayEnd,
    const float *modelLocalToWorldMatrix, int32_t ignoreInvisiblePolygons,
    CSGTreeBrushIntersection *outputBrushIntersection);
RCSG_EXPORT int32_t RayCastIntoBrushSurface(
    int32_t brushNodeID, int32_t surfaceID, const Vector3 *rayStart,
    const Vector3 *rayEnd, const float *modelLocalToWorldMatrix,
    CSGSurfaceIntersection *outputSurfaceIntersection);
RCSG_EXPORT int32_t RayCastIntoTreeMultiCount(
    int32_t modelNodeID, const Vector3 *worldRayStart,
    const Vector3 *worldRayEnd, const float *modelLocalToWorldMatrix,
    int32_t in_filterLayerParameter0, int32_t ignoreInvisiblePolygons,
    const int32_t *ignoreNodeIDs, int32_t ignoreNodeIDCount);
RCSG_EXPORT int32_t RayCastMultiGet(
    int32_t objectCount, CSGTreeBrushIntersection *outputBrushIntersection);
RCSG_EXPORT int32_t FindNodesInFrustum(int32_t modelNodeID, int32_t planeCount,
                                       const Plane *planes);
RCSG_EXPORT int32_t RetrieveUserIDsInFrustum(int32_t objectCount,
                                             int32_t *objectIDs);
RCSG_EXPORT int32_t RetrieveNodesInFrustum(int32_t objectCount,
                                           int32_t *nodeIDs);

// Outlines
RCSG_EXPORT uint64_t GetBrushOutlineGeneration(int32_t brushNodeID);
RCSG_EXPORT int32_t GetBrushOutlineSizes(
    int32_t brushNodeID, int32_t *vertexCount, int32_t *visibleOuterLineCount,
    int32_t *visibleInnerLineCount, int32_t *invisibleOuterLineCount,
    int32_t *invisibleInnerLineCount, int32_t *invalidLineCount);
RCSG_EXPORT int32_t GetBrushOutlineValues(
    int32_t brushNodeID, int32_t vertexCount, Vector3 *vertices,
    int32_t visibleOuterLineCount, int32_t *visibleOuterLines,
    int32_t visibleInnerLineCount, int32_t *visibleInnerLines,
    int32_t invisibleOuterLineCount, int32_t *invisibleOuterLines,
    int32_t invisibleInnerLineCount, int32_t *invisibleInnerLines,
    int32_t invalidLineCount, int32_t *invalidLines);
RCSG_EXPORT int32_t GetSurfaceOutlineSizes(
    int32_t brushNodeID, int32_t surfaceID, int32_t *vertexCount,
    int32_t *visibleOuterLineCount, int32_t *visibleInnerLineCount,
    int32_t *visibleTriangleCount, int32_t *invisibleOuterLineCount,
    int32_t *invisibleInnerLineCount, int32_t *invalidLineCount);
RCSG_EXPORT int32_t GetSurfaceOutlineValues(
    int32_t brushNodeID, int32_t surfaceID, int32_t vertexCount,
    Vector3 *vertices, int32_t visibleOuterLineCount,
    int32_t *visibleOuterLines, int32_t visibleInnerLineCount,
    int32_t *visibleInnerLines, int32_t visibleTriangleCount,
    int32_t *visibleTriangles, int32_t invisibleOuterLineCount,
    int32_t *invisibleOuterLines, int32_t invisibleInnerLineCount,
    int32_t *invisibleInnerLines, int32_t invalidLineCount,
    int32_t *invalidLines);

// Decomposition & UV coordinates
RCSG_EXPORT int32_t DecomposeStart(int32_t vertexCount, const Vector2 *vertices,
                                   int32_t *polygonCount);
RCSG_EXPORT int32_t DecomposeGetSizes(int32_t polygonCount,
                                      int32_t *polygonSizes);
RCSG_EXPORT int32_t DecomposeGetPolygon(int32_t polygonIndex,
                                        int32_t vertexSize, Vector2 *vertices);
RCSG_EXPORT int32_t ConvertWorldToTextureCoord(
    int32_t brushNodeID, int32_t surfaceID, Matrix4x4 modelLocalToWorldMatrix,
    Vector3 worldCoordinate, Vector2 *textureCoordinate);
RCSG_EXPORT int32_t ConvertTextureToWorldCoord(
    int32_t brushNodeID, int32_t surfaceID, float textureCoordinateU,
    float textureCoordinateV, const float *modelLocalToWorldMatrix,
    float *worldCoordinateX, float *worldCoordinateY, float *worldCoordinateZ);
RCSG_EXPORT int32_t GetSurfaceMinMaxTexCoords(int32_t brushNodeID,
                                              int32_t surfaceID,
                                              Matrix4x4 modelLocalToWorldMatrix,
                                              Vector2 *minTextureCoordinate,
                                              Vector2 *maxTextureCoordinate);

// Transform Offsets & Synchronized variants
RCSG_EXPORT int32_t GenerateTransformOffset(int32_t userID);
RCSG_EXPORT int32_t DestroyTransformOffset(int32_t transformOffsetID);
RCSG_EXPORT int32_t DestroyTransformOffsets(int32_t count,
                                            const int32_t *transformOffsetIDs);
RCSG_EXPORT int32_t IsTransformOffsetIDValid(int32_t transformOffsetID);
RCSG_EXPORT int32_t GetTransformOffsetCount();
RCSG_EXPORT int32_t GetUserIDOfTransformOffset(int32_t transformOffsetID);
RCSG_EXPORT int32_t GetAllTransformOffsets(int32_t count,
                                           int32_t *transformOffsetIDs);
RCSG_EXPORT int32_t SetTransformOffsetLocalTransformation(
    int32_t transformOffsetID, const float *transform);
RCSG_EXPORT int32_t GetTransformOffsetLocalTransformation(
    int32_t transformOffsetID, float *transform);
RCSG_EXPORT int32_t SetNodeTransformOffset(int32_t nodeID,
                                           int32_t transformOffsetID);
RCSG_EXPORT int32_t GetNodeTransformOffset(int32_t nodeID);
RCSG_EXPORT int32_t SetNodeSynchronizedWith(int32_t nodeID,
                                            int32_t targetNodeID);
RCSG_EXPORT int32_t GetNodeSynchronizedWith(int32_t nodeID);
RCSG_EXPORT int32_t GetNodeSynchronizedOrigin(int32_t nodeID);
RCSG_EXPORT int32_t GetAllSynchronizedBrushVariantCount(int32_t nodeID);
RCSG_EXPORT int32_t GetAllSynchronizedBrushVariants(int32_t nodeID,
                                                    int32_t count,
                                                    int32_t *variantNodeIDs);

} // extern "C"
