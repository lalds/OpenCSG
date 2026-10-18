#pragma once

#include "realtimecsg/csg_plugin_api.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <vector>

namespace RealtimeCSG {

struct StoredPolygon {
  int32_t firstEdge = 0;
  int32_t edgeCount = 0;
  int32_t polygonID = 0;
  SurfaceDescription surface{};
  SurfaceLayers layers{};
};

struct StoredBrushMesh {
  int32_t id = 0;
  int32_t userID = 0;
  std::vector<Vector3> vertices;
  std::vector<BrushMeshHalfEdge> halfEdges;
  std::vector<StoredPolygon> polygons;
};

struct StoredTransformOffset {
  int32_t id = 0;
  int32_t userID = 0;
  float localTransformation[16] = {1.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f,
                                   0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 1.f};
};

struct GeneratedSubmeshData {
  int32_t meshIndex = 0;
  int32_t subMeshIndex = 0;
  int32_t surfaceParameter = 0;
  std::vector<int32_t> indices;
  std::vector<Vector3> positions;
  std::vector<Vector4> tangents;
  std::vector<Vector3> normals;
  std::vector<Vector2> uvs;
  Vector3 boundsCenter{0.f, 0.f, 0.f};
  Vector3 boundsSize{0.f, 0.f, 0.f};
};

struct Node {
  int32_t nodeID = 0;
  int32_t userID = 0;
  CSGNodeType type = CSGNodeType::None;
  int32_t parentID = 0;
  int32_t treeID = 0;
  std::vector<int32_t> children;
  bool isDirty = true;

  // Branch / Brush properties
  CSGOperationType operation = CSGOperationType::Additive;

  // Brush properties
  CSGTreeBrushFlags flags = CSGTreeBrushFlags_Default;
  int32_t brushMeshID = 0;
  float localTransformation[16] = {1.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f,
                                   0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 1.f};
  AABB bounds{-1.f, 1.f, -1.f, 1.f, -1.f, 1.f};

  // Tree properties
  bool isEnabled = true;
  std::vector<GeneratedMeshDescription> cachedMeshDescriptions;
  std::vector<GeneratedSubmeshData> cachedSubmeshes;

  // Offsets & sync
  int32_t transformOffsetID = 0;
  int32_t synchronizedWithNodeID = 0;
};

class PluginManager {
public:
  static PluginManager &Instance();

  void RegisterUnityMethods(const UnityMethods *methods);
  void ClearUnityMethods();
  void ClearAll();
  void RebuildAll();
  void LogDiagnostics();

  // Node generation
  bool GenerateTree(int32_t userID, int32_t *outNodeID);
  bool GenerateBranch(int32_t userID, int32_t *outNodeID);
  bool GenerateBrush(int32_t userID, int32_t *outNodeID);
  bool DestroyNode(int32_t nodeID);
  bool DestroyNodes(int32_t nodeCount, const int32_t *nodeIDs);
  bool DestroyAllNodesWithUserID(int32_t userID);
  bool IsNodeIDValid(int32_t nodeID) const;
  CSGNodeType GetTypeOfNode(int32_t nodeID) const;
  int32_t GetUserIDOfNode(int32_t nodeID) const;
  int32_t GetNodeCount() const;
  int32_t GetTreeCount() const;
  int32_t GetBranchCount() const;
  int32_t GetBrushCount() const;
  bool GetAllTreeNodes(int32_t nodeCount, int32_t *allNodeIDs) const;
  bool GetAllTrees(int32_t nodeCount, int32_t *allNodeIDs) const;
  int32_t GetParentOfNode(int32_t nodeID) const;
  int32_t GetTreeOfNode(int32_t nodeID) const;
  int32_t FindTreeByUserID(int32_t userID) const;

  // Hierarchy
  bool AddChildNode(int32_t nodeID, int32_t childNodeID);
  bool InsertChildNode(int32_t nodeID, int32_t index, int32_t childNodeID);
  bool InsertChildNodeRange(int32_t nodeID, int32_t index, int32_t count,
                            const int32_t *nodes);
  bool RemoveChildNode(int32_t nodeID, int32_t childNodeID);
  bool RemoveChildNodeAt(int32_t nodeID, int32_t index);
  bool RemoveChildNodeRange(int32_t nodeID, int32_t index, int32_t count);
  bool ClearChildNodes(int32_t nodeID);
  int32_t GetChildNodeCount(int32_t nodeID) const;
  int32_t GetChildNodeAtIndex(int32_t nodeID, int32_t index) const;
  int32_t IndexOfChildNode(int32_t nodeID, int32_t childNodeID) const;
  bool SetChildNodes(int32_t nodeID, int32_t childCount,
                     const int32_t *childrenNodeIDs);
  bool GetChildNodes(int32_t nodeID, int32_t childCount, int32_t *children,
                     int32_t arrayIndex) const;
  int32_t GetNumberOfBrushesInTree(int32_t nodeID) const;
  bool DoesTreeContainBrush(int32_t nodeID, int32_t brushID) const;

  // Flags, Operations, Dirty
  bool IsNodeDirty(int32_t nodeID) const;
  bool SetDirty(int32_t nodeID);
  bool ClearDirty(int32_t nodeID);
  int32_t GetBranchOperationType(int32_t branchNodeID) const;
  bool SetBranchOperationType(int32_t branchNodeID, CSGOperationType operation);
  int32_t GetBrushOperationType(int32_t brushNodeID) const;
  bool SetBrushOperationType(int32_t brushNodeID, CSGOperationType operation);
  CSGTreeBrushFlags GetBrushFlags(int32_t brushNodeID) const;
  bool SetBrushFlags(int32_t brushNodeID, CSGTreeBrushFlags flags);
  bool SetTreeEnabled(int32_t modelNodeID, bool isEnabled);
  bool GetTreeEnabled(int32_t modelNodeID) const;

  // Transforms
  bool SetNodeLocalTransformation(int32_t brushNodeID,
                                  const float *transformation);
  bool GetNodeLocalTransformation(int32_t brushNodeID,
                                  float *outTransformation) const;
  bool GetInvNodeLocalTransformation(int32_t brushNodeID,
                                     float *outInvTransformation) const;
  bool GetNodeToTreeSpaceMatrix(int32_t nodeID, float *outMatrix) const;
  bool GetTreeToNodeSpaceMatrix(int32_t nodeID, float *outMatrix) const;
  bool GetBrushBounds(int32_t brushNodeID, AABB *bounds) const;

  // BrushMesh
  int32_t CreateBrushMesh(int32_t userID, int32_t vertexCount,
                          const Vector3 *vertices, int32_t halfEdgeCount,
                          const BrushMeshHalfEdge *halfEdges,
                          int32_t polygonCount,
                          const BrushMeshPolygon *polygons);
  bool UpdateBrushMesh(int32_t brushMeshIndex, int32_t vertexCount,
                       const Vector3 *vertices, int32_t halfEdgeCount,
                       const BrushMeshHalfEdge *halfEdges, int32_t polygonCount,
                       const BrushMeshPolygon *polygons);
  bool DestroyBrushMesh(int32_t brushMeshIndex);
  bool IsBrushMeshIDValid(int32_t brushMeshIndex) const;
  int32_t GetBrushMeshCount() const;
  int32_t GetBrushMeshUserID(int32_t brushMeshIndex) const;
  bool GetAllBrushMeshInstances(int32_t nodeCount, int32_t *allNodeIDs) const;
  bool SetBrushMeshID(int32_t brushNodeID, int32_t brushMeshID);
  int32_t GetBrushMeshID(int32_t brushNodeID) const;

  // Mesh Generation
  bool UpdateAllTreeMeshes();
  bool GenerateMeshDescriptions(int32_t treeNodeID, int32_t meshTypeCount,
                                const MeshQuery *meshTypes,
                                VertexChannelFlags vertexChannelMask,
                                int32_t *meshDescriptionCount);
  bool GetMeshDescriptions(int32_t treeNodeID, int32_t meshDescriptionCount,
                           GeneratedMeshDescription *meshDescriptions);
  bool GetGeneratedMesh(int32_t treeNodeID, int32_t meshIndex,
                        int32_t subMeshIndex, int32_t indexCount,
                        int32_t *indices, int32_t vertexCount,
                        Vector3 *positions, Vector4 *tangents, Vector3 *normals,
                        Vector2 *uvs, Vector3 *boundsCenter,
                        Vector3 *boundsSize);

  // Transform offsets
  int32_t GenerateTransformOffset(int32_t userID);
  bool DestroyTransformOffset(int32_t transformOffsetID);
  bool DestroyTransformOffsets(int32_t count,
                               const int32_t *transformOffsetIDs);
  bool IsTransformOffsetIDValid(int32_t transformOffsetID) const;
  int32_t GetTransformOffsetCount() const;
  int32_t GetUserIDOfTransformOffset(int32_t transformOffsetID) const;
  bool GetAllTransformOffsets(int32_t count, int32_t *transformOffsetIDs) const;
  bool SetTransformOffsetLocalTransformation(int32_t transformOffsetID,
                                             const float *transform);
  bool GetTransformOffsetLocalTransformation(int32_t transformOffsetID,
                                             float *transform) const;
  bool SetNodeTransformOffset(int32_t nodeID, int32_t transformOffsetID);
  int32_t GetNodeTransformOffset(int32_t nodeID) const;
  bool SetNodeSynchronizedWith(int32_t nodeID, int32_t targetNodeID);
  int32_t GetNodeSynchronizedWith(int32_t nodeID) const;
  int32_t GetNodeSynchronizedOrigin(int32_t nodeID) const;
  int32_t GetAllSynchronizedBrushVariantCount(int32_t nodeID) const;
  bool GetAllSynchronizedBrushVariants(int32_t nodeID, int32_t count,
                                       int32_t *variantNodeIDs) const;

private:
  friend struct PluginAccess;
  PluginManager() = default;
  void UpdateTreeHierarchy(int32_t nodeID, int32_t treeID);
  void MarkAncestorsDirty(int32_t nodeID);
  void UpdateBounds(Node &brushNode);

  mutable std::mutex mutex_;
  UnityMethods unityMethods_{};
  bool unityMethodsRegistered_ = false;

  int32_t nextNodeID_ = 1;
  int32_t nextBrushMeshID_ = 1;
  int32_t nextTransformOffsetID_ = 1;

  std::unordered_map<int32_t, Node> nodes_;
  std::unordered_map<int32_t, StoredBrushMesh> brushMeshes_;
  std::unordered_map<int32_t, StoredTransformOffset> transformOffsets_;
};

} // namespace RealtimeCSG
