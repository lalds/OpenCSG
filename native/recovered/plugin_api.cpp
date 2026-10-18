#include "plugin_manager.h"
// C# unannotated bool P/Invoke uses 32-bit BOOL at this boundary.
using RealtimeCSG::PluginManager;
extern "C" {
int32_t HasBeenCompiledInDebugMode() { return 0; }
void RegisterMethods(const UnityMethods *unityMethods) {
  try {
    PluginManager::Instance().RegisterUnityMethods(unityMethods);
  } catch (...) {
    return;
  }
}
void ClearMethods() {
  try {
    PluginManager::Instance().ClearUnityMethods();
  } catch (...) {
    return;
  }
}
void ClearAllNodes() {
  try {
    PluginManager::Instance().ClearAll();
  } catch (...) {
    return;
  }
}
void RebuildAll() {
  try {
    PluginManager::Instance().RebuildAll();
  } catch (...) {
    return;
  }
}
void LogDiagnostics() {
  try {
    PluginManager::Instance().LogDiagnostics();
  } catch (...) {
    return;
  }
}
int32_t GenerateTree(int32_t userID, int32_t *generatedTreeNodeID) {
  try {
    return PluginManager::Instance().GenerateTree(userID, generatedTreeNodeID);
  } catch (...) {
    return 0;
  }
}
int32_t GenerateBranch(int32_t userID, int32_t *generatedOperationNodeID) {
  try {
    return PluginManager::Instance().GenerateBranch(userID,
                                                    generatedOperationNodeID);
  } catch (...) {
    return 0;
  }
}
int32_t GenerateBrush(int32_t userID, int32_t *generatedNodeID) {
  try {
    return PluginManager::Instance().GenerateBrush(userID, generatedNodeID);
  } catch (...) {
    return 0;
  }
}
int32_t DestroyNode(int32_t nodeID) {
  try {
    return PluginManager::Instance().DestroyNode(nodeID);
  } catch (...) {
    return 0;
  }
}
int32_t DestroyNodes(int32_t nodeCount, const int32_t *nodeIDs) {
  try {
    return PluginManager::Instance().DestroyNodes(nodeCount, nodeIDs);
  } catch (...) {
    return 0;
  }
}
int32_t DestroyAllNodesWithUserID(int32_t userID) {
  try {
    return PluginManager::Instance().DestroyAllNodesWithUserID(userID);
  } catch (...) {
    return 0;
  }
}
int32_t IsNodeIDValid(int32_t nodeID) {
  try {
    return PluginManager::Instance().IsNodeIDValid(nodeID);
  } catch (...) {
    return 0;
  }
}
CSGNodeType GetTypeOfNode(int32_t nodeID) {
  try {
    return PluginManager::Instance().GetTypeOfNode(nodeID);
  } catch (...) {
    return CSGNodeType::None;
  }
}
int32_t GetUserIDOfNode(int32_t nodeID) {
  try {
    return PluginManager::Instance().GetUserIDOfNode(nodeID);
  } catch (...) {
    return 0;
  }
}
int32_t GetNodeCount() {
  try {
    return PluginManager::Instance().GetNodeCount();
  } catch (...) {
    return 0;
  }
}
int32_t GetTreeCount() {
  try {
    return PluginManager::Instance().GetTreeCount();
  } catch (...) {
    return 0;
  }
}
int32_t GetBranchCount() {
  try {
    return PluginManager::Instance().GetBranchCount();
  } catch (...) {
    return 0;
  }
}
int32_t GetBrushCount() {
  try {
    return PluginManager::Instance().GetBrushCount();
  } catch (...) {
    return 0;
  }
}
int32_t GetAllTreeNodes(int32_t nodeCount, int32_t *allNodeIDs) {
  try {
    return PluginManager::Instance().GetAllTreeNodes(nodeCount, allNodeIDs);
  } catch (...) {
    return 0;
  }
}
int32_t GetAllTrees(int32_t nodeCount, int32_t *allNodeIDs) {
  try {
    return PluginManager::Instance().GetAllTrees(nodeCount, allNodeIDs);
  } catch (...) {
    return 0;
  }
}
int32_t GetParentOfNode(int32_t nodeID) {
  try {
    return PluginManager::Instance().GetParentOfNode(nodeID);
  } catch (...) {
    return 0;
  }
}
int32_t GetTreeOfNode(int32_t nodeID) {
  try {
    return PluginManager::Instance().GetTreeOfNode(nodeID);
  } catch (...) {
    return 0;
  }
}
int32_t FindTreeByUserID(int32_t userID) {
  try {
    return PluginManager::Instance().FindTreeByUserID(userID);
  } catch (...) {
    return 0;
  }
}
int32_t AddChildNode(int32_t nodeID, int32_t childNodeID) {
  try {
    return PluginManager::Instance().AddChildNode(nodeID, childNodeID);
  } catch (...) {
    return 0;
  }
}
int32_t InsertChildNode(int32_t nodeID, int32_t index, int32_t childNodeID) {
  try {
    return PluginManager::Instance().InsertChildNode(nodeID, index,
                                                     childNodeID);
  } catch (...) {
    return 0;
  }
}
int32_t InsertChildNodeRange(int32_t nodeID, int32_t index, int32_t count,
                             const int32_t *nodes) {
  try {
    return PluginManager::Instance().InsertChildNodeRange(nodeID, index, count,
                                                          nodes);
  } catch (...) {
    return 0;
  }
}
int32_t RemoveChildNode(int32_t nodeID, int32_t childNodeID) {
  try {
    return PluginManager::Instance().RemoveChildNode(nodeID, childNodeID);
  } catch (...) {
    return 0;
  }
}
int32_t RemoveChildNodeAt(int32_t nodeID, int32_t index) {
  try {
    return PluginManager::Instance().RemoveChildNodeAt(nodeID, index);
  } catch (...) {
    return 0;
  }
}
int32_t RemoveChildNodeRange(int32_t nodeID, int32_t index, int32_t count) {
  try {
    return PluginManager::Instance().RemoveChildNodeRange(nodeID, index, count);
  } catch (...) {
    return 0;
  }
}
int32_t ClearChildNodes(int32_t nodeID) {
  try {
    return PluginManager::Instance().ClearChildNodes(nodeID);
  } catch (...) {
    return 0;
  }
}
int32_t GetChildNodeCount(int32_t nodeID) {
  try {
    return PluginManager::Instance().GetChildNodeCount(nodeID);
  } catch (...) {
    return 0;
  }
}
int32_t GetChildNodeAtIndex(int32_t nodeID, int32_t index) {
  try {
    return PluginManager::Instance().GetChildNodeAtIndex(nodeID, index);
  } catch (...) {
    return 0;
  }
}
int32_t IndexOfChildNode(int32_t nodeID, int32_t childNodeID) {
  try {
    return PluginManager::Instance().IndexOfChildNode(nodeID, childNodeID);
  } catch (...) {
    return 0;
  }
}
int32_t SetChildNodes(int32_t nodeID, int32_t childCount,
                      const int32_t *childrenNodeIDs) {
  try {
    return PluginManager::Instance().SetChildNodes(nodeID, childCount,
                                                   childrenNodeIDs);
  } catch (...) {
    return 0;
  }
}
int32_t GetChildNodes(int32_t nodeID, int32_t childCount, int32_t *children,
                      int32_t arrayIndex) {
  try {
    return PluginManager::Instance().GetChildNodes(nodeID, childCount, children,
                                                   arrayIndex);
  } catch (...) {
    return 0;
  }
}
int32_t GetNumberOfBrushesInTree(int32_t nodeID) {
  try {
    return PluginManager::Instance().GetNumberOfBrushesInTree(nodeID);
  } catch (...) {
    return 0;
  }
}
int32_t DoesTreeContainBrush(int32_t nodeID, int32_t brushID) {
  try {
    return PluginManager::Instance().DoesTreeContainBrush(nodeID, brushID);
  } catch (...) {
    return 0;
  }
}
int32_t IsNodeDirty(int32_t nodeID) {
  try {
    return PluginManager::Instance().IsNodeDirty(nodeID);
  } catch (...) {
    return 0;
  }
}
int32_t SetDirty(int32_t nodeID) {
  try {
    return PluginManager::Instance().SetDirty(nodeID);
  } catch (...) {
    return 0;
  }
}
int32_t ClearDirty(int32_t nodeID) {
  try {
    return PluginManager::Instance().ClearDirty(nodeID);
  } catch (...) {
    return 0;
  }
}
int32_t GetBranchOperationType(int32_t branchNodeID) {
  try {
    return PluginManager::Instance().GetBranchOperationType(branchNodeID);
  } catch (...) {
    return 0;
  }
}
int32_t SetBranchOperationType(int32_t branchNodeID,
                               CSGOperationType operation) {
  try {
    return PluginManager::Instance().SetBranchOperationType(branchNodeID,
                                                            operation);
  } catch (...) {
    return 0;
  }
}
int32_t GetBrushOperationType(int32_t brushNodeID) {
  try {
    return PluginManager::Instance().GetBrushOperationType(brushNodeID);
  } catch (...) {
    return 0;
  }
}
int32_t SetBrushOperationType(int32_t brushNodeID, CSGOperationType operation) {
  try {
    return PluginManager::Instance().SetBrushOperationType(brushNodeID,
                                                           operation);
  } catch (...) {
    return 0;
  }
}
CSGTreeBrushFlags GetBrushFlags(int32_t brushNodeID) {
  try {
    return PluginManager::Instance().GetBrushFlags(brushNodeID);
  } catch (...) {
    return CSGTreeBrushFlags_Default;
  }
}
int32_t SetBrushFlags(int32_t brushNodeID, CSGTreeBrushFlags flags) {
  try {
    return PluginManager::Instance().SetBrushFlags(brushNodeID, flags);
  } catch (...) {
    return 0;
  }
}
int32_t SetTreeEnabled(int32_t modelNodeID, int32_t isEnabled) {
  try {
    return PluginManager::Instance().SetTreeEnabled(modelNodeID, isEnabled);
  } catch (...) {
    return 0;
  }
}
int32_t GetTreeEnabled(int32_t modelNodeID) {
  try {
    return PluginManager::Instance().GetTreeEnabled(modelNodeID);
  } catch (...) {
    return 0;
  }
}
int32_t SetNodeLocalTransformation(int32_t brushNodeID,
                                   const float *brushToTreeSpace) {
  try {
    return PluginManager::Instance().SetNodeLocalTransformation(
        brushNodeID, brushToTreeSpace);
  } catch (...) {
    return 0;
  }
}
int32_t GetNodeLocalTransformation(int32_t brushNodeID,
                                   float *brushToTreeSpace) {
  try {
    return PluginManager::Instance().GetNodeLocalTransformation(
        brushNodeID, brushToTreeSpace);
  } catch (...) {
    return 0;
  }
}
int32_t GetInvNodeLocalTransformation(int32_t brushNodeID,
                                      float *invTransformation) {
  try {
    return PluginManager::Instance().GetInvNodeLocalTransformation(
        brushNodeID, invTransformation);
  } catch (...) {
    return 0;
  }
}
int32_t GetNodeToTreeSpaceMatrix(int32_t nodeID, float *matrix) {
  try {
    return PluginManager::Instance().GetNodeToTreeSpaceMatrix(nodeID, matrix);
  } catch (...) {
    return 0;
  }
}
int32_t GetTreeToNodeSpaceMatrix(int32_t nodeID, float *matrix) {
  try {
    return PluginManager::Instance().GetTreeToNodeSpaceMatrix(nodeID, matrix);
  } catch (...) {
    return 0;
  }
}
int32_t GetBrushBounds(int32_t brushNodeID, AABB *bounds) {
  try {
    return PluginManager::Instance().GetBrushBounds(brushNodeID, bounds);
  } catch (...) {
    return 0;
  }
}
int32_t CreateBrushMesh(int32_t userID, int32_t vertexCount,
                        const Vector3 *vertices, int32_t halfEdgeCount,
                        const BrushMeshHalfEdge *halfEdges,
                        int32_t polygonCount,
                        const BrushMeshPolygon *polygons) {
  try {
    return PluginManager::Instance().CreateBrushMesh(
        userID, vertexCount, vertices, halfEdgeCount, halfEdges, polygonCount,
        polygons);
  } catch (...) {
    return 0;
  }
}
int32_t UpdateBrushMesh(int32_t brushMeshIndex, int32_t vertexCount,
                        const Vector3 *vertices, int32_t halfEdgeCount,
                        const BrushMeshHalfEdge *halfEdges,
                        int32_t polygonCount,
                        const BrushMeshPolygon *polygons) {
  try {
    return PluginManager::Instance().UpdateBrushMesh(
        brushMeshIndex, vertexCount, vertices, halfEdgeCount, halfEdges,
        polygonCount, polygons);
  } catch (...) {
    return 0;
  }
}
int32_t DestroyBrushMesh(int32_t brushMeshIndex) {
  try {
    return PluginManager::Instance().DestroyBrushMesh(brushMeshIndex);
  } catch (...) {
    return 0;
  }
}
int32_t IsBrushMeshIDValid(int32_t brushMeshIndex) {
  try {
    return PluginManager::Instance().IsBrushMeshIDValid(brushMeshIndex);
  } catch (...) {
    return 0;
  }
}
int32_t GetBrushMeshCount() {
  try {
    return PluginManager::Instance().GetBrushMeshCount();
  } catch (...) {
    return 0;
  }
}
int32_t GetBrushMeshUserID(int32_t brushMeshIndex) {
  try {
    return PluginManager::Instance().GetBrushMeshUserID(brushMeshIndex);
  } catch (...) {
    return 0;
  }
}
int32_t GetAllBrushMeshInstances(int32_t nodeCount, int32_t *allNodeIDs) {
  try {
    return PluginManager::Instance().GetAllBrushMeshInstances(nodeCount,
                                                              allNodeIDs);
  } catch (...) {
    return 0;
  }
}
int32_t SetBrushMeshID(int32_t brushNodeID, int32_t brushMeshID) {
  try {
    return PluginManager::Instance().SetBrushMeshID(brushNodeID, brushMeshID);
  } catch (...) {
    return 0;
  }
}
int32_t GetBrushMeshID(int32_t brushNodeID) {
  try {
    return PluginManager::Instance().GetBrushMeshID(brushNodeID);
  } catch (...) {
    return 0;
  }
}
int32_t UpdateAllTreeMeshes() {
  try {
    return PluginManager::Instance().UpdateAllTreeMeshes();
  } catch (...) {
    return 0;
  }
}
int32_t GenerateMeshDescriptions(int32_t treeNodeID, int32_t meshTypeCount,
                                 const MeshQuery *meshTypes,
                                 VertexChannelFlags vertexChannelMask,
                                 int32_t *meshDescriptionCount) {
  try {
    return PluginManager::Instance().GenerateMeshDescriptions(
        treeNodeID, meshTypeCount, meshTypes, vertexChannelMask,
        meshDescriptionCount);
  } catch (...) {
    return 0;
  }
}
int32_t GetMeshDescriptions(int32_t treeNodeID, int32_t meshDescriptionCount,
                            GeneratedMeshDescription *meshDescriptions) {
  try {
    return PluginManager::Instance().GetMeshDescriptions(
        treeNodeID, meshDescriptionCount, meshDescriptions);
  } catch (...) {
    return 0;
  }
}
int32_t GetGeneratedMesh(int32_t treeNodeID, int32_t meshIndex,
                         int32_t subMeshIndex, int32_t indexCount,
                         int32_t *indices, int32_t vertexCount,
                         Vector3 *positions, Vector4 *tangents,
                         Vector3 *normals, Vector2 *uvs, Vector3 *boundsCenter,
                         Vector3 *boundsSize) {
  try {
    return PluginManager::Instance().GetGeneratedMesh(
        treeNodeID, meshIndex, subMeshIndex, indexCount, indices, vertexCount,
        positions, tangents, normals, uvs, boundsCenter, boundsSize);
  } catch (...) {
    return 0;
  }
}
int32_t GenerateTransformOffset(int32_t userID) {
  try {
    return PluginManager::Instance().GenerateTransformOffset(userID);
  } catch (...) {
    return 0;
  }
}
int32_t DestroyTransformOffset(int32_t transformOffsetID) {
  try {
    return PluginManager::Instance().DestroyTransformOffset(transformOffsetID);
  } catch (...) {
    return 0;
  }
}
int32_t DestroyTransformOffsets(int32_t count,
                                const int32_t *transformOffsetIDs) {
  try {
    return PluginManager::Instance().DestroyTransformOffsets(
        count, transformOffsetIDs);
  } catch (...) {
    return 0;
  }
}
int32_t IsTransformOffsetIDValid(int32_t transformOffsetID) {
  try {
    return PluginManager::Instance().IsTransformOffsetIDValid(
        transformOffsetID);
  } catch (...) {
    return 0;
  }
}
int32_t GetTransformOffsetCount() {
  try {
    return PluginManager::Instance().GetTransformOffsetCount();
  } catch (...) {
    return 0;
  }
}
int32_t GetUserIDOfTransformOffset(int32_t transformOffsetID) {
  try {
    return PluginManager::Instance().GetUserIDOfTransformOffset(
        transformOffsetID);
  } catch (...) {
    return 0;
  }
}
int32_t GetAllTransformOffsets(int32_t count, int32_t *transformOffsetIDs) {
  try {
    return PluginManager::Instance().GetAllTransformOffsets(count,
                                                            transformOffsetIDs);
  } catch (...) {
    return 0;
  }
}
int32_t SetTransformOffsetLocalTransformation(int32_t transformOffsetID,
                                              const float *transform) {
  try {
    return PluginManager::Instance().SetTransformOffsetLocalTransformation(
        transformOffsetID, transform);
  } catch (...) {
    return 0;
  }
}
int32_t GetTransformOffsetLocalTransformation(int32_t transformOffsetID,
                                              float *transform) {
  try {
    return PluginManager::Instance().GetTransformOffsetLocalTransformation(
        transformOffsetID, transform);
  } catch (...) {
    return 0;
  }
}
int32_t SetNodeTransformOffset(int32_t nodeID, int32_t transformOffsetID) {
  try {
    return PluginManager::Instance().SetNodeTransformOffset(nodeID,
                                                            transformOffsetID);
  } catch (...) {
    return 0;
  }
}
int32_t GetNodeTransformOffset(int32_t nodeID) {
  try {
    return PluginManager::Instance().GetNodeTransformOffset(nodeID);
  } catch (...) {
    return 0;
  }
}
int32_t SetNodeSynchronizedWith(int32_t nodeID, int32_t targetNodeID) {
  try {
    return PluginManager::Instance().SetNodeSynchronizedWith(nodeID,
                                                             targetNodeID);
  } catch (...) {
    return 0;
  }
}
int32_t GetNodeSynchronizedWith(int32_t nodeID) {
  try {
    return PluginManager::Instance().GetNodeSynchronizedWith(nodeID);
  } catch (...) {
    return 0;
  }
}
int32_t GetNodeSynchronizedOrigin(int32_t nodeID) {
  try {
    return PluginManager::Instance().GetNodeSynchronizedOrigin(nodeID);
  } catch (...) {
    return 0;
  }
}
int32_t GetAllSynchronizedBrushVariantCount(int32_t nodeID) {
  try {
    return PluginManager::Instance().GetAllSynchronizedBrushVariantCount(
        nodeID);
  } catch (...) {
    return 0;
  }
}
int32_t GetAllSynchronizedBrushVariants(int32_t nodeID, int32_t count,
                                        int32_t *variantNodeIDs) {
  try {
    return PluginManager::Instance().GetAllSynchronizedBrushVariants(
        nodeID, count, variantNodeIDs);
  } catch (...) {
    return 0;
  }
}
}
