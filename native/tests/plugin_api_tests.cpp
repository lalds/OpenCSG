#include "realtimecsg/csg_plugin_api.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>
namespace {
void require(bool ok, const char *message) {
  if (!ok)
    throw std::runtime_error(message);
}
struct Box {
  std::array<Vector3, 8> vertices;
  std::array<BrushMeshHalfEdge, 24> edges{};
  std::array<BrushMeshPolygon, 6> polygons{};
  Box(Vector3 lo, Vector3 hi, int material) {
    vertices = {Vector3{lo.x, lo.y, lo.z}, {hi.x, lo.y, lo.z},
                {hi.x, hi.y, lo.z},        {lo.x, hi.y, lo.z},
                {lo.x, lo.y, hi.z},        {hi.x, lo.y, hi.z},
                {hi.x, hi.y, hi.z},        {lo.x, hi.y, hi.z}};
    int faces[6][4] = {{3, 2, 1, 0}, {4, 5, 6, 7}, {0, 1, 5, 4},
                       {1, 2, 6, 5}, {2, 3, 7, 6}, {3, 0, 4, 7}};
    for (int p = 0; p < 6; ++p) {
      polygons[p].firstEdge = p * 4;
      polygons[p].edgeCount = 4;
      polygons[p].polygonID = 100 + p;
      polygons[p].surface.UV0 = {{1, 0, 0, .25f}, {0, 1, 1, -.5f}};
      polygons[p].layers = {LayerUsageFlags_RenderReceiveCastShadows, material,
                            material + 100, 0};
      for (int c = 0; c < 4; ++c) {
        int twin = -1;
        for (int q = 0; q < 6; ++q)
          for (int d = 0; d < 4; ++d)
            if (faces[p][c] == faces[q][(d + 3) % 4] &&
                faces[p][(c + 3) % 4] == faces[q][d])
              twin = q * 4 + d;
        edges[p * 4 + c] = {faces[p][c], twin};
      }
    }
  }
  int create() {
    return CreateBrushMesh(71, 8, vertices.data(), 24, edges.data(), 6,
                           polygons.data());
  }
};
struct Output {
  std::vector<GeneratedMeshDescription> descriptions;
  double volume = 0;
};
Output output(int tree) {
  MeshQuery query{uint32_t(LayerUsageFlags_Renderable) | (1u << 24),
                  uint32_t(LayerUsageFlags_Renderable) |
                      (uint32_t(VertexChannelFlags_All) << 24)};
  int count = -1;
  require(
      GenerateMeshDescriptions(tree, 1, &query, VertexChannelFlags_All, &count),
      "Description generation failed");
  Output result;
  result.descriptions.resize(count);
  require(GetMeshDescriptions(tree, count, result.descriptions.data()),
          "Description copy failed");
  for (const auto &d : result.descriptions) {
    std::vector<int> indices(d.indexCount);
    std::vector<Vector3> positions(d.vertexCount), normals(d.vertexCount);
    std::vector<Vector4> tangents(d.vertexCount);
    std::vector<Vector2> uv(d.vertexCount);
    Vector3 center{}, size{};
    require(GetGeneratedMesh(tree, d.meshQueryIndex, d.subMeshQueryIndex,
                             d.indexCount, indices.data(), d.vertexCount,
                             positions.data(), tangents.data(), normals.data(),
                             uv.data(), &center, &size),
            "Mesh copy failed");
    for (int index : indices)
      require(index >= 0 && index < d.vertexCount, "Invalid mesh index");
    for (int i = 0; i < d.indexCount; i += 3) {
      auto a = positions[indices[i]], b = positions[indices[i + 1]],
           c = positions[indices[i + 2]];
      result.volume += (a.x * (double(b.y) * c.z - double(b.z) * c.y) +
                        a.y * (double(b.z) * c.x - double(b.x) * c.z) +
                        a.z * (double(b.x) * c.y - double(b.y) * c.x)) /
                       6;
    }
  }
  return result;
}
} // namespace
int main() {
  try {
    UnityMethods callbacks{};
    callbacks.debugLogError = [](const char *message, int32_t) {
      std::cerr << message << '\n';
    };
    RegisterMethods(&callbacks);
    static_assert(sizeof(BrushMeshPolygon) == 68 &&
                  sizeof(SurfaceDescription) == 40 &&
                  sizeof(GeneratedMeshDescription) == 48);
    static_assert(
        VertexChannelFlags_Position == 0 && VertexChannelFlags_Tangent == 2 &&
        VertexChannelFlags_Normal == 4 && VertexChannelFlags_All == 14);
    ClearAllNodes();
    int tree = 0, a = 0, b = 0;
    require(GenerateTree(1, &tree) && GenerateBrush(2, &a) &&
                GenerateBrush(3, &b),
            "Node creation failed");
    Box box({-1, -1, -1}, {1, 1, 1}, 41),
        cutter({-.5f, -.5f, -2}, {.5f, .5f, 2}, -17);
    int am = box.create(), bm = cutter.create();
    require(am > 0 && bm > 0, "Valid source mesh rejected");
    require(SetBrushMeshID(a, am) && SetBrushMeshID(b, bm) &&
                AddChildNode(tree, a) && AddChildNode(tree, b),
            "Source attach failed");
    require(!AddChildNode(a, tree) && GetParentOfNode(a) == tree,
            "Invalid hierarchy changed state");
    const double expected[] = {10, 6, 2};
    for (int operation = 0; operation < 3; ++operation) {
      require(SetBrushOperationType(b, CSGOperationType(operation)),
              "Operation rejected");
      require(UpdateAllTreeMeshes(), "Scene update failed");
      require(IsNodeDirty(tree),
              "Tree dirty cleared before frontend can read it");
      auto data = output(tree);
      require(std::abs(data.volume - expected[operation]) < 1e-5,
              "Boolean volume differs");
      require(!IsNodeDirty(tree), "Descriptions did not acknowledge output");
      require(!UpdateAllTreeMeshes(), "Idle update did work");
    }
    require(SetBrushOperationType(b, CSGOperationType::Subtractive),
            "Subtract rejected");
    Matrix4x4 matrix{};
    matrix.values[0] = matrix.values[5] = matrix.values[10] =
        matrix.values[15] = 1;
    matrix.values[0] = -1;
    require(SetNodeLocalTransformation(b, matrix.values) &&
                UpdateAllTreeMeshes(),
            "Reflected source rejected");
    require(std::abs(output(tree).volume - 6) < 1e-5,
            "Reflected winding/volume differs");
    matrix.values[0] = 1;
    Vector3 start{-3, 0, 0}, end{3, 0, 0};
    require(RayCastIntoTreeMultiCount(tree, &start, &end, matrix.values, 0, 1,
                                      nullptr, 0) > 0,
            "Visible triangle picking failed after descriptions");
    CSGTreeBrushIntersection hit{};
    require(RayCastIntoBrush(a, &start, &end, matrix.values, 1, &hit) &&
                hit.brushUserID == 2,
            "Source picking failed");
    Vector2 low{}, high{}, uv{};
    require(GetSurfaceMinMaxTexCoords(a, 100, matrix, &low, &high),
            "By-value matrix ABI failed");
    require(ConvertWorldToTextureCoord(a, 100, matrix, {-1, -1, -1}, &uv),
            "UV projection failed");
    int vc = 0, vo = 0, vi = 0, io = 0, ii = 0, invalid = 0;
    require(GetBrushOutlineSizes(a, &vc, &vo, &vi, &io, &ii, &invalid) &&
                vc == 8 && vo == 24,
            "Source outline differs");
    auto invalidBox = box;
    invalidBox.edges[0].twinIndex = 0;
    require(invalidBox.create() == 0, "Malformed topology accepted");
    int branch = 0;
    require(GenerateBranch(4, &branch) && AddChildNode(branch, b) &&
                AddChildNode(tree, branch),
            "Reparent failed");
    require(!AddChildNode(branch, branch), "Cycle accepted");
    require(DestroyNode(branch) && IsNodeIDValid(b) && GetParentOfNode(b) == 0,
            "Destroy must detach rather than destroy children");
    require(UpdateAllTreeMeshes() && std::abs(output(tree).volume - 8) < 1e-5,
            "Detached brush remained in scene");
    // Generate previews and Undo leave a valid empty operation beside a solid.
    require(GenerateBranch(5, &branch) && AddChildNode(tree, branch),
            "Empty preview group registration failed");
    for (int operation = 0; operation < 3; ++operation) {
      require(SetBranchOperationType(branch, CSGOperationType(operation)) &&
                  UpdateAllTreeMeshes(), "Empty preview group build failed");
      require(std::abs(output(tree).volume - (operation == 2 ? 0 : 8)) < 1e-5,
              "Empty group lost Boolean semantics");
    }
    require(SetBranchOperationType(branch, CSGOperationType::Additive) &&
                AddChildNode(branch, b) && UpdateAllTreeMeshes(),
            "Cutter-only group build failed");
    require(std::abs(output(tree).volume - 8) < 1e-5,
            "Cutter-only group must be empty");
    require(SetBrushOperationType(b, CSGOperationType::Additive) &&
                UpdateAllTreeMeshes() &&
                std::abs(output(tree).volume - 10) < 1e-5,
            "Preview group failed to become a solid");
    require(DestroyNode(b) && UpdateAllTreeMeshes() &&
                std::abs(output(tree).volume - 8) < 1e-5,
            "Preview removal failed to restore empty group");
    require(DestroyNode(a) && UpdateAllTreeMeshes(),
            "Empty tree update failed");
    require(output(tree).descriptions.empty(), "Empty tree kept stale meshes");
    ClearAllNodes();
    require(GetNodeCount() == 0 && GetBrushMeshCount() == 0,
            "Clear retained objects");
    // A triangular prism has 18 halfedges: workspace capacities still need
    // rounding to the recovered allocator's required multiple of four.
    std::vector<Vector3> prism = {{0, 0, -1}, {1, 0, -1}, {0, 1, -1},
                                  {0, 0, 1},  {1, 0, 1},  {0, 1, 1}};
    std::vector<std::vector<int>> faces = {
        {2, 1, 0}, {3, 4, 5}, {0, 1, 4, 3}, {1, 2, 5, 4}, {2, 0, 3, 5}};
    std::vector<BrushMeshHalfEdge> edges;
    std::vector<BrushMeshPolygon> polygons;
    for (size_t p = 0; p < faces.size(); ++p) {
      BrushMeshPolygon polygon{};
      polygon.firstEdge = int(edges.size());
      polygon.edgeCount = int(faces[p].size());
      polygon.polygonID = int(p);
      polygon.layers.layerUsage = LayerUsageFlags_Renderable;
      polygons.push_back(polygon);
      for (int v : faces[p])
        edges.push_back({v, -1});
    }
    for (size_t p = 0; p < faces.size(); ++p)
      for (size_t c = 0; c < faces[p].size(); ++c)
        for (size_t q = 0; q < faces.size(); ++q)
          for (size_t d = 0; d < faces[q].size(); ++d)
            if (faces[p][c] ==
                    faces[q][(d + faces[q].size() - 1) % faces[q].size()] &&
                faces[p][(c + faces[p].size() - 1) % faces[p].size()] ==
                    faces[q][d])
              edges[polygons[p].firstEdge + c].twinIndex =
                  polygons[q].firstEdge + int(d);
    int prismMesh =
        CreateBrushMesh(91, int(prism.size()), prism.data(), int(edges.size()),
                        edges.data(), int(polygons.size()), polygons.data());
    require(prismMesh > 0 && GenerateTree(92, &tree) && GenerateBrush(93, &a) &&
                SetBrushMeshID(a, prismMesh) && AddChildNode(tree, a),
            "Prism registration failed");
    require(UpdateAllTreeMeshes() && std::abs(output(tree).volume - 1) < 1e-5,
            "Prism build/capacity alignment failed");
    auto invalidPrism = prism;
    invalidPrism[0].x += .25f;
    require(UpdateBrushMesh(prismMesh, int(prism.size()), invalidPrism.data(),
                            int(edges.size()), edges.data(),
                            int(polygons.size()), polygons.data()),
            "Topology-valid source update rejected");
    require(!UpdateAllTreeMeshes() && !IsNodeDirty(tree) &&
                !UpdateAllTreeMeshes(),
            "Failed geometry kept rebuilding in idle");
    require(std::abs(output(tree).volume - 1) < 1e-5,
            "Failed geometry lost last good snapshot");
    require(UpdateBrushMesh(prismMesh, int(prism.size()), prism.data(),
                            int(edges.size()), edges.data(),
                            int(polygons.size()), polygons.data()) &&
                UpdateAllTreeMeshes(),
            "Valid source did not recover after error");
    require(std::abs(output(tree).volume - 1) < 1e-5,
            "Recovered prism volume differs");
    ClearAllNodes();
    // A small valid face must not be called degenerate because of an absolute
    // float area-squared cutoff. A truly collapsed face must still be rejected.
    Box tiny({0, 0, 0}, {.000001f, .000001f, .000001f}, 41);
    int tinyMesh = tiny.create();
    require(tinyMesh && GenerateTree(101, &tree) && GenerateBrush(102, &a) &&
                SetBrushMeshID(a, tinyMesh) && AddChildNode(tree, a),
            "Tiny box registration failed");
    require(UpdateAllTreeMeshes() && !output(tree).descriptions.empty(),
            "Small valid winding was rejected");
    auto collapsed = tiny;
    for (auto &v : collapsed.vertices) v.z = 0;
    require(UpdateBrushMesh(tinyMesh, 8, collapsed.vertices.data(), 24,
                            collapsed.edges.data(), 6, collapsed.polygons.data()),
            "Collapsed source topology update failed");
    require(!UpdateAllTreeMeshes(), "Zero-area polygon was accepted");
    require(UpdateBrushMesh(tinyMesh, 8, tiny.vertices.data(), 24,
                            tiny.edges.data(), 6, tiny.polygons.data()) &&
                UpdateAllTreeMeshes() && !output(tree).descriptions.empty(),
            "Tiny box failed to recover after collapse");
    ClearAllNodes();
    // Convex partitioning must not expose render triangulation as source
    // topology. Cylinder caps need one editable n-gon, in either winding.
    std::vector<Vector2> circle;
    for (int i = 0; i < 32; ++i) {
      double angle = i * (2 * std::acos(-1.0) / 32);
      circle.push_back({float(std::cos(angle)), float(std::sin(angle))});
    }
    for (int winding = 0; winding < 2; ++winding) {
      int count = 0, size = 0;
      require(DecomposeStart(32, circle.data(), &count) && count == 1 &&
                  DecomposeGetSizes(1, &size) && size == 32,
              "Convex outline was needlessly triangulated");
      std::vector<Vector2> preserved(32);
      require(DecomposeGetPolygon(0, 32, preserved.data()), "Convex polygon copy failed");
      for (int i = 0; i < 32; ++i)
        require(preserved[i].x == circle[i].x && preserved[i].y == circle[i].y,
                "Convex partition changed source vertex order");
      std::reverse(circle.begin(), circle.end());
    }
    Vector2 concave[] = {{0,0},{2,0},{2,1},{1,1},{1,2},{0,2}};
    int partCount = 0;
    require(DecomposeStart(6, concave, &partCount) && partCount > 1,
            "Concave partition was lost");
    std::vector<int> sizes(partCount);
    require(DecomposeGetSizes(partCount, sizes.data()), "Concave sizes failed");
    double twiceArea = 0;
    for (int p = 0; p < partCount; ++p) {
      std::vector<Vector2> part(sizes[p]);
      require(DecomposeGetPolygon(p, sizes[p], part.data()), "Concave polygon copy failed");
      for (int i = 0; i < sizes[p]; ++i) {
        const auto a = part[i], b = part[(i + 1) % sizes[p]];
        twiceArea += double(a.x) * b.y - double(a.y) * b.x;
      }
    }
    require(std::abs(twiceArea - 6) < 1e-6, "Concave partition changed total area");
    std::cout << "Plugin ABI, hierarchy, Boolean volumes, reflection, dirty "
                 "lifecycle, picking, UV, outlines and validation passed\n";
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
