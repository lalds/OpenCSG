#include "plugin_manager.h"
#include "realtimecsg/recovered_scene.h"

#include <array>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <unordered_set>

namespace RealtimeCSG {
namespace {
using realtimecsg::recovered::CategorizationNode;
using realtimecsg::recovered::CSGOperation;
using realtimecsg::recovered::ExportedScene;
using realtimecsg::recovered::SceneBrushInput;
using realtimecsg::recovered::SceneBuildInput;
using realtimecsg::recovered::SceneFloat4;
using realtimecsg::recovered::SceneHalfEdge;
using realtimecsg::recovered::ScenePolygon;
using realtimecsg::recovered::SceneSurface;
using realtimecsg::recovered::SceneTriangleOrigin;
using realtimecsg::recovered::SceneVertex;
using ScenePosition = realtimecsg::recovered::Vector3;
using realtimecsg::recovered::buildConvexScene;
using realtimecsg::recovered::kSceneAbiVersion;

struct MeshSnapshot {
  ExportedScene scene;
  std::vector<SceneVertex> vertices;
  std::vector<std::int32_t> indices;
  std::vector<SceneTriangleOrigin> triangles;
  std::vector<GeneratedMeshDescription> descriptions;
  std::vector<GeneratedSubmeshData> meshes;
};

// Node IDs are monotonically allocated by PluginManager. This cache is guarded
// by the manager mutex at every access and replaced atomically after a build.
std::unordered_map<std::int32_t, MeshSnapshot> g_meshSnapshots;

struct BrushStorage {
  std::vector<ScenePosition> vertices;
  std::vector<SceneHalfEdge> edges;
  std::vector<ScenePolygon> polygons;
  std::vector<SceneSurface> surfaces;
};

float dot(ScenePosition a, ScenePosition b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}
ScenePosition normalize(double x, double y, double z) {
  const double n = std::hypot(x, y, z);
  if (!(n > 0) || !std::isfinite(n))
    throw std::runtime_error("brush polygon has a degenerate winding");
  return {float(x / n), float(y / n), float(z / n)};
}
ScenePosition point(const float *m, ::Vector3 v) {
  return {m[0] * v.x + m[4] * v.y + m[8] * v.z + m[12],
          m[1] * v.x + m[5] * v.y + m[9] * v.z + m[13],
          m[2] * v.x + m[6] * v.y + m[10] * v.z + m[14]};
}
float determinant3(const float *m) {
  return m[0] * (m[5] * m[10] - m[9] * m[6]) -
         m[4] * (m[1] * m[10] - m[9] * m[2]) +
         m[8] * (m[1] * m[6] - m[5] * m[2]);
}
SceneFloat4 uvRowToTree(const float *m, float det, Vector4 row) {
  // Convert brush-space UV projection r * p + w to the common tree-space
  // position used by scene_export: r * inverse(M) * pTree + w.
  const float inv[9] = {
      (m[5] * m[10] - m[9] * m[6]) / det, (m[8] * m[6] - m[4] * m[10]) / det,
      (m[4] * m[9] - m[8] * m[5]) / det,  (m[9] * m[2] - m[1] * m[10]) / det,
      (m[0] * m[10] - m[8] * m[2]) / det, (m[8] * m[1] - m[0] * m[9]) / det,
      (m[1] * m[6] - m[5] * m[2]) / det,  (m[4] * m[2] - m[0] * m[6]) / det,
      (m[0] * m[5] - m[4] * m[1]) / det};
  const float x = row.x * inv[0] + row.y * inv[3] + row.z * inv[6];
  const float y = row.x * inv[1] + row.y * inv[4] + row.z * inv[7];
  const float z = row.x * inv[2] + row.y * inv[5] + row.z * inv[8];
  return {x, y, z, row.w - x * m[12] - y * m[13] - z * m[14]};
}

std::uint64_t hashBytes(std::uint64_t h, const void *data, std::size_t size) {
  const auto *b = static_cast<const std::uint8_t *>(data);
  for (std::size_t i = 0; i < size; ++i) {
    h ^= b[i];
    h *= 1099511628211ull;
  }
  return h;
}
template <class T>
std::uint64_t hashVector(std::uint64_t h, const std::vector<T> &v) {
  return v.empty() ? h : hashBytes(h, v.data(), v.size() * sizeof(T));
}
int requestedParameter(const MeshQuery &q) {
  return static_cast<int>((q.layers >> 24) & 0x7fu);
}
std::uint32_t requestedLayers(const MeshQuery &q) {
  return q.layers & 0x00ffffffu;
}
std::uint32_t requestedMask(const MeshQuery &q) {
  return q.maskAndChannels & 0x00ffffffu;
}
std::uint32_t requestedChannels(const MeshQuery &q) {
  return (q.maskAndChannels >> 24) & 0xffu;
}
int layerParameter(const SurfaceLayers &l, int index) {
  switch (index) {
  case 1:
    return l.layerParameter1;
  case 2:
    return l.layerParameter2;
  case 3:
    return l.layerParameter3;
  default:
    return 0;
  }
}

bool validMeshQuery(const MeshQuery &q, VertexChannelFlags channelMask) {
  const auto allowed = static_cast<std::uint32_t>(VertexChannelFlags_All);
  return requestedParameter(q) <= 3 && (requestedChannels(q) & ~allowed) == 0 &&
         (static_cast<std::uint32_t>(channelMask) & ~allowed) == 0;
}

bool buildStorage(const Node &node, const StoredBrushMesh &source,
                  BrushStorage &out) {
  const int nv = static_cast<int>(source.vertices.size()),
            ne = static_cast<int>(source.halfEdges.size()),
            np = static_cast<int>(source.polygons.size());
  if (nv < 0 || ne < 0 || np < 0)
    return false;
  const float det = determinant3(node.localTransformation);
  if (!std::isfinite(det) || std::abs(det) < 1.0e-12f)
    return false;
  out.vertices.reserve(nv);
  for (auto v : source.vertices)
    out.vertices.push_back(point(node.localTransformation, v));
  out.edges.resize(ne);
  out.polygons.reserve(np);
  out.surfaces.reserve(np);
  std::vector<int> edgeMap(ne), reversedStart(ne, -1);
  for (int i = 0; i < ne; ++i)
    edgeMap[i] = i;
  if (det < 0) {
    for (const auto &p : source.polygons) {
      if (p.firstEdge < 0 || p.edgeCount < 3 || p.firstEdge > ne - p.edgeCount)
        return false;
      for (int j = 0; j < p.edgeCount; ++j) {
        const int old = p.firstEdge + j;
        edgeMap[old] = p.firstEdge + (p.edgeCount - 1 - j);
        reversedStart[old] =
            source.halfEdges[p.firstEdge + (j + p.edgeCount - 1) % p.edgeCount]
                .vertexIndex;
      }
    }
  }
  for (int i = 0; i < ne; ++i) {
    const auto &e = source.halfEdges[i];
    if (e.vertexIndex < 0 || e.vertexIndex >= nv || e.twinIndex < 0 ||
        e.twinIndex >= ne)
      return false;
    out.edges[edgeMap[i]] = {det < 0 ? reversedStart[i] : e.vertexIndex,
                             edgeMap[e.twinIndex]};
  }
  for (int pi = 0; pi < np; ++pi) {
    const auto &p = source.polygons[pi];
    if (p.firstEdge < 0 || p.edgeCount < 3 || p.firstEdge > ne - p.edgeCount)
      return false;
    // Accumulate area around a nearby origin in double precision. The float
    // Newell sum cancels for small faces translated far from the model origin.
    const auto origin = out.vertices[source.halfEdges[p.firstEdge].vertexIndex];
    double nx = 0, ny = 0, nz = 0;
    for (int j = 0; j < p.edgeCount; ++j) {
      auto a = out.vertices[source.halfEdges[p.firstEdge + j].vertexIndex];
      auto b =
          out.vertices[source.halfEdges[p.firstEdge + (j + 1) % p.edgeCount]
                           .vertexIndex];
      const double ax = double(a.x) - origin.x, ay = double(a.y) - origin.y,
                   az = double(a.z) - origin.z;
      const double bx = double(b.x) - origin.x, by = double(b.y) - origin.y,
                   bz = double(b.z) - origin.z;
      nx += ay * bz - az * by;
      ny += az * bx - ax * bz;
      nz += ax * by - ay * bx;
    }
    ScenePosition n = normalize(nx, ny, nz);
    // A reflected transform reverses the source winding. Reverse the
    // normal here so the stored polygon remains outward-facing.
    if (det < 0)
      n = {-n.x, -n.y, -n.z};
    const ScenePosition wn = n;
    const int first = p.firstEdge;
    out.polygons.push_back({first, p.edgeCount, pi});
    SceneSurface surf{};
    surf.plane.normal = wn;
    const auto firstVertex = out.vertices[out.edges[first].vertexIndex];
    surf.plane.distance = dot(wn, firstVertex);
    // UVMatrix consists of U/V Vector4 rows, copy each independently.
    const auto uvU =
        (p.surface.surfaceFlags & SurfaceFlags_TextureIsInWorldSpace)
            ? SceneFloat4{p.surface.UV0.U.x, p.surface.UV0.U.y,
                          p.surface.UV0.U.z, p.surface.UV0.U.w}
            : uvRowToTree(node.localTransformation, det, p.surface.UV0.U);
    const auto uvV =
        (p.surface.surfaceFlags & SurfaceFlags_TextureIsInWorldSpace)
            ? SceneFloat4{p.surface.UV0.V.x, p.surface.UV0.V.y,
                          p.surface.UV0.V.z, p.surface.UV0.V.w}
            : uvRowToTree(node.localTransformation, det, p.surface.UV0.V);
    surf.uvU = uvU;
    surf.uvV = uvV;
    surf.materialId = pi;
    out.surfaces.push_back(surf);
  }
  return true;
}

} // namespace

void ForgetGeneratedTreeLocked(int32_t id) { g_meshSnapshots.erase(id); }
void ClearGeneratedScenesLocked() { g_meshSnapshots.clear(); }

bool CopyGeneratedSceneLocked(int32_t treeID,
                              realtimecsg::recovered::ExportedScene &scene) {
  auto it = g_meshSnapshots.find(treeID);
  if (it == g_meshSnapshots.end())
    return false;
  scene = it->second.scene;
  return true;
}

bool PluginManager::UpdateAllTreeMeshes() {
  try {
    std::unique_lock<std::mutex> lock(mutex_);
    std::vector<std::pair<int32_t, std::string>> errors;
    std::vector<int> treeIDs;
    for (const auto &kv : nodes_)
      if (kv.second.type == CSGNodeType::Tree)
        treeIDs.push_back(kv.first);
    std::sort(treeIDs.begin(), treeIDs.end());
    std::unordered_map<int, MeshSnapshot> replacements;
    for (int treeID : treeIDs) {
      auto rootIt = nodes_.find(treeID);
      if (rootIt == nodes_.end())
        continue;
      if (!rootIt->second.isDirty)
        continue;
      try {
        if (!rootIt->second.isEnabled) {
          replacements.emplace(treeID, MeshSnapshot{});
          continue;
        }
        std::vector<int> orderedNodes;
        std::vector<int> stack{treeID};
        std::unordered_set<int> seen;
        while (!stack.empty()) {
          const int id = stack.back();
          stack.pop_back();
          if (!seen.insert(id).second)
            throw std::runtime_error(
                "tree hierarchy contains a cycle or duplicate node");
          auto it = nodes_.find(id);
          if (it == nodes_.end())
            throw std::runtime_error(
                "tree hierarchy references a missing node");
          orderedNodes.push_back(id);
          for (auto r = it->second.children.rbegin();
               r != it->second.children.rend(); ++r)
            stack.push_back(*r);
        }
        std::vector<CategorizationNode> catNodes;
        std::vector<int> children;
        std::vector<BrushStorage> storage;
        std::vector<SceneBrushInput> brushes;
        std::unordered_map<int, int> nodeToIndex;
        for (std::size_t i = 0; i < orderedNodes.size(); ++i)
          nodeToIndex[orderedNodes[i]] = static_cast<int>(i);
        for (int id : orderedNodes) {
          const Node &n = nodes_.at(id);
          CategorizationNode c{};
          c.operation = static_cast<CSGOperation>(n.operation);
          c.isBrush = (n.type == CSGNodeType::Brush);
          c.infinite = (n.flags & CSGTreeBrushFlags_Infinite) != 0;
          c.firstChild = static_cast<int>(children.size());
          for (int child : n.children) {
            auto ci = nodeToIndex.find(child);
            if (ci == nodeToIndex.end())
              throw std::runtime_error("invalid tree child");
            children.push_back(ci->second);
          }
          c.childCount = static_cast<int>(children.size()) - c.firstChild;
          catNodes.push_back(c);
          if (n.type == CSGNodeType::Brush) {
            auto bm = brushMeshes_.find(n.brushMeshID);
            if (bm == brushMeshes_.end())
              throw std::runtime_error("tree brush has no stored brush mesh");
            storage.emplace_back();
            auto &s = storage.back();
            if (!buildStorage(n, bm->second, s))
              throw std::runtime_error("brush mesh or transform is invalid");
            brushes.push_back(
                {nodeToIndex[id], id, s.vertices.data(),
                 static_cast<int>(s.vertices.size()), s.edges.data(),
                 static_cast<int>(s.edges.size()), s.polygons.data(),
                 static_cast<int>(s.polygons.size()), s.surfaces.data(),
                 static_cast<int>(s.surfaces.size())});
          }
        }
        // Restore pointers after all vectors have reached final capacity.
        std::size_t bi = 0;
        for (int id : orderedNodes)
          if (nodes_.at(id).type == CSGNodeType::Brush) {
            auto &s = storage[bi];
            auto &b = brushes[bi];
            b.vertices = s.vertices.data();
            b.edges = s.edges.data();
            b.polygons = s.polygons.data();
            b.surfaces = s.surfaces.data();
            ++bi;
          }
        MeshSnapshot snapshot;
        if (brushes.empty()) {
          replacements.emplace(treeID, std::move(snapshot));
          continue;
        }
        int maxVertices = 0, maxEdges = 0, maxPolygons = 0;
        for (const auto &s : storage) {
          maxVertices =
              std::max(maxVertices, static_cast<int>(s.vertices.size()));
          maxEdges = std::max(maxEdges, static_cast<int>(s.edges.size()));
          maxPolygons =
              std::max(maxPolygons, static_cast<int>(s.polygons.size()));
        }
        SceneBuildInput input{};
        input.abiVersion = kSceneAbiVersion;
        input.rootNode = 0;
        input.nodes = catNodes.data();
        input.nodeCount = static_cast<int>(catNodes.size());
        input.children = children.data();
        input.childCount = static_cast<int>(children.size());
        input.brushes = brushes.data();
        input.brushCount = static_cast<int>(brushes.size());
        input.maxGapSplitsPerBrush = 128;
        input.maxBrushVertices = static_cast<int>(
            std::min<int64_t>(65535, int64_t(maxVertices) + 512));
        input.maxBrushEdges = static_cast<int>(std::min<int64_t>(
            262140, (int64_t(maxEdges) + 4099) & ~int64_t(3)));
        input.maxBrushPolygons = static_cast<int>(
            std::min<int64_t>(65535, int64_t(maxPolygons) + 512));
        auto exported = buildConvexScene(input);
        snapshot.scene = exported;
        snapshot.vertices = std::move(exported.vertices);
        snapshot.indices = std::move(exported.indices);
        snapshot.triangles = std::move(exported.triangles);
        replacements.emplace(treeID, std::move(snapshot));
      } catch (const std::exception &error) {
        // Preserve the previous snapshot and retry only after a source changes.
        rootIt->second.isDirty = false;
        errors.emplace_back(rootIt->second.userID, error.what());
      }
    }
    for (auto &kv : replacements) {
      auto it = nodes_.find(kv.first);
      if (it != nodes_.end()) {
        it->second.cachedMeshDescriptions.clear();
        it->second.cachedSubmeshes.clear();
      }
    }
    for (auto &kv : replacements)
      g_meshSnapshots[kv.first] = std::move(kv.second);
    auto logger =
        unityMethodsRegistered_ ? unityMethods_.debugLogError : nullptr;
    lock.unlock();
    if (logger)
      for (const auto &error : errors)
        logger(error.second.c_str(), error.first);
    return !replacements.empty();
  } catch (const std::exception &e) {
    if (unityMethodsRegistered_ && unityMethods_.debugLogError)
      unityMethods_.debugLogError(e.what(), 0);
    return false;
  } catch (...) {
    if (unityMethodsRegistered_ && unityMethods_.debugLogError)
      unityMethods_.debugLogError("unknown mesh generation failure", 0);
    return false;
  }
}

bool PluginManager::GenerateMeshDescriptions(
    int32_t treeNodeID, int32_t meshTypeCount, const MeshQuery *meshTypes,
    VertexChannelFlags vertexChannelMask, int32_t *meshDescriptionCount) {
  if (meshDescriptionCount)
    *meshDescriptionCount = 0;
  if (!meshDescriptionCount || meshTypeCount < 0 ||
      (meshTypeCount > 0 && !meshTypes))
    return false;
  try {
    std::lock_guard<std::mutex> lock(mutex_);
    auto ni = nodes_.find(treeNodeID);
    auto si = g_meshSnapshots.find(treeNodeID);
    if (ni == nodes_.end() || ni->second.type != CSGNodeType::Tree ||
        si == g_meshSnapshots.end())
      return false;
    MeshSnapshot next;
    next.scene = si->second.scene;
    next.vertices = si->second.vertices;
    next.indices = si->second.indices;
    next.triangles = si->second.triangles;
    for (int qi = 0; qi < meshTypeCount; ++qi) {
      const auto &query = meshTypes[qi];
      if (!validMeshQuery(query, vertexChannelMask))
        return false;
      const auto channels = requestedChannels(query) &
                            static_cast<std::uint32_t>(vertexChannelMask);
      const auto layerQuery = requestedLayers(query),
                 layerMask = requestedMask(query);
      std::map<int, std::vector<int>> byParam;
      for (int ti = 0; ti < static_cast<int>(next.triangles.size()); ++ti) {
        const auto &tri = next.triangles[ti];
        if (tri.state != 2 && tri.state != 3)
          continue;
        const int triBrush = tri.brushIndex;
        int polygon = -1;
        if (triBrush >= 0) {
          // Scene triangle's surfaceIndex is the original polygon index.
          polygon = tri.surfaceIndex;
        }
        if (polygon < 0)
          continue;
        auto bn = nodes_.find(triBrush);
        if (bn == nodes_.end() || bn->second.type != CSGNodeType::Brush ||
            bn->second.treeID != treeNodeID)
          continue;
        auto bm = brushMeshes_.find(bn->second.brushMeshID);
        if (bm == brushMeshes_.end() ||
            polygon >= static_cast<int>(bm->second.polygons.size()))
          continue;
        const auto &layers = bm->second.polygons[polygon].layers;
        if ((static_cast<std::uint32_t>(layers.layerUsage) & layerMask) !=
            (layerQuery & layerMask))
          continue;
        const int param = layerParameter(layers, requestedParameter(query));
        byParam[param].push_back(ti);
      }
      for (auto &group : byParam) {
        const int meshIndex = static_cast<int>(next.descriptions.size());
        int subIndex = 0;
        const int parameter = requestedParameter(query) ? group.first : 0;
        GeneratedSubmeshData mesh{};
        mesh.meshIndex = meshIndex;
        mesh.surfaceParameter = parameter;
        std::unordered_map<int, int> remap;
        Vector3 mins{std::numeric_limits<float>::max(),
                     std::numeric_limits<float>::max(),
                     std::numeric_limits<float>::max()},
            maxs{-mins.x, -mins.y, -mins.z};
        auto emit = [&]() {
          if (mesh.indices.empty())
            return;
          mesh.subMeshIndex = subIndex;
          mesh.boundsCenter = {(mins.x + maxs.x) * .5f, (mins.y + maxs.y) * .5f,
                               (mins.z + maxs.z) * .5f};
          mesh.boundsSize = {maxs.x - mins.x, maxs.y - mins.y, maxs.z - mins.z};
          GeneratedMeshDescription desc{};
          desc.meshQuery = query;
          desc.surfaceParameter = parameter;
          desc.meshQueryIndex = meshIndex;
          desc.subMeshQueryIndex = subIndex;
          desc.meshQuery.maskAndChannels =
              (desc.meshQuery.maskAndChannels & 0x00ffffffu) | (channels << 24);
          desc.vertexCount = static_cast<int>(mesh.positions.size());
          desc.indexCount = static_cast<int>(mesh.indices.size());
          std::uint64_t gh = 1469598103934665603ull,
                        sh = 1469598103934665603ull;
          gh = hashVector(gh, mesh.positions);
          gh = hashVector(gh, mesh.indices);
          sh = hashVector(sh, mesh.normals);
          sh = hashVector(sh, mesh.tangents);
          sh = hashVector(sh, mesh.uvs);
          desc.geometryHashValue = static_cast<std::int64_t>(gh);
          desc.surfaceHashValue = static_cast<std::int64_t>(sh);
          next.descriptions.push_back(desc);
          next.meshes.push_back(std::move(mesh));
          ++subIndex;
          mesh = GeneratedSubmeshData{};
          mesh.meshIndex = meshIndex;
          mesh.surfaceParameter = parameter;
          remap.clear();
          mins = {std::numeric_limits<float>::max(),
                  std::numeric_limits<float>::max(),
                  std::numeric_limits<float>::max()};
          maxs = {-mins.x, -mins.y, -mins.z};
        };
        for (int ti : group.second) {
          int old[3];
          int required = 0;
          for (int k = 0; k < 3; ++k) {
            old[k] = next.indices[ti * 3 + k];
            if (old[k] < 0 || old[k] >= static_cast<int>(next.vertices.size()))
              throw std::runtime_error("scene triangle index is out of bounds");
            if (remap.find(old[k]) == remap.end())
              ++required;
          }
          if (mesh.positions.size() + required > 65535)
            emit();
          for (int k = 0; k < 3; ++k) {
            auto found = remap.find(old[k]);
            int vi;
            if (found == remap.end()) {
              vi = static_cast<int>(mesh.positions.size());
              remap.emplace(old[k], vi);
              const auto &v = next.vertices[old[k]];
              mesh.positions.push_back(
                  {v.position.x, v.position.y, v.position.z});
              if (channels & VertexChannelFlags_Normal)
                mesh.normals.push_back({v.normal.x, v.normal.y, v.normal.z});
              if (channels & VertexChannelFlags_Tangent)
                mesh.tangents.push_back(
                    {v.tangent.x, v.tangent.y, v.tangent.z, v.tangent.w});
              if (channels & VertexChannelFlags_UV0)
                mesh.uvs.push_back({v.uv.x, v.uv.y});
              mins.x = std::min(mins.x, v.position.x);
              mins.y = std::min(mins.y, v.position.y);
              mins.z = std::min(mins.z, v.position.z);
              maxs.x = std::max(maxs.x, v.position.x);
              maxs.y = std::max(maxs.y, v.position.y);
              maxs.z = std::max(maxs.z, v.position.z);
            } else
              vi = found->second;
            mesh.indices.push_back(vi);
          }
        }
        emit();
      }
    }
    ni->second.cachedMeshDescriptions = next.descriptions;
    ni->second.cachedSubmeshes = next.meshes;
    si->second = std::move(next);
    for (auto &node : nodes_)
      if (node.first == treeNodeID || node.second.treeID == treeNodeID)
        node.second.isDirty = false;
    *meshDescriptionCount =
        static_cast<int>(ni->second.cachedMeshDescriptions.size());
    return true;
  } catch (const std::exception &e) {
    if (unityMethodsRegistered_ && unityMethods_.debugLogError)
      unityMethods_.debugLogError(e.what(), treeNodeID);
    return false;
  } catch (...) {
    return false;
  }
}

bool PluginManager::GetMeshDescriptions(int32_t treeNodeID, int32_t count,
                                        GeneratedMeshDescription *out) {
  if (count < 0 || (count > 0 && !out))
    return false;
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = nodes_.find(treeNodeID);
  if (it == nodes_.end() || it->second.type != CSGNodeType::Tree ||
      static_cast<int>(it->second.cachedMeshDescriptions.size()) != count)
    return false;
  if (count)
    std::memcpy(out, it->second.cachedMeshDescriptions.data(),
                static_cast<std::size_t>(count) * sizeof(*out));
  return true;
}

bool PluginManager::GetGeneratedMesh(int32_t treeNodeID, int32_t meshIndex,
                                     int32_t subMeshIndex, int32_t indexCount,
                                     int32_t *indices, int32_t vertexCount,
                                     Vector3 *positions, Vector4 *tangents,
                                     Vector3 *normals, Vector2 *uvs,
                                     Vector3 *boundsCenter,
                                     Vector3 *boundsSize) {
  if (indexCount < 0 || vertexCount < 0 || !boundsCenter || !boundsSize ||
      (indexCount > 0 && !indices) || (vertexCount > 0 && !positions))
    return false;
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = g_meshSnapshots.find(treeNodeID);
  auto ni = nodes_.find(treeNodeID);
  if (it == g_meshSnapshots.end() || ni == nodes_.end() ||
      ni->second.type != CSGNodeType::Tree)
    return false;
  auto found = std::find_if(
      it->second.meshes.begin(), it->second.meshes.end(), [&](const auto &m) {
        return m.meshIndex == meshIndex && m.subMeshIndex == subMeshIndex;
      });
  if (found == it->second.meshes.end() ||
      static_cast<int>(found->indices.size()) != indexCount ||
      static_cast<int>(found->positions.size()) != vertexCount ||
      (!tangents && !found->tangents.empty()) ||
      (!normals && !found->normals.empty()) || (!uvs && !found->uvs.empty()))
    return false;
  if (indexCount)
    std::memcpy(indices, found->indices.data(), indexCount * sizeof(int32_t));
  if (vertexCount) {
    std::memcpy(positions, found->positions.data(),
                vertexCount * sizeof(Vector3));
    if (tangents && !found->tangents.empty())
      std::memcpy(tangents, found->tangents.data(),
                  vertexCount * sizeof(Vector4));
    if (normals && !found->normals.empty())
      std::memcpy(normals, found->normals.data(),
                  vertexCount * sizeof(Vector3));
    if (uvs && !found->uvs.empty())
      std::memcpy(uvs, found->uvs.data(), vertexCount * sizeof(Vector2));
  }
  *boundsCenter = found->boundsCenter;
  *boundsSize = found->boundsSize;
  return true;
}

} // namespace RealtimeCSG
