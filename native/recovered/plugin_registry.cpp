#include "plugin_manager.h"

#include <array>
#include <cmath>
#include <limits>
#include <set>
#include <string>

namespace RealtimeCSG {
void ForgetGeneratedTreeLocked(int32_t);
void ClearGeneratedScenesLocked();
namespace {

bool Finite(float x) { return std::isfinite(x); }

bool ValidMatrix(const float *m) {
  if (!m)
    return false;
  for (int i = 0; i < 16; ++i)
    if (!Finite(m[i]))
      return false;
  if (std::abs(m[3]) > 1e-6f || std::abs(m[7]) > 1e-6f ||
      std::abs(m[11]) > 1e-6f || std::abs(m[15] - 1.f) > 1e-6f)
    return false;
  // Gauss-Jordan also rejects singular matrices.
  double a[4][8]{};
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c)
      a[r][c] = m[c * 4 + r];
    a[r][r + 4] = 1.0;
  }
  for (int c = 0; c < 4; ++c) {
    int p = c;
    for (int r = c + 1; r < 4; ++r)
      if (std::abs(a[r][c]) > std::abs(a[p][c]))
        p = r;
    if (std::abs(a[p][c]) < 1e-12)
      return false;
    for (int j = 0; j < 8; ++j)
      std::swap(a[p][j], a[c][j]);
    const double d = a[c][c];
    for (int j = 0; j < 8; ++j)
      a[c][j] /= d;
    for (int r = 0; r < 4; ++r)
      if (r != c) {
        const double f = a[r][c];
        for (int j = 0; j < 8; ++j)
          a[r][j] -= f * a[c][j];
      }
  }
  return true;
}

bool Invert(const float *m, float *out) {
  if (!m || !out)
    return false;
  double a[4][8]{};
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c)
      a[r][c] = m[c * 4 + r];
    a[r][r + 4] = 1.0;
  }
  for (int c = 0; c < 4; ++c) {
    int p = c;
    for (int r = c + 1; r < 4; ++r)
      if (std::abs(a[r][c]) > std::abs(a[p][c]))
        p = r;
    if (std::abs(a[p][c]) < 1e-12)
      return false;
    for (int j = 0; j < 8; ++j)
      std::swap(a[p][j], a[c][j]);
    const double d = a[c][c];
    for (int j = 0; j < 8; ++j)
      a[c][j] /= d;
    for (int r = 0; r < 4; ++r)
      if (r != c) {
        const double f = a[r][c];
        for (int j = 0; j < 8; ++j)
          a[r][j] -= f * a[c][j];
      }
  }
  float temp[16];
  for (int r = 0; r < 4; ++r)
    for (int c = 0; c < 4; ++c) {
      temp[c * 4 + r] = static_cast<float>(a[r][c + 4]);
      if (!Finite(temp[c * 4 + r]))
        return false;
    }
  std::memcpy(out, temp, sizeof(temp));
  return true;
}

void Multiply(const float *a, const float *b, float *out) {
  float t[16]{};
  for (int r = 0; r < 4; ++r)
    for (int c = 0; c < 4; ++c)
      for (int k = 0; k < 4; ++k)
        t[c * 4 + r] += a[k * 4 + r] * b[c * 4 + k];
  std::memcpy(out, t, sizeof(t));
}

bool ValidOperation(CSGOperationType op) {
  return op == CSGOperationType::Additive ||
         op == CSGOperationType::Subtractive ||
         op == CSGOperationType::Intersecting;
}

bool BuildMesh(int32_t userID, int32_t vc, const Vector3 *vs, int32_t ec,
               const BrushMeshHalfEdge *es, int32_t pc,
               const BrushMeshPolygon *ps, StoredBrushMesh &out) {
  if (vc <= 0 || ec < 3 || pc <= 0 || !vs || !es || !ps)
    return false;
  for (int32_t i = 0; i < vc; ++i)
    if (!Finite(vs[i].x) || !Finite(vs[i].y) || !Finite(vs[i].z))
      return false;
  for (int32_t i = 0; i < ec; ++i) {
    if (es[i].vertexIndex < 0 || es[i].vertexIndex >= vc ||
        es[i].twinIndex < 0 || es[i].twinIndex >= ec)
      return false;
    const auto &twin = es[es[i].twinIndex];
    if (twin.twinIndex != i)
      return false;
  }
  std::vector<uint8_t> owned(static_cast<size_t>(ec), 0);
  out.userID = userID;
  out.vertices.assign(vs, vs + vc);
  out.halfEdges.assign(es, es + ec);
  out.polygons.clear();
  out.polygons.reserve(pc);
  for (int32_t i = 0; i < pc; ++i) {
    const auto &p = ps[i];
    if (p.firstEdge < 0 || p.edgeCount < 3 || p.firstEdge > ec ||
        p.edgeCount > ec - p.firstEdge)
      return false;
    const auto &s = p.surface;
    const float surfaceValues[] = {s.UV0.U.x, s.UV0.U.y, s.UV0.U.z, s.UV0.U.w,
                                   s.UV0.V.x, s.UV0.V.y, s.UV0.V.z, s.UV0.V.w};
    for (float value : surfaceValues)
      if (!Finite(value))
        return false;
    for (int32_t e = p.firstEdge; e < p.firstEdge + p.edgeCount; ++e) {
      if (owned[static_cast<size_t>(e)]++)
        return false;
    }
    out.polygons.push_back(
        {p.firstEdge, p.edgeCount, p.polygonID, p.surface, p.layers});
  }
  for (auto x : owned)
    if (x != 1)
      return false;
  std::vector<int32_t> nextEdge(static_cast<size_t>(ec), -1);
  for (const auto &p : out.polygons)
    for (int32_t j = 0; j < p.edgeCount; ++j)
      nextEdge[static_cast<size_t>(p.firstEdge + j)] =
          p.firstEdge + (j + p.edgeCount - 1) % p.edgeCount;
  for (int32_t e = 0; e < ec; ++e) {
    const int32_t twin = es[e].twinIndex;
    if (nextEdge[static_cast<size_t>(e)] < 0 ||
        nextEdge[static_cast<size_t>(twin)] < 0 ||
        es[nextEdge[static_cast<size_t>(e)]].vertexIndex !=
            es[twin].vertexIndex ||
        es[nextEdge[static_cast<size_t>(twin)]].vertexIndex !=
            es[e].vertexIndex)
      return false;
  }
  return true;
}

} // namespace

PluginManager &PluginManager::Instance() {
  static PluginManager manager;
  return manager;
}

void PluginManager::RegisterUnityMethods(const UnityMethods *methods) {
  std::lock_guard<std::mutex> lock(mutex_);
  unityMethods_ = methods ? *methods : UnityMethods{};
  unityMethodsRegistered_ = methods != nullptr;
}
void PluginManager::ClearUnityMethods() {
  std::lock_guard<std::mutex> lock(mutex_);
  unityMethods_ = {};
  unityMethodsRegistered_ = false;
}

void PluginManager::ClearAll() {
  std::lock_guard<std::mutex> lock(mutex_);
  nodes_.clear();
  brushMeshes_.clear();
  transformOffsets_.clear();
  ClearGeneratedScenesLocked();
}
void PluginManager::RebuildAll() {
  std::lock_guard<std::mutex> lock(mutex_);
  for (auto &p : nodes_)
    if (p.second.type == CSGNodeType::Tree)
      p.second.isDirty = true;
}
void PluginManager::LogDiagnostics() {
  StringLogFn logger = nullptr;
  std::string s;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!unityMethodsRegistered_ || !unityMethods_.debugLog)
      return;
    logger = unityMethods_.debugLog;
    s = "OpenCSG recovered registry: nodes=" + std::to_string(nodes_.size()) +
        ", brushMeshes=" + std::to_string(brushMeshes_.size()) +
        ", transformOffsets=" + std::to_string(transformOffsets_.size());
  }
  logger(s.c_str(), 0);
}

bool PluginManager::GenerateTree(int32_t userID, int32_t *out) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!out)
    return false;
  Node n;
  n.nodeID = nextNodeID_++;
  n.userID = userID;
  n.type = CSGNodeType::Tree;
  n.treeID = n.nodeID;
  *out = n.nodeID;
  nodes_.emplace(n.nodeID, std::move(n));
  return true;
}
bool PluginManager::GenerateBranch(int32_t userID, int32_t *out) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!out)
    return false;
  Node n;
  n.nodeID = nextNodeID_++;
  n.userID = userID;
  n.type = CSGNodeType::Branch;
  *out = n.nodeID;
  nodes_.emplace(n.nodeID, std::move(n));
  return true;
}
bool PluginManager::GenerateBrush(int32_t userID, int32_t *out) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!out)
    return false;
  Node n;
  n.nodeID = nextNodeID_++;
  n.userID = userID;
  n.type = CSGNodeType::Brush;
  *out = n.nodeID;
  nodes_.emplace(n.nodeID, std::move(n));
  return true;
}

void PluginManager::UpdateTreeHierarchy(int32_t id, int32_t treeID) {
  auto it = nodes_.find(id);
  if (it == nodes_.end())
    return;
  it->second.treeID = treeID;
  for (int32_t child : it->second.children)
    UpdateTreeHierarchy(child, treeID);
}
void PluginManager::MarkAncestorsDirty(int32_t id) {
  std::set<int32_t> visited;
  while (id && visited.insert(id).second) {
    auto it = nodes_.find(id);
    if (it == nodes_.end())
      break;
    it->second.isDirty = true;
    id = it->second.parentID;
  }
}
void PluginManager::UpdateBounds(Node &n) {
  auto mi = brushMeshes_.find(n.brushMeshID);
  if (mi == brushMeshes_.end() || mi->second.vertices.empty()) {
    n.bounds = {-1, 1, -1, 1, -1, 1};
    return;
  }
  float mn[3] = {std::numeric_limits<float>::infinity(),
                 std::numeric_limits<float>::infinity(),
                 std::numeric_limits<float>::infinity()};
  float mx[3] = {-mn[0], -mn[1], -mn[2]};
  const float *m = n.localTransformation;
  for (const auto &v : mi->second.vertices) {
    float q[3] = {m[0] * v.x + m[4] * v.y + m[8] * v.z + m[12],
                  m[1] * v.x + m[5] * v.y + m[9] * v.z + m[13],
                  m[2] * v.x + m[6] * v.y + m[10] * v.z + m[14]};
    for (int k = 0; k < 3; ++k) {
      mn[k] = std::min(mn[k], q[k]);
      mx[k] = std::max(mx[k], q[k]);
    }
  }
  n.bounds = {mn[0], mx[0], mn[1], mx[1], mn[2], mx[2]};
}

bool PluginManager::DestroyNode(int32_t id) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = nodes_.find(id);
  if (it == nodes_.end())
    return false;
  const int32_t parent = it->second.parentID;
  if (parent) {
    auto p = nodes_.find(parent);
    if (p != nodes_.end()) {
      auto &c = p->second.children;
      c.erase(std::remove(c.begin(), c.end(), id), c.end());
      MarkAncestorsDirty(parent);
    }
  }
  auto children = it->second.children;
  ForgetGeneratedTreeLocked(id);
  nodes_.erase(it);
  for (int32_t c : children) {
    auto ci = nodes_.find(c);
    if (ci != nodes_.end()) {
      ci->second.parentID = 0;
      UpdateTreeHierarchy(c, 0);
    }
  }
  return true;
}
bool PluginManager::DestroyNodes(int32_t count, const int32_t *ids) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (count < 0 || (count && !ids))
    return false;
  std::set<int32_t> uniq;
  for (int32_t i = 0; i < count; ++i)
    if (!nodes_.count(ids[i]) || !uniq.insert(ids[i]).second)
      return false;
  for (int32_t i = 0; i < count; ++i) {
    auto it = nodes_.find(ids[i]);
    if (it == nodes_.end())
      continue;
    if (it->second.parentID) {
      auto p = nodes_.find(it->second.parentID);
      if (p != nodes_.end()) {
        auto &c = p->second.children;
        c.erase(std::remove(c.begin(), c.end(), ids[i]), c.end());
        MarkAncestorsDirty(p->first);
      }
    }
    for (int32_t c : it->second.children) {
      auto ci = nodes_.find(c);
      if (ci != nodes_.end() && !uniq.count(c)) {
        ci->second.parentID = 0;
        UpdateTreeHierarchy(c, 0);
      }
    }
    ForgetGeneratedTreeLocked(ids[i]);
    nodes_.erase(it);
  }
  return true;
}
bool PluginManager::DestroyAllNodesWithUserID(int32_t uid) {
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<int32_t> ids;
  for (auto &p : nodes_)
    if (p.second.userID == uid)
      ids.push_back(p.first);
  std::sort(ids.begin(), ids.end());
  for (int32_t id : ids) {
    auto it = nodes_.find(id);
    if (it == nodes_.end())
      continue;
    if (it->second.parentID) {
      auto p = nodes_.find(it->second.parentID);
      if (p != nodes_.end()) {
        auto &c = p->second.children;
        c.erase(std::remove(c.begin(), c.end(), id), c.end());
        MarkAncestorsDirty(p->first);
      }
    }
    for (int32_t c : it->second.children) {
      auto ci = nodes_.find(c);
      if (ci != nodes_.end() && ci->second.userID != uid) {
        ci->second.parentID = 0;
        UpdateTreeHierarchy(c, 0);
      }
    }
    ForgetGeneratedTreeLocked(id);
    nodes_.erase(it);
  }
  return true;
}
bool PluginManager::IsNodeIDValid(int32_t id) const {
  std::lock_guard<std::mutex> lock(mutex_);
  return nodes_.count(id) != 0;
}
CSGNodeType PluginManager::GetTypeOfNode(int32_t id) const {
  std::lock_guard<std::mutex> lock(mutex_);
  auto i = nodes_.find(id);
  return i == nodes_.end() ? CSGNodeType::None : i->second.type;
}
int32_t PluginManager::GetUserIDOfNode(int32_t id) const {
  std::lock_guard<std::mutex> lock(mutex_);
  auto i = nodes_.find(id);
  return i == nodes_.end() ? 0 : i->second.userID;
}
int32_t PluginManager::GetNodeCount() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return static_cast<int32_t>(nodes_.size());
}
int32_t PluginManager::GetTreeCount() const {
  std::lock_guard<std::mutex> lock(mutex_);
  int n = 0;
  for (auto &p : nodes_)
    n += p.second.type == CSGNodeType::Tree;
  return n;
}
int32_t PluginManager::GetBranchCount() const {
  std::lock_guard<std::mutex> lock(mutex_);
  int n = 0;
  for (auto &p : nodes_)
    n += p.second.type == CSGNodeType::Branch;
  return n;
}
int32_t PluginManager::GetBrushCount() const {
  std::lock_guard<std::mutex> lock(mutex_);
  int n = 0;
  for (auto &p : nodes_)
    n += p.second.type == CSGNodeType::Brush;
  return n;
}
bool PluginManager::GetAllTreeNodes(int32_t count, int32_t *out) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (count < 0 || (count && !out))
    return false;
  std::vector<int> v;
  for (auto &p : nodes_)
    v.push_back(p.first);
  std::sort(v.begin(), v.end());
  if (count != static_cast<int>(v.size()))
    return false;
  std::copy(v.begin(), v.end(), out);
  return true;
}
bool PluginManager::GetAllTrees(int32_t count, int32_t *out) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (count < 0 || (count && !out))
    return false;
  std::vector<int> v;
  for (auto &p : nodes_)
    if (p.second.type == CSGNodeType::Tree)
      v.push_back(p.first);
  std::sort(v.begin(), v.end());
  if (count != static_cast<int>(v.size()))
    return false;
  std::copy(v.begin(), v.end(), out);
  return true;
}
int32_t PluginManager::GetParentOfNode(int32_t id) const {
  std::lock_guard<std::mutex> lock(mutex_);
  auto i = nodes_.find(id);
  return i == nodes_.end() ? 0 : i->second.parentID;
}
int32_t PluginManager::GetTreeOfNode(int32_t id) const {
  std::lock_guard<std::mutex> lock(mutex_);
  auto i = nodes_.find(id);
  return i == nodes_.end() ? 0 : i->second.treeID;
}
int32_t PluginManager::FindTreeByUserID(int32_t uid) const {
  std::lock_guard<std::mutex> lock(mutex_);
  for (auto &p : nodes_)
    if (p.second.type == CSGNodeType::Tree && p.second.userID == uid)
      return p.first;
  return 0;
}

bool PluginManager::AddChildNode(int32_t parent, int32_t child) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto p = nodes_.find(parent), c = nodes_.find(child);
  if (p == nodes_.end() || c == nodes_.end() ||
      (p->second.type != CSGNodeType::Tree &&
       p->second.type != CSGNodeType::Branch) ||
      c->second.type == CSGNodeType::Tree || parent == child ||
      std::find(p->second.children.begin(), p->second.children.end(), child) !=
          p->second.children.end())
    return false;
  for (int32_t a = parent; a;) {
    if (a == child)
      return false;
    auto i = nodes_.find(a);
    if (i == nodes_.end())
      break;
    a = i->second.parentID;
  }
  const int32_t old = c->second.parentID;
  if (old) {
    auto o = nodes_.find(old);
    if (o != nodes_.end()) {
      auto &v = o->second.children;
      v.erase(std::remove(v.begin(), v.end(), child), v.end());
      MarkAncestorsDirty(old);
    }
  }
  p = nodes_.find(parent);
  c = nodes_.find(child);
  p->second.children.push_back(child);
  c->second.parentID = parent;
  int32_t tree =
      p->second.type == CSGNodeType::Tree ? parent : p->second.treeID;
  UpdateTreeHierarchy(child, tree);
  MarkAncestorsDirty(parent);
  return true;
}
bool PluginManager::InsertChildNode(int32_t parent, int32_t index,
                                    int32_t child) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto p = nodes_.find(parent), c = nodes_.find(child);
  if (p == nodes_.end() || c == nodes_.end() ||
      (p->second.type != CSGNodeType::Tree &&
       p->second.type != CSGNodeType::Branch) ||
      c->second.type == CSGNodeType::Tree || parent == child || index < 0 ||
      index > static_cast<int32_t>(p->second.children.size()) ||
      std::find(p->second.children.begin(), p->second.children.end(), child) !=
          p->second.children.end())
    return false;
  for (int32_t a = parent; a;) {
    if (a == child)
      return false;
    auto i = nodes_.find(a);
    if (i == nodes_.end())
      break;
    a = i->second.parentID;
  }
  const int32_t old = c->second.parentID;
  if (old) {
    auto o = nodes_.find(old);
    if (o != nodes_.end()) {
      auto &v = o->second.children;
      v.erase(std::remove(v.begin(), v.end(), child), v.end());
      MarkAncestorsDirty(old);
    }
  }
  p = nodes_.find(parent);
  c = nodes_.find(child);
  p->second.children.insert(p->second.children.begin() + index, child);
  c->second.parentID = parent;
  UpdateTreeHierarchy(
      child, p->second.type == CSGNodeType::Tree ? parent : p->second.treeID);
  MarkAncestorsDirty(parent);
  return true;
}
bool PluginManager::InsertChildNodeRange(int32_t parent, int32_t index,
                                         int32_t count, const int32_t *input) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto p = nodes_.find(parent);
  if (p == nodes_.end() ||
      (p->second.type != CSGNodeType::Tree &&
       p->second.type != CSGNodeType::Branch) ||
      count < 0 || (count && !input) || index < 0 ||
      index > static_cast<int32_t>(p->second.children.size()))
    return false;
  std::vector<int32_t> ids;
  if (count)
    ids.assign(input, input + count);
  std::set<int32_t> unique;
  for (int32_t child : ids) {
    auto c = nodes_.find(child);
    if (c == nodes_.end() || c->second.type == CSGNodeType::Tree ||
        child == parent || !unique.insert(child).second ||
        std::find(p->second.children.begin(), p->second.children.end(),
                  child) != p->second.children.end())
      return false;
    for (int32_t a = parent; a;) {
      if (a == child)
        return false;
      auto ai = nodes_.find(a);
      if (ai == nodes_.end())
        break;
      a = ai->second.parentID;
    }
  }
  for (int32_t child : ids) {
    auto c = nodes_.find(child);
    int32_t old = c->second.parentID;
    if (old) {
      auto o = nodes_.find(old);
      if (o != nodes_.end()) {
        auto &v = o->second.children;
        v.erase(std::remove(v.begin(), v.end(), child), v.end());
        MarkAncestorsDirty(old);
      }
    }
  }
  p = nodes_.find(parent);
  p->second.children.insert(p->second.children.begin() + index, ids.begin(),
                            ids.end());
  const int32_t tree =
      p->second.type == CSGNodeType::Tree ? parent : p->second.treeID;
  for (int32_t child : ids) {
    nodes_[child].parentID = parent;
    UpdateTreeHierarchy(child, tree);
  }
  MarkAncestorsDirty(parent);
  return true;
}
bool PluginManager::RemoveChildNode(int32_t parent, int32_t child) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto p = nodes_.find(parent);
  if (p == nodes_.end())
    return false;
  auto i =
      std::find(p->second.children.begin(), p->second.children.end(), child);
  if (i == p->second.children.end())
    return false;
  p->second.children.erase(i);
  auto c = nodes_.find(child);
  if (c != nodes_.end() && c->second.parentID == parent) {
    c->second.parentID = 0;
    UpdateTreeHierarchy(child, 0);
  }
  MarkAncestorsDirty(parent);
  return true;
}
bool PluginManager::RemoveChildNodeAt(int32_t parent, int32_t index) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto p = nodes_.find(parent);
  if (p == nodes_.end() || index < 0 ||
      index >= static_cast<int32_t>(p->second.children.size()))
    return false;
  int32_t child = p->second.children[index];
  p->second.children.erase(p->second.children.begin() + index);
  auto c = nodes_.find(child);
  if (c != nodes_.end() && c->second.parentID == parent) {
    c->second.parentID = 0;
    UpdateTreeHierarchy(child, 0);
  }
  MarkAncestorsDirty(parent);
  return true;
}
bool PluginManager::RemoveChildNodeRange(int32_t parent, int32_t index,
                                         int32_t count) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto p = nodes_.find(parent);
  if (p == nodes_.end() || count < 0 || index < 0 ||
      index > static_cast<int32_t>(p->second.children.size()) ||
      count > static_cast<int32_t>(p->second.children.size()) - index)
    return false;
  for (int32_t j = 0; j < count; ++j) {
    int32_t child = p->second.children[index + j];
    auto c = nodes_.find(child);
    if (c != nodes_.end() && c->second.parentID == parent) {
      c->second.parentID = 0;
      UpdateTreeHierarchy(child, 0);
    }
  }
  p->second.children.erase(p->second.children.begin() + index,
                           p->second.children.begin() + index + count);
  MarkAncestorsDirty(parent);
  return true;
}
bool PluginManager::ClearChildNodes(int32_t parent) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto p = nodes_.find(parent);
  if (p == nodes_.end())
    return false;
  for (int32_t child : p->second.children) {
    auto c = nodes_.find(child);
    if (c != nodes_.end() && c->second.parentID == parent) {
      c->second.parentID = 0;
      UpdateTreeHierarchy(child, 0);
    }
  }
  if (!p->second.children.empty()) {
    p->second.children.clear();
    MarkAncestorsDirty(parent);
  }
  return true;
}
int32_t PluginManager::GetChildNodeCount(int32_t id) const {
  std::lock_guard<std::mutex> lock(mutex_);
  auto i = nodes_.find(id);
  return i == nodes_.end() ? 0
                           : static_cast<int32_t>(i->second.children.size());
}
int32_t PluginManager::GetChildNodeAtIndex(int32_t id, int32_t index) const {
  std::lock_guard<std::mutex> lock(mutex_);
  auto i = nodes_.find(id);
  return i == nodes_.end() || index < 0 ||
                 index >= static_cast<int32_t>(i->second.children.size())
             ? 0
             : i->second.children[index];
}
int32_t PluginManager::IndexOfChildNode(int32_t id, int32_t child) const {
  std::lock_guard<std::mutex> lock(mutex_);
  auto i = nodes_.find(id);
  if (i == nodes_.end())
    return -1;
  auto c =
      std::find(i->second.children.begin(), i->second.children.end(), child);
  return c == i->second.children.end()
             ? -1
             : static_cast<int32_t>(c - i->second.children.begin());
}
bool PluginManager::SetChildNodes(int32_t parent, int32_t count,
                                  const int32_t *input) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto p = nodes_.find(parent);
  if (p == nodes_.end() ||
      (p->second.type != CSGNodeType::Tree &&
       p->second.type != CSGNodeType::Branch) ||
      count < 0 || (count && !input))
    return false;
  std::vector<int32_t> ids;
  if (count)
    ids.assign(input, input + count);
  std::set<int32_t> unique;
  for (int32_t child : ids) {
    auto c = nodes_.find(child);
    if (c == nodes_.end() || child == parent ||
        c->second.type == CSGNodeType::Tree || !unique.insert(child).second)
      return false;
    for (int32_t a = parent; a;) {
      if (a == child)
        return false;
      auto ai = nodes_.find(a);
      if (ai == nodes_.end())
        break;
      a = ai->second.parentID;
    }
  }
  // All validation is complete before detaching anything.
  for (int32_t oldChild : p->second.children) {
    auto c = nodes_.find(oldChild);
    if (c != nodes_.end() && c->second.parentID == parent &&
        !unique.count(oldChild)) {
      c->second.parentID = 0;
      UpdateTreeHierarchy(oldChild, 0);
    }
  }
  for (int32_t child : ids) {
    auto c = nodes_.find(child);
    if (c->second.parentID && c->second.parentID != parent) {
      auto o = nodes_.find(c->second.parentID);
      if (o != nodes_.end()) {
        auto &v = o->second.children;
        v.erase(std::remove(v.begin(), v.end(), child), v.end());
        MarkAncestorsDirty(o->first);
      }
    }
  }
  p = nodes_.find(parent);
  p->second.children = ids;
  int32_t tree =
      p->second.type == CSGNodeType::Tree ? parent : p->second.treeID;
  for (int32_t child : ids) {
    nodes_[child].parentID = parent;
    UpdateTreeHierarchy(child, tree);
  }
  MarkAncestorsDirty(parent);
  return true;
}
bool PluginManager::GetChildNodes(int32_t id, int32_t count, int32_t *out,
                                  int32_t arrayIndex) const {
  std::lock_guard<std::mutex> lock(mutex_);
  auto i = nodes_.find(id);
  if (i == nodes_.end() || count < 0 || arrayIndex < 0 || (count && !out) ||
      arrayIndex > static_cast<int32_t>(i->second.children.size()) ||
      count > static_cast<int32_t>(i->second.children.size()) - arrayIndex)
    return false;
  std::copy_n(i->second.children.begin() + arrayIndex, count, out);
  return true;
}
int32_t PluginManager::GetNumberOfBrushesInTree(int32_t id) const {
  std::lock_guard<std::mutex> lock(mutex_);
  auto root = nodes_.find(id);
  if (root == nodes_.end())
    return 0;
  int count = 0;
  std::vector<int32_t> todo{id};
  std::set<int32_t> seen;
  while (!todo.empty()) {
    auto n = todo.back();
    todo.pop_back();
    if (!seen.insert(n).second)
      continue;
    auto i = nodes_.find(n);
    if (i == nodes_.end())
      continue;
    if (i->second.type == CSGNodeType::Brush)
      ++count;
    todo.insert(todo.end(), i->second.children.begin(),
                i->second.children.end());
  }
  return count;
}
bool PluginManager::DoesTreeContainBrush(int32_t id, int32_t brush) const {
  std::lock_guard<std::mutex> lock(mutex_);
  auto r = nodes_.find(id), b = nodes_.find(brush);
  if (r == nodes_.end() || b == nodes_.end() ||
      b->second.type != CSGNodeType::Brush)
    return false;
  int32_t n = brush;
  std::set<int32_t> seen;
  while (n && seen.insert(n).second) {
    if (n == id)
      return true;
    auto i = nodes_.find(n);
    if (i == nodes_.end())
      break;
    n = i->second.parentID;
  }
  return false;
}

bool PluginManager::IsNodeDirty(int32_t id) const {
  std::lock_guard<std::mutex> lock(mutex_);
  auto i = nodes_.find(id);
  return i != nodes_.end() && i->second.isDirty;
}
bool PluginManager::SetDirty(int32_t id) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!nodes_.count(id))
    return false;
  MarkAncestorsDirty(id);
  return true;
}
bool PluginManager::ClearDirty(int32_t id) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto i = nodes_.find(id);
  if (i == nodes_.end())
    return false;
  i->second.isDirty = false;
  return true;
}
int32_t PluginManager::GetBranchOperationType(int32_t id) const {
  std::lock_guard<std::mutex> lock(mutex_);
  auto i = nodes_.find(id);
  return i == nodes_.end() || i->second.type != CSGNodeType::Branch
             ? -1
             : static_cast<int32_t>(i->second.operation);
}
bool PluginManager::SetBranchOperationType(int32_t id, CSGOperationType op) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto i = nodes_.find(id);
  if (i == nodes_.end() || i->second.type != CSGNodeType::Branch ||
      !ValidOperation(op))
    return false;
  if (i->second.operation != op) {
    i->second.operation = op;
    MarkAncestorsDirty(id);
  }
  return true;
}
int32_t PluginManager::GetBrushOperationType(int32_t id) const {
  std::lock_guard<std::mutex> lock(mutex_);
  auto i = nodes_.find(id);
  return i == nodes_.end() || i->second.type != CSGNodeType::Brush
             ? -1
             : static_cast<int32_t>(i->second.operation);
}
bool PluginManager::SetBrushOperationType(int32_t id, CSGOperationType op) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto i = nodes_.find(id);
  if (i == nodes_.end() || i->second.type != CSGNodeType::Brush ||
      !ValidOperation(op))
    return false;
  if (i->second.operation != op) {
    i->second.operation = op;
    MarkAncestorsDirty(id);
  }
  return true;
}
CSGTreeBrushFlags PluginManager::GetBrushFlags(int32_t id) const {
  std::lock_guard<std::mutex> lock(mutex_);
  auto i = nodes_.find(id);
  return i == nodes_.end() || i->second.type != CSGNodeType::Brush
             ? CSGTreeBrushFlags_Default
             : i->second.flags;
}
bool PluginManager::SetBrushFlags(int32_t id, CSGTreeBrushFlags flags) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto i = nodes_.find(id);
  if (i == nodes_.end() || i->second.type != CSGNodeType::Brush ||
      (flags & ~CSGTreeBrushFlags_Infinite))
    return false;
  if (i->second.flags != flags) {
    i->second.flags = flags;
    MarkAncestorsDirty(id);
  }
  return true;
}
bool PluginManager::SetTreeEnabled(int32_t id, bool enabled) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto i = nodes_.find(id);
  if (i == nodes_.end() || i->second.type != CSGNodeType::Tree)
    return false;
  if (i->second.isEnabled != enabled) {
    i->second.isEnabled = enabled;
    MarkAncestorsDirty(id);
  }
  return true;
}
bool PluginManager::GetTreeEnabled(int32_t id) const {
  std::lock_guard<std::mutex> lock(mutex_);
  auto i = nodes_.find(id);
  return i != nodes_.end() && i->second.type == CSGNodeType::Tree &&
         i->second.isEnabled;
}

bool PluginManager::SetNodeLocalTransformation(int32_t id,
                                               const float *transform) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto i = nodes_.find(id);
  if (i == nodes_.end() || i->second.type != CSGNodeType::Brush ||
      !ValidMatrix(transform))
    return false;
  std::memcpy(i->second.localTransformation, transform, 16 * sizeof(float));
  UpdateBounds(i->second);
  MarkAncestorsDirty(id);
  return true;
}
bool PluginManager::GetNodeLocalTransformation(int32_t id, float *out) const {
  std::lock_guard<std::mutex> lock(mutex_);
  auto i = nodes_.find(id);
  if (i == nodes_.end() || i->second.type != CSGNodeType::Brush || !out)
    return false;
  std::memcpy(out, i->second.localTransformation, 16 * sizeof(float));
  return true;
}
bool PluginManager::GetInvNodeLocalTransformation(int32_t id,
                                                  float *out) const {
  std::lock_guard<std::mutex> lock(mutex_);
  auto i = nodes_.find(id);
  return i != nodes_.end() && i->second.type == CSGNodeType::Brush &&
         Invert(i->second.localTransformation, out);
}
bool PluginManager::GetNodeToTreeSpaceMatrix(int32_t id, float *out) const {
  std::lock_guard<std::mutex> lock(mutex_);
  auto i = nodes_.find(id);
  if (i == nodes_.end() || !out)
    return false;
  for (int k = 0; k < 16; ++k)
    out[k] = (k % 5 == 0) ? 1.f : 0.f;
  std::vector<int32_t> chain;
  std::set<int32_t> seen;
  int32_t n = id;
  while (n && seen.insert(n).second) {
    auto x = nodes_.find(n);
    if (x == nodes_.end())
      return false;
    chain.push_back(n);
    n = x->second.parentID;
  }
  if (n)
    return false;
  for (int32_t current : chain) {
    auto x = nodes_.find(current);
    if (x->second.type == CSGNodeType::Brush)
      Multiply(out, x->second.localTransformation, out);
  }
  return true;
}
bool PluginManager::GetTreeToNodeSpaceMatrix(int32_t id, float *out) const {
  std::lock_guard<std::mutex> lock(mutex_);
  auto i = nodes_.find(id);
  if (i == nodes_.end() || !out)
    return false;
  float m[16];
  for (int k = 0; k < 16; ++k)
    m[k] = (k % 5 == 0) ? 1.f : 0.f;
  std::vector<int32_t> chain;
  std::set<int32_t> seen;
  int32_t n = id;
  while (n && seen.insert(n).second) {
    auto x = nodes_.find(n);
    if (x == nodes_.end())
      return false;
    chain.push_back(n);
    n = x->second.parentID;
  }
  if (n)
    return false;
  for (int32_t current : chain) {
    auto x = nodes_.find(current);
    if (x->second.type == CSGNodeType::Brush) {
      float inv[16];
      if (!Invert(x->second.localTransformation, inv))
        return false;
      Multiply(m, inv, m);
    }
  }
  std::memcpy(out, m, sizeof(m));
  return true;
}
bool PluginManager::GetBrushBounds(int32_t id, AABB *out) const {
  std::lock_guard<std::mutex> lock(mutex_);
  auto i = nodes_.find(id);
  if (i == nodes_.end() || i->second.type != CSGNodeType::Brush || !out)
    return false;
  *out = i->second.bounds;
  return true;
}

int32_t PluginManager::CreateBrushMesh(int32_t uid, int32_t vc,
                                       const Vector3 *vs, int32_t ec,
                                       const BrushMeshHalfEdge *es, int32_t pc,
                                       const BrushMeshPolygon *ps) {
  std::lock_guard<std::mutex> lock(mutex_);
  StoredBrushMesh mesh;
  if (!BuildMesh(uid, vc, vs, ec, es, pc, ps, mesh))
    return 0;
  mesh.id = nextBrushMeshID_++;
  int32_t id = mesh.id;
  brushMeshes_.emplace(id, std::move(mesh));
  return id;
}
bool PluginManager::UpdateBrushMesh(int32_t id, int32_t vc, const Vector3 *vs,
                                    int32_t ec, const BrushMeshHalfEdge *es,
                                    int32_t pc, const BrushMeshPolygon *ps) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto i = brushMeshes_.find(id);
  if (i == brushMeshes_.end())
    return false;
  StoredBrushMesh replacement;
  // Build into a temporary; preserve the old mesh and all referencing brushes
  // on failure.
  if (!BuildMesh(i->second.userID, vc, vs, ec, es, pc, ps, replacement))
    return false;
  replacement.id = id;
  i->second = std::move(replacement);
  for (auto &n : nodes_)
    if (n.second.type == CSGNodeType::Brush && n.second.brushMeshID == id) {
      UpdateBounds(n.second);
      MarkAncestorsDirty(n.first);
    }
  return true;
}
bool PluginManager::DestroyBrushMesh(int32_t id) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto i = brushMeshes_.find(id);
  if (i == brushMeshes_.end())
    return false;
  for (auto &n : nodes_)
    if (n.second.brushMeshID == id)
      return false;
  brushMeshes_.erase(i);
  return true;
}
bool PluginManager::IsBrushMeshIDValid(int32_t id) const {
  std::lock_guard<std::mutex> lock(mutex_);
  return brushMeshes_.count(id) != 0;
}
int32_t PluginManager::GetBrushMeshCount() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return static_cast<int32_t>(brushMeshes_.size());
}
int32_t PluginManager::GetBrushMeshUserID(int32_t id) const {
  std::lock_guard<std::mutex> lock(mutex_);
  auto i = brushMeshes_.find(id);
  return i == brushMeshes_.end() ? 0 : i->second.userID;
}
bool PluginManager::GetAllBrushMeshInstances(int32_t count,
                                             int32_t *out) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (count < 0 || (count && !out))
    return false;
  std::vector<int32_t> ids;
  for (auto &m : brushMeshes_)
    ids.push_back(m.first);
  std::sort(ids.begin(), ids.end());
  if (count != static_cast<int32_t>(ids.size()))
    return false;
  std::copy(ids.begin(), ids.end(), out);
  return true;
}
bool PluginManager::SetBrushMeshID(int32_t node, int32_t mesh) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto n = nodes_.find(node);
  if (n == nodes_.end() || n->second.type != CSGNodeType::Brush ||
      (mesh && !brushMeshes_.count(mesh)))
    return false;
  if (n->second.brushMeshID != mesh) {
    n->second.brushMeshID = mesh;
    UpdateBounds(n->second);
    MarkAncestorsDirty(node);
  }
  return true;
}
int32_t PluginManager::GetBrushMeshID(int32_t node) const {
  std::lock_guard<std::mutex> lock(mutex_);
  auto n = nodes_.find(node);
  return n == nodes_.end() || n->second.type != CSGNodeType::Brush
             ? 0
             : n->second.brushMeshID;
}

int32_t PluginManager::GenerateTransformOffset(int32_t uid) {
  std::lock_guard<std::mutex> lock(mutex_);
  StoredTransformOffset o;
  o.id = nextTransformOffsetID_++;
  o.userID = uid;
  int32_t id = o.id;
  transformOffsets_.emplace(id, o);
  return id;
}
bool PluginManager::DestroyTransformOffset(int32_t id) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto i = transformOffsets_.find(id);
  if (i == transformOffsets_.end())
    return false;
  for (auto &n : nodes_)
    if (n.second.transformOffsetID == id)
      return false;
  transformOffsets_.erase(i);
  return true;
}
bool PluginManager::DestroyTransformOffsets(int32_t count, const int32_t *ids) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (count < 0 || (count && !ids))
    return false;
  std::set<int32_t> unique;
  for (int j = 0; j < count; ++j)
    if (!transformOffsets_.count(ids[j]) || !unique.insert(ids[j]).second)
      return false;
  for (auto &n : nodes_)
    if (unique.count(n.second.transformOffsetID))
      return false;
  for (int id : unique)
    transformOffsets_.erase(id);
  return true;
}
bool PluginManager::IsTransformOffsetIDValid(int32_t id) const {
  std::lock_guard<std::mutex> lock(mutex_);
  return transformOffsets_.count(id) != 0;
}
int32_t PluginManager::GetTransformOffsetCount() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return static_cast<int32_t>(transformOffsets_.size());
}
int32_t PluginManager::GetUserIDOfTransformOffset(int32_t id) const {
  std::lock_guard<std::mutex> lock(mutex_);
  auto i = transformOffsets_.find(id);
  return i == transformOffsets_.end() ? 0 : i->second.userID;
}
bool PluginManager::GetAllTransformOffsets(int32_t count, int32_t *out) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (count < 0 || (count && !out))
    return false;
  std::vector<int32_t> ids;
  for (auto &o : transformOffsets_)
    ids.push_back(o.first);
  std::sort(ids.begin(), ids.end());
  if (count != static_cast<int32_t>(ids.size()))
    return false;
  std::copy(ids.begin(), ids.end(), out);
  return true;
}
bool PluginManager::SetTransformOffsetLocalTransformation(
    int32_t id, const float *transform) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto i = transformOffsets_.find(id);
  if (i == transformOffsets_.end() || !ValidMatrix(transform))
    return false;
  std::memcpy(i->second.localTransformation, transform, 16 * sizeof(float));
  for (auto &n : nodes_)
    if (n.second.transformOffsetID == id)
      MarkAncestorsDirty(n.first);
  return true;
}
bool PluginManager::GetTransformOffsetLocalTransformation(int32_t id,
                                                          float *out) const {
  std::lock_guard<std::mutex> lock(mutex_);
  auto i = transformOffsets_.find(id);
  if (i == transformOffsets_.end() || !out)
    return false;
  std::memcpy(out, i->second.localTransformation, 16 * sizeof(float));
  return true;
}
bool PluginManager::SetNodeTransformOffset(int32_t node, int32_t offset) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto n = nodes_.find(node);
  if (n == nodes_.end() || (offset && !transformOffsets_.count(offset)))
    return false;
  if (n->second.transformOffsetID != offset) {
    n->second.transformOffsetID = offset;
    MarkAncestorsDirty(node);
  }
  return true;
}
int32_t PluginManager::GetNodeTransformOffset(int32_t node) const {
  std::lock_guard<std::mutex> lock(mutex_);
  auto n = nodes_.find(node);
  return n == nodes_.end() ? 0 : n->second.transformOffsetID;
}
bool PluginManager::SetNodeSynchronizedWith(int32_t node, int32_t target) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto n = nodes_.find(node);
  if (n == nodes_.end() || n->second.type != CSGNodeType::Brush)
    return false;
  if (target) {
    auto t = nodes_.find(target);
    if (t == nodes_.end() || t->second.type != CSGNodeType::Brush ||
        target == node)
      return false;
    int32_t walk = target;
    std::set<int32_t> seen;
    while (walk && seen.insert(walk).second) {
      if (walk == node)
        return false;
      auto i = nodes_.find(walk);
      if (i == nodes_.end())
        break;
      walk = i->second.synchronizedWithNodeID;
    }
    if (walk)
      return false;
  }
  n->second.synchronizedWithNodeID = target;
  return true;
}
int32_t PluginManager::GetNodeSynchronizedWith(int32_t node) const {
  std::lock_guard<std::mutex> lock(mutex_);
  auto i = nodes_.find(node);
  return i == nodes_.end() ? 0 : i->second.synchronizedWithNodeID;
}
int32_t PluginManager::GetNodeSynchronizedOrigin(int32_t node) const {
  std::lock_guard<std::mutex> lock(mutex_);
  auto i = nodes_.find(node);
  if (i == nodes_.end() || i->second.type != CSGNodeType::Brush)
    return 0;
  std::set<int32_t> seen;
  int32_t current = node;
  while (current && seen.insert(current).second) {
    auto n = nodes_.find(current);
    if (n == nodes_.end() || !n->second.synchronizedWithNodeID)
      return current;
    current = n->second.synchronizedWithNodeID;
  }
  return 0;
}
int32_t PluginManager::GetAllSynchronizedBrushVariantCount(int32_t node) const {
  std::lock_guard<std::mutex> lock(mutex_);
  auto i = nodes_.find(node);
  if (i == nodes_.end() || i->second.type != CSGNodeType::Brush)
    return 0;
  auto origin = [&](int32_t id) {
    std::set<int32_t> seen;
    int32_t cur = id;
    while (cur && seen.insert(cur).second) {
      auto n = nodes_.find(cur);
      if (n == nodes_.end() || !n->second.synchronizedWithNodeID)
        return cur;
      cur = n->second.synchronizedWithNodeID;
    }
    return 0;
  };
  const int32_t root = origin(node);
  if (!root)
    return 0;
  int count = 0;
  for (auto &n : nodes_)
    if (n.second.type == CSGNodeType::Brush && n.first != node &&
        origin(n.first) == root)
      ++count;
  return count;
}
bool PluginManager::GetAllSynchronizedBrushVariants(int32_t node, int32_t count,
                                                    int32_t *out) const {
  std::lock_guard<std::mutex> lock(mutex_);
  auto i = nodes_.find(node);
  if (i == nodes_.end() || i->second.type != CSGNodeType::Brush || count < 0 ||
      (count && !out))
    return false;
  auto origin = [&](int32_t id) {
    std::set<int32_t> seen;
    int32_t cur = id;
    while (cur && seen.insert(cur).second) {
      auto n = nodes_.find(cur);
      if (n == nodes_.end() || !n->second.synchronizedWithNodeID)
        return cur;
      cur = n->second.synchronizedWithNodeID;
    }
    return 0;
  };
  const int32_t root = origin(node);
  std::vector<int32_t> variants;
  if (!root)
    return count == 0;
  for (auto &n : nodes_)
    if (n.second.type == CSGNodeType::Brush && n.first != node &&
        origin(n.first) == root)
      variants.push_back(n.first);
  std::sort(variants.begin(), variants.end());
  if (count != static_cast<int32_t>(variants.size()))
    return false;
  std::copy(variants.begin(), variants.end(), out);
  return true;
}

} // namespace RealtimeCSG
