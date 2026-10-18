#include "plugin_manager.h"
#include "realtimecsg/recovered_scene.h"
#include <array>
#include <limits>
#include <map>
#include <set>

namespace RealtimeCSG {
bool CopyGeneratedSceneLocked(int32_t, realtimecsg::recovered::ExportedScene &);
struct PluginAccess {
  static bool Brush(int32_t id, Node &node, StoredBrushMesh &mesh) {
    auto &m = PluginManager::Instance();
    std::lock_guard<std::mutex> lock(m.mutex_);
    auto it = m.nodes_.find(id);
    if (it == m.nodes_.end() || it->second.type != CSGNodeType::Brush)
      return false;
    auto source = m.brushMeshes_.find(it->second.brushMeshID);
    if (source == m.brushMeshes_.end())
      return false;
    node = it->second;
    mesh = source->second;
    return true;
  }
  static std::vector<int32_t> Brushes(int32_t tree) {
    auto &m = PluginManager::Instance();
    std::lock_guard<std::mutex> lock(m.mutex_);
    std::vector<int32_t> ids;
    for (const auto &item : m.nodes_)
      if (item.second.treeID == tree && item.second.type == CSGNodeType::Brush)
        ids.push_back(item.first);
    std::sort(ids.begin(), ids.end());
    return ids;
  }
  static bool Scene(int32_t tree,
                    realtimecsg::recovered::ExportedScene &scene) {
    auto &m = PluginManager::Instance();
    std::lock_guard<std::mutex> lock(m.mutex_);
    return CopyGeneratedSceneLocked(tree, scene);
  }
};
} // namespace RealtimeCSG
namespace {
using RealtimeCSG::Node;
using RealtimeCSG::PluginAccess;
using RealtimeCSG::StoredBrushMesh;
Vector3 add(Vector3 a, Vector3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
Vector3 sub(Vector3 a, Vector3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
Vector3 mul(Vector3 a, double s) {
  return {float(a.x * s), float(a.y * s), float(a.z * s)};
}
double dot(Vector3 a, Vector3 b) {
  return double(a.x) * b.x + double(a.y) * b.y + double(a.z) * b.z;
}
Vector3 cross(Vector3 a, Vector3 b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
Vector3 unit(Vector3 a) {
  double n = std::sqrt(dot(a, a));
  return n > 0 ? mul(a, 1 / n) : Vector3{};
}
Vector3 point(const float *m, Vector3 p) {
  return {m[0] * p.x + m[4] * p.y + m[8] * p.z + m[12],
          m[1] * p.x + m[5] * p.y + m[9] * p.z + m[13],
          m[2] * p.x + m[6] * p.y + m[10] * p.z + m[14]};
}
void product(const float *a, const float *b, float *out) {
  for (int c = 0; c < 4; ++c)
    for (int r = 0; r < 4; ++r) {
      out[c * 4 + r] = 0;
      for (int k = 0; k < 4; ++k)
        out[c * 4 + r] += a[k * 4 + r] * b[c * 4 + k];
    }
}
bool inverse(const float *m, float *out) {
  if (!m)
    return false;
  double a[4][8]{};
  for (int r = 0; r < 4; ++r)
    for (int c = 0; c < 4; ++c) {
      if (!std::isfinite(m[c * 4 + r]))
        return false;
      a[r][c] = m[c * 4 + r];
      a[r][c + 4] = r == c;
    }
  for (int c = 0; c < 4; ++c) {
    int p = c;
    for (int r = c + 1; r < 4; ++r)
      if (std::abs(a[r][c]) > std::abs(a[p][c]))
        p = r;
    if (std::abs(a[p][c]) < 1e-20)
      return false;
    for (int j = 0; j < 8; ++j)
      std::swap(a[c][j], a[p][j]);
    double q = a[c][c];
    for (double &v : a[c])
      v /= q;
    for (int r = 0; r < 4; ++r)
      if (r != c) {
        q = a[r][c];
        for (int j = 0; j < 8; ++j)
          a[r][j] -= q * a[c][j];
      }
  }
  for (int r = 0; r < 4; ++r)
    for (int c = 0; c < 4; ++c)
      out[c * 4 + r] = float(a[r][c + 4]);
  return true;
}
Plane facePlane(const StoredBrushMesh &mesh, int surface) {
  const auto &p = mesh.polygons.at(surface);
  Vector3 n{}, o = mesh.vertices[mesh.halfEdges[p.firstEdge].vertexIndex];
  for (int c = 0; c < p.edgeCount; ++c) {
    auto a = mesh.vertices[mesh.halfEdges[p.firstEdge + c].vertexIndex],
         b = mesh.vertices[mesh.halfEdges[p.firstEdge + (c + 1) % p.edgeCount]
                               .vertexIndex];
    n = add(n, cross(sub(a, o), sub(b, o)));
  }
  n = unit(n);
  return {n, float(-dot(n, o))};
}
Plane transformPlane(Plane p, const float *m) {
  float inv[16];
  if (!inverse(m, inv))
    return {};
  float v[4] = {p.normal.x, p.normal.y, p.normal.z, p.distance}, q[4]{};
  for (int r = 0; r < 4; ++r)
    for (int c = 0; c < 4; ++c)
      q[r] += inv[r * 4 + c] * v[c];
  double l = std::sqrt(double(q[0]) * q[0] + double(q[1]) * q[1] +
                       double(q[2]) * q[2]);
  if (!(l > 0))
    return {};
  return {{float(q[0] / l), float(q[1] / l), float(q[2] / l)}, float(q[3] / l)};
}
int surfaceIndex(const StoredBrushMesh &mesh, int id) {
  for (size_t i = 0; i < mesh.polygons.size(); ++i)
    if (mesh.polygons[i].polygonID == id)
      return int(i);
  return -1;
}
Vector2 uv(const Node &node, const StoredBrushMesh &mesh, int surface,
           Vector3 local) {
  const auto &s = mesh.polygons[surface].surface;
  Vector3 p = (s.surfaceFlags & SurfaceFlags_TextureIsInWorldSpace)
                  ? point(node.localTransformation, local)
                  : local;
  return {s.UV0.U.x * p.x + s.UV0.U.y * p.y + s.UV0.U.z * p.z + s.UV0.U.w,
          s.UV0.V.x * p.x + s.UV0.V.y * p.y + s.UV0.V.z * p.z + s.UV0.V.w};
}
bool rayTriangle(Vector3 start, Vector3 end, Vector3 a, Vector3 b, Vector3 c,
                 double &t) {
  auto d = sub(end, start), e = sub(b, a), f = sub(c, a), p = cross(d, f);
  double det = dot(e, p);
  if (std::abs(det) < 1e-12)
    return false;
  auto s = sub(start, a);
  double u = dot(s, p) / det;
  if (u < -1e-6 || u > 1 + 1e-6)
    return false;
  auto q = cross(s, e);
  double v = dot(d, q) / det;
  if (v < -1e-6 || u + v > 1 + 1e-6)
    return false;
  t = dot(f, q) / det;
  return t >= 0 && t <= 1;
}
bool fillHit(const Node &node, const StoredBrushMesh &mesh, int surface,
             const float *world, Vector3 start, Vector3 end, double t,
             CSGTreeBrushIntersection &out) {
  float all[16], inv[16];
  product(world, node.localTransformation, all);
  if (!inverse(all, inv))
    return false;
  auto hit = add(start, mul(sub(end, start), t)), local = point(inv, hit);
  auto plane = facePlane(mesh, surface);
  out = {node.treeID,
         node.nodeID,
         mesh.polygons[surface].polygonID,
         node.userID,
         {plane, transformPlane(plane, node.localTransformation),
          transformPlane(plane, all), hit, uv(node, mesh, surface, local),
          float(std::sqrt(dot(sub(hit, start), sub(hit, start))))}};
  return true;
}
bool castSource(int id, int surfaceID, const Vector3 *start, const Vector3 *end,
                const float *world, bool ignore,
                CSGTreeBrushIntersection *out) {
  if (!start || !end || !world || !out)
    return false;
  Node node;
  StoredBrushMesh mesh;
  if (!PluginAccess::Brush(id, node, mesh))
    return false;
  float all[16], inv[16];
  product(world, node.localTransformation, all);
  if (!inverse(all, inv))
    return false;
  auto a = point(inv, *start), b = point(inv, *end);
  double best = 2;
  int selected = -1;
  for (int p = 0; p < int(mesh.polygons.size()); ++p) {
    const auto &poly = mesh.polygons[p];
    if (surfaceID >= 0 && poly.polygonID != surfaceID)
      continue;
    if (ignore && !(poly.layers.layerUsage & LayerUsageFlags_Renderable))
      continue;
    auto o = mesh.vertices[mesh.halfEdges[poly.firstEdge].vertexIndex];
    for (int c = 1; c + 1 < poly.edgeCount; ++c) {
      double t;
      auto
          v = mesh.vertices[mesh.halfEdges[poly.firstEdge + c].vertexIndex],
          w = mesh.vertices[mesh.halfEdges[poly.firstEdge + c + 1].vertexIndex];
      if (rayTriangle(a, b, o, v, w, t) && t < best) {
        best = t;
        selected = p;
      }
    }
  }
  return selected >= 0 &&
         fillHit(node, mesh, selected, world, *start, *end, best, *out);
}
thread_local std::vector<CSGTreeBrushIntersection> rayResults;
thread_local std::vector<int32_t> frustumNodes, frustumUsers;
thread_local std::vector<std::vector<Vector2>> pieces;
struct Outline {
  std::vector<Vector3> vertices;
  std::vector<int32_t> lines, triangles;
};
bool outline(int id, int surfaceID, Outline &result) {
  Node node;
  StoredBrushMesh mesh;
  if (!PluginAccess::Brush(id, node, mesh))
    return false;
  result.vertices = mesh.vertices;
  std::set<std::pair<int, int>> edges;
  for (const auto &p : mesh.polygons) {
    if (surfaceID >= 0 && p.polygonID != surfaceID)
      continue;
    for (int c = 0; c < p.edgeCount; ++c) {
      int a = mesh.halfEdges[p.firstEdge + c].vertexIndex,
          b = mesh.halfEdges[p.firstEdge + (c + 1) % p.edgeCount].vertexIndex;
      edges.insert(std::minmax(a, b));
    }
    if (surfaceID >= 0)
      for (int c = 1; c + 1 < p.edgeCount; ++c) {
        result.triangles.push_back(mesh.halfEdges[p.firstEdge].vertexIndex);
        result.triangles.push_back(mesh.halfEdges[p.firstEdge + c].vertexIndex);
        result.triangles.push_back(
            mesh.halfEdges[p.firstEdge + c + 1].vertexIndex);
      }
  }
  for (auto edge : edges) {
    result.lines.push_back(edge.first);
    result.lines.push_back(edge.second);
  }
  return surfaceID < 0 || surfaceIndex(mesh, surfaceID) >= 0;
}
template <class T> bool copy(const std::vector<T> &source, int count, T *dest) {
  if (count != int(source.size()) || (count && !dest))
    return false;
  if (count)
    std::copy(source.begin(), source.end(), dest);
  return true;
}
} // namespace
extern "C" {
int32_t RayCastIntoBrush(int32_t id, const Vector3 *a, const Vector3 *b,
                         const float *m, int32_t ignore,
                         CSGTreeBrushIntersection *out) {
  try {
    return castSource(id, -1, a, b, m, ignore != 0, out);
  } catch (...) {
    return 0;
  }
}
int32_t RayCastIntoBrushSurface(int32_t id, int32_t surface, const Vector3 *a,
                                const Vector3 *b, const float *m,
                                CSGSurfaceIntersection *out) {
  try {
    CSGTreeBrushIntersection hit{};
    if (!out || !castSource(id, surface, a, b, m, false, &hit))
      return 0;
    *out = hit.intersection;
    return 1;
  } catch (...) {
    return 0;
  }
}
int32_t RayCastIntoTreeMultiCount(int32_t tree, const Vector3 *a,
                                  const Vector3 *b, const float *m,
                                  int32_t filter, int32_t ignore,
                                  const int32_t *ignored, int32_t count) {
  try {
    rayResults.clear();
    if (!a || !b || !m || count < 0 || (count && !ignored))
      return 0;
    realtimecsg::recovered::ExportedScene scene;
    if (!PluginAccess::Scene(tree, scene))
      return 0;
    std::map<int32_t, CSGTreeBrushIntersection> nearest;
    for (size_t t = 0; t < scene.triangles.size(); ++t) {
      auto origin = scene.triangles[t];
      if (count && std::find(ignored, ignored + count, origin.brushIndex) !=
                       ignored + count)
        continue;
      Node node;
      StoredBrushMesh mesh;
      if (!PluginAccess::Brush(origin.brushIndex, node, mesh) ||
          origin.surfaceIndex < 0 ||
          origin.surfaceIndex >= int(mesh.polygons.size()))
        continue;
      const auto &poly = mesh.polygons[origin.surfaceIndex];
      if ((ignore && !(poly.layers.layerUsage & LayerUsageFlags_Renderable)) ||
          (filter && poly.layers.layerParameter1 != filter))
        continue;
      Vector3 p[3];
      for (int c = 0; c < 3; ++c) {
        auto v = scene.vertices[scene.indices[t * 3 + c]].position;
        p[c] = point(m, {v.x, v.y, v.z});
      }
      double at;
      if (rayTriangle(*a, *b, p[0], p[1], p[2], at)) {
        CSGTreeBrushIntersection hit;
        if (fillHit(node, mesh, origin.surfaceIndex, m, *a, *b, at, hit) &&
            (!nearest.count(node.nodeID) ||
             hit.intersection.distance <
                 nearest[node.nodeID].intersection.distance))
          nearest[node.nodeID] = hit;
      }
    }
    for (const auto &item : nearest)
      rayResults.push_back(item.second);
    std::sort(rayResults.begin(), rayResults.end(),
              [](const auto &x, const auto &y) {
                return x.intersection.distance < y.intersection.distance;
              });
    return int(rayResults.size());
  } catch (...) {
    rayResults.clear();
    return 0;
  }
}
int32_t RayCastMultiGet(int32_t count, CSGTreeBrushIntersection *out) {
  return copy(rayResults, count, out);
}
int32_t FindNodesInFrustum(int32_t tree, int32_t count, const Plane *planes) {
  try {
    frustumNodes.clear();
    frustumUsers.clear();
    if (count < 0 || (count && !planes))
      return 0;
    for (int id : PluginAccess::Brushes(tree)) {
      Node node;
      StoredBrushMesh mesh;
      if (!PluginAccess::Brush(id, node, mesh))
        continue;
      bool inside = true;
      for (int p = 0; p < count; ++p) {
        bool any = false;
        for (auto v : mesh.vertices)
          if (dot(planes[p].normal, point(node.localTransformation, v)) +
                  planes[p].distance >=
              0) {
            any = true;
            break;
          }
        if (!any) {
          inside = false;
          break;
        }
      }
      if (inside) {
        frustumNodes.push_back(id);
        frustumUsers.push_back(node.userID);
      }
    }
    return int(frustumNodes.size());
  } catch (...) {
    return 0;
  }
}
int32_t RetrieveUserIDsInFrustum(int32_t count, int32_t *out) {
  return copy(frustumUsers, count, out);
}
int32_t RetrieveNodesInFrustum(int32_t count, int32_t *out) {
  return copy(frustumNodes, count, out);
}
uint64_t GetBrushOutlineGeneration(int32_t id) {
  try {
    Node node;
    StoredBrushMesh mesh;
    if (!PluginAccess::Brush(id, node, mesh))
      return 0;
    uint64_t hash = 14695981039346656037ULL;
    auto bytes = [&](const void *source, size_t count) {
      const auto *data = static_cast<const unsigned char *>(source);
      for (size_t i = 0; i < count; ++i) {
        hash ^= data[i];
        hash *= 1099511628211ULL;
      }
    };
    bytes(mesh.vertices.data(), mesh.vertices.size() * sizeof(Vector3));
    bytes(mesh.halfEdges.data(),
          mesh.halfEdges.size() * sizeof(BrushMeshHalfEdge));
    for (const auto &polygon : mesh.polygons) {
      bytes(&polygon.firstEdge, sizeof(polygon.firstEdge));
      bytes(&polygon.edgeCount, sizeof(polygon.edgeCount));
      bytes(&polygon.polygonID, sizeof(polygon.polygonID));
    }
    return hash ? hash : 1;
  } catch (...) {
    return 0;
  }
}
int32_t GetBrushOutlineSizes(int32_t id, int32_t *v, int32_t *vo, int32_t *vi,
                             int32_t *io, int32_t *ii, int32_t *invalid) {
  try {
    Outline o;
    if (!v || !vo || !vi || !io || !ii || !invalid || !outline(id, -1, o))
      return 0;
    *v = int(o.vertices.size());
    *vo = int(o.lines.size());
    *vi = *io = *ii = *invalid = 0;
    return 1;
  } catch (...) {
    return 0;
  }
}
int32_t GetBrushOutlineValues(int32_t id, int32_t vc, Vector3 *v, int32_t voc,
                              int32_t *vo, int32_t vic, int32_t *, int32_t ioc,
                              int32_t *, int32_t iic, int32_t *,
                              int32_t invalid, int32_t *) {
  try {
    Outline o;
    if (vic || ioc || iic || invalid || !outline(id, -1, o) ||
        vc != int(o.vertices.size()) || voc != int(o.lines.size()) ||
        (vc && !v) || (voc && !vo))
      return 0;
    return copy(o.vertices, vc, v) && copy(o.lines, voc, vo);
  } catch (...) {
    return 0;
  }
}
int32_t GetSurfaceOutlineSizes(int32_t id, int32_t surface, int32_t *v,
                               int32_t *vo, int32_t *vi, int32_t *tri,
                               int32_t *io, int32_t *ii, int32_t *invalid) {
  try {
    Outline o;
    if (!v || !vo || !vi || !tri || !io || !ii || !invalid ||
        !outline(id, surface, o))
      return 0;
    *v = int(o.vertices.size());
    *vo = int(o.lines.size());
    *tri = int(o.triangles.size());
    *vi = *io = *ii = *invalid = 0;
    return 1;
  } catch (...) {
    return 0;
  }
}
int32_t GetSurfaceOutlineValues(int32_t id, int32_t surface, int32_t vc,
                                Vector3 *v, int32_t voc, int32_t *vo,
                                int32_t vic, int32_t *, int32_t tc,
                                int32_t *tri, int32_t ioc, int32_t *,
                                int32_t iic, int32_t *, int32_t invalid,
                                int32_t *) {
  try {
    Outline o;
    if (vic || ioc || iic || invalid || !outline(id, surface, o) ||
        vc != int(o.vertices.size()) || voc != int(o.lines.size()) ||
        tc != int(o.triangles.size()) || (vc && !v) || (voc && !vo) ||
        (tc && !tri))
      return 0;
    return copy(o.vertices, vc, v) && copy(o.lines, voc, vo) &&
           copy(o.triangles, tc, tri);
  } catch (...) {
    return 0;
  }
}
int32_t ConvertWorldToTextureCoord(int32_t id, int32_t surface, Matrix4x4 world,
                                   Vector3 position, Vector2 *out) {
  try {
    Node node;
    StoredBrushMesh mesh;
    float all[16], inv[16];
    if (!out || !PluginAccess::Brush(id, node, mesh))
      return 0;
    int p = surfaceIndex(mesh, surface);
    if (p < 0)
      return 0;
    product(world.values, node.localTransformation, all);
    if (!inverse(all, inv))
      return 0;
    *out = uv(node, mesh, p, point(inv, position));
    return 1;
  } catch (...) {
    return 0;
  }
}
int32_t ConvertTextureToWorldCoord(int32_t id, int32_t surface, float u,
                                   float v, const float *world, float *x,
                                   float *y, float *z) {
  try {
    Node node;
    StoredBrushMesh mesh;
    if (!world || !x || !y || !z || !PluginAccess::Brush(id, node, mesh))
      return 0;
    int p = surfaceIndex(mesh, surface);
    if (p < 0)
      return 0;
    const auto &s = mesh.polygons[p].surface;
    auto plane = facePlane(mesh, p);
    bool common = s.surfaceFlags & SurfaceFlags_TextureIsInWorldSpace;
    if (common)
      plane = transformPlane(plane, node.localTransformation);
    Vector3 a{s.UV0.U.x, s.UV0.U.y, s.UV0.U.z},
        b{s.UV0.V.x, s.UV0.V.y, s.UV0.V.z};
    double determinant = dot(a, cross(b, plane.normal));
    if (std::abs(determinant) < 1e-12)
      return 0;
    auto q = mul(add(add(mul(cross(b, plane.normal), u - s.UV0.U.w),
                         mul(cross(plane.normal, a), v - s.UV0.V.w)),
                     mul(cross(a, b), -plane.distance)),
                 1 / determinant);
    auto result = point(world, common ? q : point(node.localTransformation, q));
    if (!std::isfinite(result.x) || !std::isfinite(result.y) ||
        !std::isfinite(result.z))
      return 0;
    *x = result.x;
    *y = result.y;
    *z = result.z;
    return 1;
  } catch (...) {
    return 0;
  }
}
int32_t GetSurfaceMinMaxTexCoords(int32_t id, int32_t surface, Matrix4x4,
                                  Vector2 *lo, Vector2 *hi) {
  try {
    Node node;
    StoredBrushMesh mesh;
    if (!lo || !hi || !PluginAccess::Brush(id, node, mesh))
      return 0;
    int p = surfaceIndex(mesh, surface);
    if (p < 0)
      return 0;
    const auto &poly = mesh.polygons[p];
    Vector2 lower{INFINITY, INFINITY}, upper{-INFINITY, -INFINITY};
    for (int c = 0; c < poly.edgeCount; ++c) {
      auto q =
          uv(node, mesh, p,
             mesh.vertices[mesh.halfEdges[poly.firstEdge + c].vertexIndex]);
      lower = {std::min(lower.x, q.x), std::min(lower.y, q.y)};
      upper = {std::max(upper.x, q.x), std::max(upper.y, q.y)};
    }
    *lo = lower;
    *hi = upper;
    return 1;
  } catch (...) {
    return 0;
  }
}
int32_t DecomposeStart(int32_t count, const Vector2 *vertices, int32_t *out) {
  try {
    pieces.clear();
    if (out) *out = 0;
    if (!out || count < 3 || count > 4096 || !vertices)
      return 0;
    std::vector<int> ring;
    double area = 0;
    for (int i = 0; i < count; ++i) {
      if (!std::isfinite(vertices[i].x) || !std::isfinite(vertices[i].y))
        return 0;
      for (int j = 0; j < i; ++j)
        if (vertices[i].x == vertices[j].x && vertices[i].y == vertices[j].y)
          return 0;
      ring.push_back(i);
      area += double(vertices[i].x) * vertices[(i + 1) % count].y -
              double(vertices[i].y) * vertices[(i + 1) % count].x;
    }
    if (area == 0)
      return 0;
    double sign = area > 0 ? 1 : -1;
    auto side = [](Vector2 a, Vector2 b, Vector2 c) {
      return (double(b.x) - a.x) * (double(c.y) - a.y) -
             (double(b.y) - a.y) * (double(c.x) - a.x);
    };
    // This API partitions into convex polygons, not triangles. Retain a
    // convex outline so extruded generators create one brush and whole caps.
    // Check all vertices against every edge: local turns alone accept stars.
    bool convex = true;
    for (int i = 0; i < count && convex; ++i)
      for (int j = 0; j < count; ++j)
        if (side(vertices[i], vertices[(i + 1) % count], vertices[j]) * sign < 0) {
          convex = false;
          break;
        }
    if (convex) {
      pieces.emplace_back(vertices, vertices + count);
      *out = 1;
      return 1;
    }
    while (ring.size() > 3) {
      bool found = false;
      for (size_t c = 0; c < ring.size(); ++c) {
        int a = ring[(c + ring.size() - 1) % ring.size()], b = ring[c],
            d = ring[(c + 1) % ring.size()];
        double turn = side(vertices[a], vertices[b], vertices[d]) * sign;
        if (turn <= 0)
          continue;
        bool contains = false;
        for (int k : ring)
          if (k != a && k != b && k != d &&
              side(vertices[a], vertices[b], vertices[k]) * sign >= 0 &&
              side(vertices[b], vertices[d], vertices[k]) * sign >= 0 &&
              side(vertices[d], vertices[a], vertices[k]) * sign >= 0) {
            contains = true;
            break;
          }
        if (contains)
          continue;
        pieces.push_back({vertices[a], vertices[b], vertices[d]});
        ring.erase(ring.begin() + c);
        found = true;
        break;
      }
      if (!found) {
        pieces.clear();
        return 0;
      }
    }
    pieces.push_back({vertices[ring[0]], vertices[ring[1]], vertices[ring[2]]});
    *out = int(pieces.size());
    return 1;
  } catch (...) {
    pieces.clear();
    return 0;
  }
}
int32_t DecomposeGetSizes(int32_t count, int32_t *out) {
  if (count != int(pieces.size()) || (count && !out))
    return 0;
  for (int i = 0; i < count; ++i)
    out[i] = int(pieces[i].size());
  return 1;
}
int32_t DecomposeGetPolygon(int32_t p, int32_t count, Vector2 *out) {
  return p >= 0 && p < int(pieces.size()) && copy(pieces[p], count, out);
}
}
