#include "realtimecsg/recovered_triangulation.h"
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <vector>

namespace realtimecsg::recovered {
namespace {
Vector3 subtract(Vector3 a, Vector3 b) { return {a.x-b.x,a.y-b.y,a.z-b.z}; }
Vector3 add(Vector3 a, Vector3 b) { return {a.x+b.x,a.y+b.y,a.z+b.z}; }
Vector3 cross(Vector3 a, Vector3 b) {
    return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};
}
float dot(Vector3 a, Vector3 b) { return (a.x*b.x+a.y*b.y)+a.z*b.z; }
Vector3 normalizeDouble(Vector3 a) {
    const double x = a.x, y = a.y, z = a.z;
    const double reciprocal = 1.0/std::sqrt((x*x+y*y)+z*z);
    return {static_cast<float>(x*reciprocal),static_cast<float>(y*reciprocal),static_cast<float>(z*reciprocal)};
}
bool containsOtherPoint(Vector3 normal, const std::vector<TriangleVertexRecord>& vertices,
                        const std::vector<int>& indices, int count, int a, int b, int c) {
    const auto pa = vertices[a].position, pb = vertices[b].position, pc = vertices[c].position;
    const auto ab = subtract(pa,pb), bc = subtract(pb,pc), ca = subtract(pc,pa);
    // Preserve native's cancellation-prone cross(edge, edge+normal), not the
    // algebraically simplified cross(edge, normal).
    const Vector3 normals[] = {normalizeDouble(cross(ab,add(ab,normal))),
        normalizeDouble(cross(bc,add(bc,normal))),normalizeDouble(cross(ca,add(ca,normal)))};
    const float offsets[] = {dot(normals[0],pa)-kPlaneDistanceTolerance,
        dot(normals[1],pb)-kPlaneDistanceTolerance,dot(normals[2],pc)-kPlaneDistanceTolerance};
    for (int i = 0; i < count; ++i) {
        const int id = indices[i];
        if (id == a || id == b || id == c) continue;
        const auto point = vertices[id].position;
        // Native ja/jbe treats unordered values as passing these halfspaces.
        if (0.0f > dot(normals[0],point)-offsets[0]) continue;
        if (0.0f > dot(normals[1],point)-offsets[1]) continue;
        if (!(0.0f > dot(normals[2],point)-offsets[2])) return true;
    }
    return false;
}
struct Candidate { int first, middle, last; bool found; };
Candidate selectEar(const std::vector<TriangleVertexRecord>& vertices, const std::vector<int>& indices,
                    int count, Vector3 normal, bool allowShortEdges) {
    Candidate selected{0,1,2,false};
    float best = -std::numeric_limits<float>::infinity();
    for (int last = 0; last < count; ++last) {
        const int middle = last == 0 ? count-1 : last-1;
        const int first = middle == 0 ? count-1 : middle-1;
        const auto a = vertices[indices[first]].position, b = vertices[indices[middle]].position,
                   c = vertices[indices[last]].position;
        const auto u = subtract(b,a), v = subtract(c,b);
        const float lengthU = dot(u,u), lengthV = dot(v,v);
        const bool shortEdges = kCoincidentVertexDistanceSquared >= lengthU || kCoincidentVertexDistanceSquared >= lengthV;
        float score = 0;
        if (shortEdges) { if (!allowShortEdges) continue; }
        else {
            const auto perpendicular = cross(u,v);
            const float square = dot(perpendicular,perpendicular), product = lengthU*lengthV;
            bool oriented = false;
            if (!(square >= 9.999999439624929e-11f)) {
                const float ratio = dot(u,v)/product;
                if (-0.9999998807907104f >= ratio) continue;
                oriented = ratio >= 0.9999998807907104f;
            }
            score = square/product;
            if (!oriented && !(dot(perpendicular,normal) >= square*9.999999747378752e-6f)) continue;
        }
        if (!(score > best)) continue;
        if (!shortEdges && containsOtherPoint(normal,vertices,indices,count,indices[first],indices[middle],indices[last])) continue;
        selected = {first,middle,last,true}; best = score;
    }
    return selected;
}
}
int triangulateConcavePolygon(TriangleMeshView& mesh, const SurfaceView& surfaces, int planeCount,
    ConcavePolygonView& polygon, TriangleIndexPoolView& pool, int surface, std::uint8_t flags, std::uint16_t state) {
    const int initial = polygon.count;
    if (initial < 0 || initial > polygon.capacity || pool.count < 0 || pool.count > pool.capacity
        || (initial && (!polygon.vertices || !polygon.normals || !polygon.twins)))
        throw std::invalid_argument("Invalid concave polygon views");
    if (initial < 3) return 0;
    if (mesh.triangleCount < 0 || mesh.triangleCount > mesh.triangleCapacity || mesh.triangleCount > (1<<30)
        || mesh.vertexCount < 0 || mesh.vertexCount > mesh.vertexCapacity
        || !mesh.triangles || !mesh.vertices || !surfaces.planes || surface < 0 || surface >= planeCount
        || !pool.indices) throw std::invalid_argument("Invalid concave mesh or surface");
    if (pool.count < initial-2) throw std::length_error("Concave triangle pool exhausted");
    std::vector<unsigned char> occupied(static_cast<std::size_t>(mesh.triangleCount),0);
    for (int i = pool.count-(initial-2); i < pool.count; ++i) {
        const int id = pool.indices[i];
        if (id < 0 || id >= mesh.triangleCount || occupied[id]) throw std::invalid_argument("Invalid concave triangle pool");
        occupied[id] = 1;
    }
    for (int i = 0; i < initial; ++i) {
        const int vertex = polygon.vertices[i]; const auto neighbor = polygon.twins[i]>>2;
        if (vertex < 0 || vertex >= mesh.vertexCount || neighbor >= static_cast<unsigned>(mesh.triangleCount) || occupied[neighbor])
            throw std::invalid_argument("Invalid concave vertex or boundary source");
    }
    std::vector<TriangleRecord> triangles(mesh.triangles,mesh.triangles+mesh.triangleCount);
    std::vector<TriangleVertexRecord> vertices(mesh.vertices,mesh.vertices+mesh.vertexCount);
    std::vector<int> indices(polygon.vertices,polygon.vertices+initial);
    std::vector<Vector3> normals(polygon.normals,polygon.normals+initial);
    std::vector<std::uint32_t> twins(polygon.twins,polygon.twins+initial);
    int count = initial, remaining = pool.count;
    auto linkBack = [&](std::uint32_t neighbor, std::uint32_t edge) {
        if ((neighbor&3u) != 3u) triangles[neighbor>>2].twins[neighbor&3u] = edge;
    };
    auto source = [&](std::uint32_t neighbor) {
        int value;
        // Low bits 3 read +72 in native, physically inside the record but past
        // its edgeSource[3] array. memcpy reproduces this without C++ array UB.
        std::memcpy(&value,reinterpret_cast<const std::byte*>(&triangles[neighbor>>2])+
            offsetof(TriangleRecord,edgeSource)+4*(neighbor&3u),sizeof(value));
        return value;
    };
    while (count >= 3) {
        Candidate ear{0,1,2,true}; const bool final = count == 3;
        if (!final) {
            ear = selectEar(vertices,indices,count,surfaces.planes[surface].normal,false);
            if (!ear.found) ear = selectEar(vertices,indices,count,surfaces.planes[surface].normal,true);
            // No admissible ear: native falls back to the first three corners.
            if (!ear.found) ear = {0,1,2,true};
        }
        const int a = ear.first, b = ear.middle, c = ear.last;
        const int index = pool.indices[--remaining]; const auto edge = static_cast<std::uint32_t>(index)*4;
        vertices[indices[b]].edge = edge; vertices[indices[c]].edge = edge+1; vertices[indices[a]].edge = edge+2;
        auto& triangle = triangles[index];
        triangle.vertices[0] = indices[b]; triangle.vertices[1] = indices[c]; triangle.vertices[2] = indices[a];
        std::memcpy(&triangle.cornerNormals[0],&normals[b],sizeof(Vector3));
        std::memcpy(&triangle.cornerNormals[1],&normals[c],sizeof(Vector3));
        std::memcpy(&triangle.cornerNormals[2],&normals[a],sizeof(Vector3));
        triangle.surfaceIndex = surface; triangle.flags = flags; triangle.categorizationState = state;
        triangle.twins[0] = twins[a]; linkBack(twins[a],edge);
        triangle.twins[1] = twins[b]; linkBack(twins[b],edge+1);
        triangle.edgeSource[0] = source(twins[a]); triangle.edgeSource[1] = source(twins[b]);
        if (final) {
            triangle.twins[2] = twins[c]; linkBack(twins[c],edge+2); triangle.edgeSource[2] = source(twins[c]);
            count = 0;
        } else {
            triangle.twins[2] = kInvalidTriangleEdge; twins[a] = edge+2; triangle.edgeSource[2] = -1;
            const auto tail = static_cast<std::size_t>(count-b-1);
            if (tail) {
                std::memmove(indices.data()+b,indices.data()+b+1,tail*sizeof(int));
                std::memmove(normals.data()+b,normals.data()+b+1,tail*sizeof(Vector3));
                std::memmove(twins.data()+b,twins.data()+b+1,tail*sizeof(std::uint32_t));
            }
            --count;
        }
    }
    std::memcpy(mesh.triangles,triangles.data(),triangles.size()*sizeof(TriangleRecord));
    std::memcpy(mesh.vertices,vertices.data(),vertices.size()*sizeof(TriangleVertexRecord));
    std::memcpy(polygon.vertices,indices.data(),indices.size()*sizeof(int));
    std::memcpy(polygon.normals,normals.data(),normals.size()*sizeof(Vector3));
    std::memcpy(polygon.twins,twins.data(),twins.size()*sizeof(std::uint32_t));
    polygon.count = count; pool.count = remaining;
    return initial-2;
}
}
extern "C" std::int32_t rcsg_recovered_triangulate_concave(realtimecsg::recovered::TriangleMeshView* mesh,
    const realtimecsg::recovered::SurfaceView* surfaces, std::int32_t planeCount,
    realtimecsg::recovered::ConcavePolygonView* polygon, realtimecsg::recovered::TriangleIndexPoolView* pool,
    std::int32_t surface, std::uint8_t flags, std::uint16_t state) {
    if (!mesh || !surfaces || !polygon || !pool) return -1;
    try { return realtimecsg::recovered::triangulateConcavePolygon(*mesh,*surfaces,planeCount,*polygon,*pool,surface,flags,state); }
    catch (const std::invalid_argument&) { return -1; }
    catch (const std::length_error&) { return -2; } catch (...) { return -3; }
}
