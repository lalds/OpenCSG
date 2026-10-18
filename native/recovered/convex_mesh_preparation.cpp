#include "realtimecsg/recovered_convex_csg.h"
#include "realtimecsg/recovered_mesh_optimization.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>
#include <utility>

namespace realtimecsg::recovered {
namespace {
int bounded(std::uint64_t n) {
    if (n>static_cast<std::uint64_t>(std::numeric_limits<int>::max()))
        throw std::length_error("Convex preparation exceeds index limits");
    return static_cast<int>(n);
}
template<class T> std::vector<T> copy(const T* data, int count) {
    if (count<0 || (count && !data)) throw std::invalid_argument("Missing convex preparation buffer");
    if (!count) return {};
    return {data,data+count};
}
BrushEdgeBounds bounds(const BrushMemoryView& mesh) {
    if (mesh.vertexCount<=0 || !mesh.vertices) throw std::invalid_argument("Convex source has no vertices");
    auto p=mesh.vertices[0]; BrushEdgeBounds b{p.x,p.x,p.y,p.y,p.z,p.z};
    for (int v=0;v<mesh.vertexCount;++v) {
        p=mesh.vertices[v];
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
            throw std::invalid_argument("Nonfinite convex source vertex");
        b.minX=std::min(b.minX,p.x); b.maxX=std::max(b.maxX,p.x);
        b.minY=std::min(b.minY,p.y); b.maxY=std::max(b.maxY,p.y);
        b.minZ=std::min(b.minZ,p.z); b.maxZ=std::max(b.maxZ,p.z);
    }
    return b;
}
struct ContextStorage {
    std::vector<std::uint32_t> boundary;
    std::vector<int> removed,incident,removedVertices,polygon;
    std::vector<Vector3> normals;
    PlaneOptimizationContextView view;
    ContextStorage(int tc, int vc):boundary(tc),removed(tc),incident(tc),removedVertices(vc),polygon(vc),normals(vc),
        view{{boundary.data(),0,tc},{removed.data(),0,tc},{incident.data(),0,tc},
             {removedVertices.data(),0,vc},{polygon.data(),0,vc},{normals.data(),0,vc}} {}
};
PreparedConvexBrush prepare(const ConvexMeshInput* inputs, int count, int index,
                           const std::vector<BrushEdgeBounds>& boxes, int foreignPlaneCapacity) {
    const auto& input=inputs[index]; const auto& source=*input.brush;
    auto polygons=copy(source.polygons,source.polygonCount);
    auto edges=copy(source.edges,source.edgeCount);
    auto links=copy(source.edgeLinks,source.edgeCount);
    auto vertices=copy(source.vertices,source.vertexCount);
    auto brush=source;
    brush.polygons=polygons.data(); brush.edges=edges.data(); brush.edgeLinks=links.data(); brush.vertices=vertices.data();
    // Every preparation starts a fresh group namespace. Historical or stale
    // input tags must never masquerade as valid IDs in the new group vectors.
    for (auto& link:links) link.unidentified_04=-1;
    std::vector<std::vector<BrushTriangleEdge>> refs(static_cast<std::size_t>(source.edgeCount));
    std::vector<BrushEdgeGroupView> groups(refs.size());
    for (std::size_t g=0;g<groups.size();++g) {
        refs[g].resize(static_cast<std::size_t>(std::max(foreignPlaneCapacity,1)));
        groups[g]={refs[g].data(),0,foreignPlaneCapacity,0,0,{}};
    }
    const BrushSpaceTransform identity{{1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1}};
    int groupCount=0;
    for (int p=0;p<brush.polygonCount;++p) {
        const int surface=polygons[p].surfaceIndex;
        for (int b=0;b<count;++b) if (b!=index)
            for (int f=0;f<inputs[b].planeCount;++f)
                findBrushEdges(brush,p,input.surfaces->planes[surface],inputs[b].surfaces->planes,inputs[b].planeCount,
                    boxes[b],identity,inputs[b].brushIndex,f,groups.data(),groupCount,source.edgeCount);
    }
    groups.resize(static_cast<std::size_t>(groupCount));
    // Merges can append all other groups' refs into one root. Reserving the
    // total actual ref count bounds that storage without speculative retries.
    std::size_t totalRefs=0;
    for (const auto& group:groups) totalRefs+=static_cast<std::size_t>(group.surfaceRefCount);
    const int refCapacity=bounded(totalRefs);
    for (int g=0;g<groupCount;++g) {
        refs[g].resize(static_cast<std::size_t>(std::max(refCapacity,1)));
        groups[g].surfaceRefs=refs[g].data(); groups[g].surfaceRefCapacity=refCapacity;
    }
    auto mesh=triangulateConvexBrush(brush,*input.surfaces,input.planeCount);
    const int initialTC=bounded(mesh.triangles.size()), initialVC=bounded(mesh.vertices.size());
    // At most one Split per original vertex in the second pass. Removed slots
    // are retained until that pass's compaction; account for all temporaries.
    const int tc=bounded(std::uint64_t(initialTC)+2ull*initialVC+2);
    const int vc=bounded(2ull*initialVC+1);
    if (tc>=(1<<30)) throw std::length_error("Convex preparation triangle handles overflow");
    mesh.triangles.resize(static_cast<std::size_t>(tc)); mesh.vertices.resize(static_cast<std::size_t>(vc));
    for (auto& triangle:mesh.triangles) for (auto& link:triangle.brushEdges) link={-1,kInvalidTriangleEdge};
    TriangleMeshView view{mesh.triangles.data(),mesh.vertices.data(),nullptr,nullptr,initialTC,initialVC,tc,vc};
    std::vector<std::vector<std::uint32_t>> handles(static_cast<std::size_t>(groupCount));
    std::vector<BrushEdgeTriangleList> sources(static_cast<std::size_t>(groupCount));
    std::vector<BrushEdgeConnectionList> connections(static_cast<std::size_t>(groupCount));
    const int sourceCapacity=bounded(std::uint64_t(tc)*3);
    for (int g=0;g<groupCount;++g) {
        handles[g].resize(static_cast<std::size_t>(sourceCapacity)); sources[g]={handles[g].data(),0,sourceCapacity};
    }
    ContextStorage context(tc,vc);
    optimizeTriangleMesh(view,*input.surfaces,input.planeCount,context.view,groups.data(),sources.data(),connections.data(),groupCount);
    cleanUpBrushEdges(view,groups.data(),sources.data(),connections.data(),groupCount);
    mesh.triangles.resize(static_cast<std::size_t>(view.triangleCount)); mesh.vertices.resize(static_cast<std::size_t>(view.vertexCount));
    PreparedConvexBrush result{input.brushIndex,std::move(mesh),{}};
    result.edgeGroups.reserve(static_cast<std::size_t>(groupCount));
    for (int g=0;g<groupCount;++g) {
        const auto& group=groups[g]; const auto& list=sources[g];
        result.edgeGroups.push_back({copy(group.surfaceRefs,group.surfaceRefCount),copy(list.edges,list.count),
            group.surfaceA,group.surfaceB,group.line,{}});
    }
    return result;
}
}
std::vector<PreparedConvexBrush> prepareConvexMeshes(const ConvexMeshInput* inputs, int count) {
    if (count<0 || (count && !inputs)) throw std::invalid_argument("Invalid convex preparation inputs");
    std::set<int> ids; std::size_t totalPlanes=0;
    std::vector<BrushEdgeBounds> boxes; boxes.reserve(static_cast<std::size_t>(count));
    for (int b=0;b<count;++b) {
        const auto& in=inputs[b];
        if (!in.brush || !in.surfaces || !in.surfaces->planes || in.planeCount<=0 || in.brushIndex<0
            || !ids.insert(in.brushIndex).second) throw std::invalid_argument("Invalid convex preparation brush");
        const auto& m=*in.brush;
        if (m.polygonCount<=0 || m.polygonCount>m.polygonCapacity || !m.polygons
            || m.edgeCount<0 || std::int64_t(m.edgeCount)>std::int64_t(m.edgeChunkCapacity)*4
            || m.vertexCount<0 || m.vertexCount>m.vertexCapacity)
            throw std::invalid_argument("Invalid convex preparation mesh sizes");
        for (int p=0;p<m.polygonCount;++p) if (m.polygons[p].surfaceIndex<0 || m.polygons[p].surfaceIndex>=in.planeCount
            || m.polygons[p].categorizationState<1 || m.polygons[p].categorizationState>3)
            throw std::invalid_argument("Convex preparation requires resolved source surfaces");
        for (int p=0;p<in.planeCount;++p) {
            const auto& plane=in.surfaces->planes[p]; const auto n=plane.normal;
            const float square=(n.x*n.x+n.y*n.y)+n.z*n.z;
            if (!std::isfinite(square) || !std::isfinite(plane.distance) || std::abs(square-1.f)>1e-4f)
                throw std::invalid_argument("Convex preparation requires finite unit planes");
        }
        totalPlanes+=static_cast<std::size_t>(in.planeCount); bounded(totalPlanes);
        boxes.push_back(bounds(m));
    }
    std::vector<PreparedConvexBrush> result; result.reserve(static_cast<std::size_t>(count));
    for (int b=0;b<count;++b) result.push_back(prepare(inputs,count,b,boxes,bounded(totalPlanes-inputs[b].planeCount)));
    return result;
}
}
