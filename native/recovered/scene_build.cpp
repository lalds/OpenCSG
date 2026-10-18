#include "realtimecsg/recovered_scene.h"
#include <algorithm>
#include <cmath>
#include <memory>
#include <set>
#include <stdexcept>

namespace realtimecsg::recovered {
namespace {
bool finite(Vector3 v) { return std::isfinite(v.x)&&std::isfinite(v.y)&&std::isfinite(v.z); }
double distance(const Plane& p,Vector3 v) { return (double(p.normal.x)*v.x+double(p.normal.y)*v.y)+double(p.normal.z)*v.z-p.distance; }
struct BrushScratch {
    std::vector<PolygonRecord> polygons;
    std::vector<HalfEdgeRecord> edges;
    std::vector<EdgeLinkRecord> links;
    std::vector<Vector3> vertices;
    std::vector<Plane> planes;
    BrushMemoryView memory;
    SurfaceView surfaces{};
    BrushScratch(const SceneBrushInput& source,const SceneBuildInput& limits) {
        if (source.vertexCount<=0 || source.edgeCount<=0 || source.polygonCount<=0 || source.surfaceCount<=0
            || !source.vertices || !source.edges || !source.polygons || !source.surfaces)
            throw std::invalid_argument("Missing closed convex brush data");
        if (source.vertexCount>limits.maxBrushVertices || source.edgeCount>limits.maxBrushEdges || source.polygonCount>limits.maxBrushPolygons)
            throw std::length_error("Initial brush exceeds workspace capacity");
        vertices.resize(static_cast<std::size_t>(limits.maxBrushVertices));
        polygons.resize(static_cast<std::size_t>(limits.maxBrushPolygons));
        edges.resize(static_cast<std::size_t>(limits.maxBrushEdges)); links.resize(edges.size());
        std::copy_n(source.vertices,source.vertexCount,vertices.data());
        planes.reserve(static_cast<std::size_t>(source.surfaceCount));
        for (int s=0;s<source.surfaceCount;++s) {
            const auto p=source.surfaces[s].plane; const auto n=p.normal;
            const double square=(double(n.x)*n.x+double(n.y)*n.y)+double(n.z)*n.z;
            if (!finite(n)||!std::isfinite(p.distance)||std::abs(square-1)>1e-4) throw std::invalid_argument("Source planes must be finite and unit");
            planes.push_back(p);
        }
        for (int v=0;v<source.vertexCount;++v) {
            if (!finite(vertices[v])) throw std::invalid_argument("Nonfinite source position");
            for (const auto& plane:planes) if (distance(plane,vertices[v])>kPlaneDistanceTolerance)
                throw std::invalid_argument("Source brush is not inside its convex planes");
        }
        std::vector<unsigned char> owned(static_cast<std::size_t>(source.edgeCount),0);
        for (int p=0;p<source.polygonCount;++p) {
            const auto polygon=source.polygons[p];
            if (polygon.edgeCount<3 || polygon.firstEdge<0 || polygon.firstEdge>source.edgeCount-polygon.edgeCount
                || polygon.surfaceIndex<0 || polygon.surfaceIndex>=source.surfaceCount)
                throw std::invalid_argument("Invalid source polygon span/surface");
            polygons[p]={polygon.edgeCount,polygon.firstEdge,polygon.surfaceIndex,0,0,0};
            for (int c=0;c<polygon.edgeCount;++c) {
                const int e=polygon.firstEdge+c; const auto half=source.edges[e];
                if (owned[e]++ || half.vertexIndex<0 || half.vertexIndex>=source.vertexCount || half.twinIndex<0
                    || half.twinIndex>=source.edgeCount || half.twinIndex==e || source.edges[half.twinIndex].twinIndex!=e)
                    throw std::invalid_argument("Invalid source halfedge/twin");
                if (std::abs(distance(planes[polygon.surfaceIndex],vertices[half.vertexIndex]))>kPlaneDistanceTolerance)
                    throw std::invalid_argument("Source polygon is off its surface plane");
                edges[e]={polygon.firstEdge+(c+1)%polygon.edgeCount,half.vertexIndex,half.twinIndex,p};
                links[e]={polygon.firstEdge+(c+polygon.edgeCount-1)%polygon.edgeCount,-1};
            }
        }
        for (int e=0;e<source.edgeCount;++e) {
            if (!owned[e]) throw std::invalid_argument("Source halfedge has no polygon");
            const int twin=edges[e].twinEdge;
            if (edges[e].vertexIndex!=edges[links[twin].previousEdge].vertexIndex
                || edges[links[e].previousEdge].vertexIndex!=edges[twin].vertexIndex)
                throw std::invalid_argument("Source twin endpoints differ");
        }
        for (int p=0;p<source.polygonCount;++p) {
            const auto& polygon=polygons[p]; const auto n=planes[polygon.surfaceIndex].normal;
            double area=0;
            for (int c=0;c<polygon.edgeCount;++c) {
                const auto a=vertices[edges[polygon.firstEdge+c].vertexIndex];
                const auto b=vertices[edges[polygon.firstEdge+(c+1)%polygon.edgeCount].vertexIndex];
                area+=(double(a.y)*b.z-double(a.z)*b.y)*n.x+(double(a.z)*b.x-double(a.x)*b.z)*n.y+(double(a.x)*b.y-double(a.y)*b.x)*n.z;
            }
            if (!(area>0)) throw std::invalid_argument("Source polygon winding/area is invalid");
        }
        surfaces.planes=planes.data();
        memory={polygons.data(),edges.data(),links.data(),vertices.data(),source.polygonCount,source.edgeCount,source.vertexCount,
            limits.maxBrushPolygons,limits.maxBrushEdges/4,limits.maxBrushVertices};
    }
};
}
ExportedScene buildConvexScene(const SceneBuildInput& input) {
    if (input.abiVersion!=kSceneAbiVersion || input.nodeCount<=0 || !input.nodes || input.childCount<0
        || (input.childCount&&!input.children) || input.brushCount<=0 || !input.brushes || input.maxGapSplitsPerBrush<0
        || input.maxBrushVertices<=0 || input.maxBrushEdges<=0 || input.maxBrushEdges%4 || input.maxBrushPolygons<=0)
        throw std::invalid_argument("Invalid scene build/version/workspace");
    std::vector<int> nodeToBrush(static_cast<std::size_t>(input.nodeCount),-1); std::set<int> ids;
    std::vector<CategorizationProgram> programs; programs.reserve(static_cast<std::size_t>(input.brushCount));
    for (int b=0;b<input.brushCount;++b) {
        const auto& brush=input.brushes[b];
        if (brush.nodeIndex<0 || brush.nodeIndex>=input.nodeCount || !input.nodes[brush.nodeIndex].isBrush
            || nodeToBrush[brush.nodeIndex]!=-1 || brush.brushIndex<0 || !ids.insert(brush.brushIndex).second)
            throw std::invalid_argument("Invalid scene brush/node IDs");
        nodeToBrush[brush.nodeIndex]=b;
        programs.push_back(compileCategorization(input.nodes,input.nodeCount,input.children,input.childCount,brush.nodeIndex,nullptr,input.rootNode));
    }
    for (int n=0;n<input.nodeCount;++n) if (input.nodes[n].isBrush && nodeToBrush[n]<0)
        throw std::invalid_argument("Tree brush has no source mesh");
    std::vector<std::unique_ptr<BrushScratch>> owned; owned.reserve(static_cast<std::size_t>(input.brushCount));
    std::vector<ConvexBrushView> clippers(static_cast<std::size_t>(input.nodeCount),{nullptr,0});
    for (int b=0;b<input.brushCount;++b) {
        owned.push_back(std::make_unique<BrushScratch>(input.brushes[b],input));
        clippers[input.brushes[b].nodeIndex]={owned.back()->planes.data(),static_cast<int>(owned.back()->planes.size())};
    }
    std::vector<CategorizedBrushView> categorized;
    for (int b=0;b<input.brushCount;++b) {
        auto& brush=*owned[b];
        categorizeConvexFragments(brush.memory,brush.surfaces,input.brushes[b].surfaceCount,programs[b],clippers.data(),input.nodeCount);
        categorized.push_back({&brush.memory,&brush.surfaces});
    }
    removeOppositeCoincidentFragments(categorized.data(),input.brushCount);
    std::vector<ConvexMeshInput> prepareInputs;
    for (int b=0;b<input.brushCount;++b) prepareInputs.push_back({&owned[b]->memory,&owned[b]->surfaces,input.brushes[b].surfaceCount,input.brushes[b].brushIndex});
    const auto prepared=prepareConvexMeshes(prepareInputs.data(),input.brushCount);
    std::vector<ConvexGapInput> gapInputs;
    for (int b=0;b<input.brushCount;++b) gapInputs.push_back({&prepared[b],&owned[b]->surfaces,input.brushes[b].surfaceCount});
    const auto repaired=repairConvexGaps(gapInputs.data(),input.brushCount,input.maxGapSplitsPerBrush);
    std::vector<SceneExportInput> exportInputs;
    for (int b=0;b<input.brushCount;++b) exportInputs.push_back({&repaired.brushes[b],input.brushes[b].surfaces,input.brushes[b].surfaceCount});
    return exportPreparedScene(exportInputs.data(),input.brushCount);
}
}
