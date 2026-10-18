#include "realtimecsg/recovered_convex_csg.h"
#include "realtimecsg/recovered_gap_dispatch.h"
#include "realtimecsg/recovered_mesh_cleanup.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <stdexcept>
#include <utility>

namespace realtimecsg::recovered {
namespace {
int bounded(std::uint64_t size) {
    if (size>static_cast<unsigned>(std::numeric_limits<int>::max()))
        throw std::length_error("Convex gap repair exceeds index limits");
    return static_cast<int>(size);
}
bool finite(Vector3 v) { return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z); }
float dot(Vector3 a, Vector3 b) { return (a.x*b.x+a.y*b.y)+a.z*b.z; }
bool unit(Vector3 v) { return finite(v) && std::abs(dot(v,v)-1.f)<=1e-4f; }
bool sameLine(const GapLine& a, const GapLine& b) {
    if (!(std::abs(dot(a.direction,b.direction))>kGapDirectionAlignment)) return false;
    const Vector3 delta{b.origin.x-a.origin.x,b.origin.y-a.origin.y,b.origin.z-a.origin.z};
    const float parameter=dot(delta,a.direction);
    const Vector3 distance{a.direction.x*parameter+a.origin.x-b.origin.x,
        a.direction.y*parameter+a.origin.y-b.origin.y,a.direction.z*parameter+a.origin.z-b.origin.z};
    return dot(distance,distance)<kGapLineDistanceSquared;
}
bool references(const PreparedBrushEdgeGroup& group, int brush, int a, int b) {
    return std::any_of(group.surfaceRefs.begin(),group.surfaceRefs.end(),[&](const BrushTriangleEdge& ref) {
        return ref.brushIndex==brush && (ref.edge==static_cast<unsigned>(a) || ref.edge==static_cast<unsigned>(b));
    });
}
void validate(const ConvexGapInput* inputs, int count, const std::map<int,int>& ids) {
    for (int b=0;b<count;++b) {
        const auto& input=inputs[b]; const auto& prepared=*input.brush; const auto& mesh=prepared.mesh;
        if (bounded(mesh.triangles.size())>=(1<<30)) throw std::length_error("Gap triangle handles overflow");
        bounded(mesh.vertices.size()); bounded(prepared.edgeGroups.size());
        for (int p=0;p<input.planeCount;++p)
            if (!unit(input.surfaces->planes[p].normal) || !std::isfinite(input.surfaces->planes[p].distance))
                throw std::invalid_argument("Gap repair needs finite unit planes");
        for (const auto& v:mesh.vertices) if (!finite(v.position)) throw std::invalid_argument("Nonfinite gap vertex");
        std::vector<unsigned char> listed(mesh.triangles.size()*3,0);
        for (std::size_t g=0;g<prepared.edgeGroups.size();++g) {
            const auto& group=prepared.edgeGroups[g];
            if (!unit(group.line.direction) || !finite(group.line.origin) || group.surfaceA<0 || group.surfaceB<0
                || group.surfaceA>=input.planeCount || group.surfaceB>=input.planeCount)
                throw std::invalid_argument("Invalid prepared gap group");
            for (const auto& ref:group.surfaceRefs) {
                const auto found=ids.find(ref.brushIndex);
                if (found==ids.end() || found->second==b || ref.edge>=static_cast<unsigned>(inputs[found->second].planeCount))
                    throw std::invalid_argument("Missing/self/invalid foreign gap surface");
            }
            for (auto h:group.triangleEdges) {
                if ((h&3u)==3u || h>>2>=mesh.triangles.size() || mesh.triangles[h>>2].edgeSource[h&3u]!=static_cast<int>(g)
                    || listed[(h>>2)*3+(h&3u)]++) throw std::invalid_argument("Invalid/duplicate gap source handle");
            }
        }
        for (std::size_t t=0;t<mesh.triangles.size();++t) {
            const auto& triangle=mesh.triangles[t];
            if (triangle.surfaceIndex<0 || triangle.surfaceIndex>=input.planeCount || triangle.categorizationState<1
                || triangle.categorizationState>3) throw std::invalid_argument("Invalid prepared gap surface/state");
            for (unsigned c=0;c<3;++c) {
                const int v=triangle.vertices[c], source=triangle.edgeSource[c]; const auto twin=triangle.twins[c];
                if (v<0 || static_cast<std::size_t>(v)>=mesh.vertices.size() || !unit(triangle.cornerNormals[c])
                    || source < -1 || (source>=0 && static_cast<std::size_t>(source)>=prepared.edgeGroups.size())
                    || listed[t*3+c]!=(source>=0 ? 1:0) || (twin&3u)==3u || twin>>2>=mesh.triangles.size())
                    throw std::invalid_argument("Invalid prepared gap topology/source");
                const auto& other=mesh.triangles[twin>>2];
                if (other.twins[twin&3u]!=t*4+c || triangle.vertices[c]!=other.vertices[((twin&3u)+2)%3]
                    || triangle.vertices[(c+2)%3]!=other.vertices[twin&3u])
                    throw std::invalid_argument("Nonreciprocal prepared gap edge");
            }
        }
    }
}
struct Work {
    PreparedConvexBrush output;
    std::vector<Vector3> world;
    std::vector<int> lookup;
    std::vector<std::vector<std::uint32_t>> handles;
    std::vector<BrushEdgeTriangleList> sources;
    std::vector<GapLine> lines;
    std::vector<std::vector<BrushTriangleEdge>> denseConnections;
    std::vector<GapConnectionList> connections;
    TriangleMeshView mesh;
    int splits=0;
    Work(const PreparedConvexBrush& input, int budget):output(input) {
        const int tc=bounded(input.mesh.triangles.size()+2ull*budget+2), vc=bounded(input.mesh.vertices.size()+std::uint64_t(budget)+2);
        if (tc>=(1<<30)) throw std::length_error("Gap repair triangle handles overflow");
        output.mesh.triangles.resize(static_cast<std::size_t>(tc)); output.mesh.vertices.resize(static_cast<std::size_t>(vc));
        world.resize(static_cast<std::size_t>(vc)); lookup.resize(static_cast<std::size_t>(vc),-1);
        for (std::size_t v=0;v<input.mesh.vertices.size();++v) { world[v]=input.mesh.vertices[v].position; lookup[v]=static_cast<int>(v); }
        mesh={output.mesh.triangles.data(),output.mesh.vertices.data(),world.data(),lookup.data(),
            bounded(input.mesh.triangles.size()),bounded(input.mesh.vertices.size()),tc,vc};
        resetGapNeighbors(mesh);
        handles.reserve(input.edgeGroups.size()); sources.reserve(input.edgeGroups.size()); lines.reserve(input.edgeGroups.size());
        for (const auto& group:input.edgeGroups) {
            handles.push_back(group.triangleEdges); auto& h=handles.back(); const int size=bounded(h.size());
            h.resize(static_cast<std::size_t>(bounded(h.size()+2ull*budget)));
            sources.push_back({h.data(),size,bounded(h.size())}); lines.push_back(group.line);
        }
        denseConnections.resize(input.edgeGroups.size()); connections.resize(input.edgeGroups.size());
        for (auto& group:output.edgeGroups) group.connections.clear();
    }
};
void resolve(std::vector<Work>& work, const std::map<int,int>& ids) {
    for (std::size_t b=0;b<work.size();++b) {
        auto& local=work[b];
        for (std::size_t g=0;g<local.output.edgeGroups.size();++g) {
            auto& group=local.output.edgeGroups[g]; std::vector<int> foreign;
            for (const auto& ref:group.surfaceRefs) foreign.push_back(ref.brushIndex);
            std::sort(foreign.begin(),foreign.end()); foreign.erase(std::unique(foreign.begin(),foreign.end()),foreign.end());
            for (const int id:foreign) {
                const int other=ids.at(id); const auto& candidates=work[other].output.edgeGroups;
                for (std::size_t c=0;c<candidates.size();++c) {
                    const auto& candidate=candidates[c];
                    if (!references(candidate,local.output.brushIndex,group.surfaceA,group.surfaceB)
                        || !references(group,id,candidate.surfaceA,candidate.surfaceB) || !sameLine(group.line,candidate.line)
                        || !sameLine(candidate.line,group.line)) continue;
                    group.connections.push_back({id,static_cast<unsigned>(c)});
                    local.denseConnections[g].push_back({other,static_cast<unsigned>(c)});
                }
            }
            const auto& links=local.denseConnections[g]; local.connections[g]={links.data(),bounded(links.size())};
        }
    }
}
}
ConvexGapRepairResult repairConvexGaps(const ConvexGapInput* inputs, int count, int budget) {
    if (count<0 || budget<0 || (count && !inputs)) throw std::invalid_argument("Invalid convex gap inputs/budget");
    std::map<int,int> ids;
    for (int b=0;b<count;++b) {
        const auto& in=inputs[b];
        if (!in.brush || in.brush->brushIndex<0 || !ids.emplace(in.brush->brushIndex,b).second || !in.surfaces
            || in.planeCount<=0 || !in.surfaces->planes) throw std::invalid_argument("Invalid convex gap brush/planes");
    }
    validate(inputs,count,ids);
    std::vector<Work> work; work.reserve(static_cast<std::size_t>(count));
    for (int b=0;b<count;++b) work.emplace_back(*inputs[b].brush,budget);
    resolve(work,ids);
    std::vector<GapDispatchNode> nodes; nodes.reserve(work.size());
    std::vector<GapBrushView> backlinks; backlinks.reserve(work.size());
    for (int b=0;b<count;++b) {
        nodes.push_back({{&work[b].mesh,inputs[b].surfaces->planes,b,inputs[b].planeCount},work[b].sources.data(),
            work[b].lines.data(),bounded(work[b].sources.size()),3,1});
        backlinks.push_back({&work[b].mesh,b,inputs[b].planeCount});
    }
    const BrushSpaceTransform identity{{1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1}};
    std::uint64_t total=0;
    for (;;) {
        std::uint64_t cuts=0;
        for (const auto& entry:ids) {
            const int b=entry.second; auto& w=work[b];
            const int added=fixGaps(w.mesh,inputs[b].surfaces->planes,inputs[b].planeCount,identity,w.sources.data(),
                bounded(w.sources.size()),w.lines.data(),w.connections.data(),nodes.data(),count,budget-w.splits);
            w.splits+=added; cuts+=added;
        }
        total+=cuts; bounded(total); if (!cuts) break;
    }
    for (const auto& entry:ids) {
        const int b=entry.second;
        connectGaps(work[b].mesh,b,inputs[b].planeCount,backlinks.data(),count);
    }
    ConvexGapRepairResult result{{},bounded(total)}; result.brushes.reserve(work.size());
    for (auto& w:work) {
        // Native split leaves some old vertex handles stale. Refresh only these
        // handles, without renumbering triangles used by cross-brush refs.
        for (int v=0;v<w.mesh.vertexCount;++v) w.mesh.vertices[v].edge=kInvalidTriangleEdge;
        for (int t=0;t<w.mesh.triangleCount;++t) for (unsigned c=0;c<3;++c) {
            auto& triangle=w.mesh.triangles[t]; w.mesh.vertices[triangle.vertices[c]].edge=static_cast<unsigned>(t)*4+c;
            auto& ref=triangle.brushEdges[c];
            if (ref.brushIndex>=0) ref.brushIndex=work[ref.brushIndex].output.brushIndex;
        }
        for (std::size_t g=0;g<w.sources.size();++g) {
            const auto& s=w.sources[g]; auto& output=w.output.edgeGroups[g].triangleEdges;
            if (s.count) output.assign(s.edges,s.edges+s.count); else output.clear();
        }
        w.output.mesh.triangles.resize(static_cast<std::size_t>(w.mesh.triangleCount));
        w.output.mesh.vertices.resize(static_cast<std::size_t>(w.mesh.vertexCount));
        result.brushes.push_back(std::move(w.output));
    }
    return result;
}
}
