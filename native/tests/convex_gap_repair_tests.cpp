#include "realtimecsg/recovered_convex_csg.h"
#include "realtimecsg/recovered_gap_dispatch.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <stdexcept>
using namespace realtimecsg::recovered;
namespace {
void require(bool value, const char* message) { if (!value) throw std::runtime_error(message); }
struct Body {
    std::array<TriangleRecord,64> triangles{};
    std::array<TriangleVertexRecord,64> vertices{};
    std::array<Vector3,64> world{};
    std::array<int,64> lookup{};
    std::array<std::array<std::uint32_t,64>,3> edges{};
    std::array<BrushEdgeTriangleList,3> sources{};
    std::array<GapLine,3> lines{{{{1,0,0},{0,0,0}},{{0,1,0},{0,0,0}},{{0,0,1},{0,0,0}}}};
    std::array<Plane,4> planes{{{{0,0,-1},0},{{0,-1,0},0},{{-1,0,0},0},{{1,1,1},2}}};
    TriangleMeshView mesh{triangles.data(),vertices.data(),world.data(),lookup.data(),4,4,64,64};
    BrushSpaceTransform transform{{1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1}};
    Body() {
        const float n=1.f/std::sqrt(3.f); planes[3]={{n,n,n},2*n};
        world[0] = {0,0,0}; world[1] = {2,0,0}; world[2] = {0,2,0}; world[3] = {0,0,2};
        for (int g = 0; g < 3; ++g) sources[g] = {edges[g].data(),0,64};
        const int faces[4][3] = {{0,2,1},{0,1,3},{0,3,2},{1,2,3}};
        for (int v = 0; v < 4; ++v) vertices[v] = {0,world[v]};
        for (int p = 0; p < 4; ++p) {
            auto& t = triangles[p]; t.surfaceIndex = p; t.flags = 9; t.categorizationState = 2;
            for (unsigned c = 0; c < 3; ++c) {
                t.vertices[c] = faces[p][c]; t.cornerNormals[c] = planes[p].normal;
                t.brushEdges[c] = {99,0x12345678};
                const int a = faces[p][(c+2)%3], b = faces[p][c];
                const int group = std::min(a,b) == 0 ? std::max(a,b)-1 : -1;
                t.edgeSource[c] = group;
                if (group >= 0) sources[group].edges[sources[group].count++] = static_cast<unsigned>(p)*4+c;
                for (int q = 0; q < 4; ++q) for (unsigned d = 0; d < 3; ++d)
                    if (faces[q][d] == a && faces[q][(d+2)%3] == b) t.twins[c] = static_cast<unsigned>(q)*4+d;
            }
        }
    }
    float coordinate(int vertex, int axis) const {
        const auto& p = world[vertex]; return axis == 0 ? p.x : axis == 1 ? p.y : p.z;
    }
    void cut(int axis, float value) {
        const auto& group = sources[axis];
        for (int i = 0; i < group.count; ++i) {
            const auto e = group.edges[i]; const auto& t = triangles[e>>2]; const unsigned c = e&3u;
            if (coordinate(t.vertices[(c+2)%3],axis) < value && value < coordinate(t.vertices[c],axis)) {
                const Vector3 point{axis == 0 ? value : 0,axis == 1 ? value : 0,axis == 2 ? value : 0};
                splitTriangleEdge(mesh,transform,sources.data(),3,e,point,0.5f); return;
            }
        }
        throw std::runtime_error("Fixture cut edge missing");
    }
    PreparedConvexBrush prepared(int id, const std::vector<int>& foreign) const {
        PreparedConvexBrush out{id,{{triangles.begin(),triangles.begin()+mesh.triangleCount},
            {vertices.begin(),vertices.begin()+mesh.vertexCount}}, {}};
        const int pairs[3][2]={{0,1},{0,2},{1,2}};
        for (int g=0;g<3;++g) {
            std::vector<BrushTriangleEdge> refs;
            for (int b:foreign) { refs.push_back({b,static_cast<unsigned>(pairs[g][0])}); refs.push_back({b,static_cast<unsigned>(pairs[g][1])}); }
            out.edgeGroups.push_back({refs,{sources[g].edges,sources[g].edges+sources[g].count},
                pairs[g][0],pairs[g][1],lines[g],{{id,static_cast<unsigned>(g)}}}); // old self-connection is ignored
        }
        return out;
    }
    void validate() const {
        for (int p = 0; p < mesh.triangleCount; ++p) for (unsigned c = 0; c < 3; ++c) {
            const auto& t = triangles[p]; const auto twin = t.twins[c];
            require((twin&3u) < 3 && twin>>2 < static_cast<unsigned>(mesh.triangleCount),"Missing dispatcher twin");
            const auto& other = triangles[twin>>2]; const unsigned d = twin&3u;
            require(other.twins[d] == static_cast<unsigned>(p)*4+c,"Dispatcher broke reciprocal twins");
            require(t.vertices[c] == other.vertices[(d+2)%3] && t.vertices[(c+2)%3] == other.vertices[d],"Dispatcher changed twin endpoints");
        }
    }
};

std::vector<std::byte> snapshot(const PreparedConvexBrush& brush) {
    std::vector<std::byte> bytes;
    auto append=[&](const auto& v) {
        const auto* p=reinterpret_cast<const std::byte*>(&v); bytes.insert(bytes.end(),p,p+sizeof(v));
    };
    append(brush.brushIndex);
    for (const auto& t:brush.mesh.triangles) append(t);
    for (const auto& v:brush.mesh.vertices) append(v);
    for (const auto& g:brush.edgeGroups) {
        append(g.surfaceA); append(g.surfaceB); append(g.line);
        append(g.surfaceRefs.size()); for (const auto& ref:g.surfaceRefs) append(ref);
        append(g.triangleEdges.size()); for (const auto& e:g.triangleEdges) append(e);
        append(g.connections.size()); for (const auto& c:g.connections) append(c);
    }
    return bytes;
}
void validate(const PreparedConvexBrush& brush) {
    const auto& mesh=brush.mesh; std::vector<int> seen(mesh.triangles.size()*3,0);
    for (std::size_t g=0;g<brush.edgeGroups.size();++g) for (auto h:brush.edgeGroups[g].triangleEdges) {
        require((h&3u)<3 && h>>2<mesh.triangles.size(),"Stale gap group handle");
        require(mesh.triangles[h>>2].edgeSource[h&3u]==static_cast<int>(g),"Gap group tag mismatch");
        require(++seen[(h>>2)*3+(h&3u)]==1,"Duplicated gap group handle");
    }
    double volume=0;
    for (std::size_t p=0;p<mesh.triangles.size();++p) {
        const auto& t=mesh.triangles[p];
        for (unsigned c=0;c<3;++c) {
            const auto h=t.twins[c]; const int v=t.vertices[c];
            require(v>=0 && static_cast<std::size_t>(v)<mesh.vertices.size(),"Gap vertex outside mesh");
            require((h&3u)<3 && h>>2<mesh.triangles.size(),"Gap repair opened tetrahedron");
            const auto& other=mesh.triangles[h>>2]; const unsigned d=h&3u;
            require(other.twins[d]==p*4+c && t.vertices[c]==other.vertices[(d+2)%3] &&
                t.vertices[(c+2)%3]==other.vertices[d],"Gap twins/endpoints disagree");
            const auto handle=mesh.vertices[v].edge;
            require((handle&3u)<3 && handle>>2<mesh.triangles.size() &&
                mesh.triangles[handle>>2].vertices[handle&3u]==v,"Old vertex handle not refreshed");
            require(seen[p*3+c]==(t.edgeSource[c]>=0 ? 1:0),"Source list no longer covers tags");
        }
        const auto a=mesh.vertices[t.vertices[0]].position,b=mesh.vertices[t.vertices[1]].position,c=mesh.vertices[t.vertices[2]].position;
        volume+=double(a.x)*(double(b.y)*c.z-double(b.z)*c.y)+double(a.y)*(double(b.z)*c.x-double(b.x)*c.z)+double(a.z)*(double(b.x)*c.y-double(b.y)*c.x);
    }
    require(std::abs(volume/6.-4./3.)<1e-6,"Gap repair changed tetrahedron volume");
}
void pairAndBudget() {
    Body a,b; b.cut(0,.5f); b.cut(1,1.5f); b.cut(2,1.f);
    auto first=a.prepared(11,{89}),second=b.prepared(89,{11});
    const auto beforeA=snapshot(first),beforeB=snapshot(second);
    SurfaceView sa{},sb{}; sa.planes=a.planes.data(); sb.planes=b.planes.data();
    const ConvexGapInput inputs[]={{&first,&sa,4},{&second,&sb,4}};
    bool exhausted=false;
    try { (void)repairConvexGaps(inputs,2,1); } catch (const std::length_error&) { exhausted=true; }
    require(exhausted && snapshot(first)==beforeA && snapshot(second)==beforeB,"Gap budget left partial caller mutations");
    const auto repaired=repairConvexGaps(inputs,2,3);
    require(repaired.splitCount==3 && repaired.brushes.size()==2,"Wrong pair split count");
    for (std::size_t i=0;i<2;++i) {
        const auto& brush=repaired.brushes[i]; validate(brush);
        require(brush.mesh.triangles.size()==10 && brush.mesh.vertices.size()==7,"Wrong repaired pair size");
        for (const auto& group:brush.edgeGroups) {
            require(group.connections.size()==1 && group.connections[0].brushIndex==inputs[1-i].brush->brushIndex,
                "Resolver used own brush instead of foreign sparse ID");
            require(group.triangleEdges.size()==4,"Split did not update group lists");
        }
        for (const auto& t:brush.mesh.triangles) for (const auto& ref:t.brushEdges)
            if (ref.brushIndex>=0) require(ref.brushIndex==inputs[1-i].brush->brushIndex && ref.edge>>2<10,"Invalid real brush ID in repaired links");
    }
    require(snapshot(first)==beforeA && snapshot(second)==beforeB,"Successful repair changed inputs");
    const ConvexGapInput again[]={{&repaired.brushes[0],&sa,4},{&repaired.brushes[1],&sb,4}};
    const auto repeated=repairConvexGaps(again,2,0);
    require(repeated.splitCount==0 && snapshot(repeated.brushes[0])==snapshot(repaired.brushes[0]) &&
        snapshot(repeated.brushes[1])==snapshot(repaired.brushes[1]),"Aligned scene did not remain stable");
}
void chain() {
    Body a,b,c; a.cut(2,1.f); c.cut(0,.5f); c.cut(1,1.5f);
    auto x=a.prepared(11,{37}),y=b.prepared(37,{11,89}),z=c.prepared(89,{37});
    SurfaceView s{}; s.planes=a.planes.data();
    const ConvexGapInput inputs[]={{&x,&s,4},{&y,&s,4},{&z,&s,4}};
    const auto result=repairConvexGaps(inputs,3,3);
    require(result.splitCount==6,"Endpoint did not propagate across the three-brush chain");
    for (const auto& brush:result.brushes) {
        validate(brush); require(brush.mesh.vertices.size()==7,"Chain missed a foreign endpoint");
        for (const auto& group:brush.edgeGroups) {
            require(group.triangleEdges.size()==4,"Chain source lists not synchronized");
            for (const auto& ref:group.connections) require(ref.brushIndex!=brush.brushIndex,"Chain acquired a self-connection");
        }
    }
}
void filtersAndErrors() {
    Body a,b; auto x=a.prepared(7,{99}),y=b.prepared(99,{7}); SurfaceView s{}; s.planes=a.planes.data();
    ConvexGapInput inputs[]={{&x,&s,4},{&y,&s,4}};
    // Remove reverse surface refs: matching infinite lines alone is insufficient.
    for (auto& g:y.edgeGroups) g.surfaceRefs.clear();
    const auto unconnected=repairConvexGaps(inputs,2,0);
    for (const auto& brush:unconnected.brushes) {
        for (const auto& g:brush.edgeGroups) require(g.connections.empty(),"One-way surface refs matched");
        for (const auto& t:brush.mesh.triangles) for (const auto& ref:t.brushEdges) require(ref.brushIndex==-1,"Old links were retained");
    }
    y=b.prepared(99,{7}); y.edgeGroups[0].line.origin.y=.01f;
    const auto offset=repairConvexGaps(inputs,2,0);
    require(offset.brushes[0].edgeGroups[0].connections.empty(),"Separated parallel lines matched");
    y.edgeGroups[0].line.origin={0,0,0}; y.edgeGroups[0].line.direction={-1,0,0};
    require(repairConvexGaps(inputs,2,0).brushes[0].edgeGroups[0].connections.size()==1,"Opposite line direction rejected");
    const float threshold=kGapDirectionAlignment;
    y.edgeGroups[0].line.direction={threshold,std::sqrt(1.f-threshold*threshold),0};
    require(repairConvexGaps(inputs,2,0).brushes[0].edgeGroups[0].connections.empty(),"Exact direction threshold must reject");
    const float above=std::nextafter(threshold,1.f);
    y.edgeGroups[0].line.direction={above,std::sqrt(1.f-above*above),0};
    require(repairConvexGaps(inputs,2,0).brushes[0].edgeGroups[0].connections.size()==1,"Above-threshold line was rejected");
    auto invalid=[&](const ConvexGapInput* in,int n,int budget=0) {
        const auto beforeX=snapshot(x),beforeY=snapshot(y); bool failed=false;
        try { (void)repairConvexGaps(in,n,budget); } catch (const std::invalid_argument&) { failed=true; }
        require(failed && snapshot(x)==beforeX && snapshot(y)==beforeY,"Invalid input accepted or mutated");
    };
    invalid(inputs,2,-1); invalid(nullptr,1); invalid(nullptr,-1);
    invalid(inputs,1); // foreign brush 99 is absent
    y.brushIndex=7; invalid(inputs,2); y.brushIndex=99;
    x.edgeGroups[0].surfaceRefs[0].brushIndex=7; invalid(inputs,2); x.edgeGroups[0].surfaceRefs[0].brushIndex=99;
    x.edgeGroups[0].triangleEdges.push_back(x.edgeGroups[0].triangleEdges[0]); invalid(inputs,2);
    x.edgeGroups[0].triangleEdges.pop_back(); x.mesh.triangles[0].twins[0]=kInvalidTriangleEdge; invalid(inputs,2);
    require(repairConvexGaps(nullptr,0).brushes.empty(),"Empty gap scene failed");
}
}
int main() {
    try { pairAndBudget(); chain(); filtersAndErrors(); }
    catch (const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
    std::cout<<"Foreign gap resolution, sparse IDs, endpoint propagation, volume, refreshed handles and atomic errors passed\n";
}
