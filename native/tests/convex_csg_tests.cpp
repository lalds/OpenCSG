#include "realtimecsg/recovered_convex_csg.h"
#include "realtimecsg/recovered_triangulation.h"
#include "realtimecsg/recovered_mesh_cleanup.h"
#include "realtimecsg/recovered_scene.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <functional>
#include <iostream>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>

using namespace realtimecsg::recovered;

namespace {
void require(bool value, const char* message) {
    if (!value) throw std::runtime_error(message);
}
struct Bounds { Vector3 lo, hi; };
struct PreparationCounts {
    std::size_t inputTriangles=0,outputTriangles=0,inputVertices=0,outputVertices=0,groups=0,maxGroups=0,gapCuts=0,connections=0,crossLinks=0;
} preparationCounts;

void validatePreparedGroups(const PreparedConvexBrush& prepared, const std::vector<int>& ids) {
    const auto& mesh=prepared.mesh;
    std::vector<int> occurrences(mesh.triangles.size()*3,0);
    for (std::size_t g=0;g<prepared.edgeGroups.size();++g) {
        const auto& group=prepared.edgeGroups[g];
        require(!group.triangleEdges.empty(),"Preparation retained unused source group");
        require(group.surfaceA>=0 && group.surfaceA<6 && group.surfaceB>=0 && group.surfaceB<6,"Group lost local source surfaces");
        for (const auto& ref:group.surfaceRefs) {
            require(ref.brushIndex!=prepared.brushIndex && std::find(ids.begin(),ids.end(),ref.brushIndex)!=ids.end(),"Invalid foreign brush reference");
            require(ref.edge<6,"Invalid foreign surface reference");
        }
        for (const auto h:group.triangleEdges) {
            require((h&3u)<3u && h>>2<mesh.triangles.size(),"Group retained stale triangle handle");
            require(mesh.triangles[h>>2].edgeSource[h&3u]==static_cast<int>(g),"Source list and triangle tag disagree");
            require(++occurrences[(h>>2)*3+(h&3u)]==1,"Duplicate triangle handle in source groups");
        }
    }
    for (std::size_t t=0;t<mesh.triangles.size();++t) for (unsigned c=0;c<3;++c) {
        const int group=mesh.triangles[t].edgeSource[c];
        require(group==-1 || (group>=0 && static_cast<std::size_t>(group)<prepared.edgeGroups.size()),"Source tag outside group namespace");
        require(occurrences[t*3+c]==(group>=0 ? 1:0),"Tagged edge missing from rebuilt source lists");
    }
}
void validateCrossBrushLinks(const std::vector<PreparedConvexBrush>& brushes) {
    auto lookup=[&](int id) -> const PreparedConvexBrush& {
        const auto found=std::find_if(brushes.begin(),brushes.end(),[&](const PreparedConvexBrush& b) { return b.brushIndex==id; });
        require(found!=brushes.end(),"Missing foreign brush after gap repair"); return *found;
    };
    auto close=[](Vector3 a,Vector3 b) {
        const double x=double(a.x)-b.x,y=double(a.y)-b.y,z=double(a.z)-b.z;
        return x*x+y*y+z*z < .0045*.0045;
    };
    for (const auto& brush:brushes) {
        for (std::size_t g=0;g<brush.edgeGroups.size();++g) for (const auto& connection:brush.edgeGroups[g].connections) {
            const auto& other=lookup(connection.brushIndex);
            require(other.brushIndex!=brush.brushIndex && connection.edge<other.edgeGroups.size(),"Self/stale source connection");
            const auto& reverse=other.edgeGroups[connection.edge].connections;
            require(std::any_of(reverse.begin(),reverse.end(),[&](const BrushTriangleEdge& r) {
                return r.brushIndex==brush.brushIndex && r.edge==g;
            }),"Source connection is not reciprocal");
        }
        for (std::size_t t=0;t<brush.mesh.triangles.size();++t) for (unsigned c=0;c<3;++c) {
            const auto& a=brush.mesh.triangles[t]; const auto ref=a.brushEdges[c];
            if (ref.brushIndex<0) continue;
            const auto& other=lookup(ref.brushIndex);
            require(other.brushIndex!=brush.brushIndex && (ref.edge&3u)<3 && ref.edge>>2<other.mesh.triangles.size(),"Invalid cross-brush triangle link");
            const auto& b=other.mesh.triangles[ref.edge>>2]; const unsigned d=ref.edge&3u;
            require((a.flags&8u) && (b.flags&8u),"Hidden triangle acquired a gap link");
            const auto x=brush.mesh.vertices[a.vertices[c]].position,y=brush.mesh.vertices[a.vertices[(c+2)%3]].position;
            const auto p=other.mesh.vertices[b.vertices[d]].position,q=other.mesh.vertices[b.vertices[(d+2)%3]].position;
            require((close(x,p)&&close(y,q)) || (close(x,q)&&close(y,p)),"Cross-brush endpoints do not match");
            const auto reverse=b.brushEdges[d];
            require(reverse.brushIndex==brush.brushIndex && reverse.edge==t*4+c,"Triangle gap link is not reciprocal");
        }
    }
}

void validateTriangleTopology(const TriangulatedBrush& mesh, bool closed) {
    std::vector<bool> used(mesh.vertices.size(),false);
    for (std::size_t p = 0; p < mesh.triangles.size(); ++p) {
        const auto& t = mesh.triangles[p];
        for (unsigned c = 0; c < 3; ++c) {
            require(t.vertices[c] >= 0 && static_cast<std::size_t>(t.vertices[c]) < mesh.vertices.size(),"Invalid triangle vertex");
            used[t.vertices[c]] = true;
            const auto twin = t.twins[c];
            if ((twin & 3u) == 3u) { require(!closed,"Missing triangle twin before visible export"); continue; }
            require((twin >> 2) < mesh.triangles.size(),"Triangle twin out of range");
            const auto& other = mesh.triangles[twin >> 2];
            require(other.twins[twin & 3u] == ((p << 2) | c),"Asymmetric triangle twin");
            require(t.vertices[c] == other.vertices[((twin & 3u)+2)%3]
                && t.vertices[(c+2)%3] == other.vertices[twin & 3u],"Triangle twin endpoints mismatch");
        }
    }
    for (std::size_t v = 0; v < mesh.vertices.size(); ++v) {
        if (!used[v]) { require(closed,"Visible export retained unused vertex"); continue; }
        const auto edge = mesh.vertices[v].edge;
        require((edge & 3u) < 3 && (edge >> 2) < mesh.triangles.size(),"Vertex edge invalid after compaction");
        require(mesh.triangles[edge >> 2].vertices[edge & 3u] == static_cast<int>(v),"Vertex edge references another vertex");
    }
}
struct Box {
    std::vector<PolygonRecord> polygons = std::vector<PolygonRecord>(512);
    std::vector<HalfEdgeRecord> edges = std::vector<HalfEdgeRecord>(4096);
    std::vector<EdgeLinkRecord> links = std::vector<EdgeLinkRecord>(4096);
    std::vector<Vector3> vertices = std::vector<Vector3>(2048);
    std::array<Plane, 6> planes;
    BrushMemoryView memory{polygons.data(), edges.data(), links.data(), vertices.data(),
                            6,24,8,512,1024,2048};
    SurfaceView surfaces{};

    explicit Box(Bounds b) {
        planes = {{{{0,0,-1},-b.lo.z}, {{0,0,1},b.hi.z}, {{0,-1,0},-b.lo.y},
                   {{1,0,0},b.hi.x}, {{0,1,0},b.hi.y}, {{-1,0,0},-b.lo.x}}};
        surfaces.planes = planes.data();
        vertices[0] = {b.lo.x,b.lo.y,b.lo.z}; vertices[1] = {b.hi.x,b.lo.y,b.lo.z};
        vertices[2] = {b.hi.x,b.hi.y,b.lo.z}; vertices[3] = {b.lo.x,b.hi.y,b.lo.z};
        vertices[4] = {b.lo.x,b.lo.y,b.hi.z}; vertices[5] = {b.hi.x,b.lo.y,b.hi.z};
        vertices[6] = {b.hi.x,b.hi.y,b.hi.z}; vertices[7] = {b.lo.x,b.hi.y,b.hi.z};
        const int faces[6][4] = {{3,2,1,0},{4,5,6,7},{0,1,5,4},{1,2,6,5},{2,3,7,6},{3,0,4,7}};
        for (int p = 0; p < 6; ++p) {
            polygons[p] = {4,4*p,p,0,0,0};
            for (int i = 0; i < 4; ++i) {
                edges[4*p+i] = {4*p+(i+1)%4,faces[p][i],-1,p};
                links[4*p+i] = {4*p+(i+3)%4,0};
            }
        }
        for (int e = 0; e < 24; ++e)
            for (int t = 0; t < 24; ++t)
                if (edges[e].vertexIndex == edges[links[t].previousEdge].vertexIndex
                    && edges[t].vertexIndex == edges[links[e].previousEdge].vertexIndex)
                    edges[e].twinEdge = t;
    }
    Box(const Box&) = delete;
    Box& operator=(const Box&) = delete;

    std::vector<int> face(int p) const {
        std::vector<int> result;
        int e = polygons[p].firstEdge;
        for (int i = 0; i < polygons[p].edgeCount; ++i) {
            require(e >= 0 && e < memory.edgeCount, "Edge out of range");
            require(edges[e].polygonIndex == p, "Wrong edge owner");
            require(links[edges[e].nextEdge].previousEdge == e, "Broken previous edge");
            const int t = edges[e].twinEdge;
            require(t >= 0 && t < memory.edgeCount && edges[t].twinEdge == e, "Broken twin");
            require(edges[t].vertexIndex == edges[links[e].previousEdge].vertexIndex, "Twin endpoint mismatch");
            require(edges[e].vertexIndex >= 0 && edges[e].vertexIndex < memory.vertexCount, "Invalid vertex");
            result.push_back(edges[e].vertexIndex);
            e = edges[e].nextEdge;
        }
        require(e == polygons[p].firstEdge, "Face loop not closed");
        return result;
    }
};

using Occupancy = std::function<bool(const std::vector<bool>&)>;
Vector3 rotate(Vector3 p) {
    const float y = (12*p.y-5*p.z)/13, z = (5*p.y+12*p.z)/13;
    return {(3*p.x-4*y)/5,(4*p.x+3*y)/5,z};
}
Vector3 unrotate(Vector3 p) {
    const float x = (3*p.x+4*p.y)/5, y = (-4*p.x+3*p.y)/5;
    return {x,(12*y+5*p.z)/13,(-5*y+12*p.z)/13};
}
bool occupied(Vector3 p, const std::vector<Bounds>& bounds, const Occupancy& operation) {
    std::vector<bool> inside;
    for (const auto& b : bounds)
        inside.push_back(p.x > b.lo.x && p.x < b.hi.x && p.y > b.lo.y && p.y < b.hi.y
                         && p.z > b.lo.z && p.z < b.hi.z);
    return operation(inside);
}

// Independent exact axis-aligned cell arrangement; no recovered clipping or
// category rules are used to calculate the expected volume and boundary area.
std::array<double, 2> reference(const std::vector<Bounds>& boxes, const Occupancy& operation) {
    std::array<std::vector<float>, 3> axes;
    for (const auto& b : boxes) {
        axes[0].insert(axes[0].end(), {b.lo.x,b.hi.x});
        axes[1].insert(axes[1].end(), {b.lo.y,b.hi.y});
        axes[2].insert(axes[2].end(), {b.lo.z,b.hi.z});
    }
    for (auto& axis : axes) {
        std::sort(axis.begin(), axis.end());
        axis.erase(std::unique(axis.begin(), axis.end()), axis.end());
    }
    double volume = 0, area = 0;
    for (std::size_t x = 0; x+1 < axes[0].size(); ++x)
        for (std::size_t y = 0; y+1 < axes[1].size(); ++y)
            for (std::size_t z = 0; z+1 < axes[2].size(); ++z) {
                const Vector3 center{(axes[0][x]+axes[0][x+1])*0.5f,
                    (axes[1][y]+axes[1][y+1])*0.5f,(axes[2][z]+axes[2][z+1])*0.5f};
                if (!occupied(center, boxes, operation)) continue;
                const double dx = axes[0][x+1]-axes[0][x], dy = axes[1][y+1]-axes[1][y],
                             dz = axes[2][z+1]-axes[2][z];
                volume += dx*dy*dz;
                const float eps = 0.001f;
                if (!occupied({axes[0][x]-eps,center.y,center.z},boxes,operation)) area += dy*dz;
                if (!occupied({axes[0][x+1]+eps,center.y,center.z},boxes,operation)) area += dy*dz;
                if (!occupied({center.x,axes[1][y]-eps,center.z},boxes,operation)) area += dx*dz;
                if (!occupied({center.x,axes[1][y+1]+eps,center.z},boxes,operation)) area += dx*dz;
                if (!occupied({center.x,center.y,axes[2][z]-eps},boxes,operation)) area += dx*dy;
                if (!occupied({center.x,center.y,axes[2][z+1]+eps},boxes,operation)) area += dx*dy;
            }
    return {volume,area};
}

void run(const std::vector<Bounds>& bounds, const std::vector<CategorizationNode>& nodes,
         const std::vector<int>& children, const Occupancy& operation, const std::string& obj = {}, bool rotated = false) {
    std::vector<std::unique_ptr<Box>> boxes;
    std::vector<ConvexBrushView> clippers(nodes.size(), {nullptr,0});
    std::vector<int> indices;
    for (std::size_t n = 0; n < nodes.size(); ++n) if (nodes[n].isBrush) indices.push_back(static_cast<int>(n));
    require(indices.size() == bounds.size(), "Wrong fixture brush count");
    for (std::size_t b = 0; b < bounds.size(); ++b) {
        boxes.push_back(std::make_unique<Box>(bounds[b]));
        if (rotated) {
            for (int v = 0; v < boxes.back()->memory.vertexCount; ++v)
                boxes.back()->vertices[v] = rotate(boxes.back()->vertices[v]);
            for (auto& plane : boxes.back()->planes) plane.normal = rotate(plane.normal);
        }
        clippers[indices[b]] = {boxes.back()->planes.data(),6};
    }
    std::vector<CategorizedBrushView> categorized;
    for (std::size_t b = 0; b < boxes.size(); ++b) {
        auto& box = *boxes[b];
        auto program = compileCategorization(nodes.data(), static_cast<int>(nodes.size()), children.data(),
                                             static_cast<int>(children.size()),indices[b]);
        categorizeConvexFragments(box.memory, box.surfaces, 6, program, clippers.data(),static_cast<int>(clippers.size()));
        categorized.push_back({&box.memory,&box.surfaces});
    }
    removeOppositeCoincidentFragments(categorized.data(),static_cast<int>(categorized.size()));
    std::vector<ConvexMeshInput> meshInputs;
    for (std::size_t b=0;b<boxes.size();++b) meshInputs.push_back({&boxes[b]->memory,&boxes[b]->surfaces,6,indices[b]});
    const auto initial=prepareConvexMeshes(meshInputs.data(),static_cast<int>(meshInputs.size()));
    std::vector<ConvexGapInput> gapInputs;
    for (std::size_t b=0;b<initial.size();++b) gapInputs.push_back({&initial[b],&boxes[b]->surfaces,6});
    const auto repaired=repairConvexGaps(gapInputs.data(),static_cast<int>(gapInputs.size()));
    const auto& prepared=repaired.brushes; preparationCounts.gapCuts+=static_cast<std::size_t>(repaired.splitCount);
    validateCrossBrushLinks(prepared);
    for (const auto& brush:prepared) {
        for (const auto& group:brush.edgeGroups) preparationCounts.connections+=group.connections.size();
        for (const auto& t:brush.mesh.triangles) for (const auto& ref:t.brushEdges) if (ref.brushIndex>=0) ++preparationCounts.crossLinks;
    }
    require(prepared.size()==boxes.size(),"Preparation changed brush count");
    double volume = 0, area = 0;
    std::array<double,3> closure{};
    std::ofstream out;
    int vertexOffset = 1;
    if (!obj.empty()) { out.open(obj); require(bool(out), "Cannot write OBJ"); out << "# Open convex CSG research output; source groups, recovered Optimize and visible compaction\n"; }
    for (std::size_t b = 0; b < boxes.size(); ++b) {
        auto& box = *boxes[b];
        for (int p = 0; p < box.memory.polygonCount; ++p) box.face(p);
        const auto before=triangulateConvexBrush(box.memory,box.surfaces,6);
        const auto& complete = prepared[b].mesh;
        require(prepared[b].brushIndex==indices[b],"Preparation changed brush ID");
        validatePreparedGroups(prepared[b],indices);
        preparationCounts.inputTriangles+=before.triangles.size(); preparationCounts.outputTriangles+=complete.triangles.size();
        preparationCounts.inputVertices+=before.vertices.size(); preparationCounts.outputVertices+=complete.vertices.size();
        preparationCounts.groups+=prepared[b].edgeGroups.size();
        preparationCounts.maxGroups=std::max(preparationCounts.maxGroups,prepared[b].edgeGroups.size());
        validateTriangleTopology(complete,true);
        const auto triangulated = compactVisibleTriangles(complete);
        validateTriangleTopology(triangulated,false);
        if (out.is_open()) {
            out << "o brush_" << indices[b] << '\n';
            for (const auto& vertex : triangulated.vertices) {
                const auto& p = vertex.position; out << "v " << p.x << ' ' << p.y << ' ' << p.z << '\n';
            }
        }
        for (std::size_t p = 0; p < triangulated.triangles.size(); ++p) {
            const auto& poly = triangulated.triangles[p];
            require(poly.surfaceIndex >= 0 && poly.surfaceIndex < 6, "Lost source plane");
            const auto visibilityBits = poly.flags & 0x3fu;
            require(visibilityBits == (poly.categorizationState == 1 ? 0u : poly.categorizationState == 2 ? 9u : 0x1au),
                    "Visibility flags do not match final category");
            std::array<int,3> face{{poly.vertices[0],poly.vertices[1],poly.vertices[2]}};
            for (unsigned c = 0; c < 3; ++c) {
                require(face[c] >= 0 && static_cast<std::size_t>(face[c]) < triangulated.vertices.size(),"Invalid triangle vertex");
                const auto& n = poly.cornerNormals[c];
                const auto& expectedNormal = box.planes[poly.surfaceIndex].normal;
                require(n.x == expectedNormal.x && n.y == expectedNormal.y && n.z == expectedNormal.z,"Lost corner normal");
            }
            require(poly.categorizationState == 2 || poly.categorizationState == 3, "Invalid terminal state");
            if (poly.categorizationState == 3) std::swap(face[1],face[2]);
            if (out.is_open()) {
                out << "g surface_" << poly.surfaceIndex << "_state_" << poly.categorizationState << "\nf";
                for (int v : face) out << ' ' << vertexOffset+v;
                out << '\n';
            }
            const auto& a = triangulated.vertices[face[0]].position;
            {
                const auto& c = triangulated.vertices[face[1]].position;
                const auto& d = triangulated.vertices[face[2]].position;
                const double nx = (double(c.y)-a.y)*(double(d.z)-a.z)-(double(c.z)-a.z)*(double(d.y)-a.y);
                const double ny = (double(c.z)-a.z)*(double(d.x)-a.x)-(double(c.x)-a.x)*(double(d.z)-a.z);
                const double nz = (double(c.x)-a.x)*(double(d.y)-a.y)-(double(c.y)-a.y)*(double(d.x)-a.x);
                const double twiceArea = std::sqrt(nx*nx+ny*ny+nz*nz);
                volume += (a.x*nx+a.y*ny+a.z*nz)/6.0;
                area += twiceArea/2.0;
                closure[0] += nx; closure[1] += ny; closure[2] += nz;
                // Fan diagonals through nearly collinear float vertices can
                // generate numerical slivers. Include their moments/area above,
                // but their unstable cross-product is not a meaningful normal.
                if (twiceArea < 1e-6) continue;
                const Vector3 center{(a.x+c.x+d.x)/3,(a.y+c.y+d.y)/3,(a.z+c.z+d.z)/3};
                const float ox = static_cast<float>(nx/twiceArea*0.001),
                            oy = static_cast<float>(ny/twiceArea*0.001),
                            oz = static_cast<float>(nz/twiceArea*0.001);
                Vector3 inward{center.x-ox,center.y-oy,center.z-oz}, outward{center.x+ox,center.y+oy,center.z+oz};
                if (rotated) { inward = unrotate(inward); outward = unrotate(outward); }
                if (!occupied(inward,bounds,operation) || occupied(outward,bounds,operation)) {
                    std::cerr << "Boundary failure brush " << b << " triangle " << p << " state " << poly.categorizationState
                              << " rotated " << rotated << " triangle area " << twiceArea/2
                              << " inward " << inward.x << ',' << inward.y << ',' << inward.z
                              << " outward " << outward.x << ',' << outward.y << ',' << outward.z << '\n';
                    throw std::runtime_error("Incorrect boundary orientation or position");
                }
            }
        }
        vertexOffset += static_cast<int>(triangulated.vertices.size());
    }
    const auto expected = reference(bounds,operation);
    std::vector<std::array<SceneSurface,6>> attributes(boxes.size()); std::vector<SceneExportInput> exports;
    for (std::size_t b=0;b<boxes.size();++b) {
        for (int s=0;s<6;++s) {
            Vector3 u=s==3||s==5 ? Vector3{0,1,0}:Vector3{1,0,0},v=s<2 ? Vector3{0,1,0}:Vector3{0,0,1};
            if (rotated) { u=rotate(u); v=rotate(v); }
            attributes[b][s]={boxes[b]->planes[s],{u.x,u.y,u.z,.25f},{v.x,v.y,v.z,-.5f},indices[b]*10+s%2};
        }
        exports.push_back({&prepared[b],attributes[b].data(),6});
    }
    const auto unified=exportPreparedScene(exports.data(),static_cast<int>(exports.size()));
    double exportVolume=0,exportArea=0;
    for (std::size_t i=0;i<unified.indices.size();i+=3) {
        const auto a=unified.vertices[unified.indices[i]].position,b=unified.vertices[unified.indices[i+1]].position,c=unified.vertices[unified.indices[i+2]].position;
        const double nx=(double(b.y)-a.y)*(double(c.z)-a.z)-(double(b.z)-a.z)*(double(c.y)-a.y),
            ny=(double(b.z)-a.z)*(double(c.x)-a.x)-(double(b.x)-a.x)*(double(c.z)-a.z),
            nz=(double(b.x)-a.x)*(double(c.y)-a.y)-(double(b.y)-a.y)*(double(c.x)-a.x);
        exportVolume+=(a.x*nx+a.y*ny+a.z*nz)/6; exportArea+=std::sqrt(nx*nx+ny*ny+nz*nz)/2;
    }
    require(std::abs(exportVolume-expected[0])<=1e-5*std::max(1.0,expected[0]) &&
        std::abs(exportArea-expected[1])<=1e-5*std::max(1.0,expected[1]),"Unified scene export volume/area mismatch");
    for (const auto& vertex:unified.vertices) {
        const auto n=vertex.normal; const auto t=vertex.tangent;
        require(std::abs((n.x*t.x+n.y*t.y)+n.z*t.z)<1e-5 && std::abs((t.x*t.x+t.y*t.y)+t.z*t.z-1)<1e-5 &&
            std::abs(t.w)==1 && std::isfinite(vertex.uv.x) && std::isfinite(vertex.uv.y),"Unified export tangent/UV invalid");
    }
    if (std::abs(volume-expected[0]) > 1e-5*std::max(1.0,expected[0])
        || std::abs(area-expected[1]) > 1e-5*std::max(1.0,expected[1])) {
        std::cerr << "Volume " << volume << " expected " << expected[0] << "; area " << area << " expected " << expected[1] << '\n';
        throw std::runtime_error("Boolean volume or area mismatch");
    }
    for (double component : closure) require(std::abs(component) < 1e-5*std::max(1.0,expected[1]),"Oriented boundary area does not close");
}

std::vector<CategorizationNode> pairTree(CSGOperation op) {
    return {{0,2,CSGOperation::Additive,0,0,0},{0,0,CSGOperation::Additive,1,0,0},{0,0,op,1,0,0}};
}
Occupancy pairOccupancy(int op) {
    return [op](const std::vector<bool>& v) { return op == 0 ? v[0] || v[1] : op == 1 ? v[0] && !v[1] : v[0] && v[1]; };
}
} // namespace

int main(int argc, char** argv) {
    try {
        require(coplanarRelation({{1,0,0},1},{{1,0,0},1}) == PlaneRelation::Aligned,"Same plane");
        require(coplanarRelation({{1,0,0},1},{{-1,0,0},-1}) == PlaneRelation::OppositeAligned,"Opposite plane");
        require(coplanarRelation({{1,0,0},0},{{1,0,0},5e-6f}) == PlaneRelation::Separate,"Distance threshold must be strict");
        const Bounds a{{-1,-1,-1},{1,1,1}};
        const std::vector<Bounds> cases = {
            {{-.5f,-.75f,-.25f},{1.5f,1.25f,1.75f}}, // general overlap
            {{2,2,2},{3,3,3}},                       // disjoint
            {{-.5f,-.5f,-.5f},{.5f,.5f,.5f}},        // internal cavity
            {{-2,-2,-2},{2,2,2}}, a,                 // enclosing / coincident
            {{1,-1,-1},{3,1,1}},                     // full face contact
            {{1,-.5f,-.5f},{3,.5f,.5f}},             // partial face contact
            {{1,1,-1},{3,3,1}}, {{1,1,1},{3,3,3}},  // edge / point contact
            {{0,-1,-1},{2,1,1}}};                    // overlapping coplanar faces
        int count = 0;
        for (const auto& b : cases) for (int op = 0; op < 3; ++op) {
            run({a,b},pairTree(static_cast<CSGOperation>(op)),{1,2},pairOccupancy(op)); ++count;
            run({a,b},pairTree(static_cast<CSGOperation>(op)),{1,2},pairOccupancy(op),{},true); ++count;
        }
        std::mt19937 rng(0x435347);
        auto randomBox = [&]() {
            auto value = [&]() { return float(int(rng()%25)-12)/8; };
            const Vector3 lo{value(),value(),value()};
            return Bounds{lo,{lo.x+float(1+rng()%12)/4,lo.y+float(1+rng()%12)/4,lo.z+float(1+rng()%12)/4}};
        };
        for (int i = 0; i < 150; ++i) {
            const auto b = randomBox(), c = randomBox();
            for (int op = 0; op < 3; ++op) {
                run({b,c},pairTree(static_cast<CSGOperation>(op)),{1,2},pairOccupancy(op)); ++count;
                if (i < 20) {
                    run({b,c},pairTree(static_cast<CSGOperation>(op)),{1,2},pairOccupancy(op),{},true); ++count;
                }
            }
            const std::vector<CategorizationNode> nested = {{0,2,CSGOperation::Additive,0,0,0},
                {0,0,CSGOperation::Additive,1,0,0},{0,0,CSGOperation::Additive,1,0,0},
                {0,0,CSGOperation::Intersecting,1,0,0},{2,2,CSGOperation::Subtractive,0,0,0}};
            run({a,b,c},nested,{1,4,2,3},[](const std::vector<bool>& v) { return v[0] && !(v[1] && v[2]); }); ++count;
            if (i < 20) {
                run({a,b,c},nested,{1,4,2,3},[](const std::vector<bool>& v) { return v[0] && !(v[1] && v[2]); },{},true); ++count;
            }
        }
        if (argc == 2) for (int op = 0; op < 3; ++op) {
            const char* names[] = {"union","subtract","intersect"};
            run({a,cases[0]},pairTree(static_cast<CSGOperation>(op)),{1,2},pairOccupancy(op),
                std::string(argv[1])+"-"+names[op]+".obj");
        }
        std::cout << "Convex CSG checks passed: " << count << " Boolean results\n";
        require(preparationCounts.outputTriangles<preparationCounts.inputTriangles,"Optimizer made no reductions in Boolean scenarios");
        require(preparationCounts.groups>0,"Boolean pipeline exercised no source groups");
        require(preparationCounts.maxGroups>=3,"Boolean pipeline exercised no multi-group mesh");
        require(preparationCounts.gapCuts>0 && preparationCounts.crossLinks>0,"Boolean pipeline exercised no actual gap cuts/links");
        std::cout << "Prepared triangles " << preparationCounts.inputTriangles << " -> " << preparationCounts.outputTriangles
                  << "; vertices " << preparationCounts.inputVertices << " -> " << preparationCounts.outputVertices
                  << "; surviving source groups " << preparationCounts.groups << "; max per brush " << preparationCounts.maxGroups << '\n';
        std::cout << "Gap splits " << preparationCounts.gapCuts << "; connections " << preparationCounts.connections
                  << "; directed triangle links " << preparationCounts.crossLinks << '\n';
    } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
