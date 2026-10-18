#include "realtimecsg/recovered_mesh_optimization.h"
#include <cstring>
#include <stdexcept>

namespace realtimecsg::recovered::detail {
// Runs only on the transaction's private mesh/context/group copies.
void removeTwoSurfaceVerticesWorking(TriangleMeshView& mesh, const SurfaceView& surfaces, int planeCount,
    PlaneOptimizationContextView& context, BrushEdgeGroupView* groups, BrushEdgeTriangleList* sources,
    BrushEdgeConnectionList* connections, int& groupCount) {
    const int initialVertexCount=mesh.vertexCount;
    for (int vertex=initialVertexCount-1;vertex>=0;--vertex) {
        std::size_t attempts=0;
        for (;;) {
            if (++attempts>static_cast<std::size_t>(mesh.triangleCapacity)*9+3)
                throw std::invalid_argument("Nonterminating two-surface swizzle sequence");
            TwoSurfaceVertexPlan plan;
            const auto action=planTwoSurfaceVertex(mesh,surfaces,planeCount,vertex,plan);
            if (action==TwoSurfaceVertexAction::Rejected) break;
            if (action==TwoSurfaceVertexAction::Swizzle) {
                if (swizzleTriangleEdge(mesh,surfaces,planeCount,plan.swizzleEdge)) continue;
                break;
            }
            if (action==TwoSurfaceVertexAction::RemoveInPlane) {
                removeVertexInMiddleOfPlane(mesh,surfaces,planeCount,plan.vertex,context,groups,sources,connections,groupCount);
                break;
            }
            if (mesh.triangleCapacity-mesh.triangleCount<2 || mesh.vertexCount==mesh.vertexCapacity)
                throw std::length_error("Two-surface split capacity exhausted");
            if (mesh.triangleCount>(1<<30)-2)
                throw std::invalid_argument("Two-surface triangle handles overflow");
            const auto ah=plan.boundary.firstBoundary, bh=plan.boundary.secondBoundary;
            const unsigned ac=ah&3u, bc=bh&3u;
            const auto oldA=mesh.triangles[ah>>2], oldB=mesh.triangles[bh>>2];
            const auto at=oldA.twins[ac], bt=oldB.twins[bc];
            const auto acrossA=mesh.triangles[at>>2], acrossB=mesh.triangles[bt>>2];
            const int x=oldA.vertices[(ac+2)%3], y=oldB.vertices[(bc+2)%3], center=oldA.vertices[ac];
            const int ai=mesh.triangleCount, bi=ai+1, duplicate=mesh.vertexCount;
            const auto ae=static_cast<std::uint32_t>(ai)*4, be=static_cast<std::uint32_t>(bi)*4;
            auto& a=mesh.triangles[ai]; auto& b=mesh.triangles[bi];
            a.twins[0]=be; a.twins[1]=bt; a.twins[2]=ah;
            b.twins[0]=ae; b.twins[1]=at; b.twins[2]=bh;
            a.vertices[0]=y; a.vertices[1]=center; a.vertices[2]=x;
            b.vertices[0]=x; b.vertices[1]=center; b.vertices[2]=y;
            const Vector3 an[]={acrossB.cornerNormals[bt&3u],oldA.cornerNormals[ac],oldA.cornerNormals[(ac+2)%3]};
            const Vector3 bn[]={acrossA.cornerNormals[at&3u],oldB.cornerNormals[bc],oldB.cornerNormals[(bc+2)%3]};
            std::memcpy(a.cornerNormals,an,sizeof(an)); std::memcpy(b.cornerNormals,bn,sizeof(bn));
            int indices[]={acrossA.edgeSource[at&3u],acrossB.edgeSource[bt&3u],oldA.edgeSource[ac],oldB.edgeSource[bc]};
            // New records are not active yet, so merge remaps only old faces.
            const int selected=mergeBrushIndices(mesh,groups,sources,connections,groupCount,indices,4);
            for (int c=0;c<3;++c) a.edgeSource[c]=b.edgeSource[c]=selected;
            for (auto h:{ah,bh,at,bt}) mesh.triangles[h>>2].edgeSource[h&3u]=selected;
            for (int i:{ai,bi}) for (unsigned c=0;c<3;++c) {
                const auto twin=mesh.triangles[i].twins[c];
                if ((twin&3u)!=3u) {
                    if (twin>>2>=static_cast<unsigned>(bi+1)) throw std::invalid_argument("Invalid split backlink");
                    mesh.triangles[twin>>2].twins[twin&3u]=static_cast<std::uint32_t>(i)*4+c;
                }
            }
            a.surfaceIndex=oldA.surfaceIndex; a.flags=oldA.flags; a.categorizationState=oldA.categorizationState;
            // B's metadata comes from the first second-region triangle, not
            // necessarily the last one bh. Its flags can differ inside a region.
            b.surfaceIndex=acrossA.surfaceIndex; b.flags=acrossA.flags; b.categorizationState=acrossA.categorizationState;
            for (unsigned c=0;c<3;++c) mesh.vertices[b.vertices[c]].edge=be+c;
            mesh.vertices[duplicate].edge=ae+1;
            std::memcpy(&mesh.vertices[duplicate].position,&mesh.vertices[vertex].position,sizeof(Vector3));
            auto h=ae+1; std::size_t steps=0;
            do {
                if ((h&3u)==3u || h>>2>=static_cast<unsigned>(bi+1) || ++steps>static_cast<std::size_t>(bi+1)*3)
                    throw std::invalid_argument("Nonclosing duplicated-vertex fan");
                auto& t=mesh.triangles[h>>2];
                if (t.vertices[h&3u]!=center) throw std::invalid_argument("Split fan does not share vertex");
                t.vertices[h&3u]=duplicate;
                h=t.twins[((h&3u)+1)%3];
            } while (h!=ae+1);
            ++mesh.vertexCount; mesh.triangleCount+=2;
            removeVertexInMiddleOfPlane(mesh,surfaces,planeCount,duplicate,context,groups,sources,connections,groupCount);
            removeVertexInMiddleOfPlane(mesh,surfaces,planeCount,vertex,context,groups,sources,connections,groupCount);
            break;
        }
    }
}
}
