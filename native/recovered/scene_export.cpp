#include "realtimecsg/recovered_scene.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <set>
#include <stdexcept>

namespace realtimecsg::recovered {
namespace {
bool finite(Vector3 v) { return std::isfinite(v.x)&&std::isfinite(v.y)&&std::isfinite(v.z); }
bool finite(SceneFloat4 v) { return std::isfinite(v.x)&&std::isfinite(v.y)&&std::isfinite(v.z)&&std::isfinite(v.w); }
double dot(Vector3 a,Vector3 b) { return (double(a.x)*b.x+double(a.y)*b.y)+double(a.z)*b.z; }
Vector3 cross(Vector3 a,Vector3 b) { return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x}; }
Vector3 normalize(Vector3 v) {
    const double length=std::sqrt(dot(v,v));
    if (!(length>0) || !std::isfinite(length)) throw std::invalid_argument("Invalid export tangent/normal");
    return {float(v.x/length),float(v.y/length),float(v.z/length)};
}
SceneFloat4 tangent(Vector3 normal,const SceneSurface& surface) {
    const auto n=normalize(normal);
    const Vector3 u{surface.uvU.x,surface.uvU.y,surface.uvU.z},v{surface.uvV.x,surface.uvV.y,surface.uvV.z};
    const auto a=cross(v,n),b=cross(n,u); const double determinant=dot(u,a);
    Vector3 t; float sign=1;
    if (std::isfinite(determinant) && std::abs(determinant)>1e-12) {
        const float orientation=determinant<0 ? -1.f:1.f;
        t=normalize({a.x*orientation,a.y*orientation,a.z*orientation});
        sign=dot(cross(n,t),b)*orientation<0 ? -1.f:1.f;
    } else {
        const Vector3 axis=std::abs(n.x)<.9f ? Vector3{1,0,0}:Vector3{0,1,0};
        const float projection=static_cast<float>(dot(axis,n));
        t=normalize({axis.x-projection*n.x,axis.y-projection*n.y,axis.z-projection*n.z});
    }
    return {t.x,t.y,t.z,sign};
}
int size(std::size_t n) {
    if (n>static_cast<std::size_t>(std::numeric_limits<int>::max())) throw std::length_error("Export scene exceeds int indices");
    return static_cast<int>(n);
}
bool uncertainArea(const PreparedConvexBrush& brush,const TriangleRecord& triangle) {
    Vector3 positions[3]; double coordinateScale=0;
    for (int c=0;c<3;++c) {
        const int index=triangle.vertices[c];
        if (index<0 || static_cast<std::size_t>(index)>=brush.mesh.vertices.size())
            throw std::invalid_argument("Invalid export vertex index");
        positions[c]=brush.mesh.vertices[index].position;
        const auto normal=triangle.cornerNormals[c];
        if (!finite(positions[c]) || !finite(normal) || std::abs(dot(normal,normal)-1)>1e-4)
            throw std::invalid_argument("Invalid export vertex attributes");
        coordinateScale=std::max({coordinateScale,std::abs(double(positions[c].x)),std::abs(double(positions[c].y)),std::abs(double(positions[c].z))});
    }
    const double ax=double(positions[1].x)-positions[0].x,ay=double(positions[1].y)-positions[0].y,az=double(positions[1].z)-positions[0].z;
    const double bx=double(positions[2].x)-positions[0].x,by=double(positions[2].y)-positions[0].y,bz=double(positions[2].z)-positions[0].z;
    const double cx=ay*bz-az*by,cy=az*bx-ax*bz,cz=ax*by-ay*bx;
    const double edgeScale=std::max(std::sqrt(ax*ax+ay*ay+az*az),std::sqrt(bx*bx+by*by+bz*bz));
    // New export policy, not a modification of the recovered native algorithms.
    // Below this scale, float position rounding can change the area sign.
    const double uncertainty=8*std::numeric_limits<float>::epsilon()*coordinateScale*edgeScale;
    return cx*cx+cy*cy+cz*cz<=uncertainty*uncertainty;
}
}
ExportedScene exportPreparedScene(const SceneExportInput* inputs,int count) {
    if (count<0 || (count&&!inputs)) throw std::invalid_argument("Missing scene export inputs");
    struct Face { const PreparedConvexBrush* brush; const TriangleRecord* triangle; const SceneSurface* surface; };
    std::map<int,std::vector<Face>> batches; std::set<int> ids;
    std::size_t faceCount=0;
    for (int b=0;b<count;++b) {
        const auto& input=inputs[b];
        if (!input.brush || input.brush->brushIndex<0 || !ids.insert(input.brush->brushIndex).second
            || input.surfaceCount<0 || (input.surfaceCount&&!input.surfaces)) throw std::invalid_argument("Invalid scene export brush");
        for (int s=0;s<input.surfaceCount;++s)
            if (!finite(input.surfaces[s].uvU)||!finite(input.surfaces[s].uvV)) throw std::invalid_argument("Nonfinite UV projection");
        for (const auto& triangle:input.brush->mesh.triangles) {
            if (triangle.categorizationState==1) continue;
            if ((triangle.categorizationState!=2 && triangle.categorizationState!=3) || triangle.surfaceIndex<0
                || triangle.surfaceIndex>=input.surfaceCount) throw std::invalid_argument("Unresolved scene export surface");
            if (uncertainArea(*input.brush,triangle)) continue;
            batches[input.surfaces[triangle.surfaceIndex].materialId].push_back({input.brush,&triangle,&input.surfaces[triangle.surfaceIndex]});
            if (++faceCount>static_cast<std::size_t>(std::numeric_limits<int>::max())/3) throw std::length_error("Too many export corners");
        }
    }
    ExportedScene result; result.vertices.reserve(faceCount*3); result.indices.reserve(faceCount*3); result.triangles.reserve(faceCount);
    Vector3 lower{},upper{}; bool hasBounds=false;
    for (const auto& batch:batches) {
        const int first=size(result.indices.size());
        for (const auto& face:batch.second) {
            const auto& triangle=*face.triangle; const bool reversed=triangle.categorizationState==3;
            const unsigned order[]={0,reversed ? 2u:1u,reversed ? 1u:2u};
            for (unsigned c:order) {
                const int v=triangle.vertices[c];
                if (v<0 || static_cast<std::size_t>(v)>=face.brush->mesh.vertices.size()) throw std::invalid_argument("Invalid export vertex index");
                const auto position=face.brush->mesh.vertices[v].position; auto normal=triangle.cornerNormals[c];
                if (!finite(position)||!finite(normal)||std::abs(dot(normal,normal)-1)>1e-4) throw std::invalid_argument("Invalid export vertex attributes");
                if (reversed) normal={-normal.x,-normal.y,-normal.z};
                const auto& u=face.surface->uvU; const auto& w=face.surface->uvV;
                const SceneFloat2 uv{((u.x*position.x+u.y*position.y)+u.z*position.z)+u.w,
                    ((w.x*position.x+w.y*position.y)+w.z*position.z)+w.w};
                if (!std::isfinite(uv.x)||!std::isfinite(uv.y)) throw std::invalid_argument("UV projection overflow");
                result.indices.push_back(size(result.vertices.size())); result.vertices.push_back({position,normal,tangent(normal,*face.surface),uv});
                if (!hasBounds) { lower=upper=position; hasBounds=true; }
                lower={std::min(lower.x,position.x),std::min(lower.y,position.y),std::min(lower.z,position.z)};
                upper={std::max(upper.x,position.x),std::max(upper.y,position.y),std::max(upper.z,position.z)};
            }
            result.triangles.push_back({face.brush->brushIndex,triangle.surfaceIndex,batch.first,triangle.categorizationState,triangle.flags});
        }
        result.submeshes.push_back({batch.first,first,size(result.indices.size())-first});
    }
    result.counts={size(result.vertices.size()),size(result.indices.size()),size(result.submeshes.size()),size(result.triangles.size()),count,{},{}};
    if (hasBounds) {
        result.counts.boundsCenter={float((double(lower.x)+upper.x)/2),float((double(lower.y)+upper.y)/2),float((double(lower.z)+upper.z)/2)};
        result.counts.boundsSize={upper.x-lower.x,upper.y-lower.y,upper.z-lower.z};
        if (!finite(result.counts.boundsCenter)||!finite(result.counts.boundsSize)) throw std::invalid_argument("Scene bounds overflow");
    }
    return result;
}
}
