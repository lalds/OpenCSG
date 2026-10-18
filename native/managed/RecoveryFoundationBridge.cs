#if UNITY_EDITOR
using System;
using UnityEngine;
using RealtimeCSG.Foundation;

namespace RealtimeCSG.Recovery
{
    // Geometry adapter at the existing editor's BrushMesh boundary. No old DLL calls.
    public sealed class FoundationBrush
    {
        public readonly Brush geometry;
        private readonly BrushMesh.Polygon[] sources;
        internal FoundationBrush(Brush geometry, BrushMesh.Polygon[] sources)
        { this.geometry = geometry; this.sources = sources; }
        public BrushMesh.Polygon SourcePolygon(int surfaceIndex)
        {
            if (surfaceIndex < 0 || surfaceIndex >= sources.Length) throw new ArgumentOutOfRangeException("surfaceIndex");
            return sources[surfaceIndex];
        }
    }

    public static class FoundationBridge
    {
        private static bool Finite(float value) { return !float.IsNaN(value) && !float.IsInfinity(value); }
        private static void RequireFinite(Vector4 value)
        {
            if (!Finite(value.x) || !Finite(value.y) || !Finite(value.z) || !Finite(value.w))
                throw new ArgumentException("Nonfinite brush data or transform");
        }
        private static Float3 Position(Vector3 p) { return new Float3(p.x,p.y,p.z); }
        private static Float4 Row(Vector4 p) { RequireFinite(p); return new Float4(p.x,p.y,p.z,p.w); }
        private static void Affine(Matrix4x4 matrix)
        {
            for (int i=0;i<4;++i) RequireFinite(matrix.GetRow(i));
            if (matrix.m30!=0 || matrix.m31!=0 || matrix.m32!=0 || matrix.m33!=1)
                throw new ArgumentException("Brush transforms must be affine");
        }

        // worldFromModel is needed for surfaces whose UV rows are defined in world space.
        // All CSG positions/planes are returned in model space. The input remains untouched.
        public static FoundationBrush Convert(BrushMesh source, Matrix4x4 modelFromBrush,
                                              Matrix4x4 worldFromModel, int nodeIndex, int brushIndex)
        {
            if (source==null || source.vertices==null || source.halfEdges==null || source.polygons==null)
                throw new ArgumentException("Missing Foundation BrushMesh arrays");
            if (source.vertices.Length<4 || source.polygons.Length<4 || source.halfEdges.Length<12)
                throw new ArgumentException("A closed convex brush is required");
            Affine(modelFromBrush); Affine(worldFromModel);
            float determinant=modelFromBrush.determinant;
            if (!Finite(determinant) || determinant==0) throw new ArgumentException("Singular brush transform");
            Matrix4x4 brushFromModel=modelFromBrush.inverse;
            for (int i=0;i<4;++i) RequireFinite(brushFromModel.GetRow(i));
            // Unity may return a zero inverse for a numerically singular matrix.
            Matrix4x4 identity=brushFromModel*modelFromBrush;
            for (int i=0;i<4;++i) for (int j=0;j<4;++j)
                if (!Finite(identity[i,j]) || Mathf.Abs(identity[i,j]-(i==j?1:0))>1e-3f)
                    throw new ArgumentException("Unstable brush transform inverse");
            bool reflected=determinant<0;
            var brush=new Brush {
                nodeIndex=nodeIndex, brushIndex=brushIndex, vertices=new Float3[source.vertices.Length],
                edges=new HalfEdge[source.halfEdges.Length], polygons=new Polygon[source.polygons.Length],
                surfaces=new Surface[source.polygons.Length]
            };
            for (int v=0;v<brush.vertices.Length;++v)
            {
                RequireFinite(source.vertices[v]);
                Vector3 point=modelFromBrush.MultiplyPoint3x4(source.vertices[v]); RequireFinite(point);
                brush.vertices[v]=Position(point);
            }
            int[] remap=new int[brush.edges.Length];
            for (int i=0;i<remap.Length;++i) remap[i]=-1;
            for (int p=0;p<source.polygons.Length;++p)
            {
                var polygon=source.polygons[p]; int first=polygon.firstEdge, count=polygon.edgeCount;
                if (count<3 || first<0 || first>brush.edges.Length-count)
                    throw new ArgumentException("Invalid Foundation polygon span");
                if (polygon.surface.smoothingGroup!=0)
                    throw new NotSupportedException("Recovery bridge currently supports flat surface normals only");
                if ((polygon.surface.surfaceFlags & ~SurfaceFlags.TextureIsInWorldSpace)!=0)
                    throw new NotSupportedException("Unknown Foundation surface flags");
                for (int c=0;c<count;++c)
                {
                    int e=first+c; var half=source.halfEdges[e];
                    if (remap[e]!=-1 || half.vertexIndex<0 || half.vertexIndex>=source.vertices.Length
                        || half.twinIndex<0 || half.twinIndex>=brush.edges.Length)
                        throw new ArgumentException("Invalid Foundation halfedge ownership/index");
                    remap[e]=first+(reflected?count-1-c:c);
                }
            }
            for (int e=0;e<remap.Length;++e)
                if (remap[e]<0) throw new ArgumentException("Foundation halfedge has no polygon");
            for (int p=0;p<source.polygons.Length;++p)
            {
                var polygon=source.polygons[p]; int first=polygon.firstEdge, count=polygon.edgeCount;
                Vector3 origin=source.vertices[source.halfEdges[first].vertexIndex];
                double nx=0,ny=0,nz=0;
                for (int c=0;c<count;++c)
                {
                    Vector3 a=source.vertices[source.halfEdges[first+c].vertexIndex]-origin;
                    Vector3 b=source.vertices[source.halfEdges[first+(c+1)%count].vertexIndex]-origin;
                    nx+=(double)a.y*b.z-(double)a.z*b.y;
                    ny+=(double)a.z*b.x-(double)a.x*b.z;
                    nz+=(double)a.x*b.y-(double)a.y*b.x;
                    var half=source.halfEdges[first+c];
                    // An edge stores its end vertex. Reverse its endpoints as well as its order.
                    int vertex=reflected?source.halfEdges[first+(c+count-1)%count].vertexIndex:half.vertexIndex;
                    brush.edges[remap[first+c]]=new HalfEdge { vertexIndex=vertex, twinIndex=remap[half.twinIndex] };
                }
                double length=Math.Sqrt(nx*nx+ny*ny+nz*nz);
                if (!(length>0) || double.IsInfinity(length)) throw new ArgumentException("Degenerate Foundation polygon");
                var normal=new Vector3((float)(nx/length),(float)(ny/length),(float)(nz/length));
                Vector4 equation=brushFromModel.transpose*new Vector4(normal.x,normal.y,normal.z,-Vector3.Dot(normal,origin));
                float scale=new Vector3(equation.x,equation.y,equation.z).magnitude;
                if (!Finite(scale) || scale==0) throw new ArgumentException("Degenerate transformed surface plane");
                equation/=scale; RequireFinite(equation);
                Matrix4x4 uvFromModel=(polygon.surface.surfaceFlags & SurfaceFlags.TextureIsInWorldSpace)!=0
                    ?worldFromModel:brushFromModel;
                brush.surfaces[p]=new Surface {
                    plane=new Plane { normal=new Float3(equation.x,equation.y,equation.z), distance=-equation.w },
                    uvU=Row(uvFromModel.transpose*polygon.surface.UV0.U),
                    uvV=Row(uvFromModel.transpose*polygon.surface.UV0.V), materialId=polygon.layers.layerParameter1
                };
                brush.polygons[p]=new Polygon { firstEdge=first, edgeCount=count, surfaceIndex=p };
            }
            return new FoundationBrush(brush,(BrushMesh.Polygon[])source.polygons.Clone());
        }
    }
}
#endif
