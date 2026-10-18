#if UNITY_EDITOR
using System;
using System.IO;
using UnityEngine;
using UnityEditor;
using UnityEditor.SceneManagement;
using RealtimeCSG.Foundation;

namespace RealtimeCSG.Recovery
{
    public static class FoundationSmoke
    {
        private static int cases;
        private static string context;
        private static void Require(bool value,string message)
        { if (!value) throw new InvalidOperationException(context+": "+message); }
        private static void Close(double actual,double expected,string message,double epsilon=2e-4)
        { Require(!double.IsNaN(actual) && Math.Abs(actual-expected)<=epsilon,message+": "+actual+" vs "+expected); }
        private static Vector3 Position(Float3 p) { return new Vector3(p.x,p.y,p.z); }
        private static BrushMesh Cube(Vector3 size,int material=41)
        {
            // This is the repository's original factory, not Recovery.Primitives.
            var mesh=BrushMeshUtility.CreateCube(size,new SurfaceLayers {
                layerUsage=LayerUsageFlags.RenderReceiveCastShadows|LayerUsageFlags.Collidable,
                layerParameter1=material, layerParameter2=9001, layerParameter3=72
            });
            for (int p=0;p<mesh.polygons.Length;++p)
            {
                mesh.polygons[p].polygonID=100+7*p;
                mesh.polygons[p].surface.UV0=new UVMatrix {
                    U=new Vector4(.7f,-.4f,1.1f,.25f), V=new Vector4(-.2f,.9f,.6f,-.5f)
                };
                if ((p&1)!=0) mesh.polygons[p].surface.surfaceFlags=SurfaceFlags.TextureIsInWorldSpace;
                mesh.polygons[p].layers.layerParameter1=material+p%2;
            }
            return mesh;
        }
        private static Contents Build(FoundationBrush source)
        {
            using (var scene=Scene.Build(new[] {new Node {childCount=1},new Node {isBrush=1}},new[] {1},new[] {source.geometry}))
                return scene.Read();
        }
        private static double Volume(Contents mesh)
        {
            double sum=0;
            for (int i=0;i<mesh.indices.Length;i+=3)
            {
                Vector3 a=Position(mesh.vertices[mesh.indices[i]].position);
                Vector3 b=Position(mesh.vertices[mesh.indices[i+1]].position);
                Vector3 c=Position(mesh.vertices[mesh.indices[i+2]].position);
                sum+=a.x*((double)b.y*c.z-(double)b.z*c.y)+a.y*((double)b.z*c.x-(double)b.x*c.z)+a.z*((double)b.x*c.y-(double)b.y*c.x);
            }
            return sum/6;
        }
        private static void CheckAttributes(Contents output,FoundationBrush[] brushes,Matrix4x4[] modelFromBrush,Matrix4x4 worldFromModel)
        {
            for (int t=0;t<output.triangles.Length;++t)
            {
                var provenance=output.triangles[t]; int index=Array.FindIndex(brushes,b=>b.geometry.brushIndex==provenance.brushIndex);
                Require(index>=0,"Unknown source brush");
                var source=brushes[index].SourcePolygon(provenance.surfaceIndex);
                Require(source.polygonID==100+7*provenance.surfaceIndex,"Polygon IDs lost");
                Require(source.layers.layerParameter1==provenance.materialId && source.layers.layerParameter2==9001
                    && source.layers.layerParameter3==72,"Source material/layer metadata lost");
                var triangle=new Vector3[3];
                for (int c=0;c<3;++c)
                {
                    Vertex vertex=output.vertices[output.indices[3*t+c]]; Vector3 model=Position(vertex.position); triangle[c]=model;
                    Vector3 uvPoint=(source.surface.surfaceFlags & SurfaceFlags.TextureIsInWorldSpace)!=0
                        ?worldFromModel.MultiplyPoint3x4(model):modelFromBrush[index].inverse.MultiplyPoint3x4(model);
                    Vector4 point=new Vector4(uvPoint.x,uvPoint.y,uvPoint.z,1);
                    Close(vertex.uv.x,Vector4.Dot(source.surface.UV0.U,point),"U differs from original Foundation row");
                    Close(vertex.uv.y,Vector4.Dot(source.surface.UV0.V,point),"V differs from original Foundation row");
                    Vector3 normal=Position(vertex.normal); Vector3 tangent=new Vector3(vertex.tangent.x,vertex.tangent.y,vertex.tangent.z);
                    Close(normal.magnitude,1,"Normal length"); Close(tangent.magnitude,1,"Tangent length");
                    Close(Vector3.Dot(normal,tangent),0,"Tangent orthogonality");
                }
                Vector3 geometric=Vector3.Cross(triangle[1]-triangle[0],triangle[2]-triangle[0]);
                float alignment=Vector3.Dot(geometric,Position(output.vertices[output.indices[3*t]].normal));
                Require(alignment>0,"Reflection/subtraction winding differs from normal, triangle "+t+", dot="+alignment+
                    ", points="+triangle[0].ToString("R")+" / "+triangle[1].ToString("R")+" / "+triangle[2].ToString("R"));
            }
        }
        private static void Reject(Action action,string name)
        {
            bool rejected=false;
            try { action(); } catch (ArgumentException) { rejected=true; } catch (NotSupportedException) { rejected=true; }
            Require(rejected,"Accepted invalid "+name); ++cases;
        }
        public static void BatchSmoke()
        {
            try
            {
                var world=Matrix4x4.TRS(new Vector3(7,-2,3),Quaternion.Euler(12,28,-17),new Vector3(1.2f,.8f,1.1f));
                var shear=Matrix4x4.identity; shear.m01=.35f; shear.m12=-.2f;
                var transforms=new[] {
                    Matrix4x4.identity, Matrix4x4.Translate(new Vector3(3,-2,1)),
                    Matrix4x4.Rotate(Quaternion.Euler(23,47,11)), Matrix4x4.Scale(new Vector3(1.5f,.75f,2)),
                    Matrix4x4.TRS(new Vector3(2,1,-3),Quaternion.Euler(31,17,53),new Vector3(1.5f,.75f,2)),
                    Matrix4x4.Scale(new Vector3(-1,1,1)),
                    Matrix4x4.TRS(new Vector3(-2,1,3),Quaternion.Euler(11,32,19),new Vector3(-1.5f,.75f,2)),
                    Matrix4x4.TRS(new Vector3(2,0,-1),Quaternion.Euler(9,14,31),new Vector3(1.1f,.8f,1.3f))*shear
                };
                cases=0;
                foreach (var transform in transforms)
                {
                    context="single transform "+cases;
                    var source=Cube(new Vector3(2,2,2),-17);
                    var before=(Vector3[])source.vertices.Clone(); var edges=(BrushMesh.HalfEdge[])source.halfEdges.Clone();
                    var converted=FoundationBridge.Convert(source,transform,world,1,77);
                    Contents output=Build(converted);
                    Close(Volume(output),8*Math.Abs(transform.determinant),"Transformed volume",.002);
                    CheckAttributes(output,new[] {converted},new[] {transform},world);
                    Require(output.counts.triangleCount==12,"Unexpected cube triangulation");
                    for (int i=0;i<before.Length;++i) Require(before[i]==source.vertices[i],"Source vertices changed");
                    for (int i=0;i<edges.Length;++i) Require(edges[i].vertexIndex==source.halfEdges[i].vertexIndex
                        && edges[i].twinIndex==source.halfEdges[i].twinIndex,"Source halfedges changed");
                    // Geometry and source metadata must survive later edits of caller arrays.
                    source.vertices[0]=Vector3.zero; source.polygons[0].polygonID=-999;
                    Close(Volume(Build(converted)),8*Math.Abs(transform.determinant),"Snapshot changed",.002);
                    Require(converted.SourcePolygon(0).polygonID==100,"Source metadata was borrowed");
                    ++cases;
                }
                Contents subtract=null;
                for (byte operation=0;operation<3;++operation)
                {
                    context="rotated reflected cutter operation "+operation;
                    var a=FoundationBridge.Convert(Cube(new Vector3(2,2,2)),Matrix4x4.identity,world,1,11);
                    var transform=Matrix4x4.TRS(Vector3.zero,Quaternion.Euler(0,0,30),new Vector3(-1,1,1));
                    var b=FoundationBridge.Convert(Cube(new Vector3(1,1,4),61),transform,world,2,89);
                    Contents output;
                    using (var scene=Scene.Build(Primitives.PairTree(operation),new[] {1,2},new[] {a.geometry,b.geometry})) output=scene.Read();
                    Close(Volume(output),operation==0?10:operation==1?6:2,"Rotated reflected CSG volume");
                    CheckAttributes(output,new[] {a,b},new[] {Matrix4x4.identity,transform},world);
                    if (operation==1) subtract=output;
                    ++cases;
                }
                // An invertible common affine transform preserves Boolean operations.
                // Check both inputs transformed, rather than just a transformed cutter.
                foreach (int transformIndex in new[] {4,6,7}) for (byte operation=0;operation<3;++operation)
                {
                    context="common transform "+transformIndex+", operation "+operation;
                    Matrix4x4 common=transforms[transformIndex];
                    Matrix4x4 cutter=common*Matrix4x4.TRS(Vector3.zero,Quaternion.Euler(0,0,30),new Vector3(-1,1,1));
                    var a=FoundationBridge.Convert(Cube(new Vector3(2,2,2)),common,world,1,11);
                    var b=FoundationBridge.Convert(Cube(new Vector3(1,1,4),61),cutter,world,2,89);
                    Contents output;
                    using (var scene=Scene.Build(Primitives.PairTree(operation),new[] {1,2},new[] {a.geometry,b.geometry})) output=scene.Read();
                    Close(Volume(output),(operation==0?10:operation==1?6:2)*Math.Abs(common.determinant),"Common affine CSG volume",.003);
                    CheckAttributes(output,new[] {a,b},new[] {common,cutter},world);
                    ++cases;
                }
                Reject(()=>FoundationBridge.Convert(Cube(Vector3.one),Matrix4x4.Scale(new Vector3(0,1,1)),world,1,1),"singular transform");
                var invalid=Matrix4x4.identity; invalid.m00=float.NaN;
                Reject(()=>FoundationBridge.Convert(Cube(Vector3.one),invalid,world,1,1),"NaN transform");
                invalid=Matrix4x4.identity; invalid.m30=.1f;
                Reject(()=>FoundationBridge.Convert(Cube(Vector3.one),invalid,world,1,1),"projective transform");
                var malformed=Cube(Vector3.one); malformed.polygons[0].surface.smoothingGroup=1;
                Reject(()=>FoundationBridge.Convert(malformed,Matrix4x4.identity,world,1,1),"unsupported smoothing");
                malformed=Cube(Vector3.one); malformed.polygons[1].firstEdge=0;
                Reject(()=>FoundationBridge.Convert(malformed,Matrix4x4.identity,world,1,1),"overlapping polygon spans");
                malformed=Cube(Vector3.one); malformed.halfEdges[0].vertexIndex=999;
                Reject(()=>FoundationBridge.Convert(malformed,Matrix4x4.identity,world,1,1),"invalid vertex index");
                // Native validation still rejects valid indices with broken reciprocal topology.
                malformed=Cube(Vector3.one); malformed.halfEdges[0].twinIndex=1;
                var broken=FoundationBridge.Convert(malformed,Matrix4x4.identity,world,1,1);
                bool topologyRejected=false;
                try { Build(broken); } catch (InvalidOperationException) { topologyRejected=true; }
                Require(topologyRejected,"Native build accepted malformed topology"); ++cases;
                EditorSceneManager.NewScene(NewSceneSetup.EmptyScene,NewSceneMode.Single);
                string outputPath=AssetDatabase.GenerateUniqueAssetPath("Assets/FoundationOutput");
                Directory.CreateDirectory(outputPath); AssetDatabase.Refresh();
                Mesh mesh=UnityPreview.CreateMesh(subtract);
                AssetDatabase.CreateAsset(mesh,outputPath+"/FoundationSubtract.asset");
                var go=new GameObject("Original Foundation brushes through recovered kernel",typeof(MeshFilter),typeof(MeshRenderer));
                go.GetComponent<MeshFilter>().sharedMesh=mesh;
                var materials=new Material[subtract.submeshes.Length];
                for (int i=0;i<materials.Length;++i)
                {
                    materials[i]=new Material(Shader.Find("Standard")) { color=new Color(.2f,.6f,.85f),name="Source material "+subtract.submeshes[i].materialId };
                    AssetDatabase.CreateAsset(materials[i],outputPath+"/Material_"+i+".mat");
                }
                go.GetComponent<MeshRenderer>().sharedMaterials=materials;
                AssetDatabase.SaveAssets();
                Require(EditorSceneManager.SaveScene(UnityEngine.SceneManagement.SceneManager.GetActiveScene(),outputPath+"/FoundationPreview.unity"),"Scene save failed");
                string report="{\"unity\":\""+Application.unityVersion+"\",\"cases\":"+cases+
                    ",\"volume\":"+Volume(subtract).ToString("R",System.Globalization.CultureInfo.InvariantCulture)+
                    ",\"nativeAbi\":1,\"source\":\"original Foundation BrushMeshUtility\",\"assetDirectory\":\""+outputPath+"\"}";
                File.WriteAllText("RecoveryFoundationReport.json",report);
                Debug.Log("RECOVERY_FOUNDATION_SMOKE_PASS "+report); EditorApplication.Exit(0);
            }
            catch (Exception error) { Debug.LogException(error); EditorApplication.Exit(1); }
        }
    }
}
#endif
