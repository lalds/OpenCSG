#if UNITY_EDITOR
using System;
using System.IO;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEditor;
using UnityEditor.SceneManagement;

namespace RealtimeCSG.Recovery
{
    // Explicit preview path. Existing RealtimeCSG editor/native bindings are untouched.
    public static class UnityPreview
    {
        public static Mesh CreateMesh(Contents contents)
        {
            var mesh = new Mesh { name = "Recovered native CSG", indexFormat = IndexFormat.UInt32 };
            try
            {
                var positions = new Vector3[contents.vertices.Length]; var normals = new Vector3[positions.Length];
                var tangents = new Vector4[positions.Length]; var uv = new Vector2[positions.Length];
                for (int i = 0; i < positions.Length; ++i)
                {
                    var v = contents.vertices[i];
                    positions[i] = new Vector3(v.position.x,v.position.y,v.position.z);
                    normals[i] = new Vector3(v.normal.x,v.normal.y,v.normal.z);
                    tangents[i] = new Vector4(v.tangent.x,v.tangent.y,v.tangent.z,v.tangent.w);
                    uv[i] = new Vector2(v.uv.x,v.uv.y);
                }
                mesh.vertices = positions; mesh.normals = normals; mesh.tangents = tangents; mesh.uv = uv;
                mesh.subMeshCount = contents.submeshes.Length;
                for (int i = 0; i < contents.submeshes.Length; ++i)
                {
                    var range = contents.submeshes[i]; var indices = new int[range.indexCount];
                    Array.Copy(contents.indices,range.firstIndex,indices,0,range.indexCount);
                    mesh.SetTriangles(indices,i,false);
                }
                var center = contents.counts.boundsCenter; var size = contents.counts.boundsSize;
                mesh.bounds = new Bounds(new Vector3(center.x,center.y,center.z),new Vector3(size.x,size.y,size.z));
                return mesh;
            }
            catch { UnityEngine.Object.DestroyImmediate(mesh); throw; }
        }
        private static Contents SubtractExample()
        {
            var a = Primitives.Box(new Float3(-1,-1,-1),new Float3(1,1,1),1,11);
            var b = Primitives.Box(new Float3(-.5f,-.5f,-2),new Float3(.5f,.5f,2),2,89,10);
            using (Scene scene = Scene.Build(Primitives.PairTree(1),new[] {1,2},new[] {a,b})) return scene.Read();
        }
        private static Material[] Materials(Contents contents)
        {
            Shader shader = Shader.Find("Standard");
            if (!shader) shader = Shader.Find("Hidden/InternalErrorShader");
            if (!shader) throw new InvalidOperationException("Preview shader unavailable");
            var result = new Material[contents.submeshes.Length];
            for (int i = 0; i < result.Length; ++i)
                result[i] = new Material(shader) {
                    name = "Recovery material " + contents.submeshes[i].materialId,
                    color = contents.submeshes[i].materialId >= 10 ? new Color(.95f,.45f,.12f) : new Color(.15f,.65f,.85f)
                };
            return result;
        }
        [MenuItem("Tools/RealtimeCSG Recovery/Create native subtract preview")]
        public static void CreatePreview()
        {
            Contents contents = SubtractExample(); Mesh mesh = CreateMesh(contents);
            var go = new GameObject("Recovered CSG subtract preview",typeof(MeshFilter),typeof(MeshRenderer));
            Undo.RegisterCreatedObjectUndo(go,"Create recovered CSG preview");
            go.GetComponent<MeshFilter>().sharedMesh = mesh; go.GetComponent<MeshRenderer>().sharedMaterials = Materials(contents);
            Selection.activeGameObject = go;
        }
        private static double Volume(Mesh mesh)
        {
            Vector3[] p = mesh.vertices; int[] indices = mesh.triangles; double sum = 0;
            for (int i = 0; i < indices.Length; i += 3)
            {
                Vector3 a=p[indices[i]], b=p[indices[i+1]], c=p[indices[i+2]];
                sum += a.x*((double)b.y*c.z-(double)b.z*c.y)+a.y*((double)b.z*c.x-(double)b.x*c.z)+a.z*((double)b.x*c.y-(double)b.y*c.x);
            }
            return sum/6;
        }
        public static void BatchSmoke()
        {
            try
            {
                var scene = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene,NewSceneMode.Single);
                Contents contents = SubtractExample(); Mesh mesh = CreateMesh(contents);
                double volume = Volume(mesh);
                if (Math.Abs(volume-6)>1e-5 || mesh.vertexCount!=contents.counts.vertexCount || mesh.triangles.Length!=contents.counts.indexCount
                    || mesh.normals.Length!=mesh.vertexCount || mesh.uv.Length!=mesh.vertexCount || mesh.tangents.Length!=mesh.vertexCount)
                    throw new InvalidOperationException("Unity mesh readback did not match native scene");
                string output = AssetDatabase.GenerateUniqueAssetPath("Assets/RecoveryOutput");
                Directory.CreateDirectory(output); AssetDatabase.Refresh();
                AssetDatabase.CreateAsset(mesh,output+"/RecoveredSubtract.asset");
                Material[] materials = Materials(contents);
                for (int i=0;i<materials.Length;++i) AssetDatabase.CreateAsset(materials[i],output+"/Material_"+i+".mat");
                var go = new GameObject("Recovered CSG subtract preview",typeof(MeshFilter),typeof(MeshRenderer));
                go.GetComponent<MeshFilter>().sharedMesh = mesh; go.GetComponent<MeshRenderer>().sharedMaterials = materials;
                AssetDatabase.SaveAssets();
                if (!EditorSceneManager.SaveScene(scene,output+"/RecoveryPreview.unity")) throw new InvalidOperationException("Could not save preview scene");
                string report = "{\"unity\":\""+Application.unityVersion+"\",\"vertices\":"+mesh.vertexCount+
                    ",\"indices\":"+mesh.triangles.Length+",\"submeshes\":"+mesh.subMeshCount+",\"volume\":"+
                    volume.ToString(System.Globalization.CultureInfo.InvariantCulture)+",\"nativeAbi\":1,\"assetDirectory\":\""+output+"\"}";
                File.WriteAllText("RecoveryPreviewReport.json",report);
                Debug.Log("RECOVERY_UNITY_SMOKE_PASS "+report); EditorApplication.Exit(0);
            }
            catch (Exception error) { Debug.LogException(error); EditorApplication.Exit(1); }
        }
    }
}
#endif
