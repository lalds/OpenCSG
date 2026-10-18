#if UNITY_EDITOR
using System;
using System.IO;
using UnityEngine;
using UnityEditor;
using UnityEditor.SceneManagement;
using RealtimeCSG.Foundation;

namespace RealtimeCSG.Recovery
{
    public static class PrototypeSmoke
    {
        private static RecoveryModel model;
        private static RecoveryBrush cutter;
        private static int step, builds, checks;
        private static double readyAt, deadline;
        private static string output;
        private static Mesh lastGood;
        private static void Require(bool condition,string message)
        { if (!condition) throw new InvalidOperationException("Step "+step+": "+message); ++checks; }
        private static double Volume(Mesh mesh)
        {
            if (!mesh) return 0;
            Vector3[] p=mesh.vertices; int[] indices=mesh.triangles; double result=0;
            for (int i=0;i<indices.Length;i+=3)
            {
                Vector3 a=p[indices[i]],b=p[indices[i+1]],c=p[indices[i+2]];
                result+=a.x*((double)b.y*c.z-(double)b.z*c.y)+a.y*((double)b.z*c.x-(double)b.x*c.z)+a.z*((double)b.x*c.y-(double)b.y*c.x);
            }
            return result/6;
        }
        private static void CheckVolume(double expected)
        {
            Require(model && string.IsNullOrEmpty(model.LastError),"Build error: "+(model?model.LastError:"missing model"));
            double actual=Volume(model.GeneratedMesh);
            Require(Math.Abs(actual-expected)<1e-4,"Expected volume "+expected+", received "+actual);
        }
        private static void Next()
        { ++step; readyAt=EditorApplication.timeSinceStartup+.2; deadline=EditorApplication.timeSinceStartup+20; builds=model?model.BuildCount:0; }
        public static void BatchSmoke()
        {
            try
            {
                var importer=(PluginImporter)AssetImporter.GetAtPath("Assets/Plugins/x86_64/librealtimecsg_recovered_geometry.so");
                importer.SetCompatibleWithAnyPlatform(false);
                importer.SetCompatibleWithEditor(true);
                importer.SetEditorData("OS","Linux"); importer.SetEditorData("CPU","x86_64");
                importer.SetCompatibleWithPlatform(BuildTarget.StandaloneWindows,false);
                importer.SetCompatibleWithPlatform(BuildTarget.StandaloneWindows64,false);
                importer.SetCompatibleWithPlatform(BuildTarget.StandaloneOSX,false);
                importer.SetCompatibleWithPlatform(BuildTarget.StandaloneLinux64,false);
                importer.SaveAndReimport();
                EditorSceneManager.NewScene(NewSceneSetup.EmptyScene,NewSceneMode.Single);
                output=AssetDatabase.GenerateUniqueAssetPath("Assets/PrototypeOutput");
                Directory.CreateDirectory(output); AssetDatabase.Refresh();
                Require(EditorApplication.ExecuteMenuItem("Tools/RealtimeCSG Recovery/Create editable CSG prototype"),"Prototype menu missing");
                model=UnityEngine.Object.FindObjectOfType<RecoveryModel>();
                cutter=Array.Find(model.GetComponentsInChildren<RecoveryBrush>(),b=>b.operation==CSGOperationType.Subtractive);
                var brushes=model.GetComponentsInChildren<RecoveryBrush>();
                for (int i=0;i<brushes.Length;++i)
                {
                    var material=new Material(Shader.Find("Standard")) {
                        name=i==0?"Solid material":"Cutter material",color=i==0?new Color(.2f,.65f,.85f):new Color(.95f,.4f,.1f)
                    };
                    AssetDatabase.CreateAsset(material,output+"/Material_"+i+".mat"); brushes[i].material=material;
                }
                model.Rebuild(); CheckVolume(6);
                step=0; readyAt=EditorApplication.timeSinceStartup+.4; deadline=readyAt+20;
                EditorApplication.update+=Tick;
            }
            catch (Exception error) { Fail(error); }
        }
        private static void Tick()
        {
            try
            {
                double now=EditorApplication.timeSinceStartup;
                if (now>deadline) throw new TimeoutException("Editor rebuild timed out at step "+step);
                if (now<readyAt) return;
                // Allow the component's editor update to run before checking automatic rebuilds.
                if (step!=0 && step!=1 && step!=9 && step!=13 && model && model.BuildCount<=builds && string.IsNullOrEmpty(model.LastError)) return;
                switch (step)
                {
                    case 0:
                        CheckVolume(6); Next(); return;
                    case 1:
                        Require(model.BuildCount==builds,"Idle model keeps rebuilding");
                        Undo.IncrementCurrentGroup(); Undo.SetCurrentGroupName("Prototype move cutter");
                        Undo.RecordObject(cutter.transform,"Prototype move cutter");
                        cutter.transform.localPosition=new Vector3(3,0,0); Undo.FlushUndoRecordObjects(); Next(); return;
                    case 2:
                        CheckVolume(8); Undo.PerformUndo(); Next(); return;
                    case 3:
                        CheckVolume(6); Undo.PerformRedo(); Next(); return;
                    case 4:
                        CheckVolume(8); cutter.transform.localPosition=Vector3.zero; cutter.operation=CSGOperationType.Additive; Next(); return;
                    case 5:
                        CheckVolume(10); cutter.operation=CSGOperationType.Intersecting; Next(); return;
                    case 6:
                        CheckVolume(2); cutter.enabled=false; Next(); return;
                    case 7:
                        CheckVolume(8); cutter.enabled=true; cutter.operation=CSGOperationType.Subtractive;
                        cutter.transform.localScale=new Vector3(-1,.5f,1); Next(); return;
                    case 8:
                        CheckVolume(7); lastGood=model.GeneratedMesh; cutter.size=Vector3.zero; Next(); return;
                    case 9:
                        if (string.IsNullOrEmpty(model.LastError)) return;
                        Require(model.GeneratedMesh==lastGood,"Invalid input discarded the last good mesh");
                        Require(Math.Abs(Volume(lastGood)-7)<1e-4,"Invalid input changed geometry");
                        cutter.size=new Vector3(1,1,4); cutter.transform.localScale=Vector3.one; Next(); return;
                    case 10:
                        CheckVolume(6);
                        var renderer=model.GetComponentInChildren<MeshRenderer>();
                        Require(renderer && renderer.sharedMaterials.Length==2,"Source materials not assigned to submeshes");
                        Require(Array.IndexOf(renderer.sharedMaterials,cutter.material)>=0,"Cutter material lost");
                        cutter.transform.localRotation=Quaternion.Euler(0,0,30); Next(); return;
                    case 11:
                        CheckVolume(6); cutter.transform.localRotation=Quaternion.identity;
                        model.transform.SetPositionAndRotation(new Vector3(2,1,-3),Quaternion.Euler(20,40,10)); Next(); return;
                    case 12:
                        CheckVolume(6);
                        // Cleanup must not adopt or delete an unrelated object with the output name.
                        var userChild=new GameObject("__Recovered CSG Output"); userChild.transform.SetParent(model.transform,false);
                        model.enabled=false; Require(userChild,"Cleanup removed a user object");
                        UnityEngine.Object.DestroyImmediate(userChild); model.enabled=true; Next(); return;
                    case 13:
                        if (model.BuildCount<=builds) return;
                        CheckVolume(6); Finish(); return;
                }
            }
            catch (Exception error) { Fail(error); }
        }
        private static void Finish()
        {
            EditorApplication.update-=Tick;
            Mesh copy=UnityEngine.Object.Instantiate(model.GeneratedMesh); copy.hideFlags=HideFlags.None;
            AssetDatabase.CreateAsset(copy,output+"/ExportedCSG.asset");
            // Make the saved scene useful in Game view as well as Scene view.
            var camera=new GameObject("Preview Camera",typeof(Camera));
            camera.transform.position=model.transform.TransformPoint(new Vector3(4,3,-5));
            camera.transform.LookAt(model.transform.position); camera.GetComponent<Camera>().backgroundColor=new Color(.12f,.14f,.18f);
            camera.GetComponent<Camera>().clearFlags=CameraClearFlags.SolidColor;
            var light=new GameObject("Preview Light",typeof(Light)); light.GetComponent<Light>().type=LightType.Directional;
            light.transform.rotation=Quaternion.Euler(45,-30,0);
            AssetDatabase.SaveAssets();
            string scenePath=output+"/EditablePrototype.unity";
            Require(EditorSceneManager.SaveScene(model.gameObject.scene,scenePath),"Scene save failed");
            File.WriteAllText("RecoveryPrototypeScene.txt",scenePath);
            AssetDatabase.ExportPackage(new[] {"Assets/Recovery","Assets/Editor/Recovery/RecoveryPrototypeEditor.cs",
                "Assets/Plugins/x86_64/librealtimecsg_recovered_geometry.so"},"OpenCSG-Prototype.unitypackage",ExportPackageOptions.Recurse);
            string report="{\"unity\":\""+Application.unityVersion+"\",\"checks\":"+checks+",\"builds\":"+model.BuildCount+
                ",\"volume\":"+Volume(model.GeneratedMesh).ToString("R",System.Globalization.CultureInfo.InvariantCulture)+
                ",\"scene\":\""+scenePath+"\",\"package\":\"OpenCSG-Prototype.unitypackage\"}";
            File.WriteAllText("RecoveryPrototypeReport.json",report); Debug.Log("RECOVERY_PROTOTYPE_PASS "+report);
            EditorApplication.Exit(0);
        }
        public static void ReloadSmoke()
        {
            try
            {
                string scenePath=File.ReadAllText("RecoveryPrototypeScene.txt").Trim();
                EditorSceneManager.OpenScene(scenePath);
                model=UnityEngine.Object.FindObjectOfType<RecoveryModel>(); model.Rebuild(); CheckVolume(6);
                Require(model.GetComponentsInChildren<RecoveryBrush>().Length==2,"Sources did not survive scene reload");
                Require(model.GetComponentsInChildren<MeshFilter>().Length==1,"Reload duplicated generated output");
                File.WriteAllText("RecoveryPrototypeReloadReport.json","{\"reopened\":true,\"volume\":6}");
                Debug.Log("RECOVERY_PROTOTYPE_RELOAD_PASS"); EditorApplication.Exit(0);
            }
            catch (Exception error) { Fail(error); }
        }
        private static void Fail(Exception error)
        { EditorApplication.update-=Tick; Debug.LogException(error); EditorApplication.Exit(1); }
    }
}
#endif
