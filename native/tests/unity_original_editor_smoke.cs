#if UNITY_EDITOR
using System;
using System.IO;
using System.Collections.Generic;
using System.Reflection;
using UnityEngine;
using UnityEditor;
using UnityEditor.SceneManagement;
using RealtimeCSG.Components;
using RealtimeCSG.Foundation;
using RealtimeCSG.Legacy;
using InternalRealtimeCSG;

namespace RealtimeCSG.Recovery
{
    public static class OriginalEditorSmoke
    {
        private static CSGModel model;
        private static CSGBrush solid,cutter;
        private static double deadline,ready;
        private static int step,checks;
        private static int editorErrors;
        private static void Require(bool value,string text)
        { if(!value)throw new InvalidOperationException("Original editor step "+step+": "+text);++checks; }
        private static double Volume()
        {
            var tree=(CSGTree)new CSGTreeNode { nodeID=model.modelNodeID };
            var descriptions=tree.GetMeshDescriptions(new[] {new MeshQuery(LayerUsageFlags.Renderable,vertexChannels:VertexChannelFlags.All)},VertexChannelFlags.All);
            if(descriptions==null)return 0;
            double volume=0;
            foreach(var description in descriptions)
            {
                var data=tree.GetGeneratedMesh(description); Require(data!=null,"Original GetGeneratedMesh failed");
                for(int i=0;i<data.indices.Length;i+=3)
                {
                    Vector3 a=data.positions[data.indices[i]],b=data.positions[data.indices[i+1]],c=data.positions[data.indices[i+2]];
                    volume+=a.x*((double)b.y*c.z-(double)b.z*c.y)+a.y*((double)b.z*c.x-(double)b.x*c.z)+a.z*((double)b.x*c.y-(double)b.y*c.x);
                }
            }
            return volume/6;
        }
        private static void UpdateModel()
        {
            InternalCSGModelManager.CheckForChanges(forceHierarchyUpdate:true);
            InternalCSGModelManager.UpdateMeshes(forceUpdate:true);
        }
        private static void CountErrors(string message,string stack,LogType type)
        { if(type==LogType.Error||type==LogType.Exception||type==LogType.Assert)++editorErrors; }
        private static void CheckMaterialsAndPreview()
        {
            foreach(string name in new[]{"Wall","Floor","Window","Metal"})
            {
                var material=MaterialUtility.GetRuntimeMaterial(name);
                Require(material&&material.shader&&material.shader.name=="Standard",name+" has missing/wrong built-in shader");
                Require(name=="Metal"?material.GetTexture("_MetallicGlossMap"):material.mainTexture,name+" lost its texture");
                Require(material.renderQueue<2500&&material.GetFloat("_ZWrite")==1,name+" became transparent");
                Require(MaterialUtility.GetRuntimeMaterial(name)==material,"Bundled material getter changed identity");
            }
            var preview=new GameObject("Temporary Generate group");preview.transform.SetParent(model.transform,false);
            var group=preview.AddComponent<RealtimeCSG.Components.CSGOperation>();
            for(int op=0;op<3;++op)
            {
                group.OperationType=(CSGOperationType)op;UpdateModel();
                Require(Math.Abs(Volume()-(op==2?0:10))<1e-4,"Empty group operation "+op+" failed");
            }
            group.OperationType=CSGOperationType.Additive;
            cutter.transform.SetParent(preview.transform,false);UpdateModel();
            Require(Math.Abs(Volume()-12)<1e-4,"Group containing only cutter must be empty");
            cutter.transform.SetParent(model.transform,false);UnityEngine.Object.DestroyImmediate(preview);UpdateModel();
            Require(Math.Abs(Volume()-10)<1e-4,"Preview removal changed final solid");

            // Supply the two base corners and extrusion that the mouse normally
            // supplies, then use the actual original Box generation pipeline.
            Selection.activeGameObject=solid.gameObject;
            var generator=ScriptableObject.CreateInstance<GeneratorBox>();generator.Init();
            var settings=(GeneratorBoxSettings)typeof(GeneratorBox).GetField("settings",BindingFlags.Instance|BindingFlags.NonPublic).GetValue(generator);
            settings.AddPoint(new Vector3(10,0,0));settings.AddPoint(new Vector3(12,0,2));
            var generatorType=typeof(GeneratorBase);
            var editMode=generatorType.GetField("editMode",BindingFlags.Instance|BindingFlags.NonPublic);
            editMode.SetValue(generator,Enum.Parse(editMode.FieldType,"ExtrudeShape"));
            generatorType.GetField("gridTangent",BindingFlags.Instance|BindingFlags.NonPublic).SetValue(generator,Vector3.right);
            generatorType.GetField("gridBinormal",BindingFlags.Instance|BindingFlags.NonPublic).SetValue(generator,Vector3.forward);
            generatorType.GetField("brushPosition",BindingFlags.Instance|BindingFlags.NonPublic).SetValue(generator,new Vector3(11,0,1));
            typeof(GeneratorExtrudedBase).GetField("extrusionPoints",BindingFlags.Instance|BindingFlags.NonPublic).SetValue(generator,
                new[]{new ExtrusionPoint{Position=new Vector3(11,0,1)},new ExtrusionPoint{Position=new Vector3(11,2,1)}});
            generator.CurrentCSGOperationType=CSGOperationType.Additive;
            Require(generator.UpdateBaseShape(),"Original Box Generate failed");UpdateModel();
            Require(Math.Abs(Volume()-18)<1e-4,"Original Box preview volume differs: "+Volume());
            Undo.IncrementCurrentGroup();generator.Height=3;UpdateModel();
            Require(Math.Abs(Volume()-22)<1e-4,"Original Box resize failed");
            Undo.FlushUndoRecordObjects();
            Undo.UndoRedoCallback reconcile=()=>generator.UndoRedoPerformed();
            Undo.undoRedoPerformed+=reconcile;
            try
            {
                Undo.PerformUndo();UpdateModel();
                Require(Math.Abs(generator.Height-2)<1e-4&&Math.Abs(Volume()-18)<1e-4,"Generator Undo failed");
                Undo.PerformRedo();UpdateModel();
                Require(Math.Abs(generator.Height-3)<1e-4&&Math.Abs(Volume()-22)<1e-4,"Generator Redo failed: height "+generator.Height+" volume "+Volume());
            }
            finally{Undo.undoRedoPerformed-=reconcile;}
            generator.Cancel();UnityEngine.Object.DestroyImmediate(generator);UpdateModel();
            Require(Math.Abs(Volume()-10)<1e-4,"Original Generate Cancel failed");
            Require(editorErrors==0,"Original editor logged "+editorErrors+" errors during lifecycle");
        }
        private static void CheckSphereAndPlace()
        {
            var place=ScriptableObject.CreateInstance<EditModePlace>();
            var selection=new FilteredSelection();
            selection.UpdateSelection(new HashSet<CSGNode>{solid,cutter},new HashSet<Transform>());place.SetTargets(selection);
            var hover=typeof(EditModePlace).GetField("hoverOnTarget",BindingFlags.Instance|BindingFlags.NonPublic);
            hover.SetValue(place,1);
            selection.UpdateSelection(new HashSet<CSGNode>{solid},new HashSet<Transform>());place.SetTargets(selection);
            Require((int)hover.GetValue(place)==-1,"Place retained hover from previous selection");
            hover.SetValue(place,0);place.UndoRedoPerformed();
            Require((int)hover.GetValue(place)==-1,"Place retained hover across Undo");
            UnityEngine.Object.DestroyImmediate(place);

            int splits=CSGSettings.SphereSplits;bool hemi=CSGSettings.HemiSphereMode;
            GeneratorSphere sphere=null;
            try
            {
                foreach(bool hemisphere in new[]{false,true})
                foreach(int subdivision in new[]{1,2,3})
                {
                    CSGSettings.SphereSplits=subdivision;CSGSettings.HemiSphereMode=hemisphere;
                    Selection.activeGameObject=solid.gameObject;
                    sphere=ScriptableObject.CreateInstance<GeneratorSphere>();sphere.Init();
                    var settings=(GeneratorSphereSettings)typeof(GeneratorSphere).GetField("settings",BindingFlags.Instance|BindingFlags.NonPublic).GetValue(sphere);
                    settings.AddPoint(new Vector3(10,0,0));settings.AddPoint(new Vector3(11,0,0));
                    var generatorType=typeof(GeneratorBase);
                    var mode=generatorType.GetField("editMode",BindingFlags.Instance|BindingFlags.NonPublic);
                    mode.SetValue(sphere,Enum.Parse(mode.FieldType,"EditShape"));
                    generatorType.GetField("brushPosition",BindingFlags.Instance|BindingFlags.NonPublic).SetValue(sphere,new Vector3(10,0,0));
                    generatorType.GetField("gridTangent",BindingFlags.Instance|BindingFlags.NonPublic).SetValue(sphere,Vector3.right);
                    generatorType.GetField("gridBinormal",BindingFlags.Instance|BindingFlags.NonPublic).SetValue(sphere,Vector3.forward);
                    sphere.CurrentCSGOperationType=CSGOperationType.Additive;
                    Require(sphere.UpdateBaseShape(),"Sphere generator failed");UpdateModel();
                    double added=Volume()-10;
                    Require(added>(hemisphere?1:2)&&added<(hemisphere?2.2:4.3),"Sphere generated invalid volume "+added+" splits "+subdivision+" hemi "+hemisphere);
                    sphere.SphereRadius=.5f;UpdateModel();
                    Require(Math.Abs((Volume()-10)*8-added)<1e-4,"Sphere resize changed topology/volume");
                    sphere.Cancel();UnityEngine.Object.DestroyImmediate(sphere);sphere=null;UpdateModel();
                    Require(Math.Abs(Volume()-10)<1e-4,"Sphere Cancel left geometry");
                }
            }
            finally
            {
                if(sphere){sphere.Cancel();UnityEngine.Object.DestroyImmediate(sphere);}
                CSGSettings.SphereSplits=splits;CSGSettings.HemiSphereMode=hemi;
            }
            Require(editorErrors==0,"Sphere/Place logged "+editorErrors+" errors");
        }
        private static void CheckCylinder()
        {
            int sides=CSGSettings.CircleSides;float offset=CSGSettings.CircleOffset;
            bool ends=CSGSettings.CircleSingleSurfaceEnds,recenter=CSGSettings.CircleRecenter,distance=CSGSettings.CircleDistanceToSide;
            GeneratorCylinder cylinder=null;
            try
            {
                CSGSettings.CircleSides=16;CSGSettings.CircleOffset=0;
                CSGSettings.CircleRecenter=false;CSGSettings.CircleDistanceToSide=false;
                foreach(bool singleCap in new[]{true,false})
                {
                    CSGSettings.CircleSingleSurfaceEnds=singleCap;Selection.activeGameObject=solid.gameObject;
                    cylinder=ScriptableObject.CreateInstance<GeneratorCylinder>();cylinder.Init();
                    var settings=(GeneratorCylinderSettings)typeof(GeneratorCylinder).GetField("settings",BindingFlags.Instance|BindingFlags.NonPublic).GetValue(cylinder);
                    settings.AddPoint(new Vector3(10,0,0));settings.AddPoint(new Vector3(11,0,0));
                    var generatorType=typeof(GeneratorBase);var flags=BindingFlags.Instance|BindingFlags.NonPublic;
                    var mode=generatorType.GetField("editMode",flags);mode.SetValue(cylinder,Enum.Parse(mode.FieldType,"ExtrudeShape"));
                    generatorType.GetField("brushPosition",flags).SetValue(cylinder,new Vector3(10,0,0));
                    generatorType.GetField("gridTangent",flags).SetValue(cylinder,Vector3.right);
                    generatorType.GetField("gridBinormal",flags).SetValue(cylinder,Vector3.forward);
                    typeof(GeneratorExtrudedBase).GetField("extrusionPoints",flags).SetValue(cylinder,
                        new[]{new ExtrusionPoint{Position=new Vector3(10,0,0)},new ExtrusionPoint{Position=new Vector3(10,2,0)}});
                    cylinder.CurrentCSGOperationType=CSGOperationType.Additive;
                    Require(cylinder.UpdateBaseShape(),"Cylinder Generate failed");UpdateModel();
                    var brushes=(CSGBrush[])generatorType.GetField("generatedBrushes",flags).GetValue(cylinder);
                    Require(brushes.Length==1,"Convex cylinder became multiple triangular brushes");
                    var mesh=brushes[0].ControlMesh;
                    Require(mesh.Vertices.Length==(singleCap?32:34),"Cylinder cap topology differs");
                    if(singleCap)
                    {
                        int caps=0;
                        foreach(var polygon in mesh.Polygons)
                        {
                            if(polygon.EdgeIndices.Length!=16)continue;
                            Vector3 center=Vector3.zero;
                            foreach(int edge in polygon.EdgeIndices)center+=mesh.Vertices[mesh.Edges[edge].VertexIndex];
                            center/=16;
                            Require(Math.Abs(center.x)<1e-4&&Math.Abs(center.z)<1e-4,"Cylinder cap handle is off center");++caps;
                        }
                        Require(caps==2,"Cylinder lost its two whole caps");
                    }
                    else
                    {
                        for(int i=32;i<34;++i)
                            Require(Math.Abs(mesh.Vertices[i].x)<1e-4&&Math.Abs(mesh.Vertices[i].z)<1e-4,"Cylinder fan vertex is off center");
                    }
                    double expected=16*Math.Sin(2*Math.PI/16); // regular 16-gon, radius 1, height 2
                    Require(Math.Abs(Volume()-10-expected)<1e-4,"Cylinder volume differs");
                    cylinder.Cancel();UnityEngine.Object.DestroyImmediate(cylinder);cylinder=null;UpdateModel();
                    Require(Math.Abs(Volume()-10)<1e-4,"Cylinder Cancel left geometry");
                }
            }
            finally
            {
                if(cylinder){cylinder.Cancel();UnityEngine.Object.DestroyImmediate(cylinder);}
                CSGSettings.CircleSides=sides;CSGSettings.CircleOffset=offset;CSGSettings.CircleSingleSurfaceEnds=ends;
                CSGSettings.CircleRecenter=recenter;CSGSettings.CircleDistanceToSide=distance;
            }
            Require(editorErrors==0,"Cylinder logged "+editorErrors+" errors");
        }
        public static void BatchSmoke()
        {
            try
            {
                EditorSceneManager.NewScene(NewSceneSetup.EmptyScene,NewSceneMode.Single);
                NativeMethodBindings.RegisterUnityMethods();NativeMethodBindings.RegisterExternalMethods();
                var root=new GameObject("Open RealtimeCSG Model"); model=root.AddComponent<CSGModel>();
                solid=BrushFactory.CreateCubeBrush(root.transform,"Solid",new Vector3(2,2,2));
                cutter=BrushFactory.CreateCubeBrush(root.transform,"Cutter",new Vector3(1,1,4));
                cutter.OperationType=CSGOperationType.Subtractive;
                Selection.activeGameObject=solid.gameObject;
                OpenKernelEntry.OpenEditor();
                // Null-graphics Unity reports GUI window creation errors; count
                // errors in the model/generator lifecycle after opening tools.
                editorErrors=0;Application.logMessageReceived+=CountErrors;
                ready=EditorApplication.timeSinceStartup+.5;deadline=ready+60;step=0;
                EditorApplication.update+=Tick;
            }
            catch(Exception error){Fail(error);}
        }
        private static void Tick()
        {
            try
            {
                if(EditorApplication.timeSinceStartup>deadline)throw new TimeoutException("Original editor timed out at "+step);
                if(EditorApplication.timeSinceStartup<ready)return;
                UpdateModel();
                Require(solid.brushNodeID!=0&&cutter.brushNodeID!=0&&model.modelNodeID!=0,"Original components did not register");
                if(step==0)
                {
                    double volume=Volume();Require(Math.Abs(volume-6)<1e-4,"Initial subtract volume "+volume);
                    Require(model.GetComponentsInChildren<MeshFilter>().Length>0,"Original editor did not create rendered meshes");
                    GeometryWireframe outline;
                    Require(InternalCSGModelManager.External.GetBrushOutline(solid.brushNodeID,out outline)&&outline.vertices.Length==8,"Original outline query failed");
                    var external=InternalCSGModelManager.External;
                    LegacyBrushIntersection picked;
                    Require(external.RayCastIntoBrush(solid.brushNodeID,new Vector3(-3,0,0),new Vector3(3,0,0),Matrix4x4.identity,out picked,true),"Original brush picking failed");
                    LegacyBrushIntersection[] hits;
                    Require(external.RayCastIntoModelMulti(model,new Vector3(-3,0,0),new Vector3(3,0,0),true,out hits)&&hits.Length>0,"Visible CSG picking failed");
                    Vector2 low,high,uv;
                    Matrix4x4 world=Matrix4x4.Translate(new Vector3(2,3,4));
                    Require(external.GetSurfaceMinMaxTexCoords(solid.brushNodeID,picked.surfaceIndex,world,out low,out high),"Surface bounds ABI failed");
                    Require(external.ConvertModelToTextureSpace(solid.brushNodeID,picked.surfaceIndex,world,world.MultiplyPoint3x4(picked.worldIntersection),out uv),"UV matrix-by-value ABI failed");
                    var surface=solid.Shape.Surfaces[picked.surfaceIndex];
                    Vector3 expectedUV=SurfaceUtility.GetLocalToTextureSpaceMatrix(solid.Shape.TexGens[surface.TexGenIndex],surface).MultiplyPoint3x4(picked.worldIntersection);
                    Require(Vector2.Distance(uv,new Vector2(expectedUV.x,expectedUV.y))<1e-4f,"Native UV differs from original C# projection");
                    Undo.IncrementCurrentGroup();Undo.RecordObject(cutter.transform,"Move original cutter");
                    cutter.transform.localPosition=new Vector3(3,0,0);Undo.FlushUndoRecordObjects();
                    ++step;ready=EditorApplication.timeSinceStartup+.3;return;
                }
                if(step==1)
                {
                    double volume=Volume();Require(Math.Abs(volume-8)<1e-4,"Moved cutter volume "+volume);
                    Undo.PerformUndo();++step;ready=EditorApplication.timeSinceStartup+.3;return;
                }
                if(step==2)
                {
                    double volume=Volume();Require(Math.Abs(volume-6)<1e-4,"Undo volume "+volume);
                    // Editing source vertices uses the original ControlMesh and planes.
                    Undo.RecordObject(solid,"Edit original brush vertices");
                    for(int i=0;i<solid.ControlMesh.Vertices.Length;++i)solid.ControlMesh.Vertices[i].x*=1.5f;
                    ControlMeshUtility.CalculatePlanes(solid.ControlMesh,solid.Shape);
                    solid.ControlMesh.SetDirty();
                    ++step;ready=EditorApplication.timeSinceStartup+.3;return;
                }
                double finalVolume=Volume();Require(Math.Abs(finalVolume-10)<1e-4,"Vertex edit volume "+finalVolume);
                CheckMaterialsAndPreview();
                CheckSphereAndPlace();
                CheckCylinder();
                var camera=new GameObject("Preview Camera",typeof(Camera));camera.transform.position=new Vector3(4,3,-5);camera.transform.LookAt(Vector3.zero);
                var light=new GameObject("Preview Light",typeof(Light));light.GetComponent<Light>().type=LightType.Directional;light.transform.rotation=Quaternion.Euler(45,-30,0);
                string output=AssetDatabase.GenerateUniqueAssetPath("Assets/OpenEditorOutput");Directory.CreateDirectory(output);AssetDatabase.Refresh();
                Require(EditorSceneManager.SaveScene(model.gameObject.scene,output+"/OriginalEditor.unity"),"Save scene failed");
                File.WriteAllText("OriginalEditorScene.txt",output+"/OriginalEditor.unity");
                ExportPackage();
                string report="{\"unity\":\""+Application.unityVersion+"\",\"checks\":"+checks+
                    ",\"volume\":"+finalVolume.ToString("R",System.Globalization.CultureInfo.InvariantCulture)+",\"scene\":\""+output+"/OriginalEditor.unity\"}";
                File.WriteAllText("OriginalEditorReport.json",report);Debug.Log("OPENCSG_ORIGINAL_EDITOR_PASS "+report);
                EditorApplication.update-=Tick;EditorApplication.Exit(0);
            }
            catch(Exception error){Fail(error);}
        }
        private static void ExportPackage()
        {
            foreach(string name in new[] {"libRealtimeCSG[1_559].so","librealtimecsg_recovered_geometry.so"})
            {
                var importer=(PluginImporter)AssetImporter.GetAtPath("Assets/RealtimeCSG/Plugins/Editor/External/"+name);
                importer.SetCompatibleWithAnyPlatform(false);importer.SetCompatibleWithEditor(true);
                importer.SetEditorData("OS","Linux");importer.SetEditorData("CPU","x86_64");
                importer.SetCompatibleWithPlatform(BuildTarget.StandaloneWindows,false);importer.SetCompatibleWithPlatform(BuildTarget.StandaloneWindows64,false);
                importer.SetCompatibleWithPlatform(BuildTarget.StandaloneOSX,false);importer.SetCompatibleWithPlatform(BuildTarget.StandaloneLinux64,false);
                importer.SaveAndReimport();
            }
            var assets=new List<string>();
            foreach(string path in AssetDatabase.GetAllAssetPaths())
                if(path.StartsWith("Assets/RealtimeCSG/",StringComparison.Ordinal)&&!AssetDatabase.IsValidFolder(path)&&!path.EndsWith("/OriginalEditorSmoke.cs",StringComparison.Ordinal))assets.Add(path);
            AssetDatabase.ExportPackage(assets.ToArray(),"OpenCSG-OriginalEditor.unitypackage",ExportPackageOptions.Default);
        }
        public static void ReloadSmoke()
        {
            try
            {
                EditorSceneManager.OpenScene(File.ReadAllText("OriginalEditorScene.txt").Trim());
                model=UnityEngine.Object.FindObjectOfType<CSGModel>();
                NativeMethodBindings.RegisterUnityMethods();NativeMethodBindings.RegisterExternalMethods();UpdateModel();
                Require(model&&model.GetComponentsInChildren<CSGBrush>().Length==2,"Original scene lost brush sources");
                Require(Math.Abs(Volume()-10)<1e-4,"Original scene did not restore geometry");
                File.WriteAllText("OriginalEditorReloadReport.json","{\"reopened\":true,\"volume\":10}");
                Debug.Log("OPENCSG_ORIGINAL_EDITOR_RELOAD_PASS");EditorApplication.Exit(0);
            }
            catch(Exception error){Fail(error);}
        }
        public static void RenderSmoke()
        {
            try
            {
                EditorSceneManager.OpenScene(File.ReadAllText("OriginalEditorScene.txt").Trim());
                model=UnityEngine.Object.FindObjectOfType<CSGModel>();
                NativeMethodBindings.RegisterUnityMethods();NativeMethodBindings.RegisterExternalMethods();UpdateModel();
                Require(SystemInfo.graphicsDeviceType!=UnityEngine.Rendering.GraphicsDeviceType.Null,"Render test needs graphics");
                var camera=GameObject.Find("Preview Camera").GetComponent<Camera>();
                camera.clearFlags=CameraClearFlags.SolidColor;camera.backgroundColor=new Color(.1f,.2f,.3f,1);
                var target=new RenderTexture(640,480,24);camera.targetTexture=target;camera.Render();
                RenderTexture.active=target;var pixels=new Texture2D(640,480,TextureFormat.RGB24,false);
                pixels.ReadPixels(new Rect(0,0,640,480),0,0);pixels.Apply();
                int magenta=0,foreground=0;
                var renderedPixels=pixels.GetPixels32();var background=renderedPixels[0];
                foreach(var pixel in renderedPixels)
                {
                    if(pixel.r>180&&pixel.b>180&&pixel.g<80)++magenta;
                    if(Math.Abs(pixel.r-background.r)>10||Math.Abs(pixel.g-background.g)>10||Math.Abs(pixel.b-background.b)>10)++foreground;
                }
                File.WriteAllBytes("OriginalEditorRender.png",pixels.EncodeToPNG());
                Require(magenta==0,"Rendered mesh has "+magenta+" magenta error-shader pixels");
                Require(foreground>1000,"Rendered mesh is missing/transparent");
                File.WriteAllText("OriginalEditorRenderReport.json","{\"magentaPixels\":"+magenta+",\"foregroundPixels\":"+foreground+"}");
                camera.targetTexture=null;RenderTexture.active=null;target.Release();
                Debug.Log("OPENCSG_ORIGINAL_EDITOR_RENDER_PASS");EditorApplication.Exit(0);
            }
            catch(Exception error){Fail(error);}
        }
        private static void Fail(Exception error){EditorApplication.update-=Tick;Debug.LogException(error);EditorApplication.Exit(1);}
    }
}
#endif
