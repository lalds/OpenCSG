#if UNITY_EDITOR
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using RealtimeCSG.Components;
using RealtimeCSG.Foundation;
using RealtimeCSG.Legacy;

namespace RealtimeCSG
{
    public static class OpenKernelEntry
    {
        [MenuItem("Tools/OpenCSG/Open CSG editor")]
        public static void OpenEditor()
        {
            CSGSettings.SetRealtimeCSGEnabled(true);
            if(EditModeManager.EditMode!=ToolEditMode.Edit)EditModeManager.EditMode=ToolEditMode.Edit;
            EditModeToolWindowSceneGUI.GetWindow();
            SceneView.RepaintAll();
        }

        [MenuItem("Tools/OpenCSG/Create demo model")]
        public static void CreateDemo()
        {
            Undo.IncrementCurrentGroup();int group=Undo.GetCurrentGroup();Undo.SetCurrentGroupName("Create OpenCSG demo");
            var root=new GameObject("OpenCSG Model");Undo.RegisterCreatedObjectUndo(root,"Create OpenCSG demo");
            root.AddComponent<CSGModel>();
            CSGBrush solid=BrushFactory.CreateCubeBrush(root.transform,"Solid",new Vector3(2,2,2));
            CSGBrush cutter=BrushFactory.CreateCubeBrush(root.transform,"Cutter",new Vector3(1,1,4));
            cutter.OperationType=CSGOperationType.Subtractive;
            EditorUtility.SetDirty(cutter);
            Selection.activeGameObject=solid.gameObject;
            InternalCSGModelManager.CheckForChanges(forceHierarchyUpdate:true);
            InternalCSGModelManager.UpdateMeshes(forceUpdate:true);
            EditorSceneManager.MarkSceneDirty(root.scene);
            Undo.CollapseUndoOperations(group);
            OpenEditor();
            if(SceneView.lastActiveSceneView)SceneView.lastActiveSceneView.FrameSelected();
        }
    }
}
#endif
