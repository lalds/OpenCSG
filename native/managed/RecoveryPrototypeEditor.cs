#if UNITY_EDITOR
using System;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using RealtimeCSG.Foundation;

namespace RealtimeCSG.Recovery
{
    internal static class RecoveryPrototypeEditorUtility
    {
        private const string MenuRoot = "Tools/RealtimeCSG Recovery/";

        internal static RecoveryModel FindModel(Transform start)
        {
            for (Transform current = start; current != null; current = current.parent)
            {
                RecoveryModel model = current.GetComponent<RecoveryModel>();
                if (model != null) return model;
            }
            return null;
        }

        internal static RecoveryBrush AddBrush(RecoveryModel model, string label, Vector3 size,
            CSGOperationType operation, bool recordUndo = true)
        {
            if (model == null) return null;
            GameObject child = new GameObject(label);
            if (recordUndo) Undo.RegisterCreatedObjectUndo(child, "Add recovery brush");
            child.transform.SetParent(model.transform, false);
            RecoveryBrush brush = child.AddComponent<RecoveryBrush>();
            brush.size = size;
            brush.operation = operation;
            brush.material = AssetDatabase.GetBuiltinExtraResource<Material>("Default-Material.mat");
            return brush;
        }

        internal static RecoveryModel SelectedModel()
        {
            GameObject selected = Selection.activeGameObject;
            return selected == null ? null : FindModel(selected.transform);
        }

        internal static void AddToSelection(string label, Vector3 size, CSGOperationType operation)
        {
            RecoveryModel model = SelectedModel();
            if (model == null)
            {
                EditorUtility.DisplayDialog("Recovery prototype", "Select a RecoveryModel or one of its child brushes.", "OK");
                return;
            }
            RecoveryBrush brush = AddBrush(model, label, size, operation);
            Selection.activeGameObject = brush.gameObject;
            EditorSceneManager.MarkSceneDirty(model.gameObject.scene);
        }

        [MenuItem(MenuRoot + "Create editable CSG prototype")]
        private static void CreatePrototype()
        {
            Undo.IncrementCurrentGroup();
            int group = Undo.GetCurrentGroup();
            Undo.SetCurrentGroupName("Create recovery CSG prototype");

            GameObject root = new GameObject("RecoveryModel");
            Undo.RegisterCreatedObjectUndo(root, "Create recovery CSG prototype");
            RecoveryModel model = root.AddComponent<RecoveryModel>();
            AddBrush(model, "Solid", new Vector3(2, 2, 2), CSGOperationType.Additive);
            AddBrush(model, "Cutter", new Vector3(1, 1, 4), CSGOperationType.Subtractive);
            Selection.activeGameObject = root;
            model.Rebuild();
            SceneView view = SceneView.lastActiveSceneView;
            if (view != null) view.FrameSelected();
            EditorSceneManager.MarkSceneDirty(root.scene);
            Undo.CollapseUndoOperations(group);
        }

        [MenuItem(MenuRoot + "Add additive brush")]
        private static void AddAdditiveBrush() { AddToSelection("Solid", Vector3.one * 2, CSGOperationType.Additive); }

        [MenuItem(MenuRoot + "Add subtractive brush")]
        private static void AddSubtractiveBrush() { AddToSelection("Cutter", new Vector3(1, 1, 4), CSGOperationType.Subtractive); }

        [MenuItem(MenuRoot + "Add intersecting brush")]
        private static void AddIntersectingBrush() { AddToSelection("Intersecting brush", Vector3.one * 2, CSGOperationType.Intersecting); }

        [MenuItem(MenuRoot + "Add additive brush", true)]
        [MenuItem(MenuRoot + "Add subtractive brush", true)]
        [MenuItem(MenuRoot + "Add intersecting brush", true)]
        private static bool ValidateAddBrush() { return SelectedModel() != null; }

        internal static void ExportMesh(RecoveryModel model)
        {
            if (model == null || model.GeneratedMesh == null)
            {
                EditorUtility.DisplayDialog("Export recovery mesh", "Rebuild the model before exporting its mesh.", "OK");
                return;
            }
            string path = EditorUtility.SaveFilePanelInProject("Export recovery mesh", model.name + " Mesh", "asset", "Choose a project asset path.");
            if (string.IsNullOrEmpty(path)) return;

            Mesh copy = UnityEngine.Object.Instantiate(model.GeneratedMesh);
            copy.name = System.IO.Path.GetFileNameWithoutExtension(path);
            copy.hideFlags = HideFlags.None;
            try
            {
                AssetDatabase.CreateAsset(copy, path);
                AssetDatabase.SaveAssets();
                EditorGUIUtility.PingObject(copy);
            }
            catch (Exception exception)
            {
                UnityEngine.Object.DestroyImmediate(copy);
                Debug.LogException(exception);
                EditorUtility.DisplayDialog("Export recovery mesh", exception.Message, "OK");
            }
        }
    }

    [CustomEditor(typeof(RecoveryModel))]
    [CanEditMultipleObjects]
    public sealed class RecoveryModelEditor : Editor
    {
        public override void OnInspectorGUI()
        {
            DrawDefaultInspector();
            EditorGUILayout.HelpBox("Move, rotate and scale child brushes with Unity tools. Brushes are evaluated in hierarchy order.",MessageType.Info);
            EditorGUILayout.Space();
            RecoveryModel model = (RecoveryModel)target;
            using (new EditorGUI.DisabledScope(targets.Length != 1))
            {
                if (GUILayout.Button("Rebuild"))
                {
                    Undo.RecordObject(model, "Rebuild recovery model");
                    model.Rebuild();
                    EditorUtility.SetDirty(model);
                    EditorSceneManager.MarkSceneDirty(model.gameObject.scene);
                }
                using (new EditorGUILayout.HorizontalScope())
                {
                    if (GUILayout.Button("Add solid")) RecoveryPrototypeEditorUtility.AddBrush(model, "Solid", Vector3.one * 2, CSGOperationType.Additive);
                    if (GUILayout.Button("Add cutter")) RecoveryPrototypeEditorUtility.AddBrush(model, "Cutter", new Vector3(1, 1, 4), CSGOperationType.Subtractive);
                }
                if (GUILayout.Button("Export mesh asset")) RecoveryPrototypeEditorUtility.ExportMesh(model);
            }

            if (!string.IsNullOrEmpty(model.LastError))
                EditorGUILayout.HelpBox(model.LastError, MessageType.Error);
            if (model.GeneratedMesh != null)
                EditorGUILayout.LabelField("Generated triangles", (model.GeneratedMesh.triangles.Length / 3).ToString());
            EditorGUILayout.LabelField("Build count", model.BuildCount.ToString());
            EditorGUILayout.LabelField("Status", model.LastStatus);
        }
    }

    [CustomEditor(typeof(RecoveryBrush))]
    [CanEditMultipleObjects]
    public sealed class RecoveryBrushEditor : Editor
    {
        public override void OnInspectorGUI()
        {
            DrawDefaultInspector();
            RecoveryBrush brush = (RecoveryBrush)target;
            RecoveryModel model = RecoveryPrototypeEditorUtility.FindModel(brush.transform);
            using (new EditorGUI.DisabledScope(model == null || targets.Length != 1))
            {
                if (GUILayout.Button("Rebuild parent model"))
                {
                    Undo.RecordObject(model, "Rebuild recovery model");
                    model.Rebuild();
                    EditorUtility.SetDirty(model);
                    EditorSceneManager.MarkSceneDirty(model.gameObject.scene);
                }
            }
        }
    }
}
#endif
