#if UNITY_EDITOR
using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEditor;
using RealtimeCSG.Foundation;

namespace RealtimeCSG.Recovery
{
    /// <summary>Builds a recovered native CSG preview from descendant RecoveryBrush components.</summary>
    [ExecuteAlways]
    [DisallowMultipleComponent]
    public sealed class RecoveryModel : MonoBehaviour
    {
        private const string OutputName = "__Recovered CSG Output";
        [SerializeField, HideInInspector] private GameObject outputObject;
        [SerializeField, HideInInspector] private Mesh outputMesh;
        [SerializeField, HideInInspector] private int buildCount;
        [SerializeField, HideInInspector] private string lastError;
        [SerializeField, HideInInspector] private string lastStatus;
        private ulong lastFingerprint;
        private bool hasFingerprint;
        private bool queued;
        private bool rebuilding;
        private static bool hierarchyChanged;

        public Mesh GeneratedMesh { get { return outputMesh; } }
        public string LastError { get { return lastError ?? string.Empty; } }
        public int BuildCount { get { return buildCount; } }
        public string LastStatus { get { return lastStatus ?? string.Empty; } }

        private sealed class Source
        {
            internal RecoveryBrush component;
            internal int node;
            internal int materialId;
        }

        private static void Mix(ref ulong hash, uint value)
        { hash ^= value; hash *= 1099511628211UL; }
        private static void Mix(ref ulong hash, int value) { Mix(ref hash, unchecked((uint)value)); }
        private static void Mix(ref ulong hash, float value) { Mix(ref hash, unchecked((uint)BitConverter.ToInt32(BitConverter.GetBytes(value), 0))); }
        private static void Mix(ref ulong hash, bool value) { Mix(ref hash, value ? 1 : 0); }
        private static void Mix(ref ulong hash, Matrix4x4 matrix)
        { for (int r = 0; r < 4; ++r) for (int c = 0; c < 4; ++c) Mix(ref hash, matrix[r, c]); }

        private static void Walk(Transform parent, RecoveryModel owner, List<Source> result)
        {
            for (int i = 0; i < parent.childCount; ++i)
            {
                Transform child = parent.GetChild(i);
                if (child.GetComponent<RecoveryModel>() != null || child.gameObject == owner.outputObject) continue;
                RecoveryBrush brush = child.GetComponent<RecoveryBrush>();
                if (brush != null && brush.enabled && child.gameObject.activeInHierarchy)
                    result.Add(new Source { component = brush });
                Walk(child, owner, result);
            }
        }

        private List<Source> Collect()
        {
            var result = new List<Source>();
            Walk(transform, this, result);
            return result;
        }

        private ulong Fingerprint(List<Source> sources)
        {
            ulong hash = 14695981039346656037UL;
            Mix(ref hash, sources.Count);
            Mix(ref hash, transform.localToWorldMatrix);
            for (int i = 0; i < sources.Count; ++i)
            {
                RecoveryBrush brush = sources[i].component;
                Mix(ref hash, brush.GetInstanceID());
                Mix(ref hash, brush.enabled);
                Mix(ref hash, brush.gameObject.activeInHierarchy);
                Mix(ref hash, (int)brush.operation);
                Mix(ref hash, brush.size.x); Mix(ref hash, brush.size.y); Mix(ref hash, brush.size.z);
                Mix(ref hash, brush.transform.localToWorldMatrix);
                Mix(ref hash, brush.material ? brush.material.GetInstanceID() : 0);
            }
            return hash;
        }

        private static void NotifyAll()
        {
            foreach (RecoveryModel model in Resources.FindObjectsOfTypeAll<RecoveryModel>())
                if (model != null && model.gameObject.scene.IsValid()) model.QueueRebuild();
        }

        internal static void NotifyHierarchyChanged()
        {
            hierarchyChanged = true;
        }

        private void OnEnable()
        {
            EditorApplication.update += EditorUpdate;
            Undo.undoRedoPerformed += OnUndoRedo;
            hasFingerprint = false;
            QueueRebuild();
        }

        private void OnDisable()
        {
            EditorApplication.update -= EditorUpdate;
            Undo.undoRedoPerformed -= OnUndoRedo;
            queued = false;
            DestroyOutput();
        }

        private void OnDestroy() { DestroyOutput(); }
        private void OnValidate() { hasFingerprint = false; QueueRebuild(); }
        private void OnTransformChildrenChanged() { hasFingerprint = false; QueueRebuild(); }
        private void OnTransformParentChanged() { hasFingerprint = false; QueueRebuild(); }
        private void OnUndoRedo() { hasFingerprint = false; QueueRebuild(); }

        private void QueueRebuild()
        {
            if (!isActiveAndEnabled || queued) return;
            queued = true;
        }

        private void EditorUpdate()
        {
            if (!this || !isActiveAndEnabled) return;
            if (hierarchyChanged) { hierarchyChanged = false; NotifyAll(); }
            List<Source> sources = Collect();
            ulong fingerprint = Fingerprint(sources);
            if (!hasFingerprint || fingerprint != lastFingerprint) QueueRebuild();
            if (!queued) return;
            queued = false;
            Rebuild();
        }

        public void Rebuild()
        {
            if (rebuilding || !this) return;
            rebuilding = true;
            try
            {
                var sources = Collect();
                ulong fingerprint = Fingerprint(sources);
                if (sources.Count == 0)
                {
                    ReplaceMesh(null, new Material[0]);
                    Complete(fingerprint, "Empty model");
                    return;
                }
                bool additive = false;
                for (int i = 0; i < sources.Count; ++i)
                    if (sources[i].component.operation == Foundation.CSGOperationType.Additive) { additive = true; break; }
                if (!additive) throw new InvalidOperationException("RecoveryModel requires at least one additive brush.");

                var nodes = new Node[sources.Count + 1];
                var children = new int[sources.Count];
                var brushes = new Brush[sources.Count];
                var materialById = new Dictionary<int, Material>();
                nodes[0] = new Node { firstChild = 0, childCount = sources.Count };
                Matrix4x4 worldFromModel = transform.localToWorldMatrix;
                for (int i = 0; i < sources.Count; ++i)
                {
                    RecoveryBrush source = sources[i].component;
                    int nodeIndex = i + 1;
                    int materialId = source.material ? source.material.GetInstanceID() : 0;
                    sources[i].node = nodeIndex;
                    sources[i].materialId = materialId;
                    children[i] = nodeIndex;
                    nodes[nodeIndex] = new Node { operation = (byte)source.operation, isBrush = 1 };
                    var foundation = source.CreateBrushMesh(materialId);
                    brushes[i] = FoundationBridge.Convert(foundation,
                        transform.worldToLocalMatrix * source.transform.localToWorldMatrix,
                        worldFromModel, nodeIndex, i).geometry;
                    if (source.material) materialById[materialId] = source.material;
                }

                Contents contents;
                using (Scene scene = Scene.Build(nodes, children, brushes)) contents = scene.Read();
                Mesh mesh = CreateMesh(contents);
                Material[] materials = new Material[contents.submeshes.Length];
                for (int i = 0; i < materials.Length; ++i)
                {
                    Material material;
                    materialById.TryGetValue(contents.submeshes[i].materialId, out material);
                    materials[i] = material;
                }
                ReplaceMesh(mesh, materials);
                Complete(fingerprint, "Built " + contents.counts.brushCount + " brushes, " + contents.counts.triangleCount + " triangles");
            }
            catch (Exception error)
            {
                lastFingerprint = Fingerprint(Collect());
                hasFingerprint = true;
                lastError = error.Message;
                lastStatus = "Build failed; showing last good output";
                Debug.LogError("RecoveryModel '" + name + "': " + lastError, this);
            }
            finally { rebuilding = false; }
        }

        private void Complete(ulong fingerprint, string status)
        {
            lastFingerprint = fingerprint;
            hasFingerprint = true;
            lastError = string.Empty;
            lastStatus = status;
            ++buildCount;
        }

        private static Mesh CreateMesh(Contents contents)
        {
            var mesh = new Mesh { name = "Recovered native CSG", indexFormat = IndexFormat.UInt32, hideFlags = HideFlags.DontSave };
            try
            {
                var vertices = new Vector3[contents.vertices.Length];
                var normals = new Vector3[vertices.Length];
                var tangents = new Vector4[vertices.Length];
                var uv = new Vector2[vertices.Length];
                for (int i = 0; i < vertices.Length; ++i)
                {
                    Vertex v = contents.vertices[i];
                    vertices[i] = new Vector3(v.position.x, v.position.y, v.position.z);
                    normals[i] = new Vector3(v.normal.x, v.normal.y, v.normal.z);
                    tangents[i] = new Vector4(v.tangent.x, v.tangent.y, v.tangent.z, v.tangent.w);
                    uv[i] = new Vector2(v.uv.x, v.uv.y);
                }
                mesh.vertices = vertices; mesh.normals = normals; mesh.tangents = tangents; mesh.uv = uv;
                mesh.subMeshCount = contents.submeshes.Length;
                for (int i = 0; i < contents.submeshes.Length; ++i)
                {
                    Submesh submesh = contents.submeshes[i];
                    var indices = new int[submesh.indexCount];
                    Array.Copy(contents.indices, submesh.firstIndex, indices, 0, submesh.indexCount);
                    mesh.SetTriangles(indices, i, false);
                }
                Float3 center = contents.counts.boundsCenter, size = contents.counts.boundsSize;
                mesh.bounds = new Bounds(new Vector3(center.x, center.y, center.z), new Vector3(size.x, size.y, size.z));
                return mesh;
            }
            catch { DestroyImmediate(mesh); throw; }
        }

        private void EnsureOutput()
        {
            if (OwnsOutput()) return;
            // Duplicating a model can copy references to the original model's output.
            // A user object with the same name must never be adopted or deleted.
            outputObject = null;
            outputMesh = null;
            if (!outputObject)
            {
                outputObject = new GameObject(OutputName, typeof(MeshFilter), typeof(MeshRenderer));
                outputObject.hideFlags = HideFlags.HideAndDontSave;
                outputObject.transform.SetParent(transform, false);
            }
        }

        private void ReplaceMesh(Mesh mesh, Material[] materials)
        {
            EnsureOutput();
            MeshFilter filter = outputObject.GetComponent<MeshFilter>();
            Mesh old = outputMesh;
            outputMesh = mesh;
            filter.sharedMesh = mesh;
            outputObject.GetComponent<MeshRenderer>().sharedMaterials = materials;
            if (old && old != mesh) DestroyImmediate(old);
        }

        private void DestroyOutput()
        {
            if (OwnsOutput())
            {
                if (outputMesh && !EditorUtility.IsPersistent(outputMesh)) DestroyImmediate(outputMesh);
                DestroyImmediate(outputObject);
            }
            outputMesh = null;
            outputObject = null;
        }

        private bool OwnsOutput()
        {
            return outputObject && outputObject.transform.parent == transform
                && (outputObject.hideFlags & HideFlags.DontSaveInEditor) != 0;
        }

    }
}
#endif
