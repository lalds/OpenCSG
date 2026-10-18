#if UNITY_EDITOR
using UnityEngine;
using RealtimeCSG.Foundation;

namespace RealtimeCSG.Recovery
{
    /// <summary>A simple box source for the recovered CSG preview.</summary>
    [ExecuteAlways]
    [DisallowMultipleComponent]
    public sealed class RecoveryBrush : MonoBehaviour
    {
        public Vector3 size = Vector3.one;
        public Foundation.CSGOperationType operation = Foundation.CSGOperationType.Additive;
        public Material material;

        internal BrushMesh CreateBrushMesh(int materialId)
        {
            if (!Finite(size.x) || !Finite(size.y) || !Finite(size.z) || size.x == 0 || size.y == 0 || size.z == 0)
                throw new System.ArgumentException("Brush '" + name + "' needs finite, non-zero dimensions.");
            var layers = new SurfaceLayers {
                layerUsage = LayerUsageFlags.RenderReceiveCastShadows | LayerUsageFlags.Collidable,
                layerParameter1 = materialId
            };
            return BrushMeshUtility.CreateCube(size, layers);
        }

        private static bool Finite(float value) { return !float.IsNaN(value) && !float.IsInfinity(value); }

        private void OnEnable() { RecoveryModel.NotifyHierarchyChanged(); }
        private void OnDisable() { RecoveryModel.NotifyHierarchyChanged(); }
        private void OnValidate() { RecoveryModel.NotifyHierarchyChanged(); }

        private void OnDrawGizmos()
        {
            Gizmos.matrix = transform.localToWorldMatrix;
            Gizmos.color = operation == CSGOperationType.Subtractive ? new Color(1,.45f,.15f,.8f)
                : operation == CSGOperationType.Intersecting ? new Color(.8f,.3f,1,.8f) : new Color(.2f,.8f,1,.65f);
            Gizmos.DrawWireCube(Vector3.zero,size);
        }
    }
}
#endif
