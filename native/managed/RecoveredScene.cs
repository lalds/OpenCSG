using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;

namespace RealtimeCSG.Recovery
{
    // Research bridge v1. This does not replace the original Foundation API.
    [StructLayout(LayoutKind.Sequential, Pack = 4)]
    public struct Float2 { public float x, y; }
    [StructLayout(LayoutKind.Sequential, Pack = 4)]
    public struct Float3
    {
        public float x, y, z;
        public Float3(float x, float y, float z) { this.x = x; this.y = y; this.z = z; }
    }
    [StructLayout(LayoutKind.Sequential, Pack = 4)]
    public struct Float4
    {
        public float x, y, z, w;
        public Float4(float x, float y, float z, float w) { this.x = x; this.y = y; this.z = z; this.w = w; }
    }
    [StructLayout(LayoutKind.Sequential, Pack = 4)]
    public struct Plane { public Float3 normal; public float distance; }
    [StructLayout(LayoutKind.Sequential, Pack = 4)]
    public struct Surface { public Plane plane; public Float4 uvU, uvV; public int materialId; }
    [StructLayout(LayoutKind.Sequential, Pack = 4)]
    public struct HalfEdge { public int vertexIndex, twinIndex; }
    [StructLayout(LayoutKind.Sequential, Pack = 4)]
    public struct Polygon { public int firstEdge, edgeCount, surfaceIndex; }
    [StructLayout(LayoutKind.Sequential, Pack = 4)]
    public struct Node
    {
        public int firstChild, childCount;
        public byte operation, isBrush, infinite, reserved;
    }
    [StructLayout(LayoutKind.Sequential, Pack = 4)]
    public struct Vertex { public Float3 position, normal; public Float4 tangent; public Float2 uv; }
    [StructLayout(LayoutKind.Sequential, Pack = 4)]
    public struct Submesh { public int materialId, firstIndex, indexCount; }
    [StructLayout(LayoutKind.Sequential, Pack = 4)]
    public struct TriangleOrigin { public int brushIndex, surfaceIndex, materialId; public ushort state, flags; }
    [StructLayout(LayoutKind.Sequential, Pack = 4)]
    public struct Counts
    {
        public int vertexCount, indexCount, submeshCount, triangleCount, brushCount;
        public Float3 boundsCenter, boundsSize;
    }
    public sealed class Brush
    {
        public int nodeIndex, brushIndex;
        public Float3[] vertices;
        public HalfEdge[] edges;
        public Polygon[] polygons;
        public Surface[] surfaces;
    }
    public sealed class Contents
    {
        public Counts counts;
        public Vertex[] vertices;
        public int[] indices;
        public Submesh[] submeshes;
        public TriangleOrigin[] triangles;
    }
    [StructLayout(LayoutKind.Sequential, Pack = 4)]
    internal struct NativeBrush
    {
        public int nodeIndex, brushIndex;
        public IntPtr vertices; public int vertexCount;
        public IntPtr edges; public int edgeCount;
        public IntPtr polygons; public int polygonCount;
        public IntPtr surfaces; public int surfaceCount;
    }
    [StructLayout(LayoutKind.Sequential, Pack = 4)]
    internal struct BuildInput
    {
        public uint version;
        public int rootNode;
        public IntPtr nodes; public int nodeCount;
        public IntPtr children; public int childCount;
        public IntPtr brushes; public int brushCount;
        public int maxGapSplits, maxVertices, maxEdges, maxPolygons;
    }
    [StructLayout(LayoutKind.Sequential, Pack = 4)]
    internal struct OutputBuffers
    {
        public IntPtr vertices; public int vertexCapacity;
        public IntPtr indices; public int indexCapacity;
        public IntPtr submeshes; public int submeshCapacity;
        public IntPtr triangles; public int triangleCapacity;
    }
    internal static class Native
    {
        private const string Library = "realtimecsg_recovered_geometry";
        [DllImport(Library, CallingConvention = CallingConvention.Cdecl)] internal static extern uint rcsg_recovered_scene_abi_version();
        [DllImport(Library, CallingConvention = CallingConvention.Cdecl)] internal static extern int rcsg_recovered_scene_build(ref BuildInput input, out ulong handle);
        [DllImport(Library, CallingConvention = CallingConvention.Cdecl)] internal static extern int rcsg_recovered_scene_counts(ulong handle, out Counts counts);
        [DllImport(Library, CallingConvention = CallingConvention.Cdecl)] internal static extern int rcsg_recovered_scene_copy(ulong handle, ref OutputBuffers output);
        [DllImport(Library, CallingConvention = CallingConvention.Cdecl)] internal static extern int rcsg_recovered_scene_release(ulong handle);
        [DllImport(Library, CallingConvention = CallingConvention.Cdecl)] private static extern IntPtr rcsg_recovered_scene_last_error();
        internal static void Check(int result)
        {
            if (result != 0) throw new InvalidOperationException("Recovery native status " + result + ": " + Marshal.PtrToStringAnsi(rcsg_recovered_scene_last_error()));
        }
    }
    internal sealed class Pins : IDisposable
    {
        private readonly List<GCHandle> handles = new List<GCHandle>();
        internal IntPtr Pin(Array array)
        {
            if (array == null) throw new ArgumentNullException("array");
            if (array.Length == 0) return IntPtr.Zero;
            GCHandle handle = GCHandle.Alloc(array, GCHandleType.Pinned);
            try { handles.Add(handle); } catch { handle.Free(); throw; }
            return handle.AddrOfPinnedObject();
        }
        public void Dispose() { for (int i = handles.Count - 1; i >= 0; --i) handles[i].Free(); handles.Clear(); }
    }
    public sealed class Scene : IDisposable
    {
        private readonly object gate = new object();
        private ulong handle;
        private Scene(ulong handle) { this.handle = handle; }
        public static void ValidateLayout()
        {
            if (IntPtr.Size != 8) throw new PlatformNotSupportedException("Recovery bridge currently targets 64-bit processes");
            Type[] types = { typeof(Float3), typeof(Node), typeof(Surface), typeof(NativeBrush), typeof(BuildInput), typeof(Vertex), typeof(Counts), typeof(OutputBuffers), typeof(TriangleOrigin), typeof(Submesh) };
            int[] sizes = { 12, 12, 52, 56, 60, 48, 44, 48, 16, 12 };
            for (int i = 0; i < types.Length; ++i)
                if (Marshal.SizeOf(types[i]) != sizes[i]) throw new InvalidOperationException("Recovery layout mismatch: " + types[i].Name);
            if (Marshal.OffsetOf(typeof(NativeBrush), "surfaces").ToInt32() != 44 ||
                Marshal.OffsetOf(typeof(Vertex), "uv").ToInt32() != 40 ||
                Marshal.OffsetOf(typeof(BuildInput), "brushes").ToInt32() != 32)
                throw new InvalidOperationException("Recovery field offset mismatch");
            if (Native.rcsg_recovered_scene_abi_version() != 1) throw new InvalidOperationException("Unsupported recovery ABI version");
        }
        public static Scene Build(Node[] nodes, int[] children, Brush[] brushes, int rootNode = 0,
            int maxGapSplits = 128, int maxVertices = 2048, int maxEdges = 4096, int maxPolygons = 512)
        {
            ValidateLayout();
            if (nodes == null || children == null || brushes == null) throw new ArgumentNullException("Scene input arrays");
            using (var pins = new Pins())
            {
                var raw = new NativeBrush[brushes.Length];
                for (int b = 0; b < brushes.Length; ++b)
                {
                    var brush = brushes[b];
                    if (brush == null) throw new ArgumentNullException("brushes");
                    raw[b] = new NativeBrush {
                        nodeIndex = brush.nodeIndex, brushIndex = brush.brushIndex,
                        vertices = pins.Pin(brush.vertices), vertexCount = brush.vertices.Length,
                        edges = pins.Pin(brush.edges), edgeCount = brush.edges.Length,
                        polygons = pins.Pin(brush.polygons), polygonCount = brush.polygons.Length,
                        surfaces = pins.Pin(brush.surfaces), surfaceCount = brush.surfaces.Length
                    };
                }
                var input = new BuildInput {
                    version = 1, rootNode = rootNode, nodes = pins.Pin(nodes), nodeCount = nodes.Length,
                    children = pins.Pin(children), childCount = children.Length, brushes = pins.Pin(raw), brushCount = raw.Length,
                    maxGapSplits = maxGapSplits, maxVertices = maxVertices, maxEdges = maxEdges, maxPolygons = maxPolygons
                };
                ulong token;
                Native.Check(Native.rcsg_recovered_scene_build(ref input, out token));
                try { return new Scene(token); } catch { Native.rcsg_recovered_scene_release(token); throw; }
            }
        }
        public Contents Read()
        {
            lock (gate)
            {
                if (handle == 0) throw new ObjectDisposedException("Scene");
                Counts counts; Native.Check(Native.rcsg_recovered_scene_counts(handle, out counts));
                var result = new Contents {
                    counts = counts, vertices = new Vertex[counts.vertexCount], indices = new int[counts.indexCount],
                    submeshes = new Submesh[counts.submeshCount], triangles = new TriangleOrigin[counts.triangleCount]
                };
                using (var pins = new Pins())
                {
                    var buffers = new OutputBuffers {
                        vertices = pins.Pin(result.vertices), vertexCapacity = result.vertices.Length,
                        indices = pins.Pin(result.indices), indexCapacity = result.indices.Length,
                        submeshes = pins.Pin(result.submeshes), submeshCapacity = result.submeshes.Length,
                        triangles = pins.Pin(result.triangles), triangleCapacity = result.triangles.Length
                    };
                    Native.Check(Native.rcsg_recovered_scene_copy(handle, ref buffers));
                }
                return result;
            }
        }
        public void Dispose() { Release(); GC.SuppressFinalize(this); }
        private void Release()
        {
            lock (gate) { if (handle != 0) { Native.rcsg_recovered_scene_release(handle); handle = 0; } }
        }
        ~Scene() { try { Release(); } catch { } }
    }
}
