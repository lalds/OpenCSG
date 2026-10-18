#if !UNITY_5_3_OR_NEWER
using System;
using RealtimeCSG.Recovery;

public static class ManagedSceneSmoke
{
    private static void Require(bool value, string message) { if (!value) throw new Exception(message); }
    public static double Volume(Contents contents)
    {
        double volume = 0;
        for (int i = 0; i < contents.indices.Length; i += 3)
        {
            Float3 a = contents.vertices[contents.indices[i]].position;
            Float3 b = contents.vertices[contents.indices[i+1]].position;
            Float3 c = contents.vertices[contents.indices[i+2]].position;
            volume += a.x*((double)b.y*c.z-(double)b.z*c.y) + a.y*((double)b.z*c.x-(double)b.x*c.z) + a.z*((double)b.x*c.y-(double)b.y*c.x);
        }
        return volume / 6;
    }
    public static int Main()
    {
        try
        {
            Scene.ValidateLayout();
            var a = Primitives.Box(new Float3(-1,-1,-1), new Float3(1,1,1), 1, 11);
            var b = Primitives.Box(new Float3(0,-1,-1), new Float3(2,1,1), 2, 89, 10);
            double[] expected = { 12, 4, 4 };
            for (byte operation = 0; operation < 3; ++operation)
            {
                using (Scene scene = Scene.Build(Primitives.PairTree(operation), new[] { 1, 2 }, new[] { a, b }))
                {
                    Contents mesh = scene.Read();
                    Require(Math.Abs(Volume(mesh)-expected[operation]) < 1e-5, "Managed/native Boolean volume mismatch");
                    Require(mesh.vertices.Length == mesh.counts.vertexCount && mesh.indices.Length == mesh.counts.indexCount, "Counts/copy mismatch");
                    foreach (Vertex vertex in mesh.vertices)
                        Require(!float.IsNaN(vertex.uv.x) && Math.Abs(vertex.tangent.w) == 1, "Invalid managed attributes");
                    int end = 0;
                    foreach (Submesh part in mesh.submeshes) { Require(part.firstIndex == end && part.indexCount%3 == 0, "Submesh range mismatch"); end += part.indexCount; }
                    Require(end == mesh.indices.Length, "Submesh ranges did not cover indices");
                }
            }
            b = Primitives.Box(new Float3(-.5f,-.5f,-.5f), new Float3(.5f,.5f,.5f), 2, 89, 10);
            using (Scene scene = Scene.Build(Primitives.PairTree(1), new[] {1,2}, new[] {a,b}))
            {
                a.vertices[0] = new Float3(100,100,100);
                Contents mesh = scene.Read();
                Require(Math.Abs(Volume(mesh)-7) < 1e-5, "Snapshot borrowed managed input arrays");
                bool reversed = false;
                foreach (TriangleOrigin origin in mesh.triangles) if (origin.brushIndex == 89) { reversed = true; Require(origin.state == 3, "Cutter provenance/state lost"); }
                Require(reversed, "No cutter surface in cavity");
            }
            a = Primitives.Box(new Float3(-1,-1,-1), new Float3(1,1,1), 1, 11);
            b = Primitives.Box(new Float3(2,2,2), new Float3(3,3,3), 2, 89, 10);
            Scene disposed = Scene.Build(Primitives.PairTree(2), new[] {1,2}, new[] {a,b});
            Require(disposed.Read().indices.Length == 0, "Disjoint intersection was not empty");
            disposed.Dispose(); disposed.Dispose();
            bool rejected = false;
            try { disposed.Read(); } catch (ObjectDisposedException) { rejected = true; }
            Require(rejected, "Disposed scene accepted Read");
            a.edges[0].twinIndex = -1; rejected = false;
            try { using (Scene invalid = Scene.Build(Primitives.PairTree(0), new[] {1,2}, new[] {a,b})) { } }
            catch (InvalidOperationException error) { rejected = error.Message.Contains("halfedge"); }
            Require(rejected, "Native topology error did not cross managed boundary");
            Console.WriteLine("Managed P/Invoke passed: layouts, union/subtract/intersect, UV/tangents, material ranges, cavity provenance, snapshot lifetime, empty output and disposal");
            return 0;
        }
        catch (Exception error) { Console.Error.WriteLine(error); return 1; }
    }
}
#endif
