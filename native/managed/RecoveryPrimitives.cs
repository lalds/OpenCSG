using System;

namespace RealtimeCSG.Recovery
{
    // Small explicit fixtures for bridge/Unity previews, not the existing editor's brush generators.
    public static class Primitives
    {
        public static Brush Box(Float3 lo, Float3 hi, int nodeIndex, int brushIndex, int materialBase = 0)
        {
            var result = new Brush {
                nodeIndex = nodeIndex, brushIndex = brushIndex,
                vertices = new[] {
                    new Float3(lo.x,lo.y,lo.z), new Float3(hi.x,lo.y,lo.z), new Float3(hi.x,hi.y,lo.z), new Float3(lo.x,hi.y,lo.z),
                    new Float3(lo.x,lo.y,hi.z), new Float3(hi.x,lo.y,hi.z), new Float3(hi.x,hi.y,hi.z), new Float3(lo.x,hi.y,hi.z)
                }, edges = new HalfEdge[24], polygons = new Polygon[6], surfaces = new Surface[6]
            };
            var normals = new[] { new Float3(0,0,-1),new Float3(0,0,1),new Float3(0,-1,0),new Float3(1,0,0),new Float3(0,1,0),new Float3(-1,0,0) };
            var distances = new[] { -lo.z, hi.z, -lo.y, hi.x, hi.y, -lo.x };
            int[,] faces = { {3,2,1,0},{4,5,6,7},{0,1,5,4},{1,2,6,5},{2,3,7,6},{3,0,4,7} };
            for (int p = 0; p < 6; ++p)
            {
                result.polygons[p] = new Polygon { firstEdge = p*4, edgeCount = 4, surfaceIndex = p };
                result.surfaces[p] = new Surface {
                    plane = new Plane { normal = normals[p], distance = distances[p] }, materialId = materialBase + p%2,
                    uvU = p == 3 || p == 5 ? new Float4(0,1,0,.25f) : new Float4(1,0,0,.25f),
                    uvV = p < 2 ? new Float4(0,1,0,-.5f) : new Float4(0,0,1,-.5f)
                };
                for (int c = 0; c < 4; ++c)
                {
                    int twin = -1;
                    for (int q = 0; q < 6; ++q) for (int d = 0; d < 4; ++d)
                        if (faces[p,c] == faces[q,(d+3)%4] && faces[p,(c+3)%4] == faces[q,d]) twin = q*4+d;
                    result.edges[p*4+c] = new HalfEdge { vertexIndex = faces[p,c], twinIndex = twin };
                }
            }
            return result;
        }
        public static Node[] PairTree(byte operation)
        {
            return new[] {
                new Node { firstChild = 0, childCount = 2 },
                new Node { isBrush = 1 }, new Node { isBrush = 1, operation = operation }
            };
        }
    }
}
