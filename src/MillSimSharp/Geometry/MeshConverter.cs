using System;
using System.Collections.Generic;
using System.Numerics;

namespace MillSimSharp.Geometry
{
    public static class MeshConverter
    {
        private class Vector3Comparer : IEqualityComparer<Vector3>
        {
            private const float Epsilon = 1e-6f;

            public bool Equals(Vector3 a, Vector3 b)
            {
                return Math.Abs(a.X - b.X) < Epsilon &&
                       Math.Abs(a.Y - b.Y) < Epsilon &&
                       Math.Abs(a.Z - b.Z) < Epsilon;
            }

            public int GetHashCode(Vector3 v)
            {
                return HashCode.Combine(
                    (int)(v.X / Epsilon),
                    (int)(v.Y / Epsilon),
                    (int)(v.Z / Epsilon)
                );
            }
        }
        // Convert VoxelGrid into a Mesh using marching cubes
        public static Mesh ConvertToMesh(VoxelGrid grid)
        {
            if (grid == null) throw new ArgumentNullException(nameof(grid));

            var verts = new List<Vector3>();
            var norms = new List<Vector3>();
            var inds = new List<int>();
            var vertexMap = new Dictionary<Vector3, int>(new Vector3Comparer());
            var normalSums = new Dictionary<Vector3, (Vector3 sum, int count)>(new Vector3Comparer());

            var (sizeX, sizeY, sizeZ) = grid.Dimensions;

            // Precompute node scalars for smoother interpolation
            float[][][] nodeScalar = new float[sizeX + 1][][];
            for (int i = 0; i <= sizeX; i++)
            {
                nodeScalar[i] = new float[sizeY + 1][];
                for (int j = 0; j <= sizeY; j++)
                {
                    nodeScalar[i][j] = new float[sizeZ + 1];
                }
            }
            for (int z = 0; z <= sizeZ; z++)
            {
                for (int y = 0; y <= sizeY; y++)
                {
                    for (int x = 0; x <= sizeX; x++)
                    {
                        int count = 0;
                        float sum = 0;
                        for (int dz = -1; dz <= 0; dz++)
                        {
                            for (int dy = -1; dy <= 0; dy++)
                            {
                                for (int dx = -1; dx <= 0; dx++)
                                {
                                    int vx = x + dx;
                                    int vy = y + dy;
                                    int vz = z + dz;
                                    if (vx >= 0 && vx < sizeX && vy >= 0 && vy < sizeY && vz >= 0 && vz < sizeZ)
                                    {
                                        sum += grid.GetVoxel(vx, vy, vz) ? 1.0f : 0.0f;
                                        count++;
                                    }
                                }
                            }
                        }
                        nodeScalar[x][y][z] = count > 0 ? sum / count : 0.0f;
                    }
                }
            }

            // Helper to add vertex and return index
            int AddVertex(Vector3 pos, Vector3 normal)
            {
                if (!vertexMap.TryGetValue(pos, out int index))
                {
                    index = verts.Count;
                    verts.Add(pos);
                    vertexMap[pos] = index;
                    normalSums[pos] = (normal, 1);
                }
                else
                {
                    var (sum, count) = normalSums[pos];
                    normalSums[pos] = (sum + normal, count + 1);
                }
                return index;
            }

            for (int z = 0; z < sizeZ - 1; z++)
            {
                for (int y = 0; y < sizeY - 1; y++)
                {
                    for (int x = 0; x < sizeX - 1; x++)
                    {
                        // Reuse logic from StlExporter.ProcessCube
                        bool[] corners = new bool[8];

                        float res = grid.Resolution;
                        Vector3 basePos = grid.Bounds.Min + new Vector3(x * res, y * res, z * res);

                        Vector3[] cornerPositions = new Vector3[8];
                        cornerPositions[0] = basePos;
                        cornerPositions[1] = basePos + new Vector3(res, 0, 0);
                        cornerPositions[2] = basePos + new Vector3(res, 0, res);
                        cornerPositions[3] = basePos + new Vector3(0, 0, res);
                        cornerPositions[4] = basePos + new Vector3(0, res, 0);
                        cornerPositions[5] = basePos + new Vector3(res, res, 0);
                        cornerPositions[6] = basePos + new Vector3(res, res, res);
                        cornerPositions[7] = basePos + new Vector3(0, res, res);

                        // Compute corner positions (world coordinates) and sample occupancy at those points
                        Vector3[] edgeVertices = new Vector3[12];
                        // Helper to interpolate edge vertex
                        Vector3 InterpolateEdge(int edgeIndex)
                        {
                            int v1 = MarchingCubesTable.EdgeConnections[edgeIndex][0];
                            int v2 = MarchingCubesTable.EdgeConnections[edgeIndex][1];
                            Vector3 p1 = cornerPositions[v1];
                            Vector3 p2 = cornerPositions[v2];
                            float s1 = nodeScalar[x + (v1 & 1)][y + ((v1 >> 1) & 1)][z + (v1 >> 2)];
                            float s2 = nodeScalar[x + (v2 & 1)][y + ((v2 >> 1) & 1)][z + (v2 >> 2)];
                            float t;
                            if (s1 == s2)
                            {
                                t = 0.5f;
                            }
                            else
                            {
                                t = (0.5f - s1) / (s2 - s1);
                                t = Math.Clamp(t, 0.0f, 1.0f);
                            }
                            return p1 + t * (p2 - p1);
                        }
                        for (int i = 0; i < 12; i++)
                        {
                            edgeVertices[i] = InterpolateEdge(i);
                        }
                        // Now sample corner occupancy using world coordinates
                        for (int i = 0; i < 8; i++)
                        {
                            corners[i] = grid.GetVoxelAtWorld(cornerPositions[i]);
                        }

                        int cubeIndex = 0;
                        for (int i = 0; i < 8; i++) if (corners[i]) cubeIndex |= (1 << i);
                        if (cubeIndex == 0 || cubeIndex == 255) continue;

                        for (int i = 0; MarchingCubesTable.TriangleTable[cubeIndex][i] != -1; i += 3)
                        {
                            int e1 = MarchingCubesTable.TriangleTable[cubeIndex][i];
                            int e2 = MarchingCubesTable.TriangleTable[cubeIndex][i + 1];
                            int e3 = MarchingCubesTable.TriangleTable[cubeIndex][i + 2];

                            Vector3 v1 = edgeVertices[e1];
                            Vector3 v2 = edgeVertices[e2];
                            Vector3 v3 = edgeVertices[e3];

                            Vector3 n = Vector3.Normalize(Vector3.Cross(v2 - v1, v3 - v1));

                            // Ensure triangle winding faces outward
                            var centroid = (v1 + v2 + v3) / 3.0f;
                            float eps = Math.Max(1e-4f, grid.Resolution * 0.75f);
                            var sampleInside = grid.GetVoxelAtWorld(centroid + n * eps);
                            var sampleOutside = grid.GetVoxelAtWorld(centroid - n * eps);

                            if (sampleInside && !sampleOutside)
                            {
                                // Flip winding
                                var tmp = v2; v2 = v3; v3 = tmp;
                                n = -n;
                            }
                            else if (sampleInside == sampleOutside)
                            {
                                // Fallback
                                var center = grid.Bounds.Min + grid.Bounds.Size / 2.0f;
                                var dir = Vector3.Normalize(centroid - center);
                                var dot = Vector3.Dot(n, dir);
                                if (dot < 0)
                                {
                                    var tmp = v2; v2 = v3; v3 = tmp;
                                    n = -n;
                                }
                            }

                            int i1 = AddVertex(v1, n);
                            int i2 = AddVertex(v2, n);
                            int i3 = AddVertex(v3, n);
                            inds.Add(i1);
                            inds.Add(i2);
                            inds.Add(i3);
                        }
                    }
                }
            }

            // Add outer boundary faces
            for (int z = 0; z < sizeZ; z++)
            {
                for (int y = 0; y < sizeY; y++)
                {
                    for (int x = 0; x < sizeX; x++)
                    {
                        if (!grid.GetVoxel(x, y, z)) continue;

                        var centerVoxel = grid.Bounds.Min + new Vector3((x + 0.5f) * grid.Resolution, (y + 0.5f) * grid.Resolution, (z + 0.5f) * grid.Resolution);
                        float half = grid.Resolution * 0.5f;

                        var faces = new (System.Func<int, int, int, bool> neighborCheck, Vector3[] corners, Vector3 outward)[ ]
                        {
                            ( (xi, yi, zi) => grid.GetVoxel(xi - 1, yi, zi), new Vector3[]
                                {
                                    centerVoxel + new Vector3(-half, -half, -half),
                                    centerVoxel + new Vector3(-half, +half, -half),
                                    centerVoxel + new Vector3(-half, +half, +half),
                                    centerVoxel + new Vector3(-half, -half, +half)
                                }, new Vector3(-1,0,0) ),
                            ( (xi, yi, zi) => grid.GetVoxel(xi + 1, yi, zi), new Vector3[]
                                {
                                    centerVoxel + new Vector3(+half, -half, +half),
                                    centerVoxel + new Vector3(+half, +half, +half),
                                    centerVoxel + new Vector3(+half, +half, -half),
                                    centerVoxel + new Vector3(+half, -half, -half)
                                }, new Vector3(+1,0,0) ),
                            ( (xi, yi, zi) => grid.GetVoxel(xi, yi - 1, zi), new Vector3[]
                                {
                                    centerVoxel + new Vector3(-half, -half, +half),
                                    centerVoxel + new Vector3(-half, -half, -half),
                                    centerVoxel + new Vector3(+half, -half, -half),
                                    centerVoxel + new Vector3(+half, -half, +half)
                                }, new Vector3(0,-1,0) ),
                            ( (xi, yi, zi) => grid.GetVoxel(xi, yi + 1, zi), new Vector3[]
                                {
                                    centerVoxel + new Vector3(-half, +half, -half),
                                    centerVoxel + new Vector3(-half, +half, +half),
                                    centerVoxel + new Vector3(+half, +half, +half),
                                    centerVoxel + new Vector3(+half, +half, -half)
                                }, new Vector3(0,+1,0) ),
                            ( (xi, yi, zi) => grid.GetVoxel(xi, yi, zi - 1), new Vector3[]
                                {
                                    centerVoxel + new Vector3(-half, +half, -half),
                                    centerVoxel + new Vector3(-half, -half, -half),
                                    centerVoxel + new Vector3(+half, -half, -half),
                                    centerVoxel + new Vector3(+half, +half, -half)
                                }, new Vector3(0,0,-1) ),
                            ( (xi, yi, zi) => grid.GetVoxel(xi, yi, zi + 1), new Vector3[]
                                {
                                    centerVoxel + new Vector3(-half, -half, +half),
                                    centerVoxel + new Vector3(-half, +half, +half),
                                    centerVoxel + new Vector3(+half, +half, +half),
                                    centerVoxel + new Vector3(+half, -half, +half)
                                }, new Vector3(0,0,1) )
                        };

                        foreach (var face in faces)
                        {
                            bool neighborHasMaterial = false;
                            try
                            {
                                neighborHasMaterial = face.neighborCheck(x, y, z);
                            }
                            catch { neighborHasMaterial = false; }
                            if (neighborHasMaterial) continue;

                            var fv = face.corners;
                            var tri1n = Vector3.Normalize(Vector3.Cross(fv[1] - fv[0], fv[2] - fv[0]));
                            var tri2n = Vector3.Normalize(Vector3.Cross(fv[2] - fv[0], fv[3] - fv[0]));
                            var triOut = face.outward;

                            if (Vector3.Dot(tri1n, triOut) < 0)
                            {
                                int i0 = AddVertex(fv[0], -tri1n);
                                int i2 = AddVertex(fv[2], -tri1n);
                                int i1 = AddVertex(fv[1], -tri1n);
                                inds.Add(i0); inds.Add(i2); inds.Add(i1);

                                int i3 = AddVertex(fv[3], -tri2n);
                                inds.Add(i0); inds.Add(i3); inds.Add(i2);
                            }
                            else
                            {
                                int i0 = AddVertex(fv[0], tri1n);
                                int i1 = AddVertex(fv[1], tri1n);
                                int i2 = AddVertex(fv[2], tri1n);
                                inds.Add(i0); inds.Add(i1); inds.Add(i2);

                                int i3 = AddVertex(fv[3], tri2n);
                                inds.Add(i0); inds.Add(i2); inds.Add(i3);
                            }
                        }
                    }
                }
            }

            // Average normals
            norms = new List<Vector3>(verts.Count);
            for (int i = 0; i < verts.Count; i++)
            {
                var pos = verts[i];
                var (sum, count) = normalSums[pos];
                norms.Add(Vector3.Normalize(sum / count));
            }

            return new Mesh()
            {
                Vertices = verts.ToArray(),
                Normals = norms.ToArray(),
                Indices = inds.ToArray()
            };
        }
    }
}
