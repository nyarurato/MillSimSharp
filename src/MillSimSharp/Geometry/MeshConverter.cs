using System;
using System.Collections.Generic;
using System.Numerics;
using System.Threading.Tasks;
using System.Collections.Concurrent;

namespace MillSimSharp.Geometry
{
    public static class MeshConverter
    {
        // Helper: Compute 8 corner world positions for a cube base position and resolution
        private static Vector3[] GetCubeCornerPositions(Vector3 basePos, float res)
        {
            return new Vector3[8]
            {
                basePos + new Vector3(0, 0, 0),
                basePos + new Vector3(res, 0, 0),
                basePos + new Vector3(res, res, 0),
                basePos + new Vector3(0, res, 0),
                basePos + new Vector3(0, 0, res),
                basePos + new Vector3(res, 0, res),
                basePos + new Vector3(res, res, res),
                basePos + new Vector3(0, res, res)
            };
        }

        // Helper: Interpolate the zero-crossing position on an edge given endpoints and their SDF
        private static Vector3 InterpolateEdgeVertex(Vector3 aPos, Vector3 bPos, float aVal, float bVal)
        {
            if (Math.Abs(aVal - bVal) < 1e-8f) return aPos; // avoid division by zero; fallback
            float t = aVal / (aVal - bVal);
            return aPos + (bPos - aPos) * t;
        }

        // Helper: Merge thread-local collected results into global arrays
        private static void MergeThreadLocal(IEnumerable<ThreadLocalData> threadLocalData, out List<Vector3> globalVerts, out List<Vector3> globalNormals, out List<int> globalInds)
        {
            globalVerts = new List<Vector3>();
            globalNormals = new List<Vector3>();
            globalInds = new List<int>();
            var comparer = new Vector3Comparer();
            var globalVertexMap = new Dictionary<Vector3, int>(comparer);
            var globalNormalSums = new Dictionary<Vector3, (Vector3 sum, int count)>(comparer);

            foreach (var data in threadLocalData)
            {
                var indexRemap = new int[data.Vertices.Count];
                for (int i = 0; i < data.Vertices.Count; i++)
                {
                    Vector3 v = data.Vertices[i];
                    var (sum, count) = data.NormalSums[v];
                    if (globalVertexMap.TryGetValue(v, out int existingIdx))
                    {
                        var (existingSum, existingCount) = globalNormalSums[v];
                        globalNormalSums[v] = (existingSum + sum, existingCount + count);
                        indexRemap[i] = existingIdx;
                    }
                    else
                    {
                        int newIdx = globalVerts.Count;
                        globalVerts.Add(v);
                        globalVertexMap[v] = newIdx;
                        globalNormalSums[v] = (sum, count);
                        indexRemap[i] = newIdx;
                    }
                }
                foreach (int oldIdx in data.Indices)
                {
                    globalInds.Add(indexRemap[oldIdx]);
                }
            }

            // Average normals
            foreach (var v in globalVerts)
            {
                var (sum, count) = globalNormalSums[v];
                globalNormals.Add(Vector3.Normalize(sum / count));
            }
        }
        private static void AddQuad(Vector3[] vertices, Vector3 normal, 
            Func<Vector3, Vector3, int> addVertex, List<int> indices)
        {
            // Add two triangles for the quad (v0, v1, v2, v3)
            int i0 = addVertex(vertices[0], normal);
            int i1 = addVertex(vertices[1], normal);
            int i2 = addVertex(vertices[2], normal);
            int i3 = addVertex(vertices[3], normal);

            // First triangle (0, 1, 2)
            indices.Add(i0);
            indices.Add(i1);
            indices.Add(i2);

            // Second triangle (0, 2, 3)
            indices.Add(i0);
            indices.Add(i2);
            indices.Add(i3);
        }

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

        // Thread-local data structure for parallel processing
        private struct ThreadLocalData
        {
            public List<Vector3> Vertices;
            public List<Vector3> Normals;
            public List<int> Indices;
            public Dictionary<Vector3, int> VertexMap;
            public Dictionary<Vector3, (Vector3 sum, int count)> NormalSums;
        }

        /// <summary>
        /// Converts a voxel grid to a triangle mesh using parallel processing.
        /// Generates faces for all material voxels where they border empty space or grid boundaries.
        /// </summary>
        /// <param name="grid">The voxel grid to convert</param>
        /// <returns>A triangle mesh representing the voxel surface</returns>
        public static Mesh ConvertToMesh(VoxelGrid grid)
        {
            if (grid == null) throw new ArgumentNullException(nameof(grid));

            var (sizeX, sizeY, sizeZ) = grid.Dimensions;
            float res = grid.Resolution;

            // Thread-local storage to avoid locks
            var threadLocalData = new ConcurrentBag<ThreadLocalData>();

            // Parallel processing by Z slices
            Parallel.For(-1, sizeZ, () =>
            {
                return new ThreadLocalData
                {
                    Vertices = new List<Vector3>(),
                    Normals = new List<Vector3>(),
                    Indices = new List<int>(),
                    VertexMap = new Dictionary<Vector3, int>(new Vector3Comparer()),
                    NormalSums = new Dictionary<Vector3, (Vector3 sum, int count)>(new Vector3Comparer())
                };
            },
            (z, loopState, data) =>
            {
                // Helper to add vertex and return index
                int AddVertex(Vector3 pos, Vector3 normal)
                {
                    if (!data.VertexMap.TryGetValue(pos, out int index))
                    {
                        index = data.Vertices.Count;
                        data.Vertices.Add(pos);
                        data.VertexMap[pos] = index;
                        data.NormalSums[pos] = (normal, 1);
                    }
                    else
                    {
                        var (sum, count) = data.NormalSums[pos];
                        data.NormalSums[pos] = (sum + normal, count + 1);
                    }
                    return index;
                }

                for (int y = -1; y < sizeY; y++)
                {
                    for (int x = -1; x < sizeX; x++)
                    {
                        // Check if this voxel has material
                        if (!grid.GetVoxel(x, y, z))
                            continue;

                        // Check each face and add if neighbor is empty
                        float half = res * 0.5f;
                        Vector3 center = grid.Bounds.Min + new Vector3(
                            (x + 0.5f) * res,
                            (y + 0.5f) * res,
                            (z + 0.5f) * res
                        );

                        // -X face
                        if (x == 0 || !grid.GetVoxel(x - 1, y, z))
                        {
                            Vector3 n = new Vector3(-1, 0, 0);
                            Vector3[] face = new Vector3[4]
                            {
                                center + new Vector3(-half, -half, -half),
                                center + new Vector3(-half, -half, +half),
                                center + new Vector3(-half, +half, +half),
                                center + new Vector3(-half, +half, -half)
                            };
                            AddQuad(face, n, AddVertex, data.Indices);
                        }

                        // +X face
                        if (x == sizeX - 1 || !grid.GetVoxel(x + 1, y, z))
                        {
                            Vector3 n = new Vector3(1, 0, 0);
                            Vector3[] face = new Vector3[4]
                            {
                                center + new Vector3(+half, -half, -half),
                                center + new Vector3(+half, +half, -half),
                                center + new Vector3(+half, +half, +half),
                                center + new Vector3(+half, -half, +half)
                            };
                            AddQuad(face, n, AddVertex, data.Indices);
                        }

                        // -Y face
                        if (y == 0 || !grid.GetVoxel(x, y - 1, z))
                        {
                            Vector3 n = new Vector3(0, -1, 0);
                            Vector3[] face = new Vector3[4]
                            {
                                center + new Vector3(-half, -half, -half),
                                center + new Vector3(+half, -half, -half),
                                center + new Vector3(+half, -half, +half),
                                center + new Vector3(-half, -half, +half)
                            };
                            AddQuad(face, n, AddVertex, data.Indices);
                        }

                        // +Y face
                        if (y == sizeY - 1 || !grid.GetVoxel(x, y + 1, z))
                        {
                            Vector3 n = new Vector3(0, 1, 0);
                            Vector3[] face = new Vector3[4]
                            {
                                center + new Vector3(-half, +half, -half),
                                center + new Vector3(-half, +half, +half),
                                center + new Vector3(+half, +half, +half),
                                center + new Vector3(+half, +half, -half)
                            };
                            AddQuad(face, n, AddVertex, data.Indices);
                        }

                        // -Z face
                        if (z == 0 || !grid.GetVoxel(x, y, z - 1))
                        {
                            Vector3 n = new Vector3(0, 0, -1);
                            Vector3[] face = new Vector3[4]
                            {
                                center + new Vector3(-half, -half, -half),
                                center + new Vector3(-half, +half, -half),
                                center + new Vector3(+half, +half, -half),
                                center + new Vector3(+half, -half, -half)
                            };
                            AddQuad(face, n, AddVertex, data.Indices);
                        }

                        // +Z face
                        if (z == sizeZ - 1 || !grid.GetVoxel(x, y, z + 1))
                        {
                            Vector3 n = new Vector3(0, 0, 1);
                            Vector3[] face = new Vector3[4]
                            {
                                center + new Vector3(-half, -half, +half),
                                center + new Vector3(+half, -half, +half),
                                center + new Vector3(+half, +half, +half),
                                center + new Vector3(-half, +half, +half)
                            };
                            AddQuad(face, n, AddVertex, data.Indices);
                        }
                    }
                }
                return data;
            },
            (data) => threadLocalData.Add(data));

            MergeThreadLocal(threadLocalData, out var globalVerts, out var globalNormals, out var globalInds);

            return new Mesh()
            {
                Vertices = globalVerts.ToArray(),
                Normals = globalNormals.ToArray(),
                Indices = globalInds.ToArray()
            };
        }

        /// <summary>
        /// Convert a Signed Distance Field grid into a triangle mesh using the Marching Cubes algorithm.
        /// This generates a smoother mesh compared to voxel-face extrusion.
        /// </summary>
        /// <param name="sdf">The signed distance field grid.</param>
        /// <returns>Generated Mesh with per-vertex normals computed via SDF gradients.</returns>
        public static Mesh ConvertToMeshFromSDF(SDFGrid sdf)
        {
            if (sdf == null) throw new ArgumentNullException(nameof(sdf));

            var (sizeX, sizeY, sizeZ) = sdf.Dimensions;
            float res = sdf.Resolution;

            var threadLocalData = new ConcurrentBag<ThreadLocalData>();

            Parallel.For(-1, sizeZ, () =>
            {
                return new ThreadLocalData
                {
                    Vertices = new List<Vector3>(),
                    Normals = new List<Vector3>(),
                    Indices = new List<int>(),
                    VertexMap = new Dictionary<Vector3, int>(new Vector3Comparer()),
                    NormalSums = new Dictionary<Vector3, (Vector3 sum, int count)>(new Vector3Comparer())
                };
            },
            (z, loopState, data) =>
            {
                int AddVertex(Vector3 pos, Vector3 normal)
                {
                    if (!data.VertexMap.TryGetValue(pos, out int index))
                    {
                        index = data.Vertices.Count;
                        data.Vertices.Add(pos);
                        data.VertexMap[pos] = index;
                        data.NormalSums[pos] = (normal, 1);
                    }
                    else
                    {
                        var (sum, count) = data.NormalSums[pos];
                        data.NormalSums[pos] = (sum + normal, count + 1);
                    }
                    return index;
                }

                for (int y = -1; y < sizeY; y++)
                {
                    for (int x = -1; x < sizeX; x++)
                    {
                        // Sample SDF at cube corners (node-based sampling). This avoids interpolation
                        // across out-of-bounds values and provides robust detection of boundary
                        // crossings for marching cubes. Allow x/y/z to be -1 to include outer cubes.
                        Vector3 basePos = sdf.Bounds.Min + new Vector3(x * res, y * res, z * res);

                        float[] val = new float[8];
                        Vector3[] cornerPos = GetCubeCornerPositions(basePos, res);
                        for (int i = 0; i < cornerPos.Length; i++)
                        {
                            val[i] = sdf.GetDistance(cornerPos[i]);
                        }

                        // Determine cube index
                        int cubeIndex = 0;
                        for (int i = 0; i < 8; i++)
                        {
                            if (val[i] < 0) cubeIndex |= (1 << i);
                        }

                        // If cube is entirely inside or outside, skip
                        if (cubeIndex == 0 || cubeIndex == 255)
                            continue;

                        // For each edge, interpolate vertex if needed
                        Vector3[] vertList = new Vector3[12];
                        for (int e = 0; e < vertList.Length; e++)
                        {
                            var edge = MarchingCubesTable.EdgeConnections[e];
                            int a = edge[0];
                            int b = edge[1];
                            float va = val[a];
                            float vb = val[b];
                            if ((va < 0 && vb >= 0) || (va >= 0 && vb < 0))
                            {
                                vertList[e] = InterpolateEdgeVertex(cornerPos[a], cornerPos[b], va, vb);
                            }
                            else
                            {
                                vertList[e] = new Vector3(float.NaN);
                            }
                        }

                        // Build triangles using table
                        int[] tri = MarchingCubesTable.TriangleTable[cubeIndex];
                        for (int ti = 0; ti < tri.Length; ti += 3)
                        {
                            if (tri[ti] < 0) break;
                            Vector3 v0 = vertList[tri[ti + 0]];
                            Vector3 v1 = vertList[tri[ti + 1]];
                            Vector3 v2 = vertList[tri[ti + 2]];

                            // Skip degenerate/invalid vertices (can occur at edges without interpolation)
                            if (float.IsNaN(v0.X) || float.IsNaN(v1.X) || float.IsNaN(v2.X))
                                continue;

                            // Compute normals from SDF gradient
                            // Use negative gradient so normals point outward from material
                            Vector3 n0 = -sdf.GetGradient(v0);
                            Vector3 n1 = -sdf.GetGradient(v1);
                            Vector3 n2 = -sdf.GetGradient(v2);

                            int i0 = AddVertex(v0, n0);
                            int i1 = AddVertex(v1, n1);
                            int i2 = AddVertex(v2, n2);

                            // Ensure the triangle winding matches the SDF gradient orientation so culling
                            // doesn't hide the surface (winding must point in the same direction of the gradient).
                            // Compute geometric face normal (right-hand rule) and flip indices if necessary.
                            // Compute geometric face normal; skip degenerate (near-zero area) faces
                            Vector3 faceNormalVec = Vector3.Cross(v1 - v0, v2 - v0);
                            if (faceNormalVec.Length() < 1e-6f)
                                continue;
                            Vector3 faceNormal = Vector3.Normalize(faceNormalVec);
                            Vector3 avgGradient = (n0 + n1 + n2) / 3.0f;
                            float avgLen = avgGradient.Length();
                            if (avgLen < 1e-6f)
                                continue;
                            avgGradient = Vector3.Normalize(avgGradient);
                            // If dot is negative, flip to make face normal point in the same direction as gradient
                            if (Vector3.Dot(faceNormal, avgGradient) < 0)
                            {
                                // swap i1 and i2 to flip winding
                                data.Indices.Add(i0);
                                data.Indices.Add(i2);
                                data.Indices.Add(i1);
                            }
                            else
                            {
                                data.Indices.Add(i0);
                                data.Indices.Add(i1);
                                data.Indices.Add(i2);
                            }
                        }
                    }
                }

                return data;
            },
            (data) => threadLocalData.Add(data));

            MergeThreadLocal(threadLocalData, out var globalVerts, out var globalNormals, out var globalInds);

            return new Mesh()
            {
                Vertices = globalVerts.ToArray(),
                Normals = globalNormals.ToArray(),
                Indices = globalInds.ToArray()
            };
        }

        /// <summary>
        /// Convenience method to convert voxel grid to SDF then to mesh.
        /// </summary>
        public static Mesh ConvertToMeshViaSDF(VoxelGrid grid, int narrowBandWidth = 10)
        {
            var sdf = SDFGrid.FromVoxelGrid(grid, narrowBandWidth);
            return ConvertToMeshFromSDF(sdf);
        }
    }
}
