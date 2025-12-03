using System;
using System.Collections.Generic;
using System.Numerics;
using System.Threading.Tasks;
using System.Collections.Concurrent;

namespace MillSimSharp.Geometry
{
    public static class MeshConverter
    {
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
            Parallel.For(0, sizeZ, () =>
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

                for (int y = 0; y < sizeY; y++)
                {
                    for (int x = 0; x < sizeX; x++)
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

            // Merge thread-local results
            var globalVerts = new List<Vector3>();
            var globalNormals = new List<Vector3>();
            var globalInds = new List<int>();
            var globalVertexMap = new Dictionary<Vector3, int>(new Vector3Comparer());
            var globalNormalSums = new Dictionary<Vector3, (Vector3 sum, int count)>(new Vector3Comparer());

            foreach (var data in threadLocalData)
            {
                // Map old indices to new indices
                var indexRemap = new int[data.Vertices.Count];

                for (int i = 0; i < data.Vertices.Count; i++)
                {
                    Vector3 v = data.Vertices[i];
                    var (sum, count) = data.NormalSums[v];

                    if (globalVertexMap.TryGetValue(v, out int existingIdx))
                    {
                        // Accumulate normals
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

                // Add indices with remapping
                foreach (int oldIdx in data.Indices)
                {
                    globalInds.Add(indexRemap[oldIdx]);
                }
            }

            // Average normals
            for (int i = 0; i < globalVerts.Count; i++)
            {
                var pos = globalVerts[i];
                var (sum, count) = globalNormalSums[pos];
                globalNormals.Add(Vector3.Normalize(sum / count));
            }

            return new Mesh()
            {
                Vertices = globalVerts.ToArray(),
                Normals = globalNormals.ToArray(),
                Indices = globalInds.ToArray()
            };
        }
    }
}
