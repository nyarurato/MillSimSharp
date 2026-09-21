using System;
using System.Collections.Generic;
using System.Numerics;
using System.Threading.Tasks;
using System.Collections.Concurrent;

namespace MillSimSharp.Geometry
{
    /// <summary>
    /// Static class for converting voxel grids and SDF grids to triangle meshes.
    /// </summary>
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
            var comparer = new VoxelVertexComparer();
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
                    VertexMap = new Dictionary<Vector3, int>(new VoxelVertexComparer()),
                    NormalSums = new Dictionary<Vector3, (Vector3 sum, int count)>(new VoxelVertexComparer())
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

                        Func<Vector3, Vector3, int> addVertex = AddVertex;

                        if (x == 0 || !grid.GetVoxel(x - 1, y, z))
                            VoxelMeshUtil.EmitFace(0, center, half, addVertex, data.Indices);
                        if (x == sizeX - 1 || !grid.GetVoxel(x + 1, y, z))
                            VoxelMeshUtil.EmitFace(1, center, half, addVertex, data.Indices);
                        if (y == 0 || !grid.GetVoxel(x, y - 1, z))
                            VoxelMeshUtil.EmitFace(2, center, half, addVertex, data.Indices);
                        if (y == sizeY - 1 || !grid.GetVoxel(x, y + 1, z))
                            VoxelMeshUtil.EmitFace(3, center, half, addVertex, data.Indices);
                        if (z == 0 || !grid.GetVoxel(x, y, z - 1))
                            VoxelMeshUtil.EmitFace(4, center, half, addVertex, data.Indices);
                        if (z == sizeZ - 1 || !grid.GetVoxel(x, y, z + 1))
                            VoxelMeshUtil.EmitFace(5, center, half, addVertex, data.Indices);
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
        /// Convert a Signed Distance Field grid into a triangle mesh using the Dual Contouring algorithm.
        /// This generates a mesh with better feature preservation compared to Marching Cubes.
        /// </summary>
        /// <param name="sdf">The signed distance field grid</param>
        /// <returns>A triangle mesh representing the zero isosurface</returns>
        public static Mesh ConvertToMeshFromSDF(SDFGrid sdf)
        {
            if (sdf == null) throw new ArgumentNullException(nameof(sdf));

            // Use Dual Contouring algorithm
            return DualContouring.Generate(sdf);
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
