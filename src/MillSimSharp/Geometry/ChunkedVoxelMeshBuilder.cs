using System;
using System.Collections.Generic;
using System.Numerics;

namespace MillSimSharp.Geometry
{
    /// <summary>
    /// Incremental voxel mesh builder that caches one mesh per chunk and rebuilds only the chunks
    /// affected by a change. Faces are owned by their own voxel (neighbor lookups are global), so
    /// no seam stitching between chunks is required.
    /// </summary>
    public sealed class ChunkedVoxelMeshBuilder
    {
        private readonly VoxelGrid _grid;
        private readonly int _chunkSize;
        private readonly Dictionary<(int x, int y, int z), ChunkData> _chunks = new Dictionary<(int, int, int), ChunkData>();
        private readonly VoxelVertexComparer _comparer;

        /// <summary>
        /// Creates a chunked mesh builder for the given voxel grid.
        /// </summary>
        /// <param name="grid">Voxel grid to mesh.</param>
        /// <param name="chunkSize">Chunk edge length in voxels (default: 16).</param>
        public ChunkedVoxelMeshBuilder(VoxelGrid grid, int chunkSize = 16)
        {
            _grid = grid ?? throw new ArgumentNullException(nameof(grid));
            if (chunkSize <= 0) throw new ArgumentException("Chunk size must be positive.", nameof(chunkSize));
            _chunkSize = chunkSize;
            _comparer = new VoxelVertexComparer(VoxelVertexComparer.ForResolution(grid.Resolution));
        }

        /// <summary>
        /// Gets the number of chunk rebuilds performed so far (diagnostics).
        /// </summary>
        public int ChunkBuildCount { get; private set; }

        /// <summary>
        /// Gets the number of currently cached chunks.
        /// </summary>
        public int CachedChunkCount => _chunks.Count;

        /// <summary>
        /// Enumerates the coordinates of the currently cached chunks (including chunks without
        /// geometry). The returned list is a snapshot.
        /// </summary>
        public IReadOnlyList<(int X, int Y, int Z)> GetChunkCoordinates()
        {
            var coordinates = new List<(int X, int Y, int Z)>(_chunks.Count);
            foreach (var key in _chunks.Keys)
            {
                coordinates.Add(key);
            }
            return coordinates;
        }

        /// <summary>
        /// Gets the cached mesh of one chunk: world-space vertices with chunk-local indices and
        /// per-chunk averaged, normalized normals. Returns an empty mesh when the chunk has not
        /// been built.
        /// </summary>
        /// <param name="chunkX">Chunk index X.</param>
        /// <param name="chunkY">Chunk index Y.</param>
        /// <param name="chunkZ">Chunk index Z.</param>
        public Mesh GetChunkMesh(int chunkX, int chunkY, int chunkZ)
        {
            if (!_chunks.TryGetValue((chunkX, chunkY, chunkZ), out ChunkData chunk))
            {
                return new Mesh();
            }

            var normals = new Vector3[chunk.Normals.Count];
            for (int i = 0; i < normals.Length; i++)
            {
                Vector3 sum = chunk.Normals[i];
                normals[i] = sum.LengthSquared() > 1e-12f ? Vector3.Normalize(sum) : Vector3.UnitY;
            }

            return new Mesh
            {
                Vertices = chunk.Vertices.ToArray(),
                Normals = normals,
                Indices = chunk.Indices.ToArray()
            };
        }

        /// <summary>
        /// Rebuilds all chunks and returns the combined mesh.
        /// <para>
        /// This combines every cached chunk into one vertex-welded mesh, which costs a full-grid
        /// pass. For incremental rendering use <see cref="UpdateChunks"/> with
        /// <see cref="GetChunkMesh"/> instead.
        /// </para>
        /// </summary>
        /// <returns>Combined mesh of the whole grid.</returns>
        public Mesh BuildAll()
        {
            _chunks.Clear();

            var (sx, sy, sz) = _grid.Dimensions;
            for (int cz = 0; cz * _chunkSize < sz; cz++)
                for (int cy = 0; cy * _chunkSize < sy; cy++)
                    for (int cx = 0; cx * _chunkSize < sx; cx++)
                    {
                        _chunks[(cx, cy, cz)] = BuildChunk(cx, cy, cz);
                    }

            return Combine();
        }

        /// <summary>
        /// Rebuilds the chunks affected by the given changed voxel bounds and returns the combined
        /// mesh.
        /// The bounds are expanded by one voxel because a removed voxel also changes the exposed
        /// faces of its neighbors.
        /// <para>
        /// This combines every cached chunk into one vertex-welded mesh (a full-grid pass even
        /// though only the dirty chunks are rebuilt). For incremental rendering use
        /// <see cref="UpdateChunks"/> and upload only the returned chunks.
        /// </para>
        /// </summary>
        /// <param name="minX">Minimum changed voxel index X.</param>
        /// <param name="minY">Minimum changed voxel index Y.</param>
        /// <param name="minZ">Minimum changed voxel index Z.</param>
        /// <param name="maxX">Maximum changed voxel index X.</param>
        /// <param name="maxY">Maximum changed voxel index Y.</param>
        /// <param name="maxZ">Maximum changed voxel index Z.</param>
        /// <returns>Combined mesh of the whole grid.</returns>
        public Mesh Update(int minX, int minY, int minZ, int maxX, int maxY, int maxZ)
        {
            RebuildDirtyChunks(minX, minY, minZ, maxX, maxY, maxZ);
            return Combine();
        }

        /// <summary>
        /// Rebuilds the chunks affected by the given changed voxel bounds and returns their
        /// coordinates in rebuild order, without combining the mesh. Use this together with
        /// <see cref="GetChunkMesh"/> to update only the changed chunks.
        /// </summary>
        /// <param name="minX">Minimum changed voxel index X.</param>
        /// <param name="minY">Minimum changed voxel index Y.</param>
        /// <param name="minZ">Minimum changed voxel index Z.</param>
        /// <param name="maxX">Maximum changed voxel index X.</param>
        /// <param name="maxY">Maximum changed voxel index Y.</param>
        /// <param name="maxZ">Maximum changed voxel index Z.</param>
        /// <returns>Coordinates of the rebuilt chunks (empty when nothing changed).</returns>
        public IReadOnlyList<(int X, int Y, int Z)> UpdateChunks(int minX, int minY, int minZ, int maxX, int maxY, int maxZ)
        {
            return RebuildDirtyChunks(minX, minY, minZ, maxX, maxY, maxZ);
        }

        private List<(int X, int Y, int Z)> RebuildDirtyChunks(int minX, int minY, int minZ, int maxX, int maxY, int maxZ)
        {
            var rebuilt = new List<(int X, int Y, int Z)>();
            var (sx, sy, sz) = _grid.Dimensions;

            minX = Math.Max(0, minX - 1);
            minY = Math.Max(0, minY - 1);
            minZ = Math.Max(0, minZ - 1);
            maxX = Math.Min(sx - 1, maxX + 1);
            maxY = Math.Min(sy - 1, maxY + 1);
            maxZ = Math.Min(sz - 1, maxZ + 1);
            if (minX > maxX || minY > maxY || minZ > maxZ) return rebuilt;

            int minCx = minX / _chunkSize, maxCx = maxX / _chunkSize;
            int minCy = minY / _chunkSize, maxCy = maxY / _chunkSize;
            int minCz = minZ / _chunkSize, maxCz = maxZ / _chunkSize;

            for (int cz = minCz; cz <= maxCz; cz++)
                for (int cy = minCy; cy <= maxCy; cy++)
                    for (int cx = minCx; cx <= maxCx; cx++)
                    {
                        _chunks[(cx, cy, cz)] = BuildChunk(cx, cy, cz);
                        rebuilt.Add((cx, cy, cz));
                    }

            return rebuilt;
        }

        private ChunkData BuildChunk(int cx, int cy, int cz)
        {
            ChunkBuildCount++;

            var (sx, sy, sz) = _grid.Dimensions;
            int minX = cx * _chunkSize;
            int minY = cy * _chunkSize;
            int minZ = cz * _chunkSize;
            int maxX = Math.Min(minX + _chunkSize - 1, sx - 1);
            int maxY = Math.Min(minY + _chunkSize - 1, sy - 1);
            int maxZ = Math.Min(minZ + _chunkSize - 1, sz - 1);

            var vertices = new List<Vector3>();
            var normals = new List<Vector3>();
            var indices = new List<int>();
            var vertexMap = new Dictionary<Vector3, int>(_comparer);

            float res = _grid.Resolution;
            float half = res * 0.5f;

            int AddVertex(Vector3 position, Vector3 normal)
            {
                if (vertexMap.TryGetValue(position, out int existing))
                {
                    normals[existing] += normal;
                    return existing;
                }

                int index = vertices.Count;
                vertices.Add(position);
                normals.Add(normal);
                vertexMap[position] = index;
                return index;
            }

            Func<Vector3, Vector3, int> addVertex = AddVertex;

            for (int z = minZ; z <= maxZ; z++)
                for (int y = minY; y <= maxY; y++)
                    for (int x = minX; x <= maxX; x++)
                    {
                        if (!_grid.GetVoxel(x, y, z)) continue;

                        Vector3 center = _grid.Bounds.Min + new Vector3(
                            (x + 0.5f) * res,
                            (y + 0.5f) * res,
                            (z + 0.5f) * res);

                        if (x == 0 || !_grid.GetVoxel(x - 1, y, z))
                            VoxelMeshUtil.EmitFace(0, center, half, addVertex, indices);
                        if (x == sx - 1 || !_grid.GetVoxel(x + 1, y, z))
                            VoxelMeshUtil.EmitFace(1, center, half, addVertex, indices);
                        if (y == 0 || !_grid.GetVoxel(x, y - 1, z))
                            VoxelMeshUtil.EmitFace(2, center, half, addVertex, indices);
                        if (y == sy - 1 || !_grid.GetVoxel(x, y + 1, z))
                            VoxelMeshUtil.EmitFace(3, center, half, addVertex, indices);
                        if (z == 0 || !_grid.GetVoxel(x, y, z - 1))
                            VoxelMeshUtil.EmitFace(4, center, half, addVertex, indices);
                        if (z == sz - 1 || !_grid.GetVoxel(x, y, z + 1))
                            VoxelMeshUtil.EmitFace(5, center, half, addVertex, indices);
                    }

            return new ChunkData
            {
                Vertices = vertices,
                Normals = normals,
                Indices = indices
            };
        }

        private Mesh Combine()
        {
            var vertices = new List<Vector3>();
            var normalSums = new List<Vector3>();
            var indices = new List<int>();
            var vertexMap = new Dictionary<Vector3, int>(_comparer);

            foreach (var chunk in _chunks.Values)
            {
                var remap = new int[chunk.Vertices.Count];

                for (int i = 0; i < chunk.Vertices.Count; i++)
                {
                    Vector3 position = chunk.Vertices[i];
                    if (vertexMap.TryGetValue(position, out int existing))
                    {
                        normalSums[existing] += chunk.Normals[i];
                        remap[i] = existing;
                    }
                    else
                    {
                        int index = vertices.Count;
                        vertices.Add(position);
                        normalSums.Add(chunk.Normals[i]);
                        vertexMap[position] = index;
                        remap[i] = index;
                    }
                }

                foreach (int oldIndex in chunk.Indices)
                {
                    indices.Add(remap[oldIndex]);
                }
            }

            var normals = new List<Vector3>(vertices.Count);
            foreach (var sum in normalSums)
            {
                normals.Add(sum.LengthSquared() > 1e-12f ? Vector3.Normalize(sum) : Vector3.UnitY);
            }

            return new Mesh
            {
                Vertices = vertices.ToArray(),
                Normals = normals.ToArray(),
                Indices = indices.ToArray()
            };
        }

        private struct ChunkData
        {
            public List<Vector3> Vertices;
            public List<Vector3> Normals;
            public List<int> Indices;
        }
    }
}
