using System;
using System.Collections.Generic;
using System.Numerics;
using MillSimSharp.Geometry;
using NUnit.Framework;

namespace MillSimSharp.Tests.Geometry
{
    /// <summary>
    /// Tests for incremental (dirty chunk) voxel mesh generation.
    /// </summary>
    [TestFixture]
    public class ChunkedVoxelMeshBuilderTest
    {
        private static BoundingBox StockBounds => BoundingBox.FromCenterAndSize(Vector3.Zero, new Vector3(40, 40, 40));

        private static string Quantize(Vector3 v)
        {
            return $"{(long)MathF.Round(v.X * 1000f)},{(long)MathF.Round(v.Y * 1000f)},{(long)MathF.Round(v.Z * 1000f)}";
        }

        private static List<string> TriangleKeys(Mesh mesh)
        {
            var keys = new List<string>();
            for (int i = 0; i + 2 < mesh.Indices.Length; i += 3)
            {
                var triangle = new List<string>
                {
                    Quantize(mesh.Vertices[mesh.Indices[i]]),
                    Quantize(mesh.Vertices[mesh.Indices[i + 1]]),
                    Quantize(mesh.Vertices[mesh.Indices[i + 2]])
                };
                triangle.Sort(StringComparer.Ordinal);
                keys.Add(string.Join("|", triangle));
            }

            keys.Sort(StringComparer.Ordinal);
            return keys;
        }

        [Test]
        public void BuildAll_MatchesMeshConverter()
        {
            var grid = new VoxelGrid(StockBounds, 1.0f);
            grid.RemoveVoxelsInSphere(Vector3.Zero, 8f);

            var reference = MeshConverter.ConvertToMesh(grid);
            var chunked = new ChunkedVoxelMeshBuilder(grid, chunkSize: 8).BuildAll();

            Assert.That(TriangleKeys(chunked), Is.EqualTo(TriangleKeys(reference)));
        }

        [Test]
        public void Update_MatchesFullRebuild_AfterSmallEdit()
        {
            var grid = new VoxelGrid(StockBounds, 1.0f);
            grid.RemoveVoxelsInSphere(new Vector3(-10, 0, 0), 5f);

            var builder = new ChunkedVoxelMeshBuilder(grid, chunkSize: 16);
            builder.BuildAll();

            grid.RemoveVoxelsInSphere(new Vector3(10, 0, 0), 5f);
            var updated = builder.Update(25, 15, 15, 35, 25, 25);

            var fresh = new ChunkedVoxelMeshBuilder(grid, chunkSize: 16).BuildAll();

            Assert.That(TriangleKeys(updated), Is.EqualTo(TriangleKeys(fresh)),
                "Incremental chunk update must match a full rebuild");
        }

        [Test]
        public void Update_RebuildsOnlyAffectedChunks()
        {
            var grid = new VoxelGrid(StockBounds, 1.0f);
            grid.RemoveVoxelsInSphere(new Vector3(-10, 0, 0), 5f);

            var builder = new ChunkedVoxelMeshBuilder(grid, chunkSize: 16);
            builder.BuildAll();
            int before = builder.ChunkBuildCount;

            grid.RemoveVoxelsInSphere(new Vector3(10, 0, 0), 5f);
            builder.Update(25, 15, 15, 35, 25, 25);

            int rebuilds = builder.ChunkBuildCount - before;
            Assert.That(rebuilds, Is.GreaterThan(0), "At least one chunk must be rebuilt");
            Assert.That(rebuilds, Is.LessThanOrEqualTo(8), "Only chunks near the change may be rebuilt");
        }

        [Test]
        public void ChunkApi_GetChunkMesh_MatchesCombinedMesh()
        {
            var grid = new VoxelGrid(StockBounds, 1.0f);
            grid.RemoveVoxelsInSphere(Vector3.Zero, 8f);

            var builder = new ChunkedVoxelMeshBuilder(grid, chunkSize: 8);
            Mesh combined = builder.BuildAll();

            var chunkTriangles = new List<string>();
            int chunkCount = 0;
            foreach (var (cx, cy, cz) in builder.GetChunkCoordinates())
            {
                chunkTriangles.AddRange(TriangleKeys(builder.GetChunkMesh(cx, cy, cz)));
                chunkCount++;
            }
            chunkTriangles.Sort(StringComparer.Ordinal);

            Assert.That(chunkCount, Is.GreaterThan(1), "the test grid must span multiple chunks");
            Assert.That(chunkTriangles, Is.EqualTo(TriangleKeys(combined)),
                "the union of the per-chunk meshes must match the combined mesh");
        }

        [Test]
        public void ChunkApi_UpdateChunks_ReturnsRebuiltChunksAndMatchesFullRebuild()
        {
            var grid = new VoxelGrid(StockBounds, 1.0f);
            grid.RemoveVoxelsInSphere(new Vector3(-10, 0, 0), 5f);

            var builder = new ChunkedVoxelMeshBuilder(grid, chunkSize: 16);
            builder.BuildAll();
            int buildsBefore = builder.ChunkBuildCount;

            grid.RemoveVoxelsInSphere(new Vector3(10, 0, 0), 5f);
            var rebuilt = builder.UpdateChunks(25, 15, 15, 35, 25, 25);

            Assert.That(rebuilt, Is.Not.Empty);
            Assert.That(builder.ChunkBuildCount - buildsBefore, Is.EqualTo(rebuilt.Count),
                "only the returned chunks may be rebuilt");

            var chunkTriangles = new List<string>();
            foreach (var (cx, cy, cz) in builder.GetChunkCoordinates())
            {
                chunkTriangles.AddRange(TriangleKeys(builder.GetChunkMesh(cx, cy, cz)));
            }
            chunkTriangles.Sort(StringComparer.Ordinal);

            var fresh = new ChunkedVoxelMeshBuilder(grid, chunkSize: 16).BuildAll();
            Assert.That(chunkTriangles, Is.EqualTo(TriangleKeys(fresh)),
                "the per-chunk meshes after UpdateChunks must match a full rebuild");
        }

        [Test]
        public void ChunkApi_GetChunkMesh_UnknownChunk_IsEmpty()
        {
            var grid = new VoxelGrid(StockBounds, 1.0f);
            var builder = new ChunkedVoxelMeshBuilder(grid, chunkSize: 8);

            Mesh chunk = builder.GetChunkMesh(99, 99, 99);

            Assert.That(chunk.Vertices, Is.Empty);
            Assert.That(chunk.Indices, Is.Empty);
        }
    }
}
