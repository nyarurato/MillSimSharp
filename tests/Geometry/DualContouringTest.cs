using System;
using System.Collections.Generic;
using System.Numerics;
using MillSimSharp.Geometry;
using NUnit.Framework;

namespace MillSimSharp.Tests.Geometry
{
    /// <summary>
    /// Topology tests for the Dual Contouring mesh generator.
    /// </summary>
    [TestFixture]
    public class DualContouringTest
    {
        private static Mesh BuildSphereMesh(float resolution, out BoundingBox bbox)
        {
            bbox = BoundingBox.FromCenterAndSize(Vector3.Zero, new Vector3(24, 24, 24));
            var grid = new VoxelGrid(bbox, resolution);
            grid.RemoveVoxelsInSphere(Vector3.Zero, 5f);
            var sdf = SDFGrid.FromVoxelGrid(grid, narrowBandWidth: 12);
            return MeshConverter.ConvertToMeshFromSDF(sdf);
        }

        private static void AddEdge(Dictionary<(int, int), int> edges, int a, int b)
        {
            var key = a < b ? (a, b) : (b, a);
            edges.TryGetValue(key, out int count);
            edges[key] = count + 1;
        }

        [TestCase(1.0f)]
        [TestCase(0.5f)]
        public void DualContouring_MeshIsWatertight_ForSphere(float resolution)
        {
            var mesh = BuildSphereMesh(resolution, out _);
            Assert.That(mesh.Indices.Length, Is.GreaterThan(0));

            var edgeCounts = new Dictionary<(int, int), int>();
            for (int i = 0; i < mesh.Indices.Length; i += 3)
            {
                int a = mesh.Indices[i];
                int b = mesh.Indices[i + 1];
                int c = mesh.Indices[i + 2];
                AddEdge(edgeCounts, a, b);
                AddEdge(edgeCounts, b, c);
                AddEdge(edgeCounts, c, a);
            }

            int openEdges = 0;
            int nonManifoldEdges = 0;
            int selfLoops = 0;
            var openExamples = new List<string>();
            foreach (var entry in edgeCounts)
            {
                if (entry.Key.Item1 == entry.Key.Item2) selfLoops++;
                if (entry.Value == 1)
                {
                    openEdges++;
                    if (openExamples.Count < 6)
                    {
                        Vector3 p1 = mesh.Vertices[entry.Key.Item1];
                        Vector3 p2 = mesh.Vertices[entry.Key.Item2];
                        openExamples.Add($"({p1.X:F2},{p1.Y:F2},{p1.Z:F2})-({p2.X:F2},{p2.Y:F2},{p2.Z:F2})");
                    }
                }
                else if (entry.Value > 2) nonManifoldEdges++;
            }

            Assert.That(openEdges, Is.EqualTo(0),
                $"open={openEdges} nonManifold={nonManifoldEdges} selfLoops={selfLoops} examples: {string.Join("; ", openExamples)}");
            Assert.That(nonManifoldEdges, Is.EqualTo(0), "Mesh has non-manifold edges");
        }

        [Test]
        public void DualContouring_NoDuplicateVertices_AfterMerge()
        {
            var mesh = BuildSphereMesh(0.5f, out _);

            var unique = new HashSet<(long, long, long)>();
            int duplicates = 0;
            foreach (var v in mesh.Vertices)
            {
                var key = (
                    (long)MathF.Round(v.X * 1e5f),
                    (long)MathF.Round(v.Y * 1e5f),
                    (long)MathF.Round(v.Z * 1e5f));
                if (!unique.Add(key)) duplicates++;
            }

            Assert.That(duplicates, Is.EqualTo(0), "Vertices must be merged after generation");
        }
    }
}
