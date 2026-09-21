using System;
using System.Collections.Generic;
using System.Numerics;
using MillSimSharp.Geometry;
using NUnit.Framework;

namespace MillSimSharp.Tests.Geometry
{
    /// <summary>
    /// Topology and orientation tests for the Dual Contouring mesh generator.
    /// </summary>
    [TestFixture]
    public class DualContouringTest
    {
        private static Mesh BuildSphereMesh(float resolution)
        {
            var bbox = BoundingBox.FromCenterAndSize(Vector3.Zero, new Vector3(24, 24, 24));
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

        private static void AssertWatertight(Mesh mesh)
        {
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
            foreach (var entry in edgeCounts)
            {
                if (entry.Value == 1) openEdges++;
                else if (entry.Value > 2) nonManifoldEdges++;
            }

            Assert.That(openEdges, Is.EqualTo(0), "Mesh has open edges (not watertight)");
            Assert.That(nonManifoldEdges, Is.EqualTo(0), "Mesh has non-manifold edges");
        }

        [TestCase(1.0f)]
        [TestCase(0.5f)]
        public void DualContouring_MeshIsWatertight_ForSphere(float resolution)
        {
            AssertWatertight(BuildSphereMesh(resolution));
        }

        [Test]
        public void DualContouring_NativeSdfCut_IsWatertight()
        {
            // SDF-native grids keep material at the boundary voxels (no EDT boundary values).
            // The outer shell must still be closed.
            var bbox = BoundingBox.FromCenterAndSize(Vector3.Zero, new Vector3(20, 20, 20));
            var sdf = new SDFGrid(bbox, 1.0f, narrowBandWidth: 2);
            sdf.RemoveFiniteCylinder(new Vector3(-10, 0, 0), new Vector3(10, 0, 0), 3f);

            AssertWatertight(MeshConverter.ConvertToMeshFromSDF(sdf));
        }

        [Test]
        public void DualContouring_SolidBlock_FacesPointOutward()
        {
            var bbox = BoundingBox.FromCenterAndSize(Vector3.Zero, new Vector3(20, 20, 20));
            var grid = new VoxelGrid(bbox, 1.0f);
            var sdf = SDFGrid.FromVoxelGrid(grid, narrowBandWidth: 5);
            var mesh = MeshConverter.ConvertToMeshFromSDF(sdf);

            AssertWatertight(mesh);

            int inward = 0;
            for (int i = 0; i < mesh.Indices.Length; i += 3)
            {
                Vector3 v0 = mesh.Vertices[mesh.Indices[i]];
                Vector3 v1 = mesh.Vertices[mesh.Indices[i + 1]];
                Vector3 v2 = mesh.Vertices[mesh.Indices[i + 2]];
                Vector3 normal = Vector3.Cross(v1 - v0, v2 - v0);
                if (normal.LengthSquared() < 1e-12f) continue;

                Vector3 centroid = (v0 + v1 + v2) / 3f;
                if (Vector3.Dot(Vector3.Normalize(normal), Vector3.Normalize(centroid - bbox.Center)) <= 0)
                    inward++;
            }

            Assert.That(inward, Is.EqualTo(0), "All solid block faces must point outward");
        }

        [Test]
        public void SDFGrid_BoundarySamples_AreAirOrSurface()
        {
            // A boundary sample must never be reported as material; otherwise the outer shell
            // of the DC mesh is generated outside the grid and edges of the stock are lost.
            var bbox = BoundingBox.FromCenterAndSize(Vector3.Zero, new Vector3(20, 20, 20));
            var sdf = new SDFGrid(bbox, 1.0f, narrowBandWidth: 2);

            Assert.That(sdf.GetDistance(new Vector3(bbox.Max.X, 0, 0)), Is.GreaterThanOrEqualTo(0f));
            Assert.That(sdf.GetDistance(new Vector3(bbox.Min.X, 0, 0)), Is.GreaterThanOrEqualTo(0f));
            Assert.That(sdf.GetDistance(new Vector3(0, 0, bbox.Max.Z)), Is.GreaterThanOrEqualTo(0f));
            Assert.That(sdf.GetDistance(new Vector3(0, 0, bbox.Min.Z)), Is.GreaterThanOrEqualTo(0f));
            Assert.That(sdf.GetDistance(bbox.Max + new Vector3(1, 1, 1)), Is.GreaterThan(0f));
        }

        [Test]
        public void DualContouring_NoDuplicateVertices_AfterMerge()
        {
            var mesh = BuildSphereMesh(0.5f);

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
