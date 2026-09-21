using System;
using System.Collections.Generic;
using System.Numerics;
using MillSimSharp.Geometry;
using NUnit.Framework;

namespace MillSimSharp.Tests.Geometry
{
    /// <summary>
    /// Dual contouring accuracy on analytic shapes beyond the sphere: a planar surface (exact QEF
    /// test) and a cubic cavity (topology / normal orientation / degeneracy).
    /// </summary>
    [TestFixture]
    public class DualContouringAnalyticTest
    {
        private static Mesh BuildHalfSpaceMesh(float resolution)
        {
            // Material occupies x < 0; the analytic surface is the plane x = 0.
            var bbox = BoundingBox.FromCenterAndSize(Vector3.Zero, new Vector3(20, 20, 20));
            var grid = new VoxelGrid(bbox, resolution);
            var (sx, sy, sz) = grid.Dimensions;

            for (int x = 0; x < sx; x++)
                for (int y = 0; y < sy; y++)
                    for (int z = 0; z < sz; z++)
                    {
                        float worldX = grid.Bounds.Min.X + (x + 0.5f) * resolution;
                        if (worldX > 0f) grid.SetVoxel(x, y, z, false);
                    }

            var sdf = SDFGrid.FromVoxelGrid(grid, narrowBandWidth: 10);
            return MeshConverter.ConvertToMeshFromSDF(sdf);
        }

        private static Mesh BuildCubicCavityMesh()
        {
            var bbox = BoundingBox.FromCenterAndSize(Vector3.Zero, new Vector3(24, 24, 24));
            var grid = new VoxelGrid(bbox, 1.0f);
            var (sx, sy, sz) = grid.Dimensions;

            for (int x = 0; x < sx; x++)
                for (int y = 0; y < sy; y++)
                    for (int z = 0; z < sz; z++)
                    {
                        Vector3 center = grid.Bounds.Min + new Vector3(
                            (x + 0.5f), (y + 0.5f), (z + 0.5f));
                        if (MathF.Abs(center.X) <= 4f &&
                            MathF.Abs(center.Y) <= 4f &&
                            MathF.Abs(center.Z) <= 4f)
                        {
                            grid.SetVoxel(x, y, z, false);
                        }
                    }

            var sdf = SDFGrid.FromVoxelGrid(grid, narrowBandWidth: 10);
            return MeshConverter.ConvertToMeshFromSDF(sdf);
        }

        [TestCase(1.0f)]
        [TestCase(0.5f)]
        public void DualContouring_PlaneVertexDeviation_IsNearZero(float resolution)
        {
            Mesh mesh = BuildHalfSpaceMesh(resolution);

            int count = 0;
            float maxDeviation = 0;
            foreach (Vector3 vertex in mesh.Vertices)
            {
                // Only the plane surface: skip the outer shell (x = +/-10) and the side walls
                // (y/z = +/-10), which carry vertices at every x.
                if (MathF.Abs(vertex.X) > 3f || MathF.Abs(vertex.Y) > 8f || MathF.Abs(vertex.Z) > 8f) continue;

                maxDeviation = MathF.Max(maxDeviation, MathF.Abs(vertex.X));
                count++;
            }

            TestContext.Out.WriteLine(
                $"plane res={resolution} vertices={count} maxDeviation={maxDeviation:F5}");

            Assert.That(count, Is.GreaterThan(100), "the plane must contribute a substantial vertex set");
            Assert.That(maxDeviation, Is.LessThanOrEqualTo(0.25f * resolution),
                "DC vertices on a planar surface must stay near x = 0");
        }

        [Test]
        public void DualContouring_Box_HasNoNaNOrDegenerateTriangles()
        {
            Mesh mesh = BuildCubicCavityMesh();

            Assert.That(mesh.Vertices.Length, Is.GreaterThan(100));

            foreach (Vector3 vertex in mesh.Vertices)
            {
                Assert.That(float.IsNaN(vertex.X) || float.IsInfinity(vertex.X) ||
                            float.IsNaN(vertex.Y) || float.IsInfinity(vertex.Y) ||
                            float.IsNaN(vertex.Z) || float.IsInfinity(vertex.Z),
                    Is.False, $"vertex must be finite: {vertex}");
            }

            int degenerateTriangles = 0;
            for (int i = 0; i < mesh.Indices.Length; i += 3)
            {
                Vector3 a = mesh.Vertices[mesh.Indices[i]];
                Vector3 b = mesh.Vertices[mesh.Indices[i + 1]];
                Vector3 c = mesh.Vertices[mesh.Indices[i + 2]];
                if (Vector3.Cross(b - a, c - a).LengthSquared() <= 1e-10f) degenerateTriangles++;
            }

            Assert.That(degenerateTriangles, Is.EqualTo(0), "no degenerate triangles expected");
            AssertWatertight(mesh);

            // The SDF gradient points from material toward air; triangle normals on the cavity
            // surface must agree. Outer-shell triangles are excluded (the analytic cavity gradient
            // is only meaningful near the cavity).
            int opposite = 0;
            int checkedTriangles = 0;
            for (int i = 0; i < mesh.Indices.Length; i += 3)
            {
                Vector3 a = mesh.Vertices[mesh.Indices[i]];
                Vector3 b = mesh.Vertices[mesh.Indices[i + 1]];
                Vector3 c = mesh.Vertices[mesh.Indices[i + 2]];
                Vector3 normal = Vector3.Cross(b - a, c - a);
                if (normal.LengthSquared() <= 1e-10f) continue;

                Vector3 centroid = (a + b + c) / 3f;
                if (MathF.Abs(centroid.X) > 6f || MathF.Abs(centroid.Y) > 6f || MathF.Abs(centroid.Z) > 6f)
                    continue;

                Vector3 gradient = BuildCubicCavityGradient(centroid);
                if (gradient.LengthSquared() < 1e-6f) continue;

                checkedTriangles++;
                if (Vector3.Dot(Vector3.Normalize(normal), gradient) <= 0f) opposite++;
            }

            TestContext.Out.WriteLine(
                $"box cavity: triangles={mesh.Indices.Length / 3} checked={checkedTriangles} opposite={opposite}");
            Assert.That(checkedTriangles, Is.GreaterThan(100));
            Assert.That(opposite, Is.EqualTo(0), "triangle normals must agree with the SDF gradient");
        }

        private static Vector3 BuildCubicCavityGradient(Vector3 point)
        {
            // Central differences of the analytic cavity SDF (the mesh is built from the EDT field;
            // the analytic gradient is an independent reference for the normal direction).
            const float h = 1e-3f;
            float dx = BoxDistance(point + new Vector3(h, 0, 0)) - BoxDistance(point - new Vector3(h, 0, 0));
            float dy = BoxDistance(point + new Vector3(0, h, 0)) - BoxDistance(point - new Vector3(0, h, 0));
            float dz = BoxDistance(point + new Vector3(0, 0, h)) - BoxDistance(point - new Vector3(0, 0, h));
            return new Vector3(dx, dy, dz);
        }

        private static float BoxDistance(Vector3 point)
        {
            // Signed distance to the cavity cube [-4,4]^3: positive inside the cavity (air),
            // negative in material. Matches the stock convention (negative = material).
            Vector3 q = new Vector3(
                MathF.Abs(point.X) - 4f,
                MathF.Abs(point.Y) - 4f,
                MathF.Abs(point.Z) - 4f);
            Vector3 outside = Vector3.Max(q, Vector3.Zero);
            float boxSdf = outside.Length() + MathF.Min(MathF.Max(q.X, MathF.Max(q.Y, q.Z)), 0f);
            return -boxSdf;
        }

        private static void AssertWatertight(Mesh mesh)
        {
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

            Assert.That(openEdges, Is.EqualTo(0), "mesh must be watertight");
            Assert.That(nonManifoldEdges, Is.EqualTo(0), "mesh must be manifold");
        }

        private static void AddEdge(Dictionary<(int, int), int> edges, int a, int b)
        {
            var key = a < b ? (a, b) : (b, a);
            edges.TryGetValue(key, out int count);
            edges[key] = count + 1;
        }
    }
}
