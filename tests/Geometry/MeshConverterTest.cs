using NUnit.Framework;
using MillSimSharp.Geometry;
using System.Numerics;

namespace MillSimSharp.Tests.Geometry
{
    [TestFixture]
    public class MeshConverterTest
    {
        [Test]
        public void ConvertToMesh_GeneratesTriangles()
        {
            var bbox = BoundingBox.FromCenterAndSize(Vector3.Zero, new Vector3(20, 20, 20));
            var grid = new VoxelGrid(bbox, resolution: 1.0f);
            // remove a sphere to create some geometry
            grid.RemoveVoxelsInSphere(Vector3.Zero, 8.0f);

            var mesh = MeshConverter.ConvertToMesh(grid);

            Assert.That(mesh, Is.Not.Null);
            Assert.That(mesh.Vertices.Length, Is.GreaterThan(0));
            Assert.That(mesh.Indices.Length % 3, Is.EqualTo(0));
                // Ensure mesh contains triangles at the outer boundary (workpiece shell)
                int boundaryCount = 0;
                var (sx, sy, sz) = grid.Dimensions;
                var eps = grid.Resolution * 1.25f;
                var min = grid.Bounds.Min;
                var max = grid.Bounds.Max;
                for (int i = 0; i < mesh.Indices.Length; i += 3)
                {
                    var v1 = mesh.Vertices[mesh.Indices[i]];
                    var v2 = mesh.Vertices[mesh.Indices[i + 1]];
                    var v3 = mesh.Vertices[mesh.Indices[i + 2]];
                    var centroid = (v1 + v2 + v3) / 3.0f;
                    if (Math.Abs(centroid.X - min.X) <= eps || Math.Abs(centroid.X - max.X) <= eps ||
                        Math.Abs(centroid.Y - min.Y) <= eps || Math.Abs(centroid.Y - max.Y) <= eps ||
                        Math.Abs(centroid.Z - min.Z) <= eps || Math.Abs(centroid.Z - max.Z) <= eps)
                    {
                        boundaryCount++;
                    }
                }
                Assert.That(boundaryCount, Is.GreaterThan(0));
        }
    }
}
