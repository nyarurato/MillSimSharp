using NUnit.Framework;
using MillSimSharp.Geometry;
using System.Numerics;
using System.Reflection;

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
            
            // Verify mesh has normals for each vertex
            Assert.That(mesh.Normals.Length, Is.EqualTo(mesh.Vertices.Length));
            
            // Verify we have triangles for the sphere surface
            int triangleCount = mesh.Indices.Length / 3;
            Assert.That(triangleCount, Is.GreaterThan(100)); // Marching cubes generates many triangles
        }
    }
}
