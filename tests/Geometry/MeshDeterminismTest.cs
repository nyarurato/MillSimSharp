using System;
using System.Numerics;
using MillSimSharp.Geometry;
using NUnit.Framework;

namespace MillSimSharp.Tests.Geometry
{
    /// <summary>
    /// Determinism of mesh generation: the dual contouring pipeline uses parallel cell processing,
    /// so repeated meshing of the same field must produce byte-identical output.
    /// </summary>
    [TestFixture]
    public class MeshDeterminismTest
    {
        private static SDFGrid BuildSphereSdf(float resolution)
        {
            var bbox = BoundingBox.FromCenterAndSize(Vector3.Zero, new Vector3(24, 24, 24));
            var grid = new VoxelGrid(bbox, resolution);
            grid.RemoveVoxelsInSphere(Vector3.Zero, 5f);
            return SDFGrid.FromVoxelGrid(grid, narrowBandWidth: 12);
        }

        [TestCase(1.0f)]
        [TestCase(0.5f)]
        public void ConvertToMeshFromSDF_IsDeterministic(float resolution)
        {
            SDFGrid sdf = BuildSphereSdf(resolution);

            Mesh first = MeshConverter.ConvertToMeshFromSDF(sdf);
            Mesh second = MeshConverter.ConvertToMeshFromSDF(sdf);

            Assert.That(second.Vertices.Length, Is.EqualTo(first.Vertices.Length));
            Assert.That(second.Indices.Length, Is.EqualTo(first.Indices.Length));
            Assert.That(second.Normals.Length, Is.EqualTo(first.Normals.Length));

            for (int i = 0; i < first.Vertices.Length; i++)
            {
                Assert.That(second.Vertices[i].X, Is.EqualTo(first.Vertices[i].X), $"vertex {i} X");
                Assert.That(second.Vertices[i].Y, Is.EqualTo(first.Vertices[i].Y), $"vertex {i} Y");
                Assert.That(second.Vertices[i].Z, Is.EqualTo(first.Vertices[i].Z), $"vertex {i} Z");
            }

            for (int i = 0; i < first.Normals.Length; i++)
            {
                Assert.That(second.Normals[i].X, Is.EqualTo(first.Normals[i].X), $"normal {i} X");
                Assert.That(second.Normals[i].Y, Is.EqualTo(first.Normals[i].Y), $"normal {i} Y");
                Assert.That(second.Normals[i].Z, Is.EqualTo(first.Normals[i].Z), $"normal {i} Z");
            }

            for (int i = 0; i < first.Indices.Length; i++)
            {
                Assert.That(second.Indices[i], Is.EqualTo(first.Indices[i]), $"index {i}");
            }
        }
    }
}
