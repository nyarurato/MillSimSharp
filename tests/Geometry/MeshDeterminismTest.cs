using System;
using System.Numerics;
using MillSimSharp.Geometry;
using NUnit.Framework;

namespace MillSimSharp.Tests.Geometry
{
    /// <summary>
    /// Determinism of mesh generation: both the dual contouring pipeline and the voxel surface
    /// mesher process cells in parallel, so repeated meshing of the same field must produce
    /// byte-identical output.
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

        private static VoxelGrid BuildTexturedVoxelGrid(float resolution)
        {
            var bbox = BoundingBox.FromCenterAndSize(Vector3.Zero, new Vector3(16, 16, 16));
            var grid = new VoxelGrid(bbox, resolution);

            // Sphere pocket on one side.
            grid.RemoveVoxelsInSphere(new Vector3(-4, 0, 0), 3.0f);

            // Rectangular notch on the opposite side (explicit voxel writes).
            for (int z = -6; z <= -4; z++)
                for (int y = 0; y <= 4; y++)
                    for (int x = 2; x <= 5; x++)
                        grid.SetVoxelAtWorld(new Vector3(x, y, z), false);

            return grid;
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

        [TestCase(1.0f)]
        [TestCase(0.5f)]
        public void ConvertToMesh_VoxelGrid_IsDeterministic(float resolution)
        {
            var grid = BuildTexturedVoxelGrid(resolution);

            Mesh reference = MeshConverter.ConvertToMesh(grid);
            Assert.That(reference.Vertices.Length, Is.GreaterThan(0));

            for (int run = 0; run < 4; run++)
            {
                Mesh mesh = MeshConverter.ConvertToMesh(grid);

                Assert.That(mesh.Vertices, Is.EqualTo(reference.Vertices), $"vertices differ on run {run}");
                Assert.That(mesh.Normals, Is.EqualTo(reference.Normals), $"normals differ on run {run}");
                Assert.That(mesh.Indices, Is.EqualTo(reference.Indices), $"indices differ on run {run}");
            }
        }

        [Test]
        public void ConvertToMesh_VoxelGrid_ChangesWithMaterial()
        {
            var grid = BuildTexturedVoxelGrid(1.0f);
            Mesh baseline = MeshConverter.ConvertToMesh(grid);

            // Carve one surface voxel of the stock corner: the mesh must change.
            grid.SetVoxelAtWorld(new Vector3(7.5f, 7.5f, 7.5f), false);
            Mesh modified = MeshConverter.ConvertToMesh(grid);

            Assert.That(modified.Vertices, Is.Not.EqualTo(baseline.Vertices));
        }
    }
}
