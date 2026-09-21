using NUnit.Framework;
using MillSimSharp.Geometry;
using System.Numerics;
using System.Diagnostics;
using System;

namespace MillSimSharp.Tests.Performance
{
    [TestFixture]
    public class VoxelGridPerformanceTest
    {
        private const long MaxMilliseconds = 10_000;

        [Test]
        [Category("Performance")]
        public void BenchmarkRemoveVoxelsInSphere_Small()
        {
            // 20x20x20mm grid, 1mm resolution, 2mm radius sphere
            var bbox = BoundingBox.FromCenterAndSize(Vector3.Zero, new Vector3(20, 20, 20));
            var grid = new VoxelGrid(bbox, 1.0f);

            var sw = Stopwatch.StartNew();
            grid.RemoveVoxelsInSphere(Vector3.Zero, 2.0f);
            sw.Stop();

            Console.WriteLine($"Execution time: {sw.ElapsedMilliseconds}ms");
            Assert.That(sw.ElapsedMilliseconds, Is.LessThan(MaxMilliseconds),
                $"Execution time must stay below {MaxMilliseconds}ms");
        }

        [Test]
        [Category("Performance")]
        public void BenchmarkRemoveVoxelsInSphere_Medium()
        {
            // 100x100x100mm grid, 1mm resolution, 10mm radius sphere
            var bbox = BoundingBox.FromCenterAndSize(Vector3.Zero, new Vector3(100, 100, 100));
            var grid = new VoxelGrid(bbox, 1.0f);

            var sw = Stopwatch.StartNew();
            grid.RemoveVoxelsInSphere(Vector3.Zero, 10.0f);
            sw.Stop();

            Console.WriteLine($"Medium Sphere: {sw.ElapsedMilliseconds}ms");
            Assert.That(sw.ElapsedMilliseconds, Is.LessThan(MaxMilliseconds),
                $"Execution time must stay below {MaxMilliseconds}ms");
        }

        [Test]
        [Category("Performance")]
        public void BenchmarkRemoveVoxelsInSphere_Large()
        {
            // 200x200x200mm grid, 2mm resolution, 20mm radius sphere
            var bbox = BoundingBox.FromCenterAndSize(Vector3.Zero, new Vector3(200, 200, 200));
            var grid = new VoxelGrid(bbox, 2.0f);

            var sw = Stopwatch.StartNew();
            grid.RemoveVoxelsInSphere(Vector3.Zero, 20.0f);
            sw.Stop();

            Console.WriteLine($"Large Sphere: {sw.ElapsedMilliseconds}ms");
            Assert.That(sw.ElapsedMilliseconds, Is.LessThan(MaxMilliseconds),
                $"Execution time must stay below {MaxMilliseconds}ms");
        }

        [Test]
        [Category("Performance")]
        public void BenchmarkRemoveVoxelsInCylinder_Medium()
        {
            // 100x100x100mm grid, 1mm resolution
            var bbox = BoundingBox.FromCenterAndSize(Vector3.Zero, new Vector3(100, 100, 100));
            var grid = new VoxelGrid(bbox, 1.0f);

            var start = new Vector3(-30, 0, 0);
            var end = new Vector3(30, 0, 0);

            var sw = Stopwatch.StartNew();
            grid.RemoveVoxelsInCylinder(start, end, 10.0f);
            sw.Stop();

            Console.WriteLine($"Medium Cylinder: {sw.ElapsedMilliseconds}ms");
            Assert.That(sw.ElapsedMilliseconds, Is.LessThan(MaxMilliseconds),
                $"Execution time must stay below {MaxMilliseconds}ms");
        }

        [Test]
        [Category("Performance")]
        public void BenchmarkMultipleCuts()
        {
            // Simulate realistic milling scenario
            var bbox = BoundingBox.FromCenterAndSize(Vector3.Zero, new Vector3(100, 100, 50));
            var grid = new VoxelGrid(bbox, 1.0f);

            var sw = Stopwatch.StartNew();

            // 10 passes
            for (int i = 0; i < 10; i++)
            {
                float y = -40 + i * 8;
                grid.RemoveVoxelsInCylinder(
                    new Vector3(-40, y, 0),
                    new Vector3(40, y, 0),
                    5.0f
                );
            }

            sw.Stop();

            Console.WriteLine($"10 Cylinder Cuts: {sw.ElapsedMilliseconds}ms");
            Assert.That(sw.ElapsedMilliseconds, Is.LessThan(MaxMilliseconds),
                $"Execution time must stay below {MaxMilliseconds}ms");
        }

        [Test]
        [Category("Performance")]
        public void BenchmarkSdfBuild_Medium()
        {
            // 50x50x50mm grid, 1mm resolution, sphere cut
            var bbox = BoundingBox.FromCenterAndSize(Vector3.Zero, new Vector3(50, 50, 50));
            var grid = new VoxelGrid(bbox, 1.0f);
            grid.RemoveVoxelsInSphere(Vector3.Zero, 10.0f);

            var sw = Stopwatch.StartNew();
            var sdf = SDFGrid.FromVoxelGrid(grid, narrowBandWidth: 5);
            sw.Stop();

            Console.WriteLine($"SDF build (50^3): {sw.ElapsedMilliseconds}ms");
            Assert.That(sdf, Is.Not.Null);
            Assert.That(sw.ElapsedMilliseconds, Is.LessThan(MaxMilliseconds),
                $"SDF build time must stay below {MaxMilliseconds}ms");
        }

        [Test]
        [Category("Performance")]
        public void BenchmarkSdfMesh_Medium()
        {
            var bbox = BoundingBox.FromCenterAndSize(Vector3.Zero, new Vector3(30, 30, 30));
            var grid = new VoxelGrid(bbox, 1.0f);
            grid.RemoveVoxelsInSphere(Vector3.Zero, 8.0f);
            var sdf = SDFGrid.FromVoxelGrid(grid, narrowBandWidth: 5);

            var sw = Stopwatch.StartNew();
            var mesh = MeshConverter.ConvertToMeshFromSDF(sdf);
            sw.Stop();

            Console.WriteLine($"SDF mesh (30^3): {sw.ElapsedMilliseconds}ms, triangles={mesh.Indices.Length / 3}");
            Assert.That(mesh.Indices.Length, Is.GreaterThan(0));
            Assert.That(sw.ElapsedMilliseconds, Is.LessThan(MaxMilliseconds),
                $"SDF mesh time must stay below {MaxMilliseconds}ms");
        }
    }
}
