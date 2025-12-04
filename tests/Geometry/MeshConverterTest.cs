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

        [Test]
        public void ConvertToMeshViaSDF_GeneratesTriangles()
        {
            var bbox = BoundingBox.FromCenterAndSize(Vector3.Zero, new Vector3(20, 20, 20));
            var grid = new VoxelGrid(bbox, resolution: 1.0f);
            grid.RemoveVoxelsInSphere(Vector3.Zero, 8.0f);

            var mesh = MeshConverter.ConvertToMeshViaSDF(grid, narrowBandWidth: 10, fastMode: true);

            Assert.That(mesh, Is.Not.Null);
            Assert.That(mesh.Vertices.Length, Is.GreaterThan(0));
            Assert.That(mesh.Indices.Length % 3, Is.EqualTo(0));
            Assert.That(mesh.Normals.Length, Is.EqualTo(mesh.Vertices.Length));

            int triangleCount = mesh.Indices.Length / 3;
            Assert.That(triangleCount, Is.GreaterThan(50)); // Expect a reasonable number of triangles
        }

        [Test]
        public void ConvertToMeshFromSDF_TriangleWindingMatchesGradient()
        {
            var bbox = BoundingBox.FromCenterAndSize(Vector3.Zero, new Vector3(20, 20, 20));
            var grid = new VoxelGrid(bbox, resolution: 1.0f);
            grid.RemoveVoxelsInSphere(Vector3.Zero, 8.0f);

            var sdf = SDFGrid.FromVoxelGrid(grid, fastMode: true);
            var mesh = MeshConverter.ConvertToMeshFromSDF(sdf);

            Assert.That(mesh.Vertices.Length, Is.GreaterThan(0));
            Assert.That(mesh.Indices.Length % 3, Is.EqualTo(0));

            for (int ti = 0; ti < mesh.Indices.Length; ti += 3)
            {
                var i0 = mesh.Indices[ti + 0];
                var i1 = mesh.Indices[ti + 1];
                var i2 = mesh.Indices[ti + 2];

                var v0 = mesh.Vertices[i0];
                var v1 = mesh.Vertices[i1];
                var v2 = mesh.Vertices[i2];

                var n0 = mesh.Normals[i0];
                var n1 = mesh.Normals[i1];
                var n2 = mesh.Normals[i2];

                var faceNormal = Vector3.Normalize(Vector3.Cross(v1 - v0, v2 - v0));
                var avgGradient = Vector3.Normalize((n0 + n1 + n2) / 3.0f);

                Assert.That(Vector3.Dot(faceNormal, avgGradient), Is.GreaterThan(0.0f - 1e-3f));
            }
        }

        [Test]
        public void ConvertToMeshFromSDF_VertexMergingDiagnostics()
        {
            var bbox = BoundingBox.FromCenterAndSize(Vector3.Zero, new Vector3(20, 20, 20));
            var grid = new VoxelGrid(bbox, resolution: 1.0f);
            grid.RemoveVoxelsInSphere(Vector3.Zero, 8.0f);

            var sdf = SDFGrid.FromVoxelGrid(grid, fastMode: true);
            var mesh = MeshConverter.ConvertToMeshFromSDF(sdf);

            int totalVerts = mesh.Vertices.Length;

            // Compute unique positions with two tolerances
            float eps1 = 1e-6f;
            float eps2 = 1e-3f;
            int uniq1 = 0, uniq2 = 0;
            var unique1 = new System.Collections.Generic.List<Vector3>();
            var unique2 = new System.Collections.Generic.List<Vector3>();

            bool AddUnique(List<Vector3> list, Vector3 v, float eps)
            {
                foreach (var u in list) if (Vector3.Distance(u, v) < eps) return false;
                list.Add(v);
                return true;
            }

            foreach (var v in mesh.Vertices)
            {
                if (AddUnique(unique1, v, eps1)) uniq1++;
                if (AddUnique(unique2, v, eps2)) uniq2++;
            }

            Console.WriteLine($"Vertex merging diagnostics: total={totalVerts}, uniq_epsilon1={uniq1}, uniq_epsilon2={uniq2}");
            Assert.That(uniq2, Is.LessThanOrEqualTo(totalVerts));

            // Analyze vertex index usage
            var indexCounts = new int[mesh.Vertices.Length];
            for (int i = 0; i < mesh.Indices.Length; i++)
            {
                int idx = mesh.Indices[i];
                indexCounts[idx]++;
            }
            int maxUsage = 0; int minUsage = int.MaxValue; double avgUsage = 0;
            for (int i = 0; i < indexCounts.Length; i++) { if (indexCounts[i] > maxUsage) maxUsage = indexCounts[i]; if (indexCounts[i] < minUsage) minUsage = indexCounts[i]; avgUsage += indexCounts[i]; }
            avgUsage /= indexCounts.Length;
            Console.WriteLine($"Index usage: min={minUsage}, max={maxUsage}, avg={avgUsage:F2}");
        }

        [Test]
        public void MeshDensity_VaryingResolution()
        {
            var bbox = BoundingBox.FromCenterAndSize(Vector3.Zero, new Vector3(20, 20, 20));
            var resList = new[] { 2.0f, 1.0f, 0.5f, 0.25f };
            foreach (var res in resList)
            {
                var grid = new VoxelGrid(bbox, resolution: res);
                grid.RemoveVoxelsInSphere(Vector3.Zero, 8.0f);
                var mesh = MeshConverter.ConvertToMeshViaSDF(grid, narrowBandWidth: 10, fastMode: true);
                Console.WriteLine($"res={res:F3}, vertices={mesh.Vertices.Length}, triangles={mesh.Indices.Length / 3}");
            }
            Assert.Pass();
        }

        [Test]
        public void ConvertToMeshFromSDF_IncludesOuterShell()
        {
            var bbox = BoundingBox.FromCenterAndSize(Vector3.Zero, new Vector3(20, 20, 20));
            var grid = new VoxelGrid(bbox, resolution: 1.0f);
            // No removals -> solid block. Expect outer shell to be present around bounds.

            var sdf = SDFGrid.FromVoxelGrid(grid, fastMode: true);
            var mesh = MeshConverter.ConvertToMeshFromSDF(sdf);

            Assert.That(mesh.Vertices.Length, Is.GreaterThan(0));

            bool hasBoundary = false;
            foreach (var v in mesh.Vertices)
            {
                if (Math.Abs(v.X - bbox.Min.X) < 0.75f || Math.Abs(v.X - bbox.Max.X) < 0.75f ||
                    Math.Abs(v.Y - bbox.Min.Y) < 0.75f || Math.Abs(v.Y - bbox.Max.Y) < 0.75f ||
                    Math.Abs(v.Z - bbox.Min.Z) < 0.75f || Math.Abs(v.Z - bbox.Max.Z) < 0.75f)
                {
                    hasBoundary = true;
                    break;
                }
            }
            Assert.That(hasBoundary, Is.True, "Expected SDF mesh to include outer boundary vertices near voxel bounds.");

            // Additionally ensure both min and max bounds are present for each axis on a small grid
            var smallBbox = BoundingBox.FromCenterAndSize(Vector3.Zero, new Vector3(10,10,10));
            var smallGrid = new VoxelGrid(smallBbox, resolution: 1.0f);
            var smallSdf = SDFGrid.FromVoxelGrid(smallGrid, fastMode: true);
            var smallMesh = MeshConverter.ConvertToMeshFromSDF(smallSdf);
            int minX=0, maxX=0, minY=0, maxY=0, minZ=0, maxZ=0;
            float tol = 1.25f; // tolerance for mesh being near the outer boundary in world units
            foreach (var v in smallMesh.Vertices)
            {
                if (Math.Abs(v.X - smallBbox.Min.X) < tol) minX++;
                if (Math.Abs(v.X - smallBbox.Max.X) < tol) maxX++;
                if (Math.Abs(v.Y - smallBbox.Min.Y) < tol) minY++;
                if (Math.Abs(v.Y - smallBbox.Max.Y) < tol) maxY++;
                if (Math.Abs(v.Z - smallBbox.Min.Z) < tol) minZ++;
                if (Math.Abs(v.Z - smallBbox.Max.Z) < tol) maxZ++;
            }
            
            Assert.That(minX, Is.GreaterThan(0));
            Assert.That(maxX, Is.GreaterThan(0));
            Assert.That(minY, Is.GreaterThan(0));
            Assert.That(maxY, Is.GreaterThan(0));
            Assert.That(minZ, Is.GreaterThan(0));
            Assert.That(maxZ, Is.GreaterThan(0));
        }

        [Test]
        public void ConvertToMeshFromSDF_CubeIndexCrossingsPerSlice()
        {
            var bbox = BoundingBox.FromCenterAndSize(Vector3.Zero, new Vector3(10, 10, 10));
            var grid = new VoxelGrid(bbox, resolution: 1.0f);
            var sdf = SDFGrid.FromVoxelGrid(grid, fastMode: true);

            var (sizeX, sizeY, sizeZ) = sdf.Dimensions;
            bool[] xCross = new bool[sizeX + 1]; // slices for base x=-1..sizeX-1

            for (int z = -1; z < sizeZ; z++)
            {
                for (int y = -1; y < sizeY; y++)
                {
                    for (int x = -1; x < sizeX; x++)
                    {
                        Vector3 basePos = sdf.Bounds.Min + new Vector3(x * sdf.Resolution,
                                                    y * sdf.Resolution,
                                                    z * sdf.Resolution);
                        float[] val = new float[8];
                        Vector3[] cornerPos = new Vector3[8];
                        cornerPos[0] = basePos + new Vector3(0,0,0);
                        cornerPos[1] = basePos + new Vector3(sdf.Resolution,0,0);
                        cornerPos[2] = basePos + new Vector3(sdf.Resolution,sdf.Resolution,0);
                        cornerPos[3] = basePos + new Vector3(0,sdf.Resolution,0);
                        cornerPos[4] = basePos + new Vector3(0,0,sdf.Resolution);
                        cornerPos[5] = basePos + new Vector3(sdf.Resolution,0,sdf.Resolution);
                        cornerPos[6] = basePos + new Vector3(sdf.Resolution,sdf.Resolution,sdf.Resolution);
                        cornerPos[7] = basePos + new Vector3(0,sdf.Resolution,sdf.Resolution);
                        for (int i = 0; i < 8; i++) val[i] = sdf.GetDistance(cornerPos[i]);
                        int cubeIndex = 0;
                        for (int i = 0; i < 8; i++) if (val[i] < 0) cubeIndex |= (1 << i);
                        if (cubeIndex != 0 && cubeIndex != 255)
                        {
                            xCross[x + 1] = true; // x+1 to map -1..sizeX-1 into 0..sizeX
                        }
                        // if this is the right-most slice, print cubeIndex for debugging
                        if (x == sizeX - 1 && y == 0 && z == 0)
                        {
                            Console.WriteLine($"Debug corner vals for base x={x},y={y},z={z} -> [{string.Join(",", val)}], cubeIndex={cubeIndex}");
                        }
                    }
                }
            }

            Assert.That(xCross[0], Is.True, "Expected crossing on left-most slice (x=-1)");
            Assert.That(xCross[xCross.Length - 1], Is.True, "Expected crossing on right-most slice (x=sizeX-1)");
        }

        // Debug SDF tests removed in favor of unit tests that validate functionality.
    }
}
