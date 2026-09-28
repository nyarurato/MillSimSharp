using System;
using System.Collections.Generic;
using System.IO;
using System.Numerics;
using MillSimSharp.Geometry;
using MillSimSharp.IO;
using NUnit.Framework;

namespace MillSimSharp.Tests.IO
{
    /// <summary>
    /// Tests for OBJ / PLY / ASCII STL export.
    /// </summary>
    [TestFixture]
    public class MeshExportTest
    {
        private static Mesh CreateTriangleMesh()
        {
            return new Mesh
            {
                Vertices = new[]
                {
                    new Vector3(0, 0, 0),
                    new Vector3(1, 0, 0),
                    new Vector3(0, 1, 0)
                },
                Normals = new[]
                {
                    Vector3.UnitZ,
                    Vector3.UnitZ,
                    Vector3.UnitZ
                },
                Indices = new[] { 0, 1, 2 }
            };
        }

        /// <summary>
        /// Two triangles whose vertex normals deliberately differ from the geometric facet normals
        /// (the YZ triangle stores UnitY while its geometric normal is UnitX).
        /// </summary>
        private static Mesh CreateTwoTriangleMesh()
        {
            return new Mesh
            {
                Vertices = new[]
                {
                    new Vector3(0, 0, 0),
                    new Vector3(2, 0, 0),
                    new Vector3(0, 3, 0),
                    new Vector3(0, 0, 0),
                    new Vector3(0, 2, 0),
                    new Vector3(0, 0, 3)
                },
                Normals = new[]
                {
                    Vector3.UnitZ,
                    Vector3.UnitZ,
                    Vector3.UnitZ,
                    Vector3.UnitY,
                    Vector3.UnitY,
                    Vector3.UnitY
                },
                Indices = new[] { 0, 1, 2, 3, 4, 5 }
            };
        }

        private static string TempFile(string extension)
        {
            return Path.Combine(Path.GetTempPath(), $"millsim_export_{Guid.NewGuid():N}{extension}");
        }

        private static List<(Vector3 Normal, Vector3 V1, Vector3 V2, Vector3 V3)> ReadBinaryStl(byte[] data)
        {
            var triangles = new List<(Vector3 Normal, Vector3 V1, Vector3 V2, Vector3 V3)>();
            using var stream = new MemoryStream(data);
            using var reader = new BinaryReader(stream);

            reader.ReadBytes(80); // header
            uint triangleCount = reader.ReadUInt32();
            for (uint i = 0; i < triangleCount; i++)
            {
                Vector3 normal = ReadVector3(reader);
                Vector3 v1 = ReadVector3(reader);
                Vector3 v2 = ReadVector3(reader);
                Vector3 v3 = ReadVector3(reader);
                reader.ReadUInt16(); // attribute byte count
                triangles.Add((normal, v1, v2, v3));
            }

            return triangles;
        }

        private static Vector3 ReadVector3(BinaryReader reader)
        {
            return new Vector3(reader.ReadSingle(), reader.ReadSingle(), reader.ReadSingle());
        }

        private static Vector3 GeometricNormal(Vector3 v1, Vector3 v2, Vector3 v3)
        {
            Vector3 normal = Vector3.Cross(v2 - v1, v3 - v1);
            float length = normal.Length();
            return length > 0f ? normal / length : Vector3.Zero;
        }

        private static void AssertVectorsEqual(Vector3 expected, Vector3 actual, float tolerance)
        {
            Assert.That(actual.X, Is.EqualTo(expected.X).Within(tolerance));
            Assert.That(actual.Y, Is.EqualTo(expected.Y).Within(tolerance));
            Assert.That(actual.Z, Is.EqualTo(expected.Z).Within(tolerance));
        }

        [Test]
        public void Obj_Export_WritesVerticesAndFaces()
        {
            var mesh = CreateTriangleMesh();
            string path = TempFile(".obj");

            try
            {
                ObjExporter.Export(mesh, path);
                string text = File.ReadAllText(path);

                Assert.That(text, Does.Contain("v 0 0 0"));
                Assert.That(text, Does.Contain("vn 0 0 1"));
                Assert.That(text, Does.Contain("f 1//1 2//2 3//3"));
            }
            finally
            {
                if (File.Exists(path)) File.Delete(path);
            }
        }

        [Test]
        public void Ply_Export_WritesHeaderAndFace()
        {
            var mesh = CreateTriangleMesh();
            string path = TempFile(".ply");

            try
            {
                PlyExporter.Export(mesh, path);
                string text = File.ReadAllText(path);

                Assert.That(text, Does.Contain("format ascii 1.0"));
                Assert.That(text, Does.Contain("element vertex 3"));
                Assert.That(text, Does.Contain("element face 1"));
                Assert.That(text, Does.Contain("3 0 1 2"));
            }
            finally
            {
                if (File.Exists(path)) File.Delete(path);
            }
        }

        [Test]
        public void AsciiStl_Export_WritesFacets()
        {
            var mesh = CreateTriangleMesh();
            string path = TempFile(".ascii.stl");

            try
            {
                StlExporter.ExportAscii(mesh, path);
                string text = File.ReadAllText(path);

                Assert.That(text, Does.Contain("solid MillSimSharp"));
                Assert.That(text, Does.Contain("facet normal 0 0 1"));
                Assert.That(text, Does.Contain("endsolid MillSimSharp"));
            }
            finally
            {
                if (File.Exists(path)) File.Delete(path);
            }
        }

        [Test]
        public void StlExport_FacetNormal_MatchesTriangleGeometry()
        {
            var triangles = ReadBinaryStl(StlExporter.ExportToBytes(CreateTwoTriangleMesh()));

            Assert.That(triangles, Has.Count.EqualTo(2));
            AssertVectorsEqual(Vector3.UnitZ, triangles[0].Normal, 1e-6f);
            AssertVectorsEqual(Vector3.UnitX, triangles[1].Normal, 1e-6f);

            foreach (var triangle in triangles)
            {
                Vector3 geometric = GeometricNormal(triangle.V1, triangle.V2, triangle.V3);
                Assert.That(Vector3.Dot(triangle.Normal, geometric), Is.GreaterThan(0.999f),
                    "the facet normal must match the triangle geometry, not the vertex normal");
            }
        }

        [Test]
        public void StlExport_VoxelMesh_FacetNormalsMatchGeometry()
        {
            var bounds = BoundingBox.FromCenterAndSize(Vector3.Zero, new Vector3(8, 8, 8));
            var grid = new VoxelGrid(bounds, 1.0f);

            var triangles = ReadBinaryStl(StlExporter.ExportToBytes(grid));

            Assert.That(triangles.Count, Is.GreaterThan(100), "the box shell must produce many facets");
            foreach (var triangle in triangles)
            {
                Vector3 geometric = GeometricNormal(triangle.V1, triangle.V2, triangle.V3);

                // Signed: an inverted facet normal must fail, not just a normal with the wrong magnitude.
                Assert.That(Vector3.Dot(triangle.Normal, geometric), Is.GreaterThan(0.999f),
                    "every voxel-mesh facet normal must match its triangle geometry and winding");
            }
        }

        [Test]
        public void StlExport_DegenerateTriangle_WritesZeroNormal()
        {
            var mesh = new Mesh
            {
                Vertices = new[]
                {
                    new Vector3(0, 0, 0),
                    new Vector3(1, 0, 0),
                    new Vector3(2, 0, 0) // collinear vertices: zero-area triangle
                },
                Normals = new[]
                {
                    Vector3.UnitZ,
                    Vector3.UnitZ,
                    Vector3.UnitZ
                },
                Indices = new[] { 0, 1, 2 }
            };

            var triangles = ReadBinaryStl(StlExporter.ExportToBytes(mesh));

            Assert.That(triangles, Has.Count.EqualTo(1), "a degenerate triangle must still be written");
            Assert.That(triangles[0].Normal, Is.EqualTo(Vector3.Zero));
        }

        [Test]
        public void StlExport_SameMeshTwice_IsByteIdentical()
        {
            var mesh = CreateTwoTriangleMesh();

            byte[] first = StlExporter.ExportToBytes(mesh);
            byte[] second = StlExporter.ExportToBytes(mesh);

            Assert.That(second, Is.EqualTo(first));
        }
    }
}
