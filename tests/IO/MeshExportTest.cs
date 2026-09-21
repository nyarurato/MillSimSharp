using System;
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

        private static string TempFile(string extension)
        {
            return Path.Combine(Path.GetTempPath(), $"millsim_export_{Guid.NewGuid():N}{extension}");
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
    }
}
