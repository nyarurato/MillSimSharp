using System;
using System.Globalization;
using System.IO;
using System.Text;
using MillSimSharp.Geometry;

namespace MillSimSharp.IO
{
    /// <summary>
    /// Exports meshes to ASCII PLY format.
    /// </summary>
    public static class PlyExporter
    {
        /// <summary>
        /// Exports a mesh to an ASCII PLY file.
        /// </summary>
        /// <param name="mesh">Mesh to export.</param>
        /// <param name="filePath">Output file path.</param>
        public static void Export(Mesh mesh, string filePath)
        {
            if (mesh == null) throw new ArgumentNullException(nameof(mesh));
            if (filePath == null) throw new ArgumentNullException(nameof(filePath));

            using var writer = new StreamWriter(filePath, false, new UTF8Encoding(false));
            Write(mesh, writer);
        }

        /// <summary>
        /// Writes a mesh as ASCII PLY text.
        /// </summary>
        /// <param name="mesh">Mesh to export.</param>
        /// <param name="writer">Target writer.</param>
        public static void Write(Mesh mesh, TextWriter writer)
        {
            if (mesh == null) throw new ArgumentNullException(nameof(mesh));
            if (writer == null) throw new ArgumentNullException(nameof(writer));

            var culture = CultureInfo.InvariantCulture;
            bool hasNormals = mesh.Normals.Length == mesh.Vertices.Length;
            int faceCount = mesh.Indices.Length / 3;

            writer.WriteLine("ply");
            writer.WriteLine("format ascii 1.0");
            writer.WriteLine("comment MillSimSharp PLY export");
            writer.WriteLine($"element vertex {mesh.Vertices.Length}");
            writer.WriteLine("property float x");
            writer.WriteLine("property float y");
            writer.WriteLine("property float z");
            if (hasNormals)
            {
                writer.WriteLine("property float nx");
                writer.WriteLine("property float ny");
                writer.WriteLine("property float nz");
            }
            writer.WriteLine($"element face {faceCount}");
            writer.WriteLine("property list uchar int vertex_indices");
            writer.WriteLine("end_header");

            for (int i = 0; i < mesh.Vertices.Length; i++)
            {
                var v = mesh.Vertices[i];
                if (hasNormals)
                {
                    var n = mesh.Normals[i];
                    writer.WriteLine(FormattableString.Invariant($"{v.X} {v.Y} {v.Z} {n.X} {n.Y} {n.Z}"));
                }
                else
                {
                    writer.WriteLine(FormattableString.Invariant($"{v.X} {v.Y} {v.Z}"));
                }
            }

            for (int i = 0; i + 2 < mesh.Indices.Length; i += 3)
            {
                writer.WriteLine($"3 {mesh.Indices[i]} {mesh.Indices[i + 1]} {mesh.Indices[i + 2]}");
            }
        }
    }
}
