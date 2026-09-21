using System;
using System.Globalization;
using System.IO;
using System.Text;
using MillSimSharp.Geometry;

namespace MillSimSharp.IO
{
    /// <summary>
    /// Exports meshes to Wavefront OBJ format (ASCII).
    /// </summary>
    public static class ObjExporter
    {
        /// <summary>
        /// Exports a mesh to an OBJ file.
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
        /// Writes a mesh as OBJ text.
        /// </summary>
        /// <param name="mesh">Mesh to export.</param>
        /// <param name="writer">Target writer.</param>
        public static void Write(Mesh mesh, TextWriter writer)
        {
            if (mesh == null) throw new ArgumentNullException(nameof(mesh));
            if (writer == null) throw new ArgumentNullException(nameof(writer));

            var culture = CultureInfo.InvariantCulture;
            bool hasNormals = mesh.Normals.Length == mesh.Vertices.Length;

            writer.WriteLine("# MillSimSharp OBJ export");

            foreach (var v in mesh.Vertices)
            {
                writer.WriteLine(FormattableString.Invariant($"v {v.X} {v.Y} {v.Z}"));
            }

            if (hasNormals)
            {
                foreach (var n in mesh.Normals)
                {
                    writer.WriteLine(FormattableString.Invariant($"vn {n.X} {n.Y} {n.Z}"));
                }
            }

            for (int i = 0; i + 2 < mesh.Indices.Length; i += 3)
            {
                int a = mesh.Indices[i] + 1;
                int b = mesh.Indices[i + 1] + 1;
                int c = mesh.Indices[i + 2] + 1;

                if (hasNormals)
                    writer.WriteLine($"f {a}//{a} {b}//{b} {c}//{c}");
                else
                    writer.WriteLine($"f {a} {b} {c}");
            }
        }
    }
}
