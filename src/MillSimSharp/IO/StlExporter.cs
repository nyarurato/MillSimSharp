using System;
using System.Collections.Generic;
using System.IO;
using System.Numerics;
using MillSimSharp.Geometry;

namespace MillSimSharp.IO
{
    /// <summary>
    /// Exports voxel grids to STL mesh format using the Marching Cubes algorithm.
    /// </summary>
    public static class StlExporter
    {
        /// <summary>
        /// Exports a voxel grid to an STL file (binary format).
        /// </summary>
        /// <param name="grid">The voxel grid to export.</param>
        /// <param name="filePath">Output file path.</param>
        public static void Export(VoxelGrid grid, string filePath)
        {
            byte[] stlData = ExportToBytes(grid);
            File.WriteAllBytes(filePath, stlData);
        }

        /// <summary>
        /// Exports a voxel grid to STL binary data.
        /// </summary>
        /// <param name="grid">The voxel grid to export.</param>
        /// <returns>Binary STL data.</returns>
        public static byte[] ExportToBytes(VoxelGrid grid)
        {
            List<Triangle> triangles = GenerateTriangles(grid);

            using (MemoryStream stream = new MemoryStream())
            using (BinaryWriter writer = new BinaryWriter(stream))
            {
                // STL header (80 bytes)
                byte[] header = new byte[80];
                string headerText = "MillSimSharp Voxel Export";
                byte[] headerBytes = System.Text.Encoding.ASCII.GetBytes(headerText);
                Array.Copy(headerBytes, header, Math.Min(headerBytes.Length, 80));
                writer.Write(header);

                // Number of triangles
                writer.Write((uint)triangles.Count);

                // Write each triangle
                foreach (Triangle tri in triangles)
                {
                    // Normal vector
                    WriteVector3(writer, tri.Normal);

                    // Vertices
                    WriteVector3(writer, tri.V1);
                    WriteVector3(writer, tri.V2);
                    WriteVector3(writer, tri.V3);

                    // Attribute byte count (unused, set to 0)
                    writer.Write((ushort)0);
                }

                return stream.ToArray();
            }
        }

        private static void WriteVector3(BinaryWriter writer, Vector3 v)
        {
            writer.Write((float)v.X);
            writer.Write((float)v.Y);
            writer.Write((float)v.Z);
        }

        private static List<Triangle> GenerateTriangles(VoxelGrid grid)
        {
            List<Triangle> triangles = new List<Triangle>();
            var mesh = Geometry.MeshConverter.ConvertToMesh(grid);

            // Mesh was created by expanding triangles for each triangle; convert to Triangle list
            for (int i = 0; i < mesh.Indices.Length; i += 3)
            {
                Vector3 v1 = mesh.Vertices[mesh.Indices[i + 0]];
                Vector3 v2 = mesh.Vertices[mesh.Indices[i + 1]];
                Vector3 v3 = mesh.Vertices[mesh.Indices[i + 2]];
                Vector3 n = mesh.Normals[mesh.Indices[i + 0]];
                triangles.Add(new Triangle(v1, v2, v3, n));
            }
            return triangles;
        }
        

        private struct Triangle
        {
            public Vector3 V1;
            public Vector3 V2;
            public Vector3 V3;
            public Vector3 Normal;

            public Triangle(Vector3 v1, Vector3 v2, Vector3 v3, Vector3 normal)
            {
                V1 = v1;
                V2 = v2;
                V3 = v3;
                Normal = normal;
            }
        }
    }
}
