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
            var (sizeX, sizeY, sizeZ) = grid.Dimensions;

            // Process each cube in the voxel grid
            for (int z = 0; z < sizeZ - 1; z++)
            {
                for (int y = 0; y < sizeY - 1; y++)
                {
                    for (int x = 0; x < sizeX - 1; x++)
                    {
                        ProcessCube(grid, x, y, z, triangles);
                    }
                }
            }

            return triangles;
        }

        private static void ProcessCube(VoxelGrid grid, int x, int y, int z, List<Triangle> triangles)
        {
            // Get the 8 corner values of the cube
            bool[] corners = new bool[8];
            corners[0] = grid.GetVoxel(x, y, z);
            corners[1] = grid.GetVoxel(x + 1, y, z);
            corners[2] = grid.GetVoxel(x + 1, y, z + 1);
            corners[3] = grid.GetVoxel(x, y, z + 1);
            corners[4] = grid.GetVoxel(x, y + 1, z);
            corners[5] = grid.GetVoxel(x + 1, y + 1, z);
            corners[6] = grid.GetVoxel(x + 1, y + 1, z + 1);
            corners[7] = grid.GetVoxel(x, y + 1, z + 1);

            // Calculate cube index (0-255)
            int cubeIndex = 0;
            for (int i = 0; i < 8; i++)
            {
                if (corners[i])
                    cubeIndex |= (1 << i);
            }

            // Skip if cube is entirely inside or outside
            if (cubeIndex == 0 || cubeIndex == 255)
                return;

            // Get corner positions in world space
            Vector3[] cornerPositions = new Vector3[8];
            float res = grid.Resolution;
            Vector3 basePos = grid.Bounds.Min + new Vector3(x * res, y * res, z * res);

            cornerPositions[0] = basePos;
            cornerPositions[1] = basePos + new Vector3(res, 0, 0);
            cornerPositions[2] = basePos + new Vector3(res, 0, res);
            cornerPositions[3] = basePos + new Vector3(0, 0, res);
            cornerPositions[4] = basePos + new Vector3(0, res, 0);
            cornerPositions[5] = basePos + new Vector3(res, res, 0);
            cornerPositions[6] = basePos + new Vector3(res, res, res);
            cornerPositions[7] = basePos + new Vector3(0, res, res);

            // Calculate edge vertices
            Vector3[] edgeVertices = new Vector3[12];
            for (int i = 0; i < 12; i++)
            {
                int v1 = MarchingCubesTable.EdgeConnections[i, 0];
                int v2 = MarchingCubesTable.EdgeConnections[i, 1];

                // Linear interpolation between corners
                // Since we're dealing with binary voxels, midpoint is used
                edgeVertices[i] = (cornerPositions[v1] + cornerPositions[v2]) * 0.5f;
            }

            // Generate triangles from table
            for (int i = 0; MarchingCubesTable.TriangleTable[cubeIndex, i] != -1; i += 3)
            {
                int e1 = MarchingCubesTable.TriangleTable[cubeIndex, i];
                int e2 = MarchingCubesTable.TriangleTable[cubeIndex, i + 1];
                int e3 = MarchingCubesTable.TriangleTable[cubeIndex, i + 2];

                Vector3 v1 = edgeVertices[e1];
                Vector3 v2 = edgeVertices[e2];
                Vector3 v3 = edgeVertices[e3];

                // Calculate normal
                Vector3 edge1 = v2 - v1;
                Vector3 edge2 = v3 - v1;
                Vector3 normal = Vector3.Normalize(Vector3.Cross(edge1, edge2));

                triangles.Add(new Triangle(v1, v2, v3, normal));
            }
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
