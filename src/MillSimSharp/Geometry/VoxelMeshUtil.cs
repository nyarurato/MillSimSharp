using System;
using System.Collections.Generic;
using System.Numerics;

namespace MillSimSharp.Geometry
{
    /// <summary>
    /// Shared helpers for voxel face extraction and vertex comparison.
    /// </summary>
    internal static class VoxelMeshUtil
    {
        /// <summary>
        /// Face normals indexed by face id: -X, +X, -Y, +Y, -Z, +Z.
        /// </summary>
        public static readonly Vector3[] FaceNormals =
        {
            new Vector3(-1, 0, 0),
            new Vector3(1, 0, 0),
            new Vector3(0, -1, 0),
            new Vector3(0, 1, 0),
            new Vector3(0, 0, -1),
            new Vector3(0, 0, 1)
        };

        /// <summary>
        /// Emits a single axis-aligned face of a voxel as two triangles.
        /// </summary>
        public static void EmitFace(int faceIndex, Vector3 center, float half,
            Func<Vector3, Vector3, int> addVertex, List<int> indices)
        {
            Vector3 n = FaceNormals[faceIndex];
            Vector3[] face;

            switch (faceIndex)
            {
                case 0: // -X
                    face = new Vector3[4]
                    {
                        center + new Vector3(-half, -half, -half),
                        center + new Vector3(-half, -half, +half),
                        center + new Vector3(-half, +half, +half),
                        center + new Vector3(-half, +half, -half)
                    };
                    break;
                case 1: // +X
                    face = new Vector3[4]
                    {
                        center + new Vector3(+half, -half, -half),
                        center + new Vector3(+half, +half, -half),
                        center + new Vector3(+half, +half, +half),
                        center + new Vector3(+half, -half, +half)
                    };
                    break;
                case 2: // -Y
                    face = new Vector3[4]
                    {
                        center + new Vector3(-half, -half, -half),
                        center + new Vector3(+half, -half, -half),
                        center + new Vector3(+half, -half, +half),
                        center + new Vector3(-half, -half, +half)
                    };
                    break;
                case 3: // +Y
                    face = new Vector3[4]
                    {
                        center + new Vector3(-half, +half, -half),
                        center + new Vector3(-half, +half, +half),
                        center + new Vector3(+half, +half, +half),
                        center + new Vector3(+half, +half, -half)
                    };
                    break;
                case 4: // -Z
                    face = new Vector3[4]
                    {
                        center + new Vector3(-half, -half, -half),
                        center + new Vector3(-half, +half, -half),
                        center + new Vector3(+half, +half, -half),
                        center + new Vector3(+half, -half, -half)
                    };
                    break;
                default: // +Z
                    face = new Vector3[4]
                    {
                        center + new Vector3(-half, -half, +half),
                        center + new Vector3(+half, -half, +half),
                        center + new Vector3(+half, +half, +half),
                        center + new Vector3(-half, +half, +half)
                    };
                    break;
            }

            AddQuad(face, n, addVertex, indices);
        }

        /// <summary>
        /// Adds two triangles for a quad.
        /// </summary>
        public static void AddQuad(Vector3[] vertices, Vector3 normal,
            Func<Vector3, Vector3, int> addVertex, List<int> indices)
        {
            int i0 = addVertex(vertices[0], normal);
            int i1 = addVertex(vertices[1], normal);
            int i2 = addVertex(vertices[2], normal);
            int i3 = addVertex(vertices[3], normal);

            indices.Add(i0);
            indices.Add(i1);
            indices.Add(i2);

            indices.Add(i0);
            indices.Add(i2);
            indices.Add(i3);
        }
    }

    /// <summary>
    /// Position comparer used to merge mesh vertices (epsilon = 1e-3).
    /// </summary>
    internal sealed class VoxelVertexComparer : IEqualityComparer<Vector3>
    {
        public const float Epsilon = 1e-3f;

        public bool Equals(Vector3 a, Vector3 b)
        {
            return Math.Abs(a.X - b.X) < Epsilon &&
                   Math.Abs(a.Y - b.Y) < Epsilon &&
                   Math.Abs(a.Z - b.Z) < Epsilon;
        }

        public int GetHashCode(Vector3 v)
        {
            return HashCode.Combine(
                (int)(v.X / Epsilon),
                (int)(v.Y / Epsilon),
                (int)(v.Z / Epsilon)
            );
        }
    }
}
