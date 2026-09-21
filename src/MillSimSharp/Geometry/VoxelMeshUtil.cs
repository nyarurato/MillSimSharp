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
    /// Position comparer used to merge mesh vertices (quantization step 1e-3). Equality uses the
    /// same quantization as <see cref="GetHashCode"/>, so the <see cref="IEqualityComparer{T}"/>
    /// contract (Equals implies equal hash codes) is satisfied.
    /// </summary>
    internal sealed class VoxelVertexComparer : IEqualityComparer<Vector3>
    {
        public const float Epsilon = 1e-3f;

        public bool Equals(Vector3 a, Vector3 b)
        {
            return Quantize(a) == Quantize(b);
        }

        public int GetHashCode(Vector3 v)
        {
            return Quantize(v).GetHashCode();
        }

        private static (long X, long Y, long Z) Quantize(Vector3 v)
        {
            return (
                (long)MathF.Round(v.X / Epsilon),
                (long)MathF.Round(v.Y / Epsilon),
                (long)MathF.Round(v.Z / Epsilon));
        }
    }
}
