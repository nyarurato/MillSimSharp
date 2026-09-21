using System;
using System.Numerics;

namespace MillSimSharp.Geometry
{
    /// <summary>
    /// Simple mesh class to hold vertices, normals, and triangle indices.
    /// </summary>
    public class Mesh
    {
        /// <summary>
        /// Array of vertex positions.
        /// </summary>
        public Vector3[] Vertices { get; set; } = Array.Empty<Vector3>();
        /// <summary>
        /// Array of vertex normals.
        /// </summary>
        public Vector3[] Normals { get; set; } = Array.Empty<Vector3>();
        /// <summary>
        /// Array of triangle vertex indices.
        /// </summary>
        public int[] Indices { get; set; } = Array.Empty<int>();

        /// <summary>
        /// Default constructor.
        /// </summary>
        public Mesh() { }
    }
}
