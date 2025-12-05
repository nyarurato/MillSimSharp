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
        public Vector3[] Vertices { get; set; }
        /// <summary>
        /// Array of vertex normals.
        /// </summary>
        public Vector3[] Normals { get; set; }
        /// <summary>
        /// Array of triangle vertex indices.
        /// </summary>
        public int[] Indices { get; set; }

        /// <summary>
        /// Default constructor.
        /// </summary>
        public Mesh() { }
    }
}
