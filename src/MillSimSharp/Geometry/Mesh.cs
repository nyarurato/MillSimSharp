using System.Numerics;

namespace MillSimSharp.Geometry
{
    public class Mesh
    {
        public Vector3[] Vertices { get; set; }
        public Vector3[] Normals { get; set; }
        public int[] Indices { get; set; }

        public Mesh() { }
    }
}
