using System;
using System.Collections.Generic;
using System.Numerics;
using MillSimSharp.Geometry;
using MillSimSharp.Simulation;
using MillSimSharp.Toolpath;

namespace MillSimSharp.Viewer.Rendering
{
    /// <summary>
    /// Builds a display-only triangle mesh for an end mill at a given tool pose.
    /// The tool is generated in tool-local coordinates (tip at origin, axis +Z) and transformed
    /// to world space. Ball end mills get a spherical tip; flat end mills get flat caps.
    /// </summary>
    internal static class ToolModelBuilder
    {
        private const int Segments = 32;
        private const int Rings = 16;

        public static Mesh BuildEndMill(Tool tool, System.Numerics.Vector3 tipPosition, ToolOrientation orientation)
        {
            float radius = tool.Diameter / 2f;
            float length = tool.Length;
            bool isBall = tool.Type == ToolType.Ball;

            Matrix4x4 rotation = orientation.GetRotationMatrix();

            System.Numerics.Vector3 ToWorld(System.Numerics.Vector3 local)
                => tipPosition + System.Numerics.Vector3.Transform(local, rotation);
            System.Numerics.Vector3 NormalToWorld(System.Numerics.Vector3 local)
                => System.Numerics.Vector3.Normalize(System.Numerics.Vector3.Transform(local, rotation));

            var vertices = new List<System.Numerics.Vector3>();
            var normals = new List<System.Numerics.Vector3>();
            var indices = new List<int>();

            float cylinderStart = isBall ? radius : 0f;
            float cylinderEnd = MathF.Max(length, cylinderStart);

            // Cylinder side
            int sideBase = vertices.Count;
            for (int i = 0; i <= Segments; i++)
            {
                float theta = i * 2f * MathF.PI / Segments;
                float cos = MathF.Cos(theta);
                float sin = MathF.Sin(theta);
                var n = new System.Numerics.Vector3(cos, sin, 0f);

                vertices.Add(ToWorld(new System.Numerics.Vector3(radius * cos, radius * sin, cylinderStart)));
                normals.Add(NormalToWorld(n));
                vertices.Add(ToWorld(new System.Numerics.Vector3(radius * cos, radius * sin, cylinderEnd)));
                normals.Add(NormalToWorld(n));
            }

            for (int i = 0; i < Segments; i++)
            {
                int a = sideBase + i * 2;
                int b = a + 1;
                int c = a + 2;
                int d = a + 3;
                indices.Add(a); indices.Add(c); indices.Add(d);
                indices.Add(a); indices.Add(d); indices.Add(b);
            }

            // Top cap (+Z)
            int topCenter = vertices.Count;
            vertices.Add(ToWorld(new System.Numerics.Vector3(0f, 0f, cylinderEnd)));
            normals.Add(NormalToWorld(System.Numerics.Vector3.UnitZ));
            int topRing = vertices.Count;
            for (int i = 0; i <= Segments; i++)
            {
                float theta = i * 2f * MathF.PI / Segments;
                float cos = MathF.Cos(theta);
                float sin = MathF.Sin(theta);
                vertices.Add(ToWorld(new System.Numerics.Vector3(radius * cos, radius * sin, cylinderEnd)));
                normals.Add(NormalToWorld(System.Numerics.Vector3.UnitZ));
            }
            for (int i = 0; i < Segments; i++)
            {
                indices.Add(topCenter);
                indices.Add(topRing + i);
                indices.Add(topRing + i + 1);
            }

            if (isBall)
            {
                // Spherical tip centered at z = radius (the physical tip is the sphere bottom)
                int sphereBase = vertices.Count;
                var center = new System.Numerics.Vector3(0f, 0f, radius);
                for (int ring = 0; ring <= Rings; ring++)
                {
                    float phi = MathF.PI * ring / Rings;
                    float cosPhi = MathF.Cos(phi);
                    float sinPhi = MathF.Sin(phi);
                    for (int seg = 0; seg <= Segments; seg++)
                    {
                        float theta = seg * 2f * MathF.PI / Segments;
                        float cosTheta = MathF.Cos(theta);
                        float sinTheta = MathF.Sin(theta);
                        var n = new System.Numerics.Vector3(sinPhi * cosTheta, sinPhi * sinTheta, cosPhi);
                        vertices.Add(ToWorld(center + radius * n));
                        normals.Add(NormalToWorld(n));
                    }
                }

                int stride = Segments + 1;
                for (int ring = 0; ring < Rings; ring++)
                {
                    for (int seg = 0; seg < Segments; seg++)
                    {
                        int a = sphereBase + ring * stride + seg;
                        int b = a + 1;
                        int c = a + stride;
                        int d = c + 1;
                        indices.Add(a); indices.Add(c); indices.Add(d);
                        indices.Add(a); indices.Add(d); indices.Add(b);
                    }
                }
            }
            else
            {
                // Bottom cap (-Z)
                int bottomCenter = vertices.Count;
                vertices.Add(ToWorld(System.Numerics.Vector3.Zero));
                normals.Add(NormalToWorld(-System.Numerics.Vector3.UnitZ));
                int bottomRing = vertices.Count;
                for (int i = 0; i <= Segments; i++)
                {
                    float theta = i * 2f * MathF.PI / Segments;
                    float cos = MathF.Cos(theta);
                    float sin = MathF.Sin(theta);
                    vertices.Add(ToWorld(new System.Numerics.Vector3(radius * cos, radius * sin, 0f)));
                    normals.Add(NormalToWorld(-System.Numerics.Vector3.UnitZ));
                }
                for (int i = 0; i < Segments; i++)
                {
                    indices.Add(bottomCenter);
                    indices.Add(bottomRing + i + 1);
                    indices.Add(bottomRing + i);
                }
            }

            return new Mesh
            {
                Vertices = vertices.ToArray(),
                Normals = normals.ToArray(),
                Indices = indices.ToArray()
            };
        }
    }
}
