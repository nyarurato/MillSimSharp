using OpenTK.Graphics.OpenGL4;
using OpenTK.Mathematics;
using MillSimSharp.Geometry;
using System;
using System.Collections.Generic;

namespace MillSimSharp.Viewer.Rendering
{
    /// <summary>
    /// Renders voxel grids using instanced rendering.
    /// </summary>
    public class VoxelRenderer : IDisposable
    {
        private int _vao;
        private int _vbo;
        private int _ebo;
        private int _instanceVbo;
        private int _instanceCount;

        // Unit cube vertices (8 corners)
        private static readonly float[] CubeVertices = new float[]
        {
            // Positions (centered at origin, size 1)      // Normals
            // Front face (Z+)
            -0.5f, -0.5f,  0.5f,    0.0f,  0.0f,  1.0f,
             0.5f, -0.5f,  0.5f,    0.0f,  0.0f,  1.0f,
             0.5f,  0.5f,  0.5f,    0.0f,  0.0f,  1.0f,
            -0.5f,  0.5f,  0.5f,    0.0f,  0.0f,  1.0f,
            // Back face (Z-)
            -0.5f, -0.5f, -0.5f,    0.0f,  0.0f, -1.0f,
             0.5f, -0.5f, -0.5f,    0.0f,  0.0f, -1.0f,
             0.5f,  0.5f, -0.5f,    0.0f,  0.0f, -1.0f,
            -0.5f,  0.5f, -0.5f,    0.0f,  0.0f, -1.0f,
            // Top face (Y+)
            -0.5f,  0.5f, -0.5f,    0.0f,  1.0f,  0.0f,
            -0.5f,  0.5f,  0.5f,    0.0f,  1.0f,  0.0f,
             0.5f,  0.5f,  0.5f,    0.0f,  1.0f,  0.0f,
             0.5f,  0.5f, -0.5f,    0.0f,  1.0f,  0.0f,
            // Bottom face (Y-)
            -0.5f, -0.5f, -0.5f,    0.0f, -1.0f,  0.0f,
            -0.5f, -0.5f,  0.5f,    0.0f, -1.0f,  0.0f,
             0.5f, -0.5f,  0.5f,    0.0f, -1.0f,  0.0f,
             0.5f, -0.5f, -0.5f,    0.0f, -1.0f,  0.0f,
            // Right face (X+)
             0.5f, -0.5f, -0.5f,    1.0f,  0.0f,  0.0f,
             0.5f,  0.5f, -0.5f,    1.0f,  0.0f,  0.0f,
             0.5f,  0.5f,  0.5f,    1.0f,  0.0f,  0.0f,
             0.5f, -0.5f,  0.5f,    1.0f,  0.0f,  0.0f,
            // Left face (X-)
            -0.5f, -0.5f, -0.5f,   -1.0f,  0.0f,  0.0f,
            -0.5f,  0.5f, -0.5f,   -1.0f,  0.0f,  0.0f,
            -0.5f,  0.5f,  0.5f,   -1.0f,  0.0f,  0.0f,
            -0.5f, -0.5f,  0.5f,   -1.0f,  0.0f,  0.0f,
        };

        // Cube indices (12 triangles, 6 faces)
        private static readonly uint[] CubeIndices = new uint[]
        {
            // Front
            0, 1, 2, 2, 3, 0,
            // Back
            5, 4, 7, 7, 6, 5,
            // Top
            8, 9, 10, 10, 11, 8,
            // Bottom
            13, 12, 15, 15, 14, 13,
            // Right
            16, 17, 18, 18, 19, 16,
            // Left
            21, 20, 23, 23, 22, 21
        };

        public VoxelRenderer()
        {
            InitializeCube();
        }

        private void InitializeCube()
        {
            // Generate and bind VAO
            _vao = GL.GenVertexArray();
            GL.BindVertexArray(_vao);

            // Create VBO for cube vertices
            _vbo = GL.GenBuffer();
            GL.BindBuffer(BufferTarget.ArrayBuffer, _vbo);
            GL.BufferData(BufferTarget.ArrayBuffer, CubeVertices.Length * sizeof(float), 
                CubeVertices, BufferUsageHint.StaticDraw);

            // Position attribute (location 0)
            GL.VertexAttribPointer(0, 3, VertexAttribPointerType.Float, false, 6 * sizeof(float), 0);
            GL.EnableVertexAttribArray(0);

            // Normal attribute (location 1)
            GL.VertexAttribPointer(1, 3, VertexAttribPointerType.Float, false, 6 * sizeof(float), 3 * sizeof(float));
            GL.EnableVertexAttribArray(1);

            // Create EBO for indices
            _ebo = GL.GenBuffer();
            GL.BindBuffer(BufferTarget.ElementArrayBuffer, _ebo);
            GL.BufferData(BufferTarget.ElementArrayBuffer, CubeIndices.Length * sizeof(uint), 
                CubeIndices, BufferUsageHint.StaticDraw);

            // Create instance VBO (will be filled later)
            _instanceVbo = GL.GenBuffer();

            GL.BindVertexArray(0);
        }

        public void UpdateVoxelData(VoxelGrid grid)
        {
            var (sizeX, sizeY, sizeZ) = grid.Dimensions;
            List<float> instanceData = new List<float>();

            // Calculate min/max Z for color gradient
            float minZ = grid.Bounds.Min.Z;
            float maxZ = grid.Bounds.Max.Z;
            float zRange = maxZ - minZ;

            // Iterate through all voxels and collect material ones
            for (int z = 0; z < sizeZ; z++)
            {
                for (int y = 0; y < sizeY; y++)
                {
                    for (int x = 0; x < sizeX; x++)
                    {
                        if (grid.GetVoxel(x, y, z))
                        {
                            // Calculate world position (voxel center)
                            float worldX = grid.Bounds.Min.X + (x + 0.5f) * grid.Resolution;
                            float worldY = grid.Bounds.Min.Y + (y + 0.5f) * grid.Resolution;
                            float worldZ = grid.Bounds.Min.Z + (z + 0.5f) * grid.Resolution;

                            // Calculate color based on height (Z)
                            float t = (worldZ - minZ) / zRange;
                            Vector3 color = Vector3.Lerp(
                                new Vector3(0.2f, 0.4f, 1.0f), // Blue (bottom)
                                new Vector3(1.0f, 0.2f, 0.2f), // Red (top)
                                t
                            );

                            // Add instance data: position (3) + color (3)
                            instanceData.Add(worldX);
                            instanceData.Add(worldY);
                            instanceData.Add(worldZ);
                            instanceData.Add(color.X);
                            instanceData.Add(color.Y);
                            instanceData.Add(color.Z);
                        }
                    }
                }
            }

            _instanceCount = instanceData.Count / 6;

            if (_instanceCount > 0)
            {
                // Upload instance data to GPU
                GL.BindBuffer(BufferTarget.ArrayBuffer, _instanceVbo);
                GL.BufferData(BufferTarget.ArrayBuffer, instanceData.Count * sizeof(float), 
                    instanceData.ToArray(), BufferUsageHint.StaticDraw);

                GL.BindVertexArray(_vao);

                // Instance position attribute (location 2)
                GL.VertexAttribPointer(2, 3, VertexAttribPointerType.Float, false, 6 * sizeof(float), 0);
                GL.EnableVertexAttribArray(2);
                GL.VertexAttribDivisor(2, 1); // Advance once per instance

                // Instance color attribute (location 3)
                GL.VertexAttribPointer(3, 3, VertexAttribPointerType.Float, false, 6 * sizeof(float), 3 * sizeof(float));
                GL.EnableVertexAttribArray(3);
                GL.VertexAttribDivisor(3, 1); // Advance once per instance

                GL.BindVertexArray(0);
            }
        }

        public void Render()
        {
            if (_instanceCount == 0)
                return;

            GL.BindVertexArray(_vao);
            GL.DrawElementsInstanced(PrimitiveType.Triangles, CubeIndices.Length, 
                DrawElementsType.UnsignedInt, IntPtr.Zero, _instanceCount);
            GL.BindVertexArray(0);
        }

        public void Dispose()
        {
            GL.DeleteBuffer(_vbo);
            GL.DeleteBuffer(_ebo);
            GL.DeleteBuffer(_instanceVbo);
            GL.DeleteVertexArray(_vao);
        }
    }
}
