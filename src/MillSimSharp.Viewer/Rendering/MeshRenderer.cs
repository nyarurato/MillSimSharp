using System;
using System.Collections.Generic;
using OpenTK.Graphics.OpenGL4;
using OpenTK.Mathematics;
using MillSimSharp.Geometry;

namespace MillSimSharp.Viewer.Rendering
{
    public class MeshRenderer : IDisposable
    {
        private int _vao;
        private int _vbo;
        private int _ebo;
        private int _indexCount = 0;

        public MeshRenderer()
        {
            _vao = GL.GenVertexArray();
            GL.BindVertexArray(_vao);

            _vbo = GL.GenBuffer();
            _ebo = GL.GenBuffer();

            // Set up attribute locations
            GL.BindVertexArray(0);
        }

        public void UpdateMesh(Mesh mesh)
        {
            if (mesh == null || mesh.Vertices == null || mesh.Indices == null)
            {
                _indexCount = 0;
                return;
            }

            _indexCount = mesh.Indices.Length;

            // Interleave position and normal
            float[] data = new float[mesh.Vertices.Length * 6];
            for (int i = 0; i < mesh.Vertices.Length; i++)
            {
                data[i * 6 + 0] = mesh.Vertices[i].X;
                data[i * 6 + 1] = mesh.Vertices[i].Y;
                data[i * 6 + 2] = mesh.Vertices[i].Z;
                data[i * 6 + 3] = mesh.Normals[i].X;
                data[i * 6 + 4] = mesh.Normals[i].Y;
                data[i * 6 + 5] = mesh.Normals[i].Z;
            }

            GL.BindVertexArray(_vao);
            GL.BindBuffer(BufferTarget.ArrayBuffer, _vbo);
            GL.BufferData(BufferTarget.ArrayBuffer, data.Length * sizeof(float), data, BufferUsageHint.DynamicDraw);

            GL.BindBuffer(BufferTarget.ElementArrayBuffer, _ebo);
            GL.BufferData(BufferTarget.ElementArrayBuffer, mesh.Indices.Length * sizeof(int), mesh.Indices, BufferUsageHint.DynamicDraw);

            // Position attribute location 0
            GL.VertexAttribPointer(0, 3, VertexAttribPointerType.Float, false, 6 * sizeof(float), 0);
            GL.EnableVertexAttribArray(0);
            // Normal attribute location 1
            GL.VertexAttribPointer(1, 3, VertexAttribPointerType.Float, false, 6 * sizeof(float), 3 * sizeof(float));
            GL.EnableVertexAttribArray(1);

            GL.BindVertexArray(0);
        }

        public void Render()
        {
            if (_indexCount == 0)
                return;

            // We assume culling is enabled by default and triangle winding is outward-facing.

            GL.BindVertexArray(_vao);
            GL.DrawElements(PrimitiveType.Triangles, _indexCount, DrawElementsType.UnsignedInt, IntPtr.Zero);
            GL.BindVertexArray(0);

            // No need to toggle culling; respect global culling state.
        }

        public void Dispose()
        {
            GL.DeleteBuffer(_vbo);
            GL.DeleteBuffer(_ebo);
            GL.DeleteVertexArray(_vao);
        }
    }
}
