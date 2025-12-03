using System;
using System.Collections.Generic;
using OpenTK.Graphics.OpenGL4;
using OpenTK.Mathematics;
using MillSimSharp.Toolpath;
using System.Numerics;

namespace MillSimSharp.Viewer.Rendering
{
    public class ToolpathRenderer : IDisposable
    {
        private int _vao;
        private int _vbo;
        private int _vertexCount = 0;

        public ToolpathRenderer()
        {
            // Setup VAO/VBO
            _vao = GL.GenVertexArray();
            GL.BindVertexArray(_vao);

            _vbo = GL.GenBuffer();
            GL.BindBuffer(BufferTarget.ArrayBuffer, _vbo);
            // allocate zero initially
            GL.BufferData(BufferTarget.ArrayBuffer, IntPtr.Zero, IntPtr.Zero, BufferUsageHint.DynamicDraw);

            // layout 0: position (vec3)
            GL.EnableVertexAttribArray(0);
            GL.VertexAttribPointer(0, 3, VertexAttribPointerType.Float, false, 6 * sizeof(float), 0);
            // layout 1: color (vec3)
            GL.EnableVertexAttribArray(1);
            GL.VertexAttribPointer(1, 3, VertexAttribPointerType.Float, false, 6 * sizeof(float), 3 * sizeof(float));

            GL.BindVertexArray(0);
        }

        public void UpdateFromCommands(List<IToolpathCommand> commands, System.Numerics.Vector3 initialPosition)
        {
            if (commands == null)
            {
                _vertexCount = 0;
                return;
            }

            // Build vertices: each segment -> two vertices with color
            var data = new List<float>();
            var cur = initialPosition;
            foreach (var cmd in commands)
            {
                System.Numerics.Vector3 target;
                OpenTK.Mathematics.Vector3 color;
                if (cmd is G0Move g0)
                {
                    target = g0.Target;
                    color = new OpenTK.Mathematics.Vector3(1.0f, 1.0f, 0.0f); // yellow
                }
                else if (cmd is G1Move g1)
                {
                    target = g1.Target;
                    color = new OpenTK.Mathematics.Vector3(0.6f, 0.1f, 0.8f); // purple
                }
                else
                {
                    // we don't have a target for other command types
                    continue;
                }

                // add start vertex
                data.Add(cur.X); data.Add(cur.Y); data.Add(cur.Z);
                data.Add(color.X); data.Add(color.Y); data.Add(color.Z);

                // add end vertex
                data.Add(target.X); data.Add(target.Y); data.Add(target.Z);
                data.Add(color.X); data.Add(color.Y); data.Add(color.Z);

                cur = target;
            }

            _vertexCount = data.Count / 6;
            if (_vertexCount == 0)
                return;

            GL.BindBuffer(BufferTarget.ArrayBuffer, _vbo);
            GL.BufferData(BufferTarget.ArrayBuffer, data.Count * sizeof(float), data.ToArray(), BufferUsageHint.DynamicDraw);
            GL.BindBuffer(BufferTarget.ArrayBuffer, 0);
        }

        public void Render()
        {
            if (_vertexCount == 0)
                return;

            GL.BindVertexArray(_vao);
            GL.LineWidth(3.0f);
            GL.DrawArrays(PrimitiveType.Lines, 0, _vertexCount);
            GL.BindVertexArray(0);
        }

        public void Dispose()
        {
            GL.DeleteBuffer(_vbo);
            GL.DeleteVertexArray(_vao);
        }
    }
}
