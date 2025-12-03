using OpenTK.Graphics.OpenGL4;
using OpenTK.Mathematics;
using System;

namespace MillSimSharp.Viewer.Rendering
{
    /// <summary>
    /// Renders XYZ coordinate axes.
    /// </summary>
    public class AxisRenderer : IDisposable
    {
        private int _vao;
        private int _vbo;
        private Shader _shader;

        // X (Red), Y (Green), Z (Blue)
        private readonly float[] _vertices =
        {
            // Position (x, y, z)    // Color (r, g, b)
            0.0f, 0.0f, 0.0f,        1.0f, 0.0f, 0.0f, // Origin (Red)
            100.0f, 0.0f, 0.0f,      1.0f, 0.0f, 0.0f, // X-Axis end

            0.0f, 0.0f, 0.0f,        0.0f, 1.0f, 0.0f, // Origin (Green)
            0.0f, 100.0f, 0.0f,      0.0f, 1.0f, 0.0f, // Y-Axis end

            0.0f, 0.0f, 0.0f,        0.0f, 0.0f, 1.0f, // Origin (Blue)
            0.0f, 0.0f, 100.0f,      0.0f, 0.0f, 1.0f  // Z-Axis end
        };

        public AxisRenderer()
        {
            string baseDir = AppDomain.CurrentDomain.BaseDirectory;
            _shader = new Shader(
                System.IO.Path.Combine(baseDir, "Shaders/line.vert"),
                System.IO.Path.Combine(baseDir, "Shaders/line.frag")
            );
            InitializeBuffers();
        }

        private void InitializeBuffers()
        {
            _vao = GL.GenVertexArray();
            GL.BindVertexArray(_vao);

            _vbo = GL.GenBuffer();
            GL.BindBuffer(BufferTarget.ArrayBuffer, _vbo);
            GL.BufferData(BufferTarget.ArrayBuffer, _vertices.Length * sizeof(float), _vertices, BufferUsageHint.StaticDraw);

            // Position attribute
            GL.VertexAttribPointer(0, 3, VertexAttribPointerType.Float, false, 6 * sizeof(float), 0);
            GL.EnableVertexAttribArray(0);

            // Color attribute
            GL.VertexAttribPointer(1, 3, VertexAttribPointerType.Float, false, 6 * sizeof(float), 3 * sizeof(float));
            GL.EnableVertexAttribArray(1);

            GL.BindVertexArray(0);
        }

        public void Render(Matrix4 view, Matrix4 projection)
        {
            _shader.Use();
            _shader.SetMatrix4("uView", view);
            _shader.SetMatrix4("uProjection", projection);

            GL.BindVertexArray(_vao);
            GL.DrawArrays(PrimitiveType.Lines, 0, 6);
            GL.BindVertexArray(0);
        }

        public void Dispose()
        {
            GL.DeleteVertexArray(_vao);
            GL.DeleteBuffer(_vbo);
            _shader.Dispose();
        }
    }
}
