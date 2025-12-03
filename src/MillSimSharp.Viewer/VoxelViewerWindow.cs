using OpenTK.Graphics.OpenGL4;
using OpenTK.Mathematics;
using OpenTK.Windowing.Common;
using OpenTK.Windowing.Desktop;
using OpenTK.Windowing.GraphicsLibraryFramework;
using MillSimSharp.Geometry;
using MillSimSharp.Viewer.Rendering;
using System;
using System.IO;
using SysVector3 = System.Numerics.Vector3;

namespace MillSimSharp.Viewer
{
    public class VoxelViewerWindow : GameWindow
    {
        private VoxelRenderer? _renderer;
        private AxisRenderer? _axisRenderer;
        private Camera? _camera;
        private Shader? _voxelShader;
        private VoxelGrid? _voxelGrid;

        private Vector2 _lastMousePos;
        private bool _isMouseDragging;

        public VoxelViewerWindow(GameWindowSettings gameWindowSettings, NativeWindowSettings nativeWindowSettings)
            : base(gameWindowSettings, nativeWindowSettings)
        {
        }

        protected override void OnLoad()
        {
            base.OnLoad();

            // Set clear color
            GL.ClearColor(0.1f, 0.1f, 0.15f, 1.0f);

            // Enable depth testing
            GL.Enable(EnableCap.DepthTest);

            // Enable backface culling
            GL.Enable(EnableCap.CullFace);
            GL.CullFace(CullFaceMode.Back);

            // Load shaders
            string baseDir = AppDomain.CurrentDomain.BaseDirectory;
            _voxelShader = new Shader(
                Path.Combine(baseDir, "Shaders/voxel.vert"),
                Path.Combine(baseDir, "Shaders/voxel.frag")
            );

            // Initialize camera
            _camera = new Camera
            {
                Target = new OpenTK.Mathematics.Vector3(0, 0, 0),
                Distance = 150.0f,
                Yaw = 45.0f,
                Pitch = 30.0f
            };

            // Create demo voxel scene
            CreateDemoScene();

            // Initialize renderer
            _renderer = new VoxelRenderer();
            _axisRenderer = new AxisRenderer();
            if (_voxelGrid != null)
            {
                _renderer.UpdateVoxelData(_voxelGrid);
            }

            Console.WriteLine($"Voxel Viewer initialized");
            Console.WriteLine($"Voxels: {_voxelGrid?.CountMaterialVoxels() ?? 0}");
            Console.WriteLine($"Controls:");
            Console.WriteLine($"  - Mouse drag: Rotate camera");
            Console.WriteLine($"  - Mouse wheel: Zoom in/out");
            Console.WriteLine($"  - ESC: Exit");
        }

        private void CreateDemoScene()
        {
            // Create 100x100x100mm work area with 1mm resolution
            var bbox = BoundingBox.FromCenterAndSize(
                SysVector3.Zero,
                new SysVector3(100, 100, 100)
            );
            _voxelGrid = new VoxelGrid(bbox, resolution: 1.0f);

            // Remove a sphere in the center
            _voxelGrid.RemoveVoxelsInSphere(SysVector3.Zero, radius: 50.0f);

            // Remove some cylinders for visual interest
            _voxelGrid.RemoveVoxelsInCylinder(
                new SysVector3(-40, 0, 50),
                new SysVector3(40, 0, 50),
                radius: 10.0f
            );

            _voxelGrid.RemoveVoxelsInCylinder(
                new SysVector3(50, -40, 0),
                new SysVector3(50, 40, 0),
                radius: 10.0f
            );

            _voxelGrid.RemoveVoxelsInCylinder(
                new SysVector3(0, 50, -40),
                new SysVector3(0, 50, 40),
                radius: 10.0f
            );
        }

        protected override void OnRenderFrame(FrameEventArgs args)
        {
            base.OnRenderFrame(args);

            // Clear buffers
            GL.Clear(ClearBufferMask.ColorBufferBit | ClearBufferMask.DepthBufferBit);

            if (_voxelShader == null || _camera == null || _renderer == null)
                return;

            // Use shader
            _voxelShader.Use();

            // Set matrices
            Matrix4 view = _camera.GetViewMatrix();
            Matrix4 projection = Matrix4.CreatePerspectiveFieldOfView(
                MathHelper.DegreesToRadians(45.0f),
                (float)Size.X / Size.Y,
                0.1f,
                1000.0f
            );

            _voxelShader.SetMatrix4("uView", view);
            _voxelShader.SetMatrix4("uProjection", projection);
            _voxelShader.SetFloat("uVoxelSize", _voxelGrid?.Resolution ?? 1.0f);

            // Set light direction
            Vector3 lightDir = new Vector3(0.5f, 1.0f, 0.3f);
            lightDir.Normalize();
            _voxelShader.SetVector3("uLightDir", lightDir);

            // Render voxels
            _renderer.Render();

            // Render axes
            _axisRenderer?.Render(view, projection);

            // Swap buffers
            SwapBuffers();

            // Update FPS and Memory in title
            UpdateTitle(args.Time);
        }

        private double _timeSinceLastUpdate = 0.0;
        private int _frameCount = 0;

        private void UpdateTitle(double time)
        {
            _frameCount++;
            _timeSinceLastUpdate += time;

            if (_timeSinceLastUpdate >= 1.0)
            {
                double fps = _frameCount / _timeSinceLastUpdate;
                long memory = GC.GetTotalMemory(false) / (1024 * 1024); // MB

                Title = $"MillSimSharp Voxel Viewer - FPS: {fps:0.0} - Mem: {memory} MB - Voxels: {_voxelGrid?.CountMaterialVoxels() ?? 0}";

                _frameCount = 0;
                _timeSinceLastUpdate = 0.0;
            }
        }

        protected override void OnUpdateFrame(FrameEventArgs args)
        {
            base.OnUpdateFrame(args);

            // Handle ESC to close
            if (KeyboardState.IsKeyDown(Keys.Escape))
            {
                Close();
            }
        }

        protected override void OnResize(ResizeEventArgs e)
        {
            base.OnResize(e);
            GL.Viewport(0, 0, e.Width, e.Height);
        }

        protected override void OnMouseDown(MouseButtonEventArgs e)
        {
            base.OnMouseDown(e);

            if (e.Button == MouseButton.Left)
            {
                _isMouseDragging = true;
                _lastMousePos = MousePosition;
            }
        }

        protected override void OnMouseUp(MouseButtonEventArgs e)
        {
            base.OnMouseUp(e);

            if (e.Button == MouseButton.Left)
            {
                _isMouseDragging = false;
            }
        }

        protected override void OnMouseMove(MouseMoveEventArgs e)
        {
            base.OnMouseMove(e);

            if (_isMouseDragging && _camera != null)
            {
                Vector2 delta = MousePosition - _lastMousePos;
                _camera.ProcessMouseMove(delta.X, delta.Y);
                _lastMousePos = MousePosition;
            }
        }

        protected override void OnMouseWheel(MouseWheelEventArgs e)
        {
            base.OnMouseWheel(e);

            if (_camera != null)
            {
                _camera.ProcessMouseWheel(e.OffsetY);
            }
        }

        protected override void OnUnload()
        {
            base.OnUnload();

            _renderer?.Dispose();
            _axisRenderer?.Dispose();
            _voxelShader?.Dispose();
        }
    }
}
