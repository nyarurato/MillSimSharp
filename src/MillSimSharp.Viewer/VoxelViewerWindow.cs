using OpenTK.Graphics.OpenGL4;
using OpenTK.Mathematics;
using OpenTK.Windowing.Common;
using OpenTK.Windowing.Desktop;
using OpenTK.Windowing.GraphicsLibraryFramework;
using MillSimSharp.Geometry;
using MillSimSharp.Viewer.Rendering;
using MillSimSharp.Simulation;
using MillSimSharp.Toolpath;
using System.Diagnostics;
using SysVector3 = System.Numerics.Vector3;

namespace MillSimSharp.Viewer
{
    public class VoxelViewerWindow : GameWindow
    {
        private VoxelRenderer? _renderer;
        private AxisRenderer? _axisRenderer;
        private Camera? _camera;
        private Shader? _voxelShader;
        private Shader? _lineShader;
        private MeshRenderer? _meshRenderer;
        private Shader? _meshShader;
        private VoxelGrid? _voxelGrid;
        private ToolpathRenderer? _toolpathRenderer;
        private List<IToolpathCommand>? _pendingToolpathCommands;
        private SysVector3 _pendingToolpathStartPos;

        private Vector2 _lastMousePos;
        private bool _isMouseDragging;

        public VoxelViewerWindow(GameWindowSettings gameWindowSettings, NativeWindowSettings nativeWindowSettings)
            : base(gameWindowSettings, nativeWindowSettings)
        {
        }

        protected override void OnLoad()
        {
            var stopwatch = new Stopwatch();
            stopwatch.Start();

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

            _lineShader = new Shader(
                Path.Combine(baseDir, "Shaders/line.vert"),
                Path.Combine(baseDir, "Shaders/line.frag")
            );

            _meshShader = new Shader(
                Path.Combine(baseDir, "Shaders/mesh.vert"),
                Path.Combine(baseDir, "Shaders/mesh.frag")
            );

            // Initialize camera
            _camera = new Camera
            {
                Target = new OpenTK.Mathematics.Vector3(0, 0, 0),
                Distance = 200.0f,
                Yaw = 45.0f,
                Pitch = 30.0f
            };

            // First try to load a gcodes/test.nc file and run it; otherwise fall back to demo scene
            string gcodeFile = Path.Combine(baseDir, "gcodes", "test2.nc");
            if (File.Exists(gcodeFile))
            {
                // Create a work area that covers reasonable size for the demo G-code
                // G-code uses Z=100 for cutting, so place workpiece below that
                var bbox = BoundingBox.FromCenterAndSize(
                    new SysVector3(0, 0, 0),  // Center at Z=50 so top is at Z=100
                    new SysVector3(200, 200, 100)  // Height of 100mm
                );
                var gridStopwatch = new Stopwatch();
                gridStopwatch.Start();
                _voxelGrid = new VoxelGrid(bbox, resolution: 1.0f);
                gridStopwatch.Stop();
                Console.WriteLine($"Voxel grid creation time: {gridStopwatch.ElapsedMilliseconds} ms");
                Console.WriteLine($"Workpiece bounds: Min=({bbox.Min.X}, {bbox.Min.Y}, {bbox.Min.Z}), Max=({bbox.Max.X}, {bbox.Max.Y}, {bbox.Max.Z})");

                // Fill keeps material; implement removal via executing toolpath
                var startPos = new System.Numerics.Vector3(0, 0, 150);  // Start above workpiece
                List<MillSimSharp.Toolpath.IToolpathCommand>? commands = null;
                var parseStopwatch = new Stopwatch();
                parseStopwatch.Start();
                try
                {
                    commands = GcodeToPath.ParseFromFile(gcodeFile, startPos);
                    parseStopwatch.Stop();
                    Console.WriteLine($"Loaded G-code file: {gcodeFile}. Commands: {commands.Count}. Parse time: {parseStopwatch.ElapsedMilliseconds} ms");
                }
                catch (Exception ex)
                {
                    parseStopwatch.Stop();
                    Console.WriteLine($"Failed to parse G-code: {ex.Message}. Parse time: {parseStopwatch.ElapsedMilliseconds} ms");
                }

                var simulator = new CutterSimulator(_voxelGrid);
                var tool = new EndMill(diameter: 10.0f, length: 50.0f, isBallEnd: false);  // Longer tool
                Console.WriteLine($"Tool: Diameter={tool.Diameter}mm, Length={tool.Length}mm, Type={tool.Type}");
                
                var executor = new ToolpathExecutor(simulator, tool, startPos);
                var execStopwatch = new Stopwatch();
                execStopwatch.Start();
                
                long voxelsBeforeCut = _voxelGrid.CountMaterialVoxels();
                Console.WriteLine($"Voxels before cutting: {voxelsBeforeCut}");
                
                if (commands != null)
                {
                    executor.ExecuteCommands(commands);
                }
                execStopwatch.Stop();
                
                long voxelsAfterCut = _voxelGrid.CountMaterialVoxels();
                Console.WriteLine($"Voxels after cutting: {voxelsAfterCut}");
                Console.WriteLine($"Voxels removed: {voxelsBeforeCut - voxelsAfterCut}");
                Console.WriteLine($"Toolpath execution time: {execStopwatch.ElapsedMilliseconds} ms");

                // Store commands to update toolpath renderer later
                if (commands != null)
                {
                    _pendingToolpathCommands = commands;
                    _pendingToolpathStartPos = startPos;
                }
            }
            else
            {
                // Create demo voxel scene
                CreateDemoScene();
            }

            // Initialize renderer
            _renderer = new VoxelRenderer();
            _axisRenderer = new AxisRenderer();
            _toolpathRenderer = new ToolpathRenderer();
            _meshRenderer = new MeshRenderer();
            
            // Update toolpath renderer if we have pending commands
            if (_toolpathRenderer != null && _pendingToolpathCommands != null)
            {
                _toolpathRenderer.UpdateFromCommands(_pendingToolpathCommands, _pendingToolpathStartPos);
                Console.WriteLine($"Toolpath segments loaded: {_pendingToolpathCommands.Count}");
            }
            
            if (_voxelGrid != null)
            {
                _renderer.UpdateVoxelData(_voxelGrid);
                // Generate a mesh from the voxel grid and update mesh renderer
                var meshStopwatch = new Stopwatch();
                meshStopwatch.Start();
                var mesh = MillSimSharp.Geometry.MeshConverter.ConvertToMesh(_voxelGrid);
                meshStopwatch.Stop();
                Console.WriteLine($"Mesh conversion time: {meshStopwatch.ElapsedMilliseconds} ms");
                Console.WriteLine($"Mesh stats: Vertices={mesh.Vertices.Length}, Triangles={mesh.Indices.Length / 3}");
                _meshRenderer?.UpdateMesh(mesh);
            }
            Console.WriteLine($"Voxel Viewer initialized");
            Console.WriteLine($"Voxels: {_voxelGrid?.CountMaterialVoxels() ?? 0}");
            Console.WriteLine($"Controls:");
            Console.WriteLine($"  - Mouse drag: Rotate camera");
            Console.WriteLine($"  - Mouse wheel: Zoom in/out");
            Console.WriteLine($"  - ESC: Exit");

            stopwatch.Stop();
            Console.WriteLine($"OnLoad took {stopwatch.ElapsedMilliseconds} ms");
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

            // Render mesh if available, otherwise render instanced voxels
            if (_meshShader != null && _meshRenderer != null)
            {
                _meshShader.Use();
                _meshShader.SetMatrix4("uView", view);
                _meshShader.SetMatrix4("uProjection", projection);
                _meshShader.SetVector3("uLightDir", lightDir);

                _meshRenderer.Render();
            }
            else
            {
                _renderer.Render();
            }

            // Render axes
            _axisRenderer?.Render(view, projection);

            // Render toolpath lines (overlay)
            if (_lineShader != null && _toolpathRenderer != null)
            {
                _lineShader.Use();
                _lineShader.SetMatrix4("uView", view);
                _lineShader.SetMatrix4("uProjection", projection);
                _toolpathRenderer.Render();
            }

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
            _lineShader?.Dispose();
            _meshShader?.Dispose();
            _toolpathRenderer?.Dispose();
            _meshRenderer?.Dispose();
        }
    }
}
