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
        private Shader? _lineShader;
        private MeshRenderer? _meshRenderer;
        private bool _useSDF = true;
        private int _sdfNarrowBandWidth = 10;
        private MillSimSharp.Geometry.SDFGrid? _sdfGrid = null;
        // Async mesh generation fields
        private System.Threading.Tasks.Task<MillSimSharp.Geometry.Mesh>? _meshComputeTask;
        private MillSimSharp.Geometry.Mesh? _pendingMesh;
        private bool _meshUpdatePending = false;
        // Key state helper for toggles
        private bool _sKeyPrev = false;
        private bool _rKeyPrev = false;
        private bool _nKeyPrev = false;
        private bool _cKeyPrev = false;
        private bool _eKeyPrev = false;
        private MillSimSharp.Geometry.Mesh? _currentMesh;
        private int[] _bandOptions = new int[] { 1, 2, 5, 10 };
        private Shader? _meshShader;
        private VoxelGrid? _voxelGrid;
        private ToolpathRenderer? _toolpathRenderer;
        private List<IToolpathCommand>? _pendingToolpathCommands;
        private SysVector3 _pendingToolpathStartPos;

        private Vector2 _lastMousePos;
        private bool _isMouseDragging;
        
        // Processing state tracking
        private string _processingStatus = "";
        private bool _sdfGenerationInProgress = false;
        private bool _meshGenerationInProgress = false;

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
                var tool = new EndMill(diameter: 10.0f, length: 50.0f, isBallEnd: true);  // Longer tool
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
                // Check voxel count to determine mesh generation strategy
                long voxelsCount = _voxelGrid.CountMaterialVoxels();
                const long SDF_VOXEL_THRESHOLD = 1_000_000;
                
                if (voxelsCount > SDF_VOXEL_THRESHOLD)
                {
                    // Use SDF with narrow band for large grids
                    _useSDF = true;
                    _sdfNarrowBandWidth = 2;
                    Console.WriteLine($"Large grid detected ({voxelsCount} voxels). Using SDF with narrow band width of {_sdfNarrowBandWidth}.");
                    Console.WriteLine($"SDF grid and mesh will be generated in background. Press 'R' to regenerate if needed.");
                    Console.WriteLine($"Estimated SDF generation time: ~{EstimateSdfTime(voxelsCount, _sdfNarrowBandWidth)} seconds");
                    
                    // Start SDF generation in background to avoid blocking startup
                    var gridCopy = _voxelGrid;
                    var narrowBand = _sdfNarrowBandWidth;
                    bool useSparse = voxelsCount > SDF_VOXEL_THRESHOLD;
                    
                    _sdfGenerationInProgress = true;
                    _processingStatus = "Generating SDF...";
                    
                    Console.WriteLine($"Starting SDF generation task... (voxels={voxelsCount}, narrowBand={narrowBand}, sparse={useSparse})");
                    
                    System.Threading.Tasks.Task.Run(() =>
                    {
                        MillSimSharp.Geometry.SDFGrid? sdf = null;
                        try
                        {
                            var sdfStopwatch = new Stopwatch();
                            sdfStopwatch.Start();
                            
                            Console.WriteLine($"SDF generation thread started. About to call FromVoxelGrid...");
                            
                            // Progress reporting task - more frequent updates
                            var progressTask = System.Threading.Tasks.Task.Run(async () =>
                            {
                                int reportCount = 0;
                                while (_sdfGenerationInProgress)
                                {
                                    await System.Threading.Tasks.Task.Delay(2000); // Every 2 seconds
                                    if (_sdfGenerationInProgress)
                                    {
                                        reportCount++;
                                        Console.WriteLine($"[{reportCount}] SDF generation in progress... ({sdfStopwatch.ElapsedMilliseconds / 1000}s elapsed)");
                                    }
                                }
                            });
                            
                            sdf = MillSimSharp.Geometry.SDFGrid.FromVoxelGrid(gridCopy, narrowBand, useSparse);
                            _sdfGenerationInProgress = false;
                            sdfStopwatch.Stop();
                            Console.WriteLine($"SDF grid creation completed: {sdfStopwatch.ElapsedMilliseconds} ms ({sdfStopwatch.ElapsedMilliseconds / 1000.0:F1}s, sparse={useSparse})");
                        }
                        catch (Exception ex)
                        {
                            _sdfGenerationInProgress = false;
                            _processingStatus = "SDF generation failed";
                            Console.WriteLine($"SDF generation failed with exception: {ex.Message}");
                            Console.WriteLine($"Stack trace: {ex.StackTrace}");
                            return;
                        }
                        
                        if (sdf == null)
                        {
                            Console.WriteLine($"SDF generation returned null!");
                            return;
                        }
                        
                        // Update on main thread
                        _sdfGrid = sdf;
                        _sdfGrid.BindToVoxelGrid(gridCopy);
                        gridCopy.VoxelsChanged += (minX, minY, minZ, maxX, maxY, maxZ) =>
                        {
                            // When voxel changes happen, schedule a mesh recompute
                            StartMeshGenerationAsync();
                        };
                        
                        _processingStatus = "Generating mesh...";
                        // Auto-generate mesh after SDF is ready
                        StartMeshGenerationAsync();
                    });
                }
                else
                {
                    // For smaller grids, create SDF synchronously
                    if (_useSDF)
                    {
                        var sdfStopwatch = new Stopwatch();
                        sdfStopwatch.Start();
                        bool useSparse = voxelsCount > SDF_VOXEL_THRESHOLD;
                        _sdfGrid = MillSimSharp.Geometry.SDFGrid.FromVoxelGrid(_voxelGrid, _sdfNarrowBandWidth, useSparse);
                        sdfStopwatch.Stop();
                        Console.WriteLine($"SDF grid creation time: {sdfStopwatch.ElapsedMilliseconds} ms (sparse={useSparse})");
                        
                        _sdfGrid.BindToVoxelGrid(_voxelGrid);
                        _voxelGrid.VoxelsChanged += (minX, minY, minZ, maxX, maxY, maxZ) =>
                        {
                            // When voxel changes happen, schedule a mesh recompute
                            StartMeshGenerationAsync();
                        };
                    }
                    
                    // Kick off mesh generation asynchronously
                    StartMeshGenerationAsync();
                }
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

        /// <summary>
        /// Estimate SDF generation time based on voxel count and narrow band width
        /// </summary>
        private int EstimateSdfTime(long voxelCount, int narrowBand)
        {
            // Rough estimation based on observed performance
            // Approximately 0.5-1 second per million voxels with narrow band of 2
            double baseTime = (voxelCount / 1_000_000.0) * 0.7;
            double bandFactor = narrowBand / 2.0; // Wider band takes longer
            return (int)Math.Ceiling(baseTime * bandFactor);
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

                // Apply pending mesh update if available (do this on render thread)
                if (_meshUpdatePending && _pendingMesh != null)
                {
                    _meshRenderer.UpdateMesh(_pendingMesh);
                    // Keep a copy of the current active mesh for exports
                    _currentMesh = _pendingMesh;
                    _meshUpdatePending = false;
                    _pendingMesh = null;
                }
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
                
                string statusSuffix = string.IsNullOrEmpty(_processingStatus) ? "" : $" - {_processingStatus}";
                int triangles = _currentMesh?.Indices.Length / 3 ?? 0;
                string meshInfo = triangles > 0 ? $" - Triangles: {triangles}" : $" - Voxels: {_voxelGrid?.CountMaterialVoxels() ?? 0}";

                Title = $"MillSimSharp Voxel Viewer - FPS: {fps:0.0} - Mem: {memory} MB{meshInfo}{statusSuffix}";

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

            // S key -> toggle SDF mode
            bool sDown = KeyboardState.IsKeyDown(Keys.S);
            if (sDown && !_sKeyPrev)
            {
                _useSDF = !_useSDF;
                Console.WriteLine($"SDF mode toggled. Now using SDF: {_useSDF}. Recomputing mesh...");
                StartMeshGenerationAsync();
            }
            _sKeyPrev = sDown;

            // R key -> recompute mesh
            bool rDown = KeyboardState.IsKeyDown(Keys.R);
            if (rDown && !_rKeyPrev)
            {
                Console.WriteLine("Recomputing mesh...");
                StartMeshGenerationAsync();
            }
            _rKeyPrev = rDown;

            // N key -> cycle narrow band widths for SDF
            bool nDown = KeyboardState.IsKeyDown(Keys.N);
            if (nDown && !_nKeyPrev)
            {
                // Cycle to next band option
                int currentIndex = Array.IndexOf(_bandOptions, _sdfNarrowBandWidth);
                if (currentIndex < 0) currentIndex = 0;
                int nextIndex = (currentIndex + 1) % _bandOptions.Length;
                _sdfNarrowBandWidth = _bandOptions[nextIndex];
                Console.WriteLine($"SDF narrow band width changed to: {_sdfNarrowBandWidth}");
            }
            _nKeyPrev = nDown;

            // C key -> toggle backface culling (debug)
            bool cDown = KeyboardState.IsKeyDown(Keys.C);
            if (cDown && !_cKeyPrev)
            {
                // Toggle culling
                if (GL.IsEnabled(EnableCap.CullFace))
                {
                    GL.Disable(EnableCap.CullFace);
                    Console.WriteLine("Backface culling disabled (C)");
                }
                else
                {
                    GL.Enable(EnableCap.CullFace);
                    Console.WriteLine("Backface culling enabled (C)");
                }
            }
            _cKeyPrev = cDown;

            // E key -> export current mesh or voxel grid to STL
            bool eDown = KeyboardState.IsKeyDown(Keys.E);
            if (eDown && !_eKeyPrev)
            {
                // choose export target
                string baseDir = AppDomain.CurrentDomain.BaseDirectory;
                string outDir = Path.Combine(baseDir, "exports");
                try
                {
                    if (!Directory.Exists(outDir)) Directory.CreateDirectory(outDir);
                    string filename = DateTime.Now.ToString("yyyyMMdd_HHmmss");
                    string filePath = Path.Combine(outDir, $"export_{filename}.stl");
                    if (_currentMesh != null)
                    {
                        // Run export in background so we don't block the render loop
                        System.Threading.Tasks.Task.Run(() => {
                            MillSimSharp.IO.StlExporter.Export(_currentMesh, filePath);
                            Console.WriteLine($"Exported current mesh to: {filePath}");
                        });
                    }
                    else if (_voxelGrid != null)
                    {
                        System.Threading.Tasks.Task.Run(() => {
                            MillSimSharp.IO.StlExporter.Export(_voxelGrid, filePath);
                            Console.WriteLine($"Exported voxel grid to: {filePath}");
                        });
                    }
                    else
                    {
                        Console.WriteLine("No mesh or voxel grid available to export.");
                    }
                }
                catch (Exception ex)
                {
                    Console.WriteLine($"Failed to export STL: {ex}");
                }
            }
            _eKeyPrev = eDown;
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

        /// <summary>
        /// Start mesh generation on a background thread and apply it on the render thread once ready.
        /// </summary>
        private void StartMeshGenerationAsync()
        {
            if (_voxelGrid == null) return;

            if (_meshComputeTask != null && !_meshComputeTask.IsCompleted)
            {
                Console.WriteLine("Mesh generation already in progress...");
                return;
            }

            // Capture local state
            bool localUseSDF = _useSDF;
            int localNarrow = _sdfNarrowBandWidth;
            var gridCopy = _voxelGrid; // reference
            var sdfCopy = _sdfGrid; // may be null
            // If user toggles SDF and no SDFGrid exists yet, create one with safe fast-mode settings
            if (localUseSDF && sdfCopy == null && gridCopy != null)
            {
                long voxelsCountLocal = gridCopy.CountMaterialVoxels();
                const long SDF_VOXEL_THRESHOLD = 1_000_000;
                bool useSparseLocal = voxelsCountLocal > SDF_VOXEL_THRESHOLD;
                bool fastModeLocal = false; // always create accurate SDF for viewer to avoid holes
                try
                {
                    _sdfGrid = MillSimSharp.Geometry.SDFGrid.FromVoxelGrid(gridCopy, localNarrow, useSparseLocal, fastModeLocal);
                    _sdfGrid.BindToVoxelGrid(gridCopy);
                    sdfCopy = _sdfGrid;
                }
                catch (Exception ex)
                {
                    Console.WriteLine($"Failed to create SDF grid in viewer: {ex}");
                    // leave sdfCopy null so we fallback to ConvertToMeshViaSDF which will create an SDF internally
                }
            }

            _meshGenerationInProgress = true;
            _processingStatus = "Generating mesh...";
            Console.WriteLine($"Starting mesh generation (useSDF={localUseSDF}, narrowBand={localNarrow})...");
            
            var meshGenStopwatch = new Stopwatch();
            meshGenStopwatch.Start();
            
            // Progress reporting for mesh generation
            var meshProgressTask = System.Threading.Tasks.Task.Run(async () =>
            {
                while (_meshGenerationInProgress)
                {
                    await System.Threading.Tasks.Task.Delay(5000); // Every 5 seconds
                    if (_meshGenerationInProgress)
                    {
                        Console.WriteLine($"Mesh generation in progress... ({meshGenStopwatch.ElapsedMilliseconds / 1000}s elapsed)");
                    }
                }
            });

            _meshComputeTask = System.Threading.Tasks.Task.Run(() =>
            {
                if (localUseSDF)
                {
                    if (sdfCopy != null)
                    {
                        return MillSimSharp.Geometry.MeshConverter.ConvertToMeshFromSDF(sdfCopy);
                    }
                    else
                    {
                        return MillSimSharp.Geometry.MeshConverter.ConvertToMeshViaSDF(gridCopy, localNarrow);
                    }
                }
                else
                {
                    return MillSimSharp.Geometry.MeshConverter.ConvertToMesh(gridCopy);
                }
            });

            _meshComputeTask.ContinueWith((t) =>
            {
                meshGenStopwatch.Stop();
                _meshGenerationInProgress = false;
                
                if (t.IsCompletedSuccessfully)
                {
                    _pendingMesh = t.Result;
                    _meshUpdatePending = true;
                    _processingStatus = "";
                    Console.WriteLine($"Mesh generation finished: vertices={_pendingMesh.Vertices.Length}, triangles={_pendingMesh.Indices.Length / 3}, time={meshGenStopwatch.ElapsedMilliseconds} ms ({meshGenStopwatch.ElapsedMilliseconds / 1000.0:F1}s)");
                }
                else if (t.IsFaulted)
                {
                    _processingStatus = "Mesh generation failed";
                    // Print full exception details to help diagnose failures
                    Console.WriteLine($"Mesh generation failed after {meshGenStopwatch.ElapsedMilliseconds} ms: {t.Exception?.ToString()}");
                }
            }, System.Threading.Tasks.TaskScheduler.Default);
        }
    }
}
