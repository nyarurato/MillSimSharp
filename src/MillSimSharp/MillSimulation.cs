using System;
using System.Collections.Generic;
using System.Numerics;
using MillSimSharp.Config;
using MillSimSharp.Geometry;
using MillSimSharp.IO;
using MillSimSharp.Simulation;
using MillSimSharp.Toolpath;

namespace MillSimSharp
{
    /// <summary>
    /// Main facade class that integrates all simulation components.
    /// </summary>
    public class MillSimulation
    {
        /// <summary>
        /// Voxel grid representing the stock material.
        /// </summary>
        public VoxelGrid Grid { get; private set; }

        /// <summary>
        /// Current tool being used. The executor owns the tool state: the getter returns
        /// <see cref="ToolpathExecutor.CurrentTool"/> and the setter is equivalent to
        /// <see cref="ChangeTool"/>.
        /// </summary>
        public Tool Tool
        {
            get => Executor.CurrentTool;
            set => ChangeTool(value);
        }

        /// <summary>
        /// Cutter simulator for material removal.
        /// </summary>
        public CutterSimulator Simulator { get; private set; }

        /// <summary>
        /// Toolpath executor for command sequences.
        /// </summary>
        public ToolpathExecutor Executor { get; private set; }

        /// <summary>
        /// Stock configuration.
        /// </summary>
        public StockConfiguration StockConfig { get; private set; }

        private readonly BoundingBox _initialBounds;
        private readonly float _resolution;

        /// <summary>
        /// Creates a new milling simulation.
        /// </summary>
        /// <param name="stockConfig">Stock configuration.</param>
        /// <param name="toolConfig">Tool configuration.</param>
        /// <param name="resolution">Voxel resolution in millimeters.</param>
        public MillSimulation(StockConfiguration stockConfig, ToolConfiguration toolConfig, float resolution = 1.0f)
        {
            if (stockConfig == null) throw new ArgumentNullException(nameof(stockConfig));
            if (toolConfig == null) throw new ArgumentNullException(nameof(toolConfig));

            StockConfig = stockConfig;
            _resolution = resolution;
            _initialBounds = stockConfig.GetBoundingBox();

            Grid = new VoxelGrid(_initialBounds, resolution);
            Simulator = new CutterSimulator(Grid);
            Executor = new ToolpathExecutor(Simulator, toolConfig.CreateTool(), Vector3.Zero);
        }



        /// <summary>
        /// Executes a toolpath using the current tool.
        /// </summary>
        /// <param name="commands">List of toolpath commands to execute.</param>
        public void ExecuteToolpath(IEnumerable<IToolpathCommand> commands)
        {
            Executor.ExecuteCommands(commands);
        }

        /// <summary>
        /// Exports the current state to an STL file.
        /// </summary>
        /// <param name="filepath">Output file path.</param>
        public void ExportToStl(string filepath)
        {
            StlExporter.Export(Grid, filepath);
        }

        /// <summary>
        /// Exports the current state as a high-quality mesh generated from an SDF
        /// (Dual Contouring) to an STL file.
        /// </summary>
        /// <param name="filepath">Output file path.</param>
        /// <param name="narrowBandWidth">Width of the SDF narrow band in voxels.</param>
        public void ExportToStlViaSdf(string filepath, int narrowBandWidth = 10)
        {
            var sdf = SDFGrid.FromVoxelGrid(Grid, narrowBandWidth);
            var mesh = MeshConverter.ConvertToMeshFromSDF(sdf);
            StlExporter.Export(mesh, filepath);
        }

        /// <summary>
        /// Gets the number of voxels containing material.
        /// </summary>
        /// <returns>Material voxel count.</returns>
        public int GetMaterialVoxelCount()
        {
            return Grid.CountMaterialVoxels();
        }

        /// <summary>
        /// Resets the grid to initial state (all material).
        /// </summary>
        public void Reset()
        {
            var currentTool = Tool;
            Grid = new VoxelGrid(_initialBounds, _resolution);
            Simulator = new CutterSimulator(Grid);
            Executor = new ToolpathExecutor(Simulator, currentTool, Vector3.Zero);
        }

        /// <summary>
        /// Changes the current tool. The executor is kept, so the current pose, loaded commands,
        /// command index, progress subscriptions and estimated time are preserved.
        /// </summary>
        /// <param name="newTool">New tool to use.</param>
        public void ChangeTool(Tool newTool)
        {
            if (newTool == null) throw new ArgumentNullException(nameof(newTool));
            Executor.ChangeTool(newTool);
        }
    }
}
