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
        /// Current tool being used.
        /// </summary>
        public Tool Tool { get; set; }

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
            Tool = toolConfig.CreateTool();
            Simulator = new CutterSimulator(Grid);
            Executor = new ToolpathExecutor(Simulator, Tool, Vector3.Zero);
        }

        /// <summary>
        /// Creates a simulation from configuration files.
        /// </summary>
        /// <param name="toolPath">Path to tool configuration (default: configs/default_tool.xml).</param>
        /// <param name="stockPath">Path to stock configuration (default: configs/default_stock.xml).</param>
        /// <param name="resolution">Voxel resolution in millimeters.</param>
        /// <returns>New MillSimulation instance.</returns>
        public static MillSimulation FromConfigFiles(string toolPath = null, string stockPath = null, float resolution = 1.0f)
        {
            var toolConfig = ConfigurationLoader.LoadToolConfig(toolPath);
            var stockConfig = ConfigurationLoader.LoadStockConfig(stockPath);
            return new MillSimulation(stockConfig, toolConfig, resolution);
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
            Grid = new VoxelGrid(_initialBounds, _resolution);
            Simulator = new CutterSimulator(Grid);
            Executor = new ToolpathExecutor(Simulator, Tool, Vector3.Zero);
        }

        /// <summary>
        /// Changes the current tool.
        /// </summary>
        /// <param name="newTool">New tool to use.</param>
        public void ChangeTool(Tool newTool)
        {
            if (newTool == null) throw new ArgumentNullException(nameof(newTool));
            Tool = newTool;
            // Recreate executor with new tool
            Executor = new ToolpathExecutor(Simulator, Tool, Executor.CurrentPosition);
        }
    }
}
