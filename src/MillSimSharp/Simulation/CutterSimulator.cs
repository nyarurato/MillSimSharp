using System;
using System.Numerics;
using MillSimSharp.Geometry;

namespace MillSimSharp.Simulation
{
    /// <summary>
    /// Simulator for cutting operations on voxel grids.
    /// <para>
    /// All positions passed to this class are the <b>physical tool tip</b>. Material is removed
    /// where the tool cutting solid (<see cref="IToolGeometry"/>) placed at the tip and oriented
    /// by the tool axis occupies the stock.
    /// </para>
    /// </summary>
    public class CutterSimulator : ICutterSimulator
    {
        private readonly VoxelGrid _grid;

        /// <inheritdoc />
        public SimulationSettings Settings { get; }

        /// <summary>
        /// Creates a new CutterSimulator with the specified voxel grid.
        /// </summary>
        /// <param name="grid"></param>
        /// <exception cref="ArgumentNullException"></exception>
        public CutterSimulator(VoxelGrid grid)
        {
            _grid = grid ?? throw new ArgumentNullException(nameof(grid));
            Settings = new SimulationSettings { MaxLinearStep = 0.5f * _grid.Resolution };
        }

        /// <summary>
        /// Performs a linear cut from start to end using the specified tool (default orientation).
        /// </summary>
        /// <param name="start">Start position of the physical tool tip.</param>
        /// <param name="end">End position of the physical tool tip.</param>
        /// <param name="tool">The cutting tool.</param>
        public void CutLinear(Vector3 start, Vector3 end, Tool tool)
        {
            if (tool == null) throw new ArgumentNullException(nameof(tool));

            // 3-axis cuts are the default-orientation case of the pose sweep.
            CutLinearWithOrientation(start, end, tool, Toolpath.ToolOrientation.Default, Toolpath.ToolOrientation.Default);
        }

        /// <summary>
        /// Performs a point cut (drilling/plunging) at the specified physical tip position.
        /// </summary>
        /// <param name="position">Position of the physical tool tip.</param>
        /// <param name="tool">The cutting tool.</param>
        public void CutPoint(Vector3 position, Tool tool)
        {
            if (tool == null) throw new ArgumentNullException(nameof(tool));

            // Batch the edit so listeners (e.g. an incremental SDF) update once per cut.
            _grid.BeginEdit();
            try
            {
                ToolSweepPlanner.ExecutePointCut(tool.GetCuttingGeometry(), position,
                    (bounds, signedDistance) => _grid.RemoveVoxelsInRegion(bounds, signedDistance));
            }
            finally
            {
                _grid.EndEdit();
            }
        }

        /// <summary>
        /// Performs a linear cut with specified tool orientation (for 5-axis machining).
        /// <para>
        /// The number of interpolation steps is derived from both linear and angular motion
        /// (<see cref="SimulationSettings"/>), and the orientation is interpolated with a
        /// quaternion slerp (shortest rotation). Rotation-only moves are swept as well.
        /// </para>
        /// </summary>
        /// <param name="start">Physical tool tip position at start.</param>
        /// <param name="end">Physical tool tip position at end.</param>
        /// <param name="tool">Cutting tool to use.</param>
        /// <param name="startOrientation">Tool orientation at start.</param>
        /// <param name="endOrientation">Tool orientation at end.</param>
        public void CutLinearWithOrientation(Vector3 start, Vector3 end, Tool tool,
            Toolpath.ToolOrientation startOrientation, Toolpath.ToolOrientation endOrientation)
        {
            if (tool == null) throw new ArgumentNullException(nameof(tool));

            // Batch the edit so listeners (e.g. an incremental SDF) update once per cut.
            _grid.BeginEdit();
            try
            {
                ToolSweepPlanner.ExecuteLinearMove(
                    tool.GetCuttingGeometry(), start, end, startOrientation, endOrientation, Settings,
                    (bounds, signedDistance) => _grid.RemoveVoxelsInRegion(bounds, signedDistance));
            }
            finally
            {
                _grid.EndEdit();
            }
        }
    }
}
