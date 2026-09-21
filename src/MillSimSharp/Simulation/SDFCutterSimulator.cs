using System;
using System.Numerics;
using MillSimSharp.Geometry;

namespace MillSimSharp.Simulation
{
    /// <summary>
    /// Simulator for cutting operations on SDF grids.
    /// Provides the same interface as CutterSimulator but operates on SDFGrid instead of VoxelGrid.
    /// <para>
    /// All positions passed to this class are the <b>physical tool tip</b>. The SDF is updated by
    /// a CSG difference with the tool cutting solid (<see cref="IToolGeometry"/>) placed at the tip.
    /// </para>
    /// </summary>
    public class SDFCutterSimulator : ICutterSimulator
    {
        private readonly SDFGrid _sdfGrid;

        /// <inheritdoc />
        public SimulationSettings Settings { get; }

        /// <summary>
        /// Creates a new SDFCutterSimulator with the specified SDF grid.
        /// </summary>
        /// <param name="sdfGrid"></param>
        /// <exception cref="ArgumentNullException"></exception>
        public SDFCutterSimulator(SDFGrid sdfGrid)
        {
            _sdfGrid = sdfGrid ?? throw new ArgumentNullException(nameof(sdfGrid));
            Settings = new SimulationSettings { MaxLinearStep = 0.5f * _sdfGrid.Resolution };
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

            RemoveToolSolid(tool.GetCuttingGeometry(), position, Vector3.UnitZ);
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

            IToolGeometry geometry = tool.GetCuttingGeometry();

            Vector3 delta = end - start;
            float distance = delta.Length();
            float angularDistance = Toolpath.ToolOrientation.AngularDistanceDegrees(startOrientation, endOrientation);
            int steps = Settings.ComputeSteps(distance, angularDistance);

            Quaternion qStart = startOrientation.GetQuaternion();
            Quaternion qEnd = endOrientation.GetQuaternion();

            for (int i = 0; i <= steps; i++)
            {
                float t = i / (float)steps;
                Vector3 position = Vector3.Lerp(start, end, t);
                Quaternion q = Quaternion.Slerp(qStart, qEnd, t);
                Vector3 axisTowardSpindle = Vector3.Transform(Vector3.UnitZ, q);

                RemoveToolSolid(geometry, position, axisTowardSpindle);
            }
        }

        private void RemoveToolSolid(IToolGeometry geometry, Vector3 tip, Vector3 axisTowardSpindle)
        {
            BoundingBox worldBounds = ToolPoseMath.GetWorldBounds(geometry, tip, axisTowardSpindle);
            _sdfGrid.CarveRegion(
                worldBounds,
                point => geometry.SignedDistance(ToolPoseMath.ToLocalPoint(point, tip, axisTowardSpindle)));
        }
    }
}
