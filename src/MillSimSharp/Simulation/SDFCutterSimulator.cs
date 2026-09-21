using System;
using System.Numerics;
using MillSimSharp.Geometry;

namespace MillSimSharp.Simulation
{
    /// <summary>
    /// Simulator for cutting operations on SDF grids.
    /// Provides the same interface as CutterSimulator but operates on SDFGrid instead of VoxelGrid.
    /// <para>
    /// All positions passed to this class are the <b>physical tool tip</b>.
    /// For ball end mills the cutting sphere center is derived internally as
    /// <c>tip + AxisTowardSpindle * radius</c>.
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
        /// Performs a linear cut from start to end using the specified tool.
        /// <para>
        /// start / end は工具先端（Physical Tip）の位置です。
        /// 実装は既定姿勢の pose sweep に委譲されるため、3軸と5軸で切削形状の経路が一致します。
        /// </para>
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

            float radius = tool.Diameter / 2.0f;
            float length = tool.Length;
            float ballOffset = tool.BallCenterOffsetFromTip;
            Vector3 axisTowardSpindle = Vector3.UnitZ;

            Vector3 cuttingCenter = position + axisTowardSpindle * ballOffset;
            Vector3 top = position + axisTowardSpindle * Math.Max(length, ballOffset);

            if (tool.Type == ToolType.Ball)
            {
                // Ball: full sphere at the cutting center (the tip is the sphere bottom).
                _sdfGrid.RemoveSphere(cuttingCenter, radius);
            }

            // Tool body from the cutting center to the tool top.
            // For flat tools the cutting center equals the physical tip (flat bottom).
            _sdfGrid.RemoveFiniteCylinder(cuttingCenter, top, radius);
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

            float radius = tool.Diameter / 2.0f;
            float length = tool.Length;
            float ballOffset = tool.BallCenterOffsetFromTip;

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

                Vector3 cuttingCenter = position + axisTowardSpindle * ballOffset;
                Vector3 top = position + axisTowardSpindle * Math.Max(length, ballOffset);

                if (tool.Type == ToolType.Ball)
                {
                    // Ball: sphere at the cutting center
                    _sdfGrid.RemoveSphere(cuttingCenter, radius);
                }

                // Tool body: flat-ended cylinder from the cutting center to the tool top.
                // For flat tools this is the full tool (flat bottom at the tip plane).
                _sdfGrid.RemoveFiniteCylinder(cuttingCenter, top, radius);
            }
        }
    }
}
