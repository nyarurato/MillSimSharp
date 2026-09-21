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

        /// <summary>
        /// Creates a new SDFCutterSimulator with the specified SDF grid.
        /// </summary>
        /// <param name="sdfGrid"></param>
        /// <exception cref="ArgumentNullException"></exception>
        public SDFCutterSimulator(SDFGrid sdfGrid)
        {
            _sdfGrid = sdfGrid ?? throw new ArgumentNullException(nameof(sdfGrid));
        }

        /// <summary>
        /// Performs a linear cut from start to end using the specified tool.
        /// start / end は工具先端（Physical Tip）の位置です。
        /// ボールエンドミルは球中心の軌跡を掃引し、フラットエンドミルは平底円柱を掃引します。
        /// </summary>
        /// <param name="start">Start position of the physical tool tip.</param>
        /// <param name="end">End position of the physical tool tip.</param>
        /// <param name="tool">The cutting tool.</param>
        public void CutLinear(Vector3 start, Vector3 end, Tool tool)
        {
            if (tool == null) throw new ArgumentNullException(nameof(tool));

            float radius = tool.Diameter / 2.0f;
            float length = tool.Length;
            float ballOffset = tool.BallCenterOffsetFromTip;
            Vector3 axisTowardSpindle = Vector3.UnitZ; // 3-axis tools always point downward

            Vector3 centerStart = start + axisTowardSpindle * ballOffset;
            Vector3 centerEnd = end + axisTowardSpindle * ballOffset;
            Vector3 topOffset = axisTowardSpindle * Math.Max(length, ballOffset);

            // Step 1: sweep the cutting edge along the path
            if (tool.Type == ToolType.Ball)
            {
                // Ball: capsule around the ball-center path
                _sdfGrid.RemoveCapsule(centerStart, centerEnd, radius);
            }
            else
            {
                // Flat: flat-ended cylinder swept along the tip path
                _sdfGrid.RemoveFiniteCylinder(start, end, radius);
            }

            // Step 2: sweep the top of the tool (cutting length is measured from the physical tip)
            _sdfGrid.RemoveFiniteCylinder(start + topOffset, end + topOffset, radius);

            // Step 3: swept shaft volume between the cutting center path and the tool top
            Vector3 motion = end - start;
            float distance = motion.Length();

            if (distance > 0)
            {
                float stepSize = radius * 0.5f;
                int numSteps = Math.Max(2, (int)Math.Ceiling(distance / stepSize));

                for (int i = 0; i <= numSteps; i++)
                {
                    float t = i / (float)numSteps;
                    Vector3 tipPos = start + motion * t;
                    Vector3 centerPos = tipPos + axisTowardSpindle * ballOffset;
                    Vector3 topPos = tipPos + topOffset;
                    _sdfGrid.RemoveFiniteCylinder(centerPos, topPos, radius);
                }
            }
            else
            {
                _sdfGrid.RemoveFiniteCylinder(centerStart, start + topOffset, radius);
            }
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

            // Number of interpolation steps based on distance
            // Limit interpolation frequency to avoid excessive computation
            Vector3 delta = end - start;
            float distance = delta.Length();

            // Use fewer steps for 5-axis: one step per 2-3 voxels rather than every voxel
            float stepSize = _sdfGrid.Resolution * 2.5f;
            int steps = Math.Max(1, (int)Math.Ceiling(distance / stepSize));

            // Interpolate along the path with orientation
            for (int i = 0; i <= steps; i++)
            {
                float t = i / (float)steps;
                Vector3 position = Vector3.Lerp(start, end, t);

                // Interpolate orientation
                var orientation = new Toolpath.ToolOrientation(
                    startOrientation.A + (endOrientation.A - startOrientation.A) * t,
                    startOrientation.B + (endOrientation.B - startOrientation.B) * t,
                    startOrientation.C + (endOrientation.C - startOrientation.C) * t
                );

                Vector3 axisTowardSpindle = orientation.GetAxisTowardSpindle();
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
