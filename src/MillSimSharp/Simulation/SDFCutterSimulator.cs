using System;
using System.Numerics;
using MillSimSharp.Geometry;

namespace MillSimSharp.Simulation
{
    /// <summary>
    /// Simulator for cutting operations on SDF grids.
    /// Provides the same interface as CutterSimulator but operates on SDFGrid instead of VoxelGrid.
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
        /// Removes material both at the tool tip and along the tool shaft.
        /// </summary>
        /// <param name="start">Start position of the tool tip.</param>
        /// <param name="end">End position of the tool tip.</param>
        /// <param name="tool">The cutting tool.</param>
        public void CutLinear(Vector3 start, Vector3 end, Tool tool)
        {
            if (tool == null) throw new ArgumentNullException(nameof(tool));

            float radius = tool.Diameter / 2.0f;
            float length = tool.Length;

            // Step 1: Remove material along the tool tip path
            if (tool.Type == ToolType.Ball)
            {
                // Ball end mill: Remove capsule (cylinder with spherical ends)
                // For ball end, we need to remove spheres at start and end, plus cylinder between
                _sdfGrid.RemoveSphere(start, radius);
                _sdfGrid.RemoveSphere(end, radius);
                _sdfGrid.RemoveCylinder(start, end, radius);
            }
            else
            {
                // Flat end mill: Remove cylinder
                _sdfGrid.RemoveCylinder(start, end, radius);
            }

            // Step 2: Remove material along the tool shaft
            // The shaft extends vertically upward (in +Z direction) from the tip
            Vector3 shaftOffset = new Vector3(0, 0, length);
            Vector3 shaftStart = start + shaftOffset;
            Vector3 shaftEnd = end + shaftOffset;

            // Remove material in the shaft path
            _sdfGrid.RemoveCylinder(shaftStart, shaftEnd, radius);

            // Step 3: Remove material in the swept volume connecting tip to shaft
            Vector3 motion = end - start;
            float distance = motion.Length();

            if (distance > 0)
            {
                // Create vertical shaft cylinders at intervals along the path
                float stepSize = radius * 0.5f;
                int numSteps = Math.Max(2, (int)Math.Ceiling(distance / stepSize));

                for (int i = 0; i <= numSteps; i++)
                {
                    float t = i / (float)numSteps;
                    Vector3 tipPos = start + motion * t;
                    Vector3 shaftTop = tipPos + shaftOffset;

                    // Create vertical cylinder from tip to shaft top at this position
                    _sdfGrid.RemoveCylinder(tipPos, shaftTop, radius);
                }
            }
            else
            {
                // Zero-length movement: just remove vertical shaft at this point
                _sdfGrid.RemoveCylinder(start, shaftStart, radius);
            }
        }

        /// <summary>
        /// Performs a point cut (drilling/plunging) at the specified position.
        /// Removes material at the tool tip and along the tool shaft.
        /// </summary>
        /// <param name="position">Position of the tool tip.</param>
        /// <param name="tool">The cutting tool.</param>
        public void CutPoint(Vector3 position, Tool tool)
        {
            if (tool == null) throw new ArgumentNullException(nameof(tool));

            float radius = tool.Diameter / 2.0f;
            float length = tool.Length;

            // Remove material at the tool tip
            _sdfGrid.RemoveSphere(position, radius);

            // Remove material along the tool shaft (vertical cylinder above the tip)
            Vector3 shaftTop = position + new Vector3(0, 0, length);
            _sdfGrid.RemoveCylinder(position, shaftTop, radius);
        }

        /// <summary>
        /// Performs a linear cut with specified tool orientation (for 5-axis machining).
        /// </summary>
        /// <param name="start">Tool tip position at start.</param>
        /// <param name="end">Tool tip position at end.</param>
        /// <param name="tool">Cutting tool to use.</param>
        /// <param name="startOrientation">Tool orientation at start.</param>
        /// <param name="endOrientation">Tool orientation at end.</param>
        public void CutLinearWithOrientation(Vector3 start, Vector3 end, Tool tool,
            Toolpath.ToolOrientation startOrientation, Toolpath.ToolOrientation endOrientation)
        {
            if (tool == null) throw new ArgumentNullException(nameof(tool));

            float radius = tool.Diameter / 2.0f;
            float length = tool.Length;

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

                // Get tool direction at this orientation
                Vector3 toolDirection = orientation.GetToolDirection();

                // Calculate shaft endpoint
                Vector3 shaftEnd = position - toolDirection * length;

                // Remove material for tool tip
                if (tool.Type == ToolType.Ball)
                {
                    _sdfGrid.RemoveSphere(position, radius);
                }
                else
                {
                    // For flat end mill, remove a small sphere at the tip
                    _sdfGrid.RemoveSphere(position, radius * 0.5f);
                }

                // Remove material along the tool shaft
                _sdfGrid.RemoveCylinder(position, shaftEnd, radius);
            }
        }
    }
}
