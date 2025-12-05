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
    }
}
