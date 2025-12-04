using System;
using System.Numerics;
using MillSimSharp.Geometry;

namespace MillSimSharp.Simulation
{
    /// <summary>
    /// Simulates material removal by a cutting tool.
    /// </summary>
    public class CutterSimulator
    {
        private readonly VoxelGrid _grid;

        public CutterSimulator(VoxelGrid grid)
        {
            _grid = grid ?? throw new ArgumentNullException(nameof(grid));
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
            if (tool.Type == ToolType.Flat)
            {
                // Flat end mill: Remove cylinder with flat ends
                _grid.RemoveVoxelsInCylinder(start, end, radius, flatEnds: true);
            }
            else if (tool.Type == ToolType.Ball)
            {
                // Ball end mill: Remove capsule (cylinder with spherical ends)
                _grid.RemoveVoxelsInCylinder(start, end, radius, flatEnds: false);
            }
            else
            {
                // Fallback for other types (treat as flat for now)
                _grid.RemoveVoxelsInCylinder(start, end, radius, flatEnds: true);
            }

            // Step 2: Remove material along the tool shaft
            // The shaft extends vertically upward (in +Z direction) from the tip
            // We need to sweep the shaft volume as the tool moves from start to end
            
            // Calculate shaft top positions
            Vector3 shaftOffset = new Vector3(0, 0, length);
            Vector3 shaftStart = start + shaftOffset;
            Vector3 shaftEnd = end + shaftOffset;

            // Remove material in the shaft path (top of tool moving from shaftStart to shaftEnd)
            _grid.RemoveVoxelsInCylinder(shaftStart, shaftEnd, radius, flatEnds: true);
            
            // Remove material in the swept volume connecting tip path to shaft path
            // This is a "ruled surface" - we need to connect the bottom path (tip) to top path (shaft)
            // For simplicity, we'll add intermediate vertical cylinders
            
            // Sample the path with intermediate points
            Vector3 motion = end - start;
            float distance = motion.Length();
            
            if (distance > 0)
            {
                // Create vertical shaft cylinders at intervals along the path
                // Use resolution based on tool diameter to ensure complete coverage
                float stepSize = Math.Min(radius * 0.5f, _grid.Resolution * 2.0f);
                int numSteps = Math.Max(2, (int)Math.Ceiling(distance / stepSize));
                
                for (int i = 0; i <= numSteps; i++)
                {
                    float t = i / (float)numSteps;
                    Vector3 tipPos = start + motion * t;
                    Vector3 shaftTop = tipPos + shaftOffset;
                    
                    // Create vertical cylinder from tip to shaft top at this position
                    _grid.RemoveVoxelsInCylinder(tipPos, shaftTop, radius, flatEnds: true);
                }
            }
            else
            {
                // Zero-length movement: just remove vertical shaft at this point
                _grid.RemoveVoxelsInCylinder(start, shaftStart, radius, flatEnds: true);
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
            if (tool.Type == ToolType.Ball)
            {
                _grid.RemoveVoxelsInSphere(position, radius);
            }
            else
            {
                // For flat end mill, remove a small cylinder at the tip
                _grid.RemoveVoxelsInSphere(position, radius);
            }

            // Remove material along the tool shaft (vertical cylinder above the tip)
            Vector3 shaftTop = position + new Vector3(0, 0, length);
            _grid.RemoveVoxelsInCylinder(position, shaftTop, radius, flatEnds: true);
        }
    }
}
