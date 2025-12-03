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
        /// </summary>
        /// <param name="start">Start position of the tool tip.</param>
        /// <param name="end">End position of the tool tip.</param>
        /// <param name="tool">The cutting tool.</param>
        public void CutLinear(Vector3 start, Vector3 end, Tool tool)
        {
            if (tool == null) throw new ArgumentNullException(nameof(tool));

            float radius = tool.Diameter / 2.0f;

            // Determine cut shape based on tool type
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

            // Note: This implementation currently only removes material along the tool tip path.
            // It does not account for the tool shaft (Length) removing material above the tip.
            // For full 3-axis simulation, we would need to sweep the tool volume.
        }

        /// <summary>
        /// Performs a point cut (drilling/plunging) at the specified position.
        /// </summary>
        /// <param name="position">Position of the tool tip.</param>
        /// <param name="tool">The cutting tool.</param>
        public void CutPoint(Vector3 position, Tool tool)
        {
            if (tool == null) throw new ArgumentNullException(nameof(tool));

            float radius = tool.Diameter / 2.0f;

            if (tool.Type == ToolType.Ball)
            {
                _grid.RemoveVoxelsInSphere(position, radius);
            }
            else
            {
                // For flat end mill, point cut is theoretically a cylinder of height 0 (circle),
                // but in voxel grid it might be better represented as a small cylinder or just checking bounds.
                // However, usually CutPoint implies plunging, which is a movement.
                // If it's just static removal, Flat end mill at a point doesn't remove anything "below" it,
                // but occupies a cylindrical space.
                // Since we don't support "height" removal yet, we'll just remove a sphere for now
                // or maybe nothing if it's truly 0 length movement?
                // Let's assume it removes a sphere for consistency, or maybe a very short cylinder?
                // Actually, static point cut for flat end mill is ambiguous without depth.
                // Let's use Sphere for now as a fallback, or maybe just ignore?
                // Better: RemoveVoxelsInSphere is safest for "clearing space" around the point.
                _grid.RemoveVoxelsInSphere(position, radius);
            }
        }
    }
}
