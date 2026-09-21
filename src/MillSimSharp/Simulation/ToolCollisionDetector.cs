using System;
using System.Numerics;
using MillSimSharp.Geometry;

namespace MillSimSharp.Simulation
{
    /// <summary>
    /// Read-only collision checks between tool parts (cutting edge, shank, holder) and the stock.
    /// All positions are the physical tool tip.
    /// </summary>
    public static class ToolCollisionDetector
    {
        /// <summary>
        /// Checks whether the given tool part intersects remaining material in a voxel grid.
        /// </summary>
        /// <param name="grid">Stock voxel grid.</param>
        /// <param name="geometry">Tool part geometry (tool-local, origin = physical tip).</param>
        /// <param name="tip">Physical tool tip position.</param>
        /// <param name="axisTowardSpindle">Tool axis direction (tip -> spindle).</param>
        /// <returns>True when the part intersects material.</returns>
        public static bool IntersectsMaterial(VoxelGrid grid, IToolGeometry geometry, Vector3 tip, Vector3 axisTowardSpindle)
        {
            if (grid == null) throw new ArgumentNullException(nameof(grid));
            if (geometry == null) throw new ArgumentNullException(nameof(geometry));

            BoundingBox worldBounds = ToolPoseMath.GetWorldBounds(geometry, tip, axisTowardSpindle);
            return grid.IntersectsToolSolid(
                worldBounds,
                point => geometry.SignedDistance(ToolPoseMath.ToLocalPoint(point, tip, axisTowardSpindle)));
        }

        /// <summary>
        /// Checks whether the given tool part intersects remaining material in an SDF grid.
        /// </summary>
        /// <param name="sdf">Stock SDF grid.</param>
        /// <param name="geometry">Tool part geometry (tool-local, origin = physical tip).</param>
        /// <param name="tip">Physical tool tip position.</param>
        /// <param name="axisTowardSpindle">Tool axis direction (tip -> spindle).</param>
        /// <returns>True when the part intersects material.</returns>
        public static bool IntersectsMaterial(SDFGrid sdf, IToolGeometry geometry, Vector3 tip, Vector3 axisTowardSpindle)
        {
            if (sdf == null) throw new ArgumentNullException(nameof(sdf));
            if (geometry == null) throw new ArgumentNullException(nameof(geometry));

            BoundingBox worldBounds = ToolPoseMath.GetWorldBounds(geometry, tip, axisTowardSpindle);
            return sdf.IntersectsToolSolid(
                worldBounds,
                point => geometry.SignedDistance(ToolPoseMath.ToLocalPoint(point, tip, axisTowardSpindle)));
        }
    }
}
