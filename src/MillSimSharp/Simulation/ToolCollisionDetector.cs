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
        /// <remarks>
        /// The stock is sampled at voxel centers, so a contact that does not reach any voxel center
        /// (for example a sub-resolution tool passing through a voxel corner) may not be detected.
        /// </remarks>
        /// <param name="grid">Stock voxel grid.</param>
        /// <param name="geometry">Tool part geometry (tool-local, origin = physical tip).</param>
        /// <param name="tip">Physical tool tip position.</param>
        /// <param name="axisTowardSpindle">Tool axis direction (tip -> spindle). Any finite non-zero vector; it is normalized internally.</param>
        /// <returns>True when the part intersects material.</returns>
        /// <exception cref="ArgumentException">Thrown when the axis is zero or not finite.</exception>
        public static bool IntersectsMaterial(VoxelGrid grid, IToolGeometry geometry, Vector3 tip, Vector3 axisTowardSpindle)
        {
            if (grid == null) throw new ArgumentNullException(nameof(grid));
            if (geometry == null) throw new ArgumentNullException(nameof(geometry));

            Vector3 axis = ValidateAxis(axisTowardSpindle);
            BoundingBox worldBounds = ToolPoseMath.GetWorldBounds(geometry, tip, axis);
            return grid.IntersectsToolSolid(
                worldBounds,
                point => geometry.SignedDistance(ToolPoseMath.ToLocalPoint(point, tip, axis)));
        }

        /// <summary>
        /// Checks whether the given tool part intersects remaining material in an SDF grid.
        /// </summary>
        /// <remarks>
        /// The stock is sampled at voxel centers, so a contact that does not reach any voxel center
        /// (for example a sub-resolution tool passing through a voxel corner) may not be detected.
        /// </remarks>
        /// <param name="sdf">Stock SDF grid.</param>
        /// <param name="geometry">Tool part geometry (tool-local, origin = physical tip).</param>
        /// <param name="tip">Physical tool tip position.</param>
        /// <param name="axisTowardSpindle">Tool axis direction (tip -> spindle). Any finite non-zero vector; it is normalized internally.</param>
        /// <returns>True when the part intersects material.</returns>
        /// <exception cref="ArgumentException">Thrown when the axis is zero or not finite.</exception>
        public static bool IntersectsMaterial(SDFGrid sdf, IToolGeometry geometry, Vector3 tip, Vector3 axisTowardSpindle)
        {
            if (sdf == null) throw new ArgumentNullException(nameof(sdf));
            if (geometry == null) throw new ArgumentNullException(nameof(geometry));

            Vector3 axis = ValidateAxis(axisTowardSpindle);
            BoundingBox worldBounds = ToolPoseMath.GetWorldBounds(geometry, tip, axis);
            return sdf.IntersectsToolSolid(
                worldBounds,
                point => geometry.SignedDistance(ToolPoseMath.ToLocalPoint(point, tip, axis)));
        }

        /// <summary>
        /// Normalizes the tool axis and rejects zero / non-finite vectors.
        /// </summary>
        private static Vector3 ValidateAxis(Vector3 axisTowardSpindle)
        {
            if (!float.IsFinite(axisTowardSpindle.X) ||
                !float.IsFinite(axisTowardSpindle.Y) ||
                !float.IsFinite(axisTowardSpindle.Z))
            {
                throw new ArgumentException("Tool axis must be finite.", nameof(axisTowardSpindle));
            }

            float length = axisTowardSpindle.Length();
            if (length < 1e-6f)
            {
                throw new ArgumentException("Tool axis must be a non-zero vector.", nameof(axisTowardSpindle));
            }

            return axisTowardSpindle / length;
        }
    }
}
