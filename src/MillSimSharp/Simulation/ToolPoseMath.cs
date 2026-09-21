using System;
using System.Numerics;
using MillSimSharp.Geometry;

namespace MillSimSharp.Simulation
{
    /// <summary>
    /// Internal helpers for transforming tool geometry between tool-local and world coordinates.
    /// </summary>
    internal static class ToolPoseMath
    {
        /// <summary>
        /// Computes a conservative rotation sweep radius for a cutting solid: the largest distance
        /// from the physical tip (local origin) to the corners of <see cref="IToolGeometry.LocalBounds"/>.
        /// The bounds normally overestimate the solid, so the value is conservative. This is used to
        /// bound the path of any tool point during an orientation change (adaptive sampling).
        /// </summary>
        public static float GetRotationSweepRadius(IToolGeometry geometry)
        {
            BoundingBox local = geometry.LocalBounds;

            float x = MathF.Max(MathF.Abs(local.Min.X), local.Max.X);
            float y = MathF.Max(MathF.Abs(local.Min.Y), local.Max.Y);
            float z = MathF.Max(MathF.Abs(local.Min.Z), local.Max.Z);
            return MathF.Sqrt(x * x + y * y + z * z);
        }

        /// <summary>
        /// Converts a world point to tool-local coordinates. Cutting solids are restricted to solids
        /// of revolution around the local +Z axis, so only (radial distance, 0, axial distance) is
        /// produced. <paramref name="axisTowardSpindle"/> must be a unit vector.
        /// </summary>
        public static Vector3 ToLocalPoint(Vector3 worldPoint, Vector3 tip, Vector3 axisTowardSpindle)
        {
            Vector3 relative = worldPoint - tip;
            float axial = Vector3.Dot(relative, axisTowardSpindle);
            float radial = (relative - axisTowardSpindle * axial).Length();
            return new Vector3(radial, 0f, axial);
        }

        /// <summary>
        /// Computes a conservative world-space AABB for the cutting solid at the given pose.
        /// <paramref name="axisTowardSpindle"/> must be a unit vector. The local bounds may extend
        /// to negative local Z (below the physical tip); the axial extent is projected onto each
        /// world axis using both bounds.
        /// </summary>
        public static BoundingBox GetWorldBounds(IToolGeometry geometry, Vector3 tip, Vector3 axisTowardSpindle)
        {
            BoundingBox local = geometry.LocalBounds;

            float radius = MathF.Max(
                MathF.Max(MathF.Abs(local.Min.X), local.Max.X),
                MathF.Max(MathF.Abs(local.Min.Y), local.Max.Y));

            float xMin = MathF.Min(axisTowardSpindle.X * local.Min.Z, axisTowardSpindle.X * local.Max.Z);
            float yMin = MathF.Min(axisTowardSpindle.Y * local.Min.Z, axisTowardSpindle.Y * local.Max.Z);
            float zMin = MathF.Min(axisTowardSpindle.Z * local.Min.Z, axisTowardSpindle.Z * local.Max.Z);
            float xMax = MathF.Max(axisTowardSpindle.X * local.Min.Z, axisTowardSpindle.X * local.Max.Z);
            float yMax = MathF.Max(axisTowardSpindle.Y * local.Min.Z, axisTowardSpindle.Y * local.Max.Z);
            float zMax = MathF.Max(axisTowardSpindle.Z * local.Min.Z, axisTowardSpindle.Z * local.Max.Z);

            Vector3 min = tip - new Vector3(radius, radius, radius)
                + new Vector3(xMin, yMin, zMin);
            Vector3 max = tip + new Vector3(radius, radius, radius)
                + new Vector3(xMax, yMax, zMax);

            return new BoundingBox(min, max);
        }
    }
}
