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
        /// Converts a world point to tool-local coordinates. Cutting solids are rotationally
        /// symmetric, so only (radial distance, 0, axial distance) is produced.
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
        /// </summary>
        public static BoundingBox GetWorldBounds(IToolGeometry geometry, Vector3 tip, Vector3 axisTowardSpindle)
        {
            BoundingBox local = geometry.LocalBounds;

            float radius = MathF.Max(
                MathF.Max(MathF.Abs(local.Min.X), local.Max.X),
                MathF.Max(MathF.Abs(local.Min.Y), local.Max.Y));
            float axialMax = local.Max.Z;

            Vector3 axis = axisTowardSpindle;
            Vector3 min = tip - new Vector3(radius, radius, radius)
                + new Vector3(
                    MathF.Min(0f, axis.X * axialMax),
                    MathF.Min(0f, axis.Y * axialMax),
                    MathF.Min(0f, axis.Z * axialMax));
            Vector3 max = tip + new Vector3(radius, radius, radius)
                + new Vector3(
                    MathF.Max(0f, axis.X * axialMax),
                    MathF.Max(0f, axis.Y * axialMax),
                    MathF.Max(0f, axis.Z * axialMax));

            return new BoundingBox(min, max);
        }
    }
}
