using System;
using System.Collections.Generic;
using System.Numerics;
using MillSimSharp.Geometry;

namespace MillSimSharp.Tests.Reference
{
    /// <summary>
    /// Test-only oracle for cutting geometry.
    /// <para>
    /// This is written independently from the production helpers (it computes the tool axis and
    /// ball center directly from the Euler angle specification), so that a bug in the production
    /// conversion cannot make the tests pass accidentally.
    /// </para>
    /// <para>
    /// Keep this class under <c>tests/</c> only: it is not part of the product library.
    /// </para>
    /// </summary>
    internal static class ReferenceCutEvaluator
    {
        private static float DegToRad(float degrees) => degrees * MathF.PI / 180f;

        /// <summary>
        /// Expected cutting axis direction (spindle -> tip) for the rotation order C -> B -> A
        /// (rotate about Z, then Y, then X). Independent scalar implementation of the specification.
        /// </summary>
        public static Vector3 ExpectedCuttingAxisDirection(float aDeg, float bDeg, float cDeg)
        {
            float a = DegToRad(aDeg);
            float b = DegToRad(bDeg);
            float c = DegToRad(cDeg);

            // Default tool direction: (0, 0, -1)
            float x = 0f, y = 0f, z = -1f;

            // Rotate about Z (C-axis) first
            float x1 = x * MathF.Cos(c) - y * MathF.Sin(c);
            float y1 = x * MathF.Sin(c) + y * MathF.Cos(c);
            x = x1;
            y = y1;

            // Then about Y (B-axis)
            float x2 = x * MathF.Cos(b) + z * MathF.Sin(b);
            float z2 = -x * MathF.Sin(b) + z * MathF.Cos(b);
            x = x2;
            z = z2;

            // Then about X (A-axis)
            float y3 = y * MathF.Cos(a) - z * MathF.Sin(a);
            float z3 = y * MathF.Sin(a) + z * MathF.Cos(a);

            return Vector3.Normalize(new Vector3(x, y3, z3));
        }

        /// <summary>
        /// Expected axis direction from the physical tip toward the spindle (tip -> spindle).
        /// </summary>
        public static Vector3 ExpectedAxisTowardSpindle(float aDeg, float bDeg, float cDeg)
        {
            return -ExpectedCuttingAxisDirection(aDeg, bDeg, cDeg);
        }

        /// <summary>
        /// Expected ball center for a ball end mill whose physical tip is at <paramref name="tip"/>.
        /// </summary>
        public static Vector3 ExpectedBallCenter(Vector3 tip, float aDeg, float bDeg, float cDeg, float radius)
        {
            return tip + ExpectedAxisTowardSpindle(aDeg, bDeg, cDeg) * radius;
        }

        /// <summary>
        /// Signed distance to a ball (negative inside, positive outside), computed independently
        /// of the production geometry helpers.
        /// </summary>
        public static float ReferenceSignedDistanceBall(Vector3 point, Vector3 center, float radius)
        {
            return Vector3.Distance(point, center) - radius;
        }

        /// <summary>
        /// Signed distance to a flat-ended cylinder (negative inside). A zero-length cylinder has
        /// no interior, so every point is reported outside.
        /// </summary>
        public static float ReferenceSignedDistanceCylinder(Vector3 point, Vector3 start, Vector3 end, float radius)
        {
            Vector3 axis = end - start;
            float length = axis.Length();
            if (length < 1e-9f)
                return Vector3.Distance(point, start) + radius;

            Vector3 dir = axis / length;
            Vector3 rel = point - start;
            float axial = Vector3.Dot(rel, dir);
            float radial = (rel - dir * axial).Length();

            float radialDistance = radial - radius;
            float axialDistance = MathF.Max(-axial, axial - length);

            float outside = MathF.Sqrt(
                MathF.Max(radialDistance, 0f) * MathF.Max(radialDistance, 0f) +
                MathF.Max(axialDistance, 0f) * MathF.Max(axialDistance, 0f));

            return MathF.Min(MathF.Max(radialDistance, axialDistance), 0f) + outside;
        }

        /// <summary>
        /// Signed distance to a capsule (negative inside).
        /// </summary>
        public static float ReferenceSignedDistanceCapsule(Vector3 point, Vector3 start, Vector3 end, float radius)
        {
            Vector3 axis = end - start;
            float length = axis.Length();
            if (length < 1e-9f)
                return Vector3.Distance(point, start) - radius;

            Vector3 dir = axis / length;
            Vector3 rel = point - start;
            float t = Math.Clamp(Vector3.Dot(rel, dir), 0f, length);
            return (rel - dir * t).Length() - radius;
        }

        /// <summary>
        /// Signed distance to the ball end mill cutting solid (ball union flute cylinder) for an
        /// arbitrary orientation (negative inside).
        /// </summary>
        public static float ReferenceSignedDistanceBallTool(Vector3 point, Vector3 tip,
            float aDeg, float bDeg, float cDeg, float radius, float length)
        {
            Vector3 axis = ExpectedAxisTowardSpindle(aDeg, bDeg, cDeg);
            Vector3 center = tip + axis * radius;
            Vector3 top = tip + axis * MathF.Max(length, radius);

            float ball = ReferenceSignedDistanceBall(point, center, radius);
            float flute = ReferenceSignedDistanceCylinder(point, center, top, radius);
            return MathF.Min(ball, flute);
        }

        /// <summary>
        /// Signed distance to the flat end mill cutting solid (flat-ended cylinder) for an
        /// arbitrary orientation (negative inside).
        /// </summary>
        public static float ReferenceSignedDistanceFlatTool(Vector3 point, Vector3 tip,
            float aDeg, float bDeg, float cDeg, float radius, float length)
        {
            Vector3 axis = ExpectedAxisTowardSpindle(aDeg, bDeg, cDeg);
            return ReferenceSignedDistanceCylinder(point, tip, tip + axis * length, radius);
        }

        /// <summary>
        /// Default-orientation ball test (axis toward spindle = +Z). Matches the production
        /// convention <c>signedDistance &lt; 0</c>: points on the surface are not inside.
        /// </summary>
        public static bool IsInsideBall(Vector3 point, Vector3 tip, float radius)
        {
            Vector3 center = tip + new Vector3(0, 0, radius);
            return ReferenceSignedDistanceBall(point, center, radius) < 0f;
        }

        /// <summary>
        /// Arbitrary-orientation ball test using the independently computed ball center.
        /// </summary>
        public static bool IsInsideBall(Vector3 point, Vector3 tip, float aDeg, float bDeg, float cDeg, float radius)
        {
            Vector3 center = ExpectedBallCenter(tip, aDeg, bDeg, cDeg, radius);
            return ReferenceSignedDistanceBall(point, center, radius) < 0f;
        }

        /// <summary>
        /// Point-in-finite-cylinder test (flat ends).
        /// </summary>
        public static bool IsInsideCylinder(Vector3 point, Vector3 start, Vector3 end, float radius)
        {
            return ReferenceSignedDistanceCylinder(point, start, end, radius) < 0f;
        }

        /// <summary>
        /// Capsule test (point within <paramref name="radius"/> of a line segment).
        /// </summary>
        public static bool IsInsideCapsule(Vector3 point, Vector3 start, Vector3 end, float radius)
        {
            return ReferenceSignedDistanceCapsule(point, start, end, radius) < 0f;
        }

        /// <summary>
        /// Reference tool solid for a static ball end mill cut (CutPoint):
        /// ball at the derived center plus the flute cylinder from the ball center to the tool top.
        /// </summary>
        public static bool IsInsideBallCutPointTool(Vector3 point, Vector3 tip, float radius, float length)
        {
            return ReferenceSignedDistanceBallTool(point, tip, 0f, 0f, 0f, radius, length) < 0f;
        }

        /// <summary>
        /// Reference tool solid for a static flat end mill cut (CutPoint):
        /// flat-ended cylinder from the physical tip to the tool top.
        /// </summary>
        public static bool IsInsideFlatCutPointTool(Vector3 point, Vector3 tip, float radius, float length)
        {
            return ReferenceSignedDistanceFlatTool(point, tip, 0f, 0f, 0f, radius, length) < 0f;
        }

        /// <summary>
        /// Collects the world-space centers of all removed voxels.
        /// Useful for cheap removed-bounds regression checks.
        /// </summary>
        public static List<Vector3> CollectRemovedVoxelCenters(VoxelGrid grid)
        {
            var result = new List<Vector3>();
            var bbox = grid.Bounds;
            float res = grid.Resolution;
            var (sx, sy, sz) = grid.Dimensions;
            var dense = grid.ToDenseArray();

            for (int z = 0; z < sz; z++)
                for (int y = 0; y < sy; y++)
                    for (int x = 0; x < sx; x++)
                    {
                        if (!dense[x][y][z])
                        {
                            result.Add(bbox.Min + new Vector3(
                                (x + 0.5f) * res,
                                (y + 0.5f) * res,
                                (z + 0.5f) * res));
                        }
                    }
            return result;
        }
    }
}
