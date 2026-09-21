using System;
using System.Numerics;
using MillSimSharp.Geometry;

namespace MillSimSharp.Simulation
{
    /// <summary>
    /// Ball end mill cutting solid: a ball of radius R centered at z = R (the physical tip is
    /// the bottom of the ball) unioned with the flute cylinder from the ball center to the tool top.
    /// </summary>
    public class BallEndMillGeometry : IToolGeometry
    {
        /// <summary>
        /// Ball radius in millimeters.
        /// </summary>
        public float Radius { get; }

        /// <summary>
        /// Cutting length measured from the physical tip in millimeters.
        /// </summary>
        public float Length { get; }

        /// <summary>
        /// Creates a ball end mill geometry.
        /// </summary>
        /// <param name="radius">Ball radius in millimeters.</param>
        /// <param name="length">Cutting length from the physical tip in millimeters.</param>
        public BallEndMillGeometry(float radius, float length)
        {
            if (!float.IsFinite(radius) || radius <= 0)
                throw new ArgumentException("Radius must be a finite positive number.", nameof(radius));
            if (!float.IsFinite(length) || length <= 0)
                throw new ArgumentException("Length must be a finite positive number.", nameof(length));

            Radius = radius;
            Length = length;

            float zMax = MathF.Max(length, 2f * radius);
            LocalBounds = new BoundingBox(
                new Vector3(-radius, -radius, 0f),
                new Vector3(radius, radius, zMax));
        }

        /// <inheritdoc />
        public BoundingBox LocalBounds { get; }

        /// <inheritdoc />
        public float CuttingCenterOffset => Radius;

        /// <inheritdoc />
        public float SignedDistance(Vector3 localPoint)
        {
            // Ball with center at (0, 0, radius); the tip is the ball bottom.
            Vector3 ballCenter = new Vector3(0f, 0f, Radius);
            float ballDistance = Vector3.Distance(localPoint, ballCenter) - Radius;

            // Flute cylinder from the ball center to the tool top.
            float fluteTop = MathF.Max(Length, Radius);
            float fluteDistance = CappedCylinderDistance(localPoint, Radius, Radius, fluteTop);

            return MathF.Min(ballDistance, fluteDistance);
        }

        private static float CappedCylinderDistance(Vector3 point, float radius, float zMin, float zMax)
        {
            float radialDistance = MathF.Sqrt(point.X * point.X + point.Y * point.Y) - radius;
            float center = (zMin + zMax) * 0.5f;
            float halfLength = (zMax - zMin) * 0.5f;
            float axialDistance = MathF.Abs(point.Z - center) - halfLength;

            float outside = MathF.Sqrt(
                MathF.Max(radialDistance, 0f) * MathF.Max(radialDistance, 0f) +
                MathF.Max(axialDistance, 0f) * MathF.Max(axialDistance, 0f));

            return MathF.Min(MathF.Max(radialDistance, axialDistance), 0f) + outside;
        }
    }
}
