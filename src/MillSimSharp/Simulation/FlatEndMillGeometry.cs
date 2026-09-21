using System;
using System.Numerics;
using MillSimSharp.Geometry;

namespace MillSimSharp.Simulation
{
    /// <summary>
    /// Flat end mill cutting solid: a flat-ended cylinder from z = 0 to z = length.
    /// </summary>
    public class FlatEndMillGeometry : IToolGeometry
    {
        /// <summary>
        /// Tool radius in millimeters.
        /// </summary>
        public float Radius { get; }

        /// <summary>
        /// Cutting length measured from the physical tip in millimeters.
        /// </summary>
        public float Length { get; }

        /// <summary>
        /// Creates a flat end mill geometry.
        /// </summary>
        /// <param name="radius">Tool radius in millimeters.</param>
        /// <param name="length">Cutting length from the physical tip in millimeters.</param>
        public FlatEndMillGeometry(float radius, float length)
        {
            if (!float.IsFinite(radius) || radius <= 0)
                throw new ArgumentException("Radius must be a finite positive number.", nameof(radius));
            if (!float.IsFinite(length) || length <= 0)
                throw new ArgumentException("Length must be a finite positive number.", nameof(length));

            Radius = radius;
            Length = length;
            LocalBounds = new BoundingBox(
                new Vector3(-radius, -radius, 0f),
                new Vector3(radius, radius, length));
        }

        /// <inheritdoc />
        public BoundingBox LocalBounds { get; }

        /// <inheritdoc />
        public float CuttingCenterOffset => 0f;

        /// <inheritdoc />
        public float SignedDistance(Vector3 localPoint)
        {
            // Exact signed distance to a capped cylinder (Inigo Quilez formulation).
            float radialDistance = MathF.Sqrt(localPoint.X * localPoint.X + localPoint.Y * localPoint.Y) - Radius;
            float halfLength = Length * 0.5f;
            float axialDistance = MathF.Abs(localPoint.Z - halfLength) - halfLength;

            float outside = MathF.Sqrt(
                MathF.Max(radialDistance, 0f) * MathF.Max(radialDistance, 0f) +
                MathF.Max(axialDistance, 0f) * MathF.Max(axialDistance, 0f));

            return MathF.Min(MathF.Max(radialDistance, axialDistance), 0f) + outside;
        }
    }
}
