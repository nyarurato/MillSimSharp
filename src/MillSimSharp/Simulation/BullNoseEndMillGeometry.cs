using System;
using System.Numerics;
using MillSimSharp.Geometry;

namespace MillSimSharp.Simulation
{
    /// <summary>
    /// Bull nose (toroidal) end mill cutting solid: a cylinder of radius R with a toroidal
    /// corner of radius <see cref="CornerRadius"/> and a flat bottom of radius R - cornerRadius.
    /// </summary>
    public class BullNoseEndMillGeometry : IToolGeometry
    {
        /// <summary>
        /// Tool (cylinder) radius in millimeters.
        /// </summary>
        public float Radius { get; }

        /// <summary>
        /// Corner radius in millimeters (0 &lt; cornerRadius &lt;= radius).
        /// </summary>
        public float CornerRadius { get; }

        /// <summary>
        /// Cutting length measured from the physical tip in millimeters.
        /// </summary>
        public float Length { get; }

        /// <summary>
        /// Creates a bull nose end mill geometry.
        /// </summary>
        /// <param name="radius">Tool radius in millimeters.</param>
        /// <param name="cornerRadius">Corner radius in millimeters.</param>
        /// <param name="length">Cutting length from the physical tip in millimeters.</param>
        public BullNoseEndMillGeometry(float radius, float cornerRadius, float length)
        {
            if (!float.IsFinite(radius) || radius <= 0)
                throw new ArgumentException("Radius must be a finite positive number.", nameof(radius));
            if (!float.IsFinite(length) || length <= 0)
                throw new ArgumentException("Length must be a finite positive number.", nameof(length));
            if (!float.IsFinite(cornerRadius) || cornerRadius <= 0)
                throw new ArgumentException("Corner radius must be a finite positive number.", nameof(cornerRadius));
            if (cornerRadius > radius)
                throw new ArgumentException("Corner radius must not exceed the tool radius.", nameof(cornerRadius));

            Radius = radius;
            CornerRadius = cornerRadius;
            Length = length;

            // The toroidal corner can reach z = 2 * cornerRadius; extend the bounds so the solid
            // is always contained (analogous to the ball end mill's max(length, 2 * radius)).
            float zMax = MathF.Max(length, 2f * cornerRadius);
            LocalBounds = new BoundingBox(
                new Vector3(-radius, -radius, 0f),
                new Vector3(radius, radius, zMax));
        }

        /// <inheritdoc />
        public BoundingBox LocalBounds { get; }

        /// <inheritdoc />
        public float CuttingCenterOffset => 0f;

        /// <inheritdoc />
        public float RotationSweepRadius
        {
            get
            {
                float zMax = MathF.Max(Length, 2f * CornerRadius);
                return MathF.Sqrt(Radius * Radius + zMax * zMax);
            }
        }

        /// <inheritdoc />
        public float SignedDistance(Vector3 localPoint)
        {
            float radial = MathF.Sqrt(localPoint.X * localPoint.X + localPoint.Y * localPoint.Y);
            float z = localPoint.Z;

            // Main cylinder from the corner center to the tool top. The top is at least the corner
            // radius so the cap range stays valid for a very short cutting length.
            float fluteTop = MathF.Max(Length, CornerRadius);
            float cylinder = CappedCylinderDistance(radial, z, Radius, CornerRadius, fluteTop);

            // Flat bottom core (radius R - r) from z = 0 to the corner center height
            float core = CappedCylinderDistance(radial, z, Radius - CornerRadius, 0f, CornerRadius);

            // Toroidal corner around the circle at (radial = R - r, z = r)
            float qx = radial - (Radius - CornerRadius);
            float qz = z - CornerRadius;
            float torus = MathF.Sqrt(qx * qx + qz * qz) - CornerRadius;

            return MathF.Min(cylinder, MathF.Min(core, torus));
        }

        private static float CappedCylinderDistance(float radial, float z, float radius, float zMin, float zMax)
        {
            float radialDistance = radial - radius;
            float center = (zMin + zMax) * 0.5f;
            float halfLength = (zMax - zMin) * 0.5f;
            float axialDistance = MathF.Abs(z - center) - halfLength;

            float outside = MathF.Sqrt(
                MathF.Max(radialDistance, 0f) * MathF.Max(radialDistance, 0f) +
                MathF.Max(axialDistance, 0f) * MathF.Max(axialDistance, 0f));

            return MathF.Min(MathF.Max(radialDistance, axialDistance), 0f) + outside;
        }
    }
}
