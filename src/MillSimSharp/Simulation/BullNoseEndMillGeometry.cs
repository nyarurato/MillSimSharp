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
            if (radius <= 0) throw new ArgumentException("Radius must be positive.", nameof(radius));
            if (length <= 0) throw new ArgumentException("Length must be positive.", nameof(length));
            if (cornerRadius <= 0) throw new ArgumentException("Corner radius must be positive.", nameof(cornerRadius));
            if (cornerRadius > radius) throw new ArgumentException("Corner radius must not exceed the tool radius.", nameof(cornerRadius));

            Radius = radius;
            CornerRadius = cornerRadius;
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
            float radial = MathF.Sqrt(localPoint.X * localPoint.X + localPoint.Y * localPoint.Y);
            float z = localPoint.Z;

            // Main cylinder from the corner center up to the tool top
            float cylinder = CappedCylinderDistance(radial, z, Radius, CornerRadius, Length);

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
