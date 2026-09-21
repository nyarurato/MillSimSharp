using System;
using System.Numerics;
using MillSimSharp.Geometry;

namespace MillSimSharp.Simulation
{
    /// <summary>
    /// Tapered end mill cutting solid: a truncated cone from the tip radius to
    /// <c>TipRadius + Length * tan(angle)</c> at the tool top.
    /// <para>
    /// The signed distance is sign-exact and uses the perpendicular scaling
    /// <c>cos(angle)</c> on the lateral surface; distances off the lateral surface are approximate.
    /// </para>
    /// </summary>
    public class TaperedEndMillGeometry : IToolGeometry
    {
        /// <summary>
        /// Radius at the physical tip in millimeters.
        /// </summary>
        public float TipRadius { get; }

        /// <summary>
        /// Taper (half) angle in degrees.
        /// </summary>
        public float TaperAngleDegrees { get; }

        /// <summary>
        /// Cutting length measured from the physical tip in millimeters.
        /// </summary>
        public float Length { get; }

        /// <summary>
        /// Radius at the tool top (tip radius + length * tan(angle)).
        /// </summary>
        public float TopRadius { get; }

        /// <summary>
        /// Creates a tapered end mill geometry.
        /// </summary>
        /// <param name="tipRadius">Radius at the physical tip in millimeters.</param>
        /// <param name="taperAngleDegrees">Taper angle in degrees (non-negative).</param>
        /// <param name="length">Cutting length from the physical tip in millimeters.</param>
        public TaperedEndMillGeometry(float tipRadius, float taperAngleDegrees, float length)
        {
            if (tipRadius < 0) throw new ArgumentException("Tip radius must be non-negative.", nameof(tipRadius));
            if (length <= 0) throw new ArgumentException("Length must be positive.", nameof(length));
            if (taperAngleDegrees < 0 || taperAngleDegrees >= 90) throw new ArgumentException("Taper angle must be in [0, 90) degrees.", nameof(taperAngleDegrees));

            TipRadius = tipRadius;
            TaperAngleDegrees = taperAngleDegrees;
            Length = length;
            TopRadius = tipRadius + length * MathF.Tan(taperAngleDegrees * MathF.PI / 180f);

            float bound = MathF.Max(TipRadius, TopRadius);
            LocalBounds = new BoundingBox(
                new Vector3(-bound, -bound, 0f),
                new Vector3(bound, bound, length));
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

            float cosAngle = MathF.Cos(TaperAngleDegrees * MathF.PI / 180f);
            float radiusAtZ = TipRadius + (TopRadius - TipRadius) * (z / Length);

            // Perpendicular distance to the lateral surface
            float lateral = (radial - radiusAtZ) * cosAngle;

            // Flat caps
            float bottom = -z;
            float top = z - Length;

            return MathF.Max(lateral, MathF.Max(bottom, top));
        }
    }
}
