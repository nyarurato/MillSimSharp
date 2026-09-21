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
            if (!float.IsFinite(tipRadius) || tipRadius < 0)
                throw new ArgumentException("Tip radius must be a finite non-negative number.", nameof(tipRadius));
            if (!float.IsFinite(length) || length <= 0)
                throw new ArgumentException("Length must be a finite positive number.", nameof(length));
            if (!float.IsFinite(taperAngleDegrees) || taperAngleDegrees < 0 || taperAngleDegrees >= 90)
                throw new ArgumentException("Taper angle must be in [0, 90) degrees.", nameof(taperAngleDegrees));

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
            // Exact signed distance of the solid of revolution: the 3D distance equals the signed
            // distance in the meridian half-plane to the convex cross-section quadrilateral
            // (0,0), (TipRadius,0), (TopRadius,Length), (0,Length). The axis edge (0,0)-(0,Length)
            // is not a real surface and is excluded from the boundary distance.
            float radial = MathF.Sqrt(localPoint.X * localPoint.X + localPoint.Y * localPoint.Y);
            float z = localPoint.Z;

            float bottom = DistanceToSegment(radial, z, 0f, 0f, TipRadius, 0f);
            float lateral = DistanceToSegment(radial, z, TipRadius, 0f, TopRadius, Length);
            float top = DistanceToSegment(radial, z, 0f, Length, TopRadius, Length);
            float boundary = MathF.Min(bottom, MathF.Min(lateral, top));

            float radiusAtZ = TipRadius + (TopRadius - TipRadius) * (z / Length);
            bool inside = z >= 0f && z <= Length && radial <= radiusAtZ;
            return inside ? -boundary : boundary;
        }

        private static float DistanceToSegment(float px, float pz, float ax, float az, float bx, float bz)
        {
            float abx = bx - ax;
            float abz = bz - az;
            float apx = px - ax;
            float apz = pz - az;
            float denominator = abx * abx + abz * abz;
            float t = denominator > 1e-12f
                ? Math.Clamp((apx * abx + apz * abz) / denominator, 0f, 1f)
                : 0f;
            float dx = apx - abx * t;
            float dz = apz - abz * t;
            return MathF.Sqrt(dx * dx + dz * dz);
        }
    }
}
