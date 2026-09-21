using System;
using System.Numerics;
using MillSimSharp.Geometry;
using MillSimSharp.Simulation;

namespace MillSimSharp.Tests.Simulation
{
    /// <summary>
    /// Test-only tool geometries used by the V3/V5/V6 audit tests. They intentionally violate the
    /// assumptions of the builtin geometries (axisymmetric solid / bounds starting at the tip).
    /// </summary>
    internal sealed class TestEllipticalGeometry : IToolGeometry
    {
        public float XRadius { get; }
        public float YRadius { get; }
        public float Length { get; }

        public TestEllipticalGeometry(float xRadius, float yRadius, float length)
        {
            XRadius = xRadius;
            YRadius = yRadius;
            Length = length;
        }

        public float SignedDistance(Vector3 localPoint)
        {
            // Sign-exact 2D ellipse distance in the local XY plane, capped at z = 0..Length.
            float ex = localPoint.X / XRadius;
            float ey = localPoint.Y / YRadius;
            float radial = (MathF.Sqrt(ex * ex + ey * ey) - 1f) * MathF.Min(XRadius, YRadius);
            float axial = MathF.Max(-localPoint.Z, localPoint.Z - Length);
            return MathF.Max(radial, axial);
        }

        public BoundingBox LocalBounds => new BoundingBox(
            new Vector3(-XRadius, -YRadius, 0f),
            new Vector3(XRadius, YRadius, Length));

        public float CuttingCenterOffset => 0f;
    }

    /// <summary>
    /// Capsule centered on the local origin (local z from -HalfLength to +HalfLength), used to test
    /// the LocalBounds Min.Z contract of the pose helpers.
    /// </summary>
    internal sealed class TestSpanningCapsuleGeometry : IToolGeometry
    {
        public float Radius { get; }
        public float HalfLength { get; }

        public TestSpanningCapsuleGeometry(float radius, float halfLength)
        {
            Radius = radius;
            HalfLength = halfLength;
        }

        public float SignedDistance(Vector3 localPoint)
        {
            float axial = Math.Clamp(localPoint.Z, -HalfLength, HalfLength);
            return Vector3.Distance(localPoint, new Vector3(0f, 0f, axial)) - Radius;
        }

        public BoundingBox LocalBounds => new BoundingBox(
            new Vector3(-Radius, -Radius, -HalfLength),
            new Vector3(Radius, Radius, HalfLength));

        public float CuttingCenterOffset => 0f;
    }

    /// <summary>
    /// Tool wrapper so the test geometries can be driven through the simulator APIs.
    /// </summary>
    internal sealed class TestGeometryTool : Tool
    {
        private readonly IToolGeometry _geometry;

        public TestGeometryTool(IToolGeometry geometry, float diameter, float length)
            : base(diameter, length, ToolType.Flat)
        {
            _geometry = geometry;
        }

        public override float BallCenterOffsetFromTip => 0f;

        public override IToolGeometry GetCuttingGeometry() => _geometry;
    }
}
