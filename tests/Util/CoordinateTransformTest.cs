using NUnit.Framework;
using MillSimSharp.Util;
using MillSimSharp.Toolpath;
using MillSimSharp.Geometry;
using System;
using System.Numerics;

namespace MillSimSharp.Tests.Util
{
    [TestFixture]
    public class CoordinateTransformTest
    {
        [Test]
        public void TestMachineToWork()
        {
            var machineCoords = new Vector3(100, 50, 25);
            var workOrigin = new Vector3(50, 50, 0);

            var workCoords = CoordinateTransform.MachineToWork(machineCoords, workOrigin);

            Assert.That(workCoords, Is.EqualTo(new Vector3(50, 0, 25)));
        }

        [Test]
        public void TestWorkToMachine()
        {
            var workCoords = new Vector3(50, 0, 25);
            var workOrigin = new Vector3(50, 50, 0);

            var machineCoords = CoordinateTransform.WorkToMachine(workCoords, workOrigin);

            Assert.That(machineCoords, Is.EqualTo(new Vector3(100, 50, 25)));
        }

        [Test]
        public void TestRoundTrip()
        {
            var original = new Vector3(123, 456, 789);
            var workOrigin = new Vector3(100, 200, 300);

            var work = CoordinateTransform.MachineToWork(original, workOrigin);
            var back = CoordinateTransform.WorkToMachine(work, workOrigin);

            Assert.That(back, Is.EqualTo(original));
        }

        [Test]
        public void TestTranslateBoundingBox()
        {
            var bbox = new BoundingBox(new Vector3(0, 0, 0), new Vector3(10, 10, 10));
            var offset = new Vector3(5, 5, 5);

            var translated = CoordinateTransform.TranslateBoundingBox(bbox, offset);

            Assert.That(translated.Min, Is.EqualTo(new Vector3(5, 5, 5)));
            Assert.That(translated.Max, Is.EqualTo(new Vector3(15, 15, 15)));
        }

        [Test]
        public void InterpolateOrientation_Midpoint_UsesShortestRotation()
        {
            // C: 350° -> 10° must take the short +20° path through 0°, not the long path
            // through 180° (which independent Euler lerp produced).
            var start = new ToolOrientation(0, 0, 350);
            var end = new ToolOrientation(0, 0, 10);

            var mid = CoordinateTransform.InterpolateOrientation(start, end, 0.5f);

            Assert.That(ToolOrientation.AngularDistanceDegrees(mid, ToolOrientation.Default),
                Is.LessThan(1e-3f), "the midpoint must be the equivalent of 0° (360°)");
        }

        [Test]
        public void InterpolateOrientation_MatchesToolOrientationSlerp()
        {
            var start = new ToolOrientation(30, -20, 350);
            var end = new ToolOrientation(-10, 40, 10);

            for (int i = 0; i <= 10; i++)
            {
                float t = i / 10f;
                ToolOrientation expected = ToolOrientation.Slerp(start, end, t);
                ToolOrientation actual = CoordinateTransform.InterpolateOrientation(start, end, t);

                Assert.That(ToolOrientation.AngularDistanceDegrees(expected, actual), Is.LessThan(1e-3f),
                    $"interpolation must match ToolOrientation.Slerp at t={t}");
            }
        }

        [Test]
        public void InterpolateOrientation_Endpoints_AreExact()
        {
            var start = new ToolOrientation(30, -20, 350);
            var end = new ToolOrientation(-10, 40, 10);

            var atStart = CoordinateTransform.InterpolateOrientation(start, end, 0f);
            var atEnd = CoordinateTransform.InterpolateOrientation(start, end, 1f);

            // Compare as rotations: FromQuaternion may return an equivalent but different
            // Euler representation (e.g. 350° -> -10°).
            Assert.That(MathF.Abs(Quaternion.Dot(start.GetQuaternion(), atStart.GetQuaternion())),
                Is.GreaterThan(0.99999f), "t=0 must return the start orientation");
            Assert.That(MathF.Abs(Quaternion.Dot(end.GetQuaternion(), atEnd.GetQuaternion())),
                Is.GreaterThan(0.99999f), "t=1 must return the end orientation");
        }

        [Test]
        public void InterpolateOrientation_ClampsParameter()
        {
            var start = new ToolOrientation(10, 0, 0);
            var end = new ToolOrientation(60, 0, 0);

            var below = CoordinateTransform.InterpolateOrientation(start, end, -0.5f);
            var above = CoordinateTransform.InterpolateOrientation(start, end, 1.5f);

            Assert.That(ToolOrientation.AngularDistanceDegrees(below, start), Is.LessThan(1e-3f),
                "t below 0 must clamp to the start orientation");
            Assert.That(ToolOrientation.AngularDistanceDegrees(above, end), Is.LessThan(1e-3f),
                "t above 1 must clamp to the end orientation");
        }
    }
}
