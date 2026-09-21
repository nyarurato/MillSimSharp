using NUnit.Framework;
using MillSimSharp.Simulation;
using System;
using System.Numerics;

namespace MillSimSharp.Tests.Simulation
{
    [TestFixture]
    public class ToolTest
    {
        [Test]
        public void TestFlatEndMill()
        {
            var tool = new EndMill(10.0f, 50.0f, isBallEnd: false);

            Assert.That(tool.Type, Is.EqualTo(ToolType.Flat));
            Assert.That(tool.Diameter, Is.EqualTo(10.0f));
            Assert.That(tool.Length, Is.EqualTo(50.0f));
            Assert.That(tool.BallCenterOffsetFromTip, Is.EqualTo(0f));
            Assert.That(tool.GetCuttingGeometry(), Is.InstanceOf<FlatEndMillGeometry>());
        }

        [Test]
        public void TestBallEndMill()
        {
            var tool = new EndMill(10.0f, 50.0f, isBallEnd: true);

            Assert.That(tool.Type, Is.EqualTo(ToolType.Ball));
            Assert.That(tool.BallCenterOffsetFromTip, Is.EqualTo(5.0f));

            var geometry = tool.GetCuttingGeometry();
            Assert.That(geometry, Is.InstanceOf<BallEndMillGeometry>());

            // The physical tip lies on the ball surface
            Assert.That(geometry.SignedDistance(Vector3.Zero), Is.EqualTo(0f).Within(1e-5f));
            Assert.That(geometry.SignedDistance(new Vector3(0, 0, -1f)), Is.EqualTo(1f).Within(1e-5f));
        }

        [Test]
        public void TestInvalidArguments()
        {
            Assert.Throws<ArgumentException>(() => new EndMill(-1.0f, 10.0f));
            Assert.Throws<ArgumentException>(() => new EndMill(10.0f, -1.0f));
        }

        [Test]
        public void TestGeometryInvalidArguments()
        {
            Assert.Throws<ArgumentException>(() => new FlatEndMillGeometry(0f, 10f));
            Assert.Throws<ArgumentException>(() => new FlatEndMillGeometry(1f, 0f));
            Assert.Throws<ArgumentException>(() => new BallEndMillGeometry(-1f, 10f));
        }

        [Test]
        public void TestNonFiniteDimensions_AreRejected()
        {
            Assert.Throws<ArgumentException>(() => new EndMill(float.NaN, 10f));
            Assert.Throws<ArgumentException>(() => new EndMill(10f, float.NaN));
            Assert.Throws<ArgumentException>(() => new EndMill(float.PositiveInfinity, 10f));
            Assert.Throws<ArgumentException>(() => new EndMill(10f, float.PositiveInfinity));

            Assert.Throws<ArgumentException>(() => new FlatEndMillGeometry(float.NaN, 10f));
            Assert.Throws<ArgumentException>(() => new FlatEndMillGeometry(5f, float.NaN));
            Assert.Throws<ArgumentException>(() => new BallEndMillGeometry(float.NaN, 10f));
            Assert.Throws<ArgumentException>(() => new BallEndMillGeometry(5f, float.NaN));
            Assert.Throws<ArgumentException>(() => new BullNoseEndMillGeometry(5f, float.NaN, 10f));
            Assert.Throws<ArgumentException>(() => new BullNoseEndMillGeometry(5f, 2f, float.NaN));
            Assert.Throws<ArgumentException>(() => new TaperedEndMillGeometry(float.NaN, 10f, 20f));
            Assert.Throws<ArgumentException>(() => new TaperedEndMillGeometry(2f, float.NaN, 20f));

            Assert.Throws<ArgumentException>(() => new BullNoseEndMill(10f, 30f, float.NaN));
            Assert.Throws<ArgumentException>(() => new TaperEndMill(4f, 20f, float.NaN));
        }
    }
}
