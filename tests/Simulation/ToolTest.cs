using NUnit.Framework;
using MillSimSharp.Simulation;
using System;

namespace MillSimSharp.Tests.Simulation
{
    [TestFixture]
    public class ToolTest
    {
        [Test]
        public void TestFlatEndMill()
        {
            // 10mm diameter, 50mm length
            var tool = new EndMill(10.0f, 50.0f, isBallEnd: false);

            Assert.That(tool.Type, Is.EqualTo(ToolType.Flat));
            Assert.That(tool.Diameter, Is.EqualTo(10.0f));
            Assert.That(tool.Length, Is.EqualTo(50.0f));

            // Radius should be constant (5.0f)
            Assert.That(tool.GetRadiusAtHeight(0.0f), Is.EqualTo(5.0f));
            Assert.That(tool.GetRadiusAtHeight(10.0f), Is.EqualTo(5.0f));
            Assert.That(tool.GetRadiusAtHeight(49.9f), Is.EqualTo(5.0f));
        }

        [Test]
        public void TestBallEndMill()
        {
            // 10mm diameter (5mm radius), 50mm length
            var tool = new EndMill(10.0f, 50.0f, isBallEnd: true);

            Assert.That(tool.Type, Is.EqualTo(ToolType.Ball));

            // At tip (h=0), radius should be 0
            Assert.That(tool.GetRadiusAtHeight(0.0f), Is.EqualTo(0.0f).Within(1e-5));

            // At center of ball (h=5), radius should be 5
            Assert.That(tool.GetRadiusAtHeight(5.0f), Is.EqualTo(5.0f).Within(1e-5));

            // At h=2.5 (halfway up radius), calculate expected radius
            // r = sqrt(R^2 - (R-h)^2) = sqrt(25 - (5-2.5)^2) = sqrt(25 - 6.25) = sqrt(18.75) ≈ 4.33
            float expected = MathF.Sqrt(25.0f - 6.25f);
            Assert.That(tool.GetRadiusAtHeight(2.5f), Is.EqualTo(expected).Within(1e-5));

            // Above the ball part (h > 5), radius should be constant 5
            Assert.That(tool.GetRadiusAtHeight(10.0f), Is.EqualTo(5.0f));
        }

        [Test]
        public void TestInvalidArguments()
        {
            Assert.Throws<ArgumentException>(() => new EndMill(-1.0f, 10.0f));
            Assert.Throws<ArgumentException>(() => new EndMill(10.0f, -1.0f));
        }

        [Test]
        public void TestOutOfBounds()
        {
            var tool = new EndMill(10.0f, 50.0f);
            
            // Negative height
            Assert.That(tool.GetRadiusAtHeight(-1.0f), Is.EqualTo(0.0f));
            
            // Height > Length
            Assert.That(tool.GetRadiusAtHeight(51.0f), Is.EqualTo(0.0f));
        }
    }
}
