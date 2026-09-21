using MillSimSharp.Simulation;
using NUnit.Framework;

namespace MillSimSharp.Tests.Simulation
{
    /// <summary>
    /// Tests for interpolation step computation and adaptive sampling.
    /// </summary>
    [TestFixture]
    public class SimulationSettingsTest
    {
        [Test]
        public void ComputeSteps_CombinesLinearAndAngularMotion()
        {
            var settings = new SimulationSettings
            {
                MaxLinearStep = 1f,
                MaxAngularStep = 2f,
                MinimumSteps = 1
            };

            Assert.That(settings.ComputeSteps(10f, 0f), Is.EqualTo(10), "10mm at 1mm/step");
            Assert.That(settings.ComputeSteps(0f, 90f), Is.EqualTo(45), "90 degrees at 2 deg/step");
            Assert.That(settings.ComputeSteps(0.1f, 3f), Is.EqualTo(2), "angular motion dominates");
            Assert.That(settings.ComputeSteps(0f, 0f), Is.EqualTo(1), "at least one step");
        }

        [Test]
        public void ComputeSteps_AdaptiveRefinesBallRotation()
        {
            var settings = new SimulationSettings
            {
                MaxLinearStep = 1000f,
                MaxAngularStep = 180f,
                MaxChordError = 0.25f,
                EnableAdaptiveSampling = true
            };

            // Ball radius 5: a 180 degree rotation needs at least 5 steps for 0.25mm chord error
            int steps = settings.ComputeSteps(0f, 180f, cuttingCenterOffset: 5f);
            Assert.That(steps, Is.GreaterThanOrEqualTo(5));

            // Flat tool (offset 0) has no curved cutting center
            Assert.That(settings.ComputeSteps(0f, 180f, cuttingCenterOffset: 0f), Is.EqualTo(1));
        }

        [Test]
        public void ComputeSteps_AdaptiveCanBeDisabled()
        {
            var settings = new SimulationSettings
            {
                MaxLinearStep = 1000f,
                MaxAngularStep = 180f,
                EnableAdaptiveSampling = false
            };

            Assert.That(settings.ComputeSteps(0f, 180f, cuttingCenterOffset: 5f), Is.EqualTo(1));
        }
    }
}
