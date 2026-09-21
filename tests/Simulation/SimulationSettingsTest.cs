using System;
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

        [Test]
        public void SimulationSettings_NonPositiveOrNonFiniteValues_Throw()
        {
            var settings = new SimulationSettings();

            Assert.Throws<ArgumentOutOfRangeException>(() => settings.MaxLinearStep = 0f);
            Assert.Throws<ArgumentOutOfRangeException>(() => settings.MaxLinearStep = -1f);
            Assert.Throws<ArgumentOutOfRangeException>(() => settings.MaxLinearStep = float.NaN);
            Assert.Throws<ArgumentOutOfRangeException>(() => settings.MaxLinearStep = float.PositiveInfinity);

            Assert.Throws<ArgumentOutOfRangeException>(() => settings.MaxAngularStep = 0f);
            Assert.Throws<ArgumentOutOfRangeException>(() => settings.MaxAngularStep = float.NaN);

            Assert.Throws<ArgumentOutOfRangeException>(() => settings.MaxChordError = -0.1f);
            Assert.Throws<ArgumentOutOfRangeException>(() => settings.MaxChordError = float.NaN);

            Assert.Throws<ArgumentOutOfRangeException>(() => settings.MinimumSteps = 0);
            Assert.Throws<ArgumentOutOfRangeException>(() => settings.MinimumSteps = -5);
        }

        [Test]
        public void SimulationSettings_DefaultsAndValidBoundaryValues_AreAccepted()
        {
            var defaults = new SimulationSettings();
            Assert.That(defaults.MaxLinearStep, Is.EqualTo(0.5f));
            Assert.That(defaults.MaxAngularStep, Is.EqualTo(2f));
            Assert.That(defaults.MinimumSteps, Is.EqualTo(1));
            Assert.That(defaults.MaxChordError, Is.EqualTo(0.25f));
            Assert.That(defaults.EnableAdaptiveSampling, Is.True);

            // The smallest sensible positive finite values must be accepted without clamping.
            var settings = new SimulationSettings
            {
                MaxLinearStep = 1e-6f,
                MaxAngularStep = 1e-6f,
                MaxChordError = 1e-6f,
                MinimumSteps = 1,
            };

            Assert.That(settings.MaxLinearStep, Is.EqualTo(1e-6f));
            Assert.That(settings.MaxAngularStep, Is.EqualTo(1e-6f));
            Assert.That(settings.MaxChordError, Is.EqualTo(1e-6f));
            Assert.That(settings.MinimumSteps, Is.EqualTo(1));
        }
    }
}
