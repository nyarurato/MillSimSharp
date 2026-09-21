using System;

namespace MillSimSharp.Simulation
{
    /// <summary>
    /// Sampling settings for pose interpolation during cutting simulation.
    /// The number of interpolation steps is derived from both linear and angular motion, so that
    /// rotation-only moves and large orientation changes are simulated correctly.
    /// </summary>
    public class SimulationSettings
    {
        /// <summary>
        /// Maximum linear interpolation step in millimeters.
        /// Default is set by each simulator to 0.5 * voxel resolution.
        /// </summary>
        public float MaxLinearStep { get; set; } = 0.5f;

        /// <summary>
        /// Maximum angular interpolation step in degrees (default: 2 degrees).
        /// </summary>
        public float MaxAngularStep { get; set; } = 2f;

        /// <summary>
        /// Minimum number of interpolation steps per cutting command (default: 1).
        /// </summary>
        public int MinimumSteps { get; set; } = 1;

        /// <summary>
        /// Computes the number of interpolation steps for a move based on linear and angular motion.
        /// </summary>
        /// <param name="linearDistance">Linear distance in millimeters.</param>
        /// <param name="angularDistanceDegrees">Shortest angular distance in degrees.</param>
        /// <returns>Number of steps (at least <see cref="MinimumSteps"/> and 1).</returns>
        public int ComputeSteps(float linearDistance, float angularDistanceDegrees)
        {
            int linearSteps = (int)MathF.Ceiling(linearDistance / MathF.Max(MaxLinearStep, 1e-6f));
            int angularSteps = (int)MathF.Ceiling(angularDistanceDegrees / MathF.Max(MaxAngularStep, 1e-6f));

            int steps = Math.Max(MinimumSteps, Math.Max(linearSteps, angularSteps));
            return Math.Max(1, steps);
        }
    }
}
