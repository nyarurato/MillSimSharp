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
        private float _maxLinearStep = 0.5f;
        private float _maxAngularStep = 2f;
        private int _minimumSteps = 1;
        private float _maxChordError = 0.25f;

        /// <summary>
        /// Maximum linear interpolation step in millimeters.
        /// Must be a finite positive value (default 0.5; each simulator sets 0.5 * resolution).
        /// </summary>
        public float MaxLinearStep
        {
            get => _maxLinearStep;
            set => _maxLinearStep = ValidatePositiveFinite(value, nameof(MaxLinearStep));
        }

        /// <summary>
        /// Maximum angular interpolation step in degrees (default 2 degrees).
        /// Must be a finite positive value.
        /// </summary>
        public float MaxAngularStep
        {
            get => _maxAngularStep;
            set => _maxAngularStep = ValidatePositiveFinite(value, nameof(MaxAngularStep));
        }

        /// <summary>
        /// Minimum number of interpolation steps per cutting command (default 1).
        /// Must be at least 1.
        /// </summary>
        public int MinimumSteps
        {
            get => _minimumSteps;
            set
            {
                if (value < 1)
                {
                    throw new ArgumentOutOfRangeException(nameof(MinimumSteps), value,
                        "MinimumSteps must be at least 1.");
                }

                _minimumSteps = value;
            }
        }

        /// <summary>
        /// Maximum chord deviation of the curved cutting-center path in millimeters (default: 0.25).
        /// Must be a finite positive value.
        /// </summary>
        public float MaxChordError
        {
            get => _maxChordError;
            set => _maxChordError = ValidatePositiveFinite(value, nameof(MaxChordError));
        }

        /// <summary>
        /// Enables feature-aware refinement so that the cutting-center chord error stays below
        /// <see cref="MaxChordError"/> (default: true).
        /// </summary>
        public bool EnableAdaptiveSampling { get; set; } = true;

        private static float ValidatePositiveFinite(float value, string propertyName)
        {
            if (!float.IsFinite(value) || value <= 0f)
            {
                throw new ArgumentOutOfRangeException(propertyName, value,
                    "Value must be a finite positive number.");
            }

            return value;
        }

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

        /// <summary>
        /// Computes the number of interpolation steps including an adaptive term for the curved
        /// path traced by tool points during rotation. Pass a conservative radius from the rotation
        /// pivot (physical tip) to the farthest cutting point; the simulators derive this from
        /// <see cref="IToolGeometry.LocalBounds"/> (for a ball end mill this covers the ball and
        /// flute, for flat tools the tool corner).
        /// </summary>
        /// <param name="linearDistance">Linear distance in millimeters.</param>
        /// <param name="angularDistanceDegrees">Shortest angular distance in degrees.</param>
        /// <param name="cuttingCenterOffset">
        /// Distance from the rotation pivot (physical tip) to the tracked cutting point in
        /// millimeters. The name is kept for API compatibility; callers should pass a conservative
        /// rotation sweep radius.
        /// </param>
        /// <returns>Number of steps.</returns>
        public int ComputeSteps(float linearDistance, float angularDistanceDegrees, float cuttingCenterOffset)
        {
            int steps = ComputeSteps(linearDistance, angularDistanceDegrees);

            if (!EnableAdaptiveSampling || cuttingCenterOffset <= 0f || angularDistanceDegrees <= 0f)
                return steps;

            float angleRad = angularDistanceDegrees * MathF.PI / 180f;
            float chordError = MathF.Max(MaxChordError, 1e-4f);

            // The cutting center travels on an arc of radius = offset. For n steps the sagitta is
            // r * (1 - cos(angle / (2n))) <= chordError, so n >= angle / (2 * acos(1 - chordError / r)).
            float ratio = 1f - chordError / cuttingCenterOffset;
            if (ratio < 1f)
            {
                float denominator = 2f * MathF.Acos(Math.Clamp(ratio, -1f, 1f));
                if (denominator > 1e-6f)
                {
                    int adaptiveSteps = (int)MathF.Ceiling(angleRad / denominator);
                    steps = Math.Max(steps, adaptiveSteps);
                }
            }

            return steps;
        }
    }
}
