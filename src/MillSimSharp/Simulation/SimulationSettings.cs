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
        /// <exception cref="ArgumentOutOfRangeException">Thrown when the required step count exceeds
        /// <see cref="int.MaxValue"/> (for example an extremely long move with a very small
        /// <see cref="MaxLinearStep"/>). Increase the step size or shorten the move.</exception>
        public int ComputeSteps(float linearDistance, float angularDistanceDegrees)
        {
            int linearSteps = ComputeStepCount(linearDistance, MaxLinearStep, nameof(linearDistance), nameof(MaxLinearStep));
            int angularSteps = ComputeStepCount(angularDistanceDegrees, MaxAngularStep, nameof(angularDistanceDegrees), nameof(MaxAngularStep));

            int steps = Math.Max(MinimumSteps, Math.Max(linearSteps, angularSteps));
            return Math.Max(1, steps);
        }

        /// <summary>
        /// Computes one step count in double precision and rejects values that do not fit in
        /// <see cref="int"/> instead of wrapping into an undersampled move. The configured step size
        /// is used as given (no hidden floor).
        /// </summary>
        private static int ComputeStepCount(float distance, float maxStep, string distanceName, string settingName)
        {
            double required = Math.Ceiling((double)distance / maxStep);
            if (!(required <= int.MaxValue))
            {
                throw new ArgumentOutOfRangeException(distanceName, distance,
                    $"The move requires {required} interpolation steps, which exceeds the supported maximum of {int.MaxValue}. Increase {settingName} or shorten the move.");
            }

            return (int)required;
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
        /// <exception cref="ArgumentOutOfRangeException">Thrown when the required step count exceeds
        /// <see cref="int.MaxValue"/> (for example an extremely small <see cref="MaxChordError"/> for
        /// the given rotation radius). Increase <see cref="MaxChordError"/>.</exception>
        public int ComputeSteps(float linearDistance, float angularDistanceDegrees, float cuttingCenterOffset)
        {
            int steps = ComputeSteps(linearDistance, angularDistanceDegrees);

            if (!EnableAdaptiveSampling || cuttingCenterOffset <= 0f || angularDistanceDegrees <= 0f)
                return steps;

            float angleRad = angularDistanceDegrees * MathF.PI / 180f;

            // The cutting center travels on an arc of radius = offset. For n steps the sagitta is
            // r * (1 - cos(angle / (2n))) <= chordError, so n >= angle / (2 * acos(1 - chordError / r)).
            // The configured chord error is used as given (no hidden floor); the computation runs in
            // double so values below the old 1e-4 clamp are honored.
            double chordFraction = (double)MaxChordError / cuttingCenterOffset;
            if (!(chordFraction < 1.0))
            {
                // An allowed chord error at or above the sweep radius is satisfied by one step.
                return Math.Max(steps, 1);
            }

            // For very small fractions the acos argument loses precision, so the series
            // acos(1 - x) ~ sqrt(2x) * (1 + x / 12) is used instead.
            double denominator = chordFraction < 1e-8
                ? 2.0 * Math.Sqrt(2.0 * chordFraction) * (1.0 + chordFraction / 12.0)
                : 2.0 * Math.Acos(1.0 - chordFraction);

            if (denominator <= 0.0)
            {
                // Unreachable for a positive chord fraction; keeps the guard explicit.
                return steps;
            }

            double required = angleRad / denominator;
            if (!(required <= int.MaxValue))
            {
                throw new ArgumentOutOfRangeException(nameof(MaxChordError), MaxChordError,
                    $"The rotation requires {required} adaptive interpolation steps, which exceeds the supported maximum of {int.MaxValue}. Increase MaxChordError.");
            }

            return Math.Max(steps, (int)Math.Ceiling(required));
        }
    }
}
