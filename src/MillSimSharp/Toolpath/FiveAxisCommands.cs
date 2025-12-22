using System;
using System.Numerics;
using MillSimSharp.Simulation;

namespace MillSimSharp.Toolpath
{
    /// <summary>
    /// 5-axis linear movement command with tool orientation.
    /// </summary>
    public class G1Move5Axis : IToolpathCommand
    {
        /// <summary>
        /// Target position.
        /// </summary>
        public Vector3 Target { get; }

        /// <summary>
        /// Tool orientation at the target.
        /// </summary>
        public ToolOrientation Orientation { get; }

        /// <summary>
        /// Feed rate in mm/min.
        /// </summary>
        public float FeedRate { get; }

        /// <summary>
        /// Number of interpolation steps for orientation changes.
        /// </summary>
        public int InterpolationSteps { get; }

        /// <summary>
        /// Creates a new 5-axis linear move command.
        /// </summary>
        /// <param name="target">Target position.</param>
        /// <param name="orientation">Tool orientation at target.</param>
        /// <param name="feedRate">Feed rate in mm/min.</param>
        /// <param name="interpolationSteps">Number of steps for smooth orientation changes (default: 20).</param>
        public G1Move5Axis(Vector3 target, ToolOrientation orientation, float feedRate = 100f, int interpolationSteps = 20)
        {
            Target = target;
            Orientation = orientation;
            FeedRate = feedRate;
            InterpolationSteps = Math.Max(1, interpolationSteps);
        }

        /// <summary>
        /// Executes the 5-axis movement.
        /// </summary>
        public void Execute(ICutterSimulator simulator, Tool tool, ref Vector3 currentPosition)
        {
            Execute(simulator, tool, ref currentPosition, ToolOrientation.Default);
        }

        /// <summary>
        /// Executes the 5-axis movement with current orientation.
        /// </summary>
        /// <param name="simulator">Cutter simulator.</param>
        /// <param name="tool">Cutting tool.</param>
        /// <param name="currentPosition">Current position (updated after execution).</param>
        /// <param name="currentOrientation">Current tool orientation.</param>
        public void Execute(ICutterSimulator simulator, Tool tool, ref Vector3 currentPosition, ToolOrientation currentOrientation)
        {
            if (simulator == null) throw new ArgumentNullException(nameof(simulator));
            if (tool == null) throw new ArgumentNullException(nameof(tool));

            // Calculate the total distance
            Vector3 direction = Target - currentPosition;
            float distance = direction.Length();

            if (distance < 0.001f)
            {
                currentPosition = Target;
                return;
            }

            // Use 5-axis cutting with orientation
            simulator.CutLinearWithOrientation(currentPosition, Target, tool, currentOrientation, Orientation);

            currentPosition = Target;
        }

        public override string ToString()
        {
            return $"G1 5-Axis to ({Target.X:F3}, {Target.Y:F3}, {Target.Z:F3}) {Orientation} F{FeedRate:F1}";
        }
    }

    /// <summary>
    /// 5-axis rapid positioning command.
    /// </summary>
    public class G0Move5Axis : IToolpathCommand
    {
        /// <summary>
        /// Target position.
        /// </summary>
        public Vector3 Target { get; }

        /// <summary>
        /// Tool orientation at the target.
        /// </summary>
        public ToolOrientation Orientation { get; }

        /// <summary>
        /// Creates a new 5-axis rapid move command.
        /// </summary>
        /// <param name="target">Target position.</param>
        /// <param name="orientation">Tool orientation at target.</param>
        public G0Move5Axis(Vector3 target, ToolOrientation orientation)
        {
            Target = target;
            Orientation = orientation;
        }

        /// <summary>
        /// Executes the rapid positioning (no cutting).
        /// </summary>
        public void Execute(ICutterSimulator simulator, Tool tool, ref Vector3 currentPosition)
        {
            if (simulator == null) throw new ArgumentNullException(nameof(simulator));
            if (tool == null) throw new ArgumentNullException(nameof(tool));

            // Rapid positioning does not cut material
            currentPosition = Target;
        }

        public override string ToString()
        {
            return $"G0 5-Axis to ({Target.X:F3}, {Target.Y:F3}, {Target.Z:F3}) {Orientation}";
        }
    }
}
