using System;
using System.Numerics;
using MillSimSharp.Simulation;

namespace MillSimSharp.Toolpath
{
    /// <summary>
    /// G0 command: Rapid positioning (non-cutting move).
    /// </summary>
    public class G0Move : IToolpathCommand
    {
        /// <summary>
        /// Target position for the rapid move.
        /// </summary>
        public Vector3 Target { get; }

        public G0Move(Vector3 target)
        {
            Target = target;
        }

        public void Execute(ICutterSimulator simulator, Tool tool, ref Vector3 currentPosition)
        {
            if (simulator == null) throw new ArgumentNullException(nameof(simulator));
            if (tool == null) throw new ArgumentNullException(nameof(tool));

            // G0 is a rapid move - no material removal
            // Just update the position
            currentPosition = Target;
        }
    }
}
