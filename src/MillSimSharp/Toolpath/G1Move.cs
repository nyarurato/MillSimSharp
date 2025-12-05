using System;
using System.Numerics;
using MillSimSharp.Simulation;

namespace MillSimSharp.Toolpath
{
    /// <summary>
    /// G1 command: Linear interpolation (cutting move).
    /// </summary>
    public class G1Move : IToolpathCommand
    {
        /// <summary>
        /// Target position for the linear cut.
        /// </summary>
        public Vector3 Target { get; }

        /// <summary>
        /// Feed rate in mm/min (optional, for future use).
        /// </summary>
        public float FeedRate { get; }

        /// <summary>
        /// Creates a new G1Move command.
        /// </summary>
        /// <param name="target"></param>
        /// <param name="feedRate"></param>
        public G1Move(Vector3 target, float feedRate = 0)
        {
            Target = target;
            FeedRate = feedRate;
        }

        /// <summary>
        /// Executes the G1 move on the given cutter simulator.
        /// </summary>
        /// <param name="simulator"></param>
        /// <param name="tool"></param>
        /// <param name="currentPosition"></param>
        /// <exception cref="ArgumentNullException"></exception>
        public void Execute(ICutterSimulator simulator, Tool tool, ref Vector3 currentPosition)
        {
            if (simulator == null) throw new ArgumentNullException(nameof(simulator));
            if (tool == null) throw new ArgumentNullException(nameof(tool));

            // G1 is a cutting move - remove material along the path
            simulator.CutLinear(currentPosition, Target, tool);
            
            // Update position
            currentPosition = Target;
        }
    }
}
