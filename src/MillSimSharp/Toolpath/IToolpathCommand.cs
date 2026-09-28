using System.Numerics;
using MillSimSharp.Simulation;

namespace MillSimSharp.Toolpath
{
    /// <summary>
    /// Interface for all toolpath commands.
    /// <para>
    /// <see cref="Execute"/> is the extension point for custom commands. The executor normalizes
    /// the built-in move commands (<see cref="G0Move"/>, <see cref="G1Move"/>, <see cref="G0Move5Axis"/>,
    /// <see cref="G1Move5Axis"/>, matched by type) into a single pose-sweep plan, so step / batch /
    /// single execution share one path. Every other command type is executed through this interface.
    /// </para>
    /// </summary>
    public interface IToolpathCommand
    {
        /// <summary>
        /// Executes the command using the provided simulator and tool.
        /// </summary>
        /// <param name="simulator">The cutter simulator to use.</param>
        /// <param name="tool">The tool to use.</param>
        /// <param name="currentPosition">Current position (updated after execution).</param>
        void Execute(ICutterSimulator simulator, Tool tool, ref Vector3 currentPosition);
    }
}
