using System.Numerics;
using MillSimSharp.Simulation;

namespace MillSimSharp.Toolpath
{
    /// <summary>
    /// Interface for all toolpath commands.
    /// </summary>
    public interface IToolpathCommand
    {
        /// <summary>
        /// Executes the command using the provided simulator and tool.
        /// </summary>
        /// <param name="simulator">The cutter simulator to use.</param>
        /// <param name="tool">The tool to use.</param>
        /// <param name="currentPosition">Current position (updated after execution).</param>
        void Execute(CutterSimulator simulator, Tool tool, ref Vector3 currentPosition);
    }
}
