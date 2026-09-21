using System;
using System.Numerics;
using MillSimSharp.Simulation;

namespace MillSimSharp.Toolpath
{
    /// <summary>
    /// Changes the active tool for subsequent commands.
    /// The command itself does not modify the stock; the <see cref="ToolpathExecutor"/> applies it.
    /// </summary>
    public class ToolChange : IToolpathCommand
    {
        /// <summary>
        /// New tool to use for subsequent commands.
        /// </summary>
        public Tool NewTool { get; }

        /// <summary>
        /// Creates a tool change command.
        /// </summary>
        /// <param name="newTool">New tool.</param>
        public ToolChange(Tool newTool)
        {
            NewTool = newTool ?? throw new ArgumentNullException(nameof(newTool));
        }

        /// <summary>
        /// No-op on the simulator; handled by the executor.
        /// </summary>
        public void Execute(ICutterSimulator simulator, Tool tool, ref Vector3 currentPosition)
        {
            if (simulator == null) throw new ArgumentNullException(nameof(simulator));
            if (tool == null) throw new ArgumentNullException(nameof(tool));
        }

        /// <inheritdoc />
        public override string ToString()
        {
            return $"ToolChange to D{NewTool.Diameter:F1}mm L{NewTool.Length:F1}mm";
        }
    }
}
