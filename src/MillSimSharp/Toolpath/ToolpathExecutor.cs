using System;
using System.Collections.Generic;
using System.Numerics;
using MillSimSharp.Simulation;

namespace MillSimSharp.Toolpath
{
    /// <summary>
    /// Executes a sequence of toolpath commands.
    /// </summary>
    public class ToolpathExecutor
    {
        private readonly CutterSimulator _simulator;
        private readonly Tool _tool;

        /// <summary>
        /// Current tool position.
        /// </summary>
        public Vector3 CurrentPosition { get; private set; }

        public ToolpathExecutor(CutterSimulator simulator, Tool tool, Vector3 initialPosition)
        {
            _simulator = simulator ?? throw new ArgumentNullException(nameof(simulator));
            _tool = tool ?? throw new ArgumentNullException(nameof(tool));
            CurrentPosition = initialPosition;
        }

        /// <summary>
        /// Executes all commands in sequence.
        /// </summary>
        /// <param name="commands">List of commands to execute.</param>
        public void ExecuteCommands(IEnumerable<IToolpathCommand> commands)
        {
            if (commands == null) throw new ArgumentNullException(nameof(commands));

            var position = CurrentPosition;
            foreach (var command in commands)
            {
                command.Execute(_simulator, _tool, ref position);
            }
            CurrentPosition = position;
        }

        /// <summary>
        /// Executes a single command.
        /// </summary>
        /// <param name="command">Command to execute.</param>
        public void ExecuteCommand(IToolpathCommand command)
        {
            if (command == null) throw new ArgumentNullException(nameof(command));
            var position = CurrentPosition;
            command.Execute(_simulator, _tool, ref position);
            CurrentPosition = position;
        }
    }
}
