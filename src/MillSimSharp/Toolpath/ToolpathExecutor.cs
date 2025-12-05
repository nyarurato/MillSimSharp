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
        private List<IToolpathCommand>? _commands;
        private int _currentCommandIndex = -1;
        private Vector3 _initialPosition;

        /// <summary>
        /// Current tool position.
        /// </summary>
        public Vector3 CurrentPosition { get; private set; }
        
        /// <summary>
        /// Number of commands to execute per step (default: 1).
        /// </summary>
        public int StepSize { get; set; } = 1;
        
        /// <summary>
        /// Current command index in the loaded command list.
        /// </summary>
        public int CurrentCommandIndex => _currentCommandIndex;
        
        /// <summary>
        /// Total number of commands loaded.
        /// </summary>
        public int TotalCommands => _commands?.Count ?? 0;
        
        /// <summary>
        /// Whether all commands have been executed.
        /// </summary>
        public bool IsCompleted => _currentCommandIndex >= (TotalCommands - 1);

        public ToolpathExecutor(CutterSimulator simulator, Tool tool, Vector3 initialPosition)
        {
            _simulator = simulator ?? throw new ArgumentNullException(nameof(simulator));
            _tool = tool ?? throw new ArgumentNullException(nameof(tool));
            _initialPosition = initialPosition;
            CurrentPosition = initialPosition;
        }
        
        /// <summary>
        /// Load commands for step-by-step execution.
        /// </summary>
        /// <param name="commands">List of commands to execute.</param>
        public void LoadCommands(IEnumerable<IToolpathCommand> commands)
        {
            _commands = new List<IToolpathCommand>(commands);
            _currentCommandIndex = -1;
            CurrentPosition = _initialPosition;
        }
        
        /// <summary>
        /// Execute the next step(s) based on StepSize.
        /// </summary>
        /// <param name="count">Number of commands to execute. If -1, uses StepSize.</param>
        /// <returns>Number of commands actually executed.</returns>
        public int ExecuteNextSteps(int count = -1)
        {
            if (_commands == null) return 0;
            
            int stepsToExecute = count > 0 ? count : StepSize;
            int executed = 0;
            
            while (executed < stepsToExecute && _currentCommandIndex < _commands.Count - 1)
            {
                _currentCommandIndex++;
                var command = _commands[_currentCommandIndex];
                var position = CurrentPosition;
                command.Execute(_simulator, _tool, ref position);
                CurrentPosition = position;
                executed++;
            }
            
            return executed;
        }
        
        /// <summary>
        /// Reset to initial state.
        /// </summary>
        public void Reset()
        {
            _currentCommandIndex = -1;
            CurrentPosition = _initialPosition;
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
