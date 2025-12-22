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
        private readonly ICutterSimulator _simulator;
        private readonly Tool _tool;
        private List<IToolpathCommand>? _commands;
        private int _currentCommandIndex = -1;
        private Vector3 _initialPosition;
        private ToolOrientation _initialOrientation;

        /// <summary>
        /// Current tool position.
        /// </summary>
        public Vector3 CurrentPosition { get; private set; }

        /// <summary>
        /// Current tool orientation (for 5-axis machining).
        /// </summary>
        public ToolOrientation CurrentOrientation { get; private set; }
        
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

        /// <summary>
        /// Creates a new ToolpathExecutor.
        /// </summary>
        /// <param name="simulator"></param>
        /// <param name="tool"></param>
        /// <param name="initialPosition"></param>
        /// <param name="initialOrientation">Initial tool orientation (optional, for 5-axis).</param>
        /// <exception cref="ArgumentNullException"></exception>
        public ToolpathExecutor(ICutterSimulator simulator, Tool tool, Vector3 initialPosition, ToolOrientation? initialOrientation = null)
        {
            _simulator = simulator ?? throw new ArgumentNullException(nameof(simulator));
            _tool = tool ?? throw new ArgumentNullException(nameof(tool));
            _initialPosition = initialPosition;
            _initialOrientation = initialOrientation ?? ToolOrientation.Default;
            CurrentPosition = initialPosition;
            CurrentOrientation = _initialOrientation;
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
            CurrentOrientation = _initialOrientation;
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
            CurrentOrientation = _initialOrientation;
        }

        /// <summary>
        /// Executes all commands in sequence.
        /// </summary>
        /// <param name="commands">List of commands to execute.</param>
        public void ExecuteCommands(IEnumerable<IToolpathCommand> commands)
        {
            if (commands == null) throw new ArgumentNullException(nameof(commands));

            var position = CurrentPosition;
            var orientation = CurrentOrientation;
            
            foreach (var command in commands)
            {
                // Check if it's a 5-axis command that needs orientation
                if (command is G1Move5Axis g1Move5Axis)
                {
                    g1Move5Axis.Execute(_simulator, _tool, ref position, orientation);
                    orientation = g1Move5Axis.Orientation;
                }
                else if (command is G0Move5Axis g0Move5Axis)
                {
                    position = g0Move5Axis.Target;
                    orientation = g0Move5Axis.Orientation;
                }
                else
                {
                    command.Execute(_simulator, _tool, ref position);
                }
            }
            
            CurrentPosition = position;
            CurrentOrientation = orientation;
        }

        /// <summary>
        /// Executes a single command.
        /// </summary>
        /// <param name="command">Command to execute.</param>
        public void ExecuteCommand(IToolpathCommand command)
        {
            if (command == null) throw new ArgumentNullException(nameof(command));
            
            var position = CurrentPosition;
            var orientation = CurrentOrientation;
            
            // Check if it's a 5-axis command that needs orientation
            if (command is G1Move5Axis g1Move5Axis)
            {
                g1Move5Axis.Execute(_simulator, _tool, ref position, orientation);
                orientation = g1Move5Axis.Orientation;
            }
            else if (command is G0Move5Axis g0Move5Axis)
            {
                position = g0Move5Axis.Target;
                orientation = g0Move5Axis.Orientation;
            }
            else
            {
                command.Execute(_simulator, _tool, ref position);
            }
            
            CurrentPosition = position;
            CurrentOrientation = orientation;
        }
    }
}
