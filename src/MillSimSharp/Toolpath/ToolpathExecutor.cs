using System;
using System.Collections.Generic;
using System.Numerics;
using System.Threading;
using MillSimSharp.Simulation;

namespace MillSimSharp.Toolpath
{
    /// <summary>
    /// Executes a sequence of toolpath commands.
    /// </summary>
    public class ToolpathExecutor
    {
        private readonly ICutterSimulator _simulator;
        private Tool _tool;
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
        /// Current tool (can change through <see cref="ToolChange"/> commands).
        /// </summary>
        public Tool CurrentTool => _tool;

        /// <summary>
        /// Feed rate used for rapid (G0) moves in mm/min (default: 5000).
        /// </summary>
        public float RapidFeedRate { get; set; } = 5000f;

        /// <summary>
        /// Estimated accumulated simulation time in seconds (feed-rate based).
        /// </summary>
        public double EstimatedTimeSeconds { get; private set; }

        /// <summary>
        /// Raised after each executed command with (executedCount, totalCount).
        /// </summary>
        public event Action<int, int>? ProgressChanged;

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
            if (commands == null) throw new ArgumentNullException(nameof(commands));
            _commands = new List<IToolpathCommand>(commands);
            _currentCommandIndex = -1;
            CurrentPosition = _initialPosition;
            CurrentOrientation = _initialOrientation;
            EstimatedTimeSeconds = 0;
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
                ExecuteCore(_commands[_currentCommandIndex]);
                ProgressChanged?.Invoke(_currentCommandIndex + 1, TotalCommands);
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
            EstimatedTimeSeconds = 0;
        }

        /// <summary>
        /// Executes a single command while keeping position and orientation state consistent.
        /// All execution paths (batch, single, step-by-step) share this method so that 5-axis
        /// orientation state is never lost.
        /// </summary>
        private void ExecuteCore(IToolpathCommand command)
        {
            // Tool changes only affect the executor state
            if (command is ToolChange toolChange)
            {
                _tool = toolChange.NewTool;
                return;
            }

            var position = CurrentPosition;
            var orientation = CurrentOrientation;
            Vector3 positionBefore = position;

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

            AccumulateTime(command, positionBefore, position);
        }

        private void AccumulateTime(IToolpathCommand command, Vector3 from, Vector3 to)
        {
            float distance = Vector3.Distance(from, to);
            if (distance <= 0f) return;

            float feed;
            switch (command)
            {
                case G1Move g1:
                    feed = g1.FeedRate > 0f ? g1.FeedRate : RapidFeedRate;
                    break;
                case G1Move5Axis g1FiveAxis:
                    feed = g1FiveAxis.FeedRate > 0f ? g1FiveAxis.FeedRate : RapidFeedRate;
                    break;
                case G0Move _:
                case G0Move5Axis _:
                    feed = RapidFeedRate;
                    break;
                default:
                    return;
            }

            if (feed > 0f)
            {
                EstimatedTimeSeconds += distance / feed * 60.0; // mm / (mm/min) -> minutes -> seconds
            }
        }

        /// <summary>
        /// Executes all commands in sequence.
        /// </summary>
        /// <param name="commands">List of commands to execute.</param>
        public void ExecuteCommands(IEnumerable<IToolpathCommand> commands)
        {
            ExecuteCommands(commands, CancellationToken.None);
        }

        /// <summary>
        /// Executes all commands in sequence with cancellation support.
        /// </summary>
        /// <param name="commands">List of commands to execute.</param>
        /// <param name="cancellationToken">Cancellation token checked before each command.</param>
        /// <exception cref="OperationCanceledException">Thrown when the token is cancelled.</exception>
        public void ExecuteCommands(IEnumerable<IToolpathCommand> commands, CancellationToken cancellationToken)
        {
            if (commands == null) throw new ArgumentNullException(nameof(commands));

            var list = commands as IList<IToolpathCommand> ?? new List<IToolpathCommand>(commands);
            for (int i = 0; i < list.Count; i++)
            {
                cancellationToken.ThrowIfCancellationRequested();
                ExecuteCore(list[i]);
                ProgressChanged?.Invoke(i + 1, list.Count);
            }
        }

        /// <summary>
        /// Executes a single command.
        /// </summary>
        /// <param name="command">Command to execute.</param>
        public void ExecuteCommand(IToolpathCommand command)
        {
            if (command == null) throw new ArgumentNullException(nameof(command));

            ExecuteCore(command);
        }
    }
}
