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
        private Tool _initialTool;
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

        private int _stepSize = 1;

        /// <summary>
        /// Number of commands to execute per step (default: 1). Must be at least 1.
        /// </summary>
        public int StepSize
        {
            get => _stepSize;
            set
            {
                if (value < 1)
                {
                    throw new ArgumentOutOfRangeException(nameof(StepSize), value, "StepSize must be at least 1.");
                }

                _stepSize = value;
            }
        }

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
            _initialTool = _tool;
            _initialPosition = initialPosition;
            _initialOrientation = initialOrientation ?? ToolOrientation.Default;
            CurrentPosition = initialPosition;
            CurrentOrientation = _initialOrientation;
        }

        /// <summary>
        /// Changes the active tool and makes it the baseline for <see cref="Reset"/> /
        /// <see cref="LoadCommands"/>. Execution state (position, orientation, loaded commands,
        /// progress and estimated time) is preserved.
        /// </summary>
        /// <param name="tool">New tool to use.</param>
        /// <exception cref="ArgumentNullException">Thrown when <paramref name="tool"/> is null.</exception>
        public void ChangeTool(Tool tool)
        {
            if (tool == null) throw new ArgumentNullException(nameof(tool));

            _tool = tool;
            _initialTool = tool;
        }

        /// <summary>
        /// Load commands for step-by-step execution. Resets the executor state (command cursor,
        /// pose, tool baseline and estimated time) like <see cref="Reset"/>; it does not restore
        /// stock material removed through the simulator.
        /// </summary>
        /// <param name="commands">List of commands to execute.</param>
        public void LoadCommands(IEnumerable<IToolpathCommand> commands)
        {
            if (commands == null) throw new ArgumentNullException(nameof(commands));
            _commands = new List<IToolpathCommand>(commands);
            _currentCommandIndex = -1;
            _tool = _initialTool;
            CurrentPosition = _initialPosition;
            CurrentOrientation = _initialOrientation;
            EstimatedTimeSeconds = 0;
        }

        /// <summary>
        /// Execute the next step(s) based on StepSize.
        /// </summary>
        /// <remarks>
        /// <para>
        /// The command cursor (<see cref="CurrentCommandIndex"/>) is committed only after a command
        /// returns without throwing. A command that throws is therefore retried by the next call
        /// instead of being skipped, and a failed final command leaves <see cref="IsCompleted"/>
        /// false.
        /// </para>
        /// <para>
        /// Material removal is not transactional: a command that throws part-way through may leave
        /// partial cuts in the stock even though the cursor is not advanced. The cursor rollback is
        /// not an undo, and <see cref="ProgressChanged"/> only fires for committed commands.
        /// </para>
        /// </remarks>
        /// <param name="count">Number of commands to execute. If -1, uses StepSize.</param>
        /// <returns>Number of commands actually executed.</returns>
        public int ExecuteNextSteps(int count = -1)
        {
            if (_commands == null) return 0;

            int stepsToExecute = count > 0 ? count : StepSize;
            int executed = 0;

            while (executed < stepsToExecute && _currentCommandIndex < _commands.Count - 1)
            {
                int nextIndex = _currentCommandIndex + 1;
                ExecuteCore(_commands[nextIndex]);
                _currentCommandIndex = nextIndex;
                ProgressChanged?.Invoke(_currentCommandIndex + 1, TotalCommands);
                executed++;
            }

            return executed;
        }

        /// <summary>
        /// Reset to initial state: command cursor, pose, tool baseline and estimated time.
        /// <para>
        /// This resets the executor only. Stock material removed through the simulator is not
        /// restored (the executor does not own the material state).
        /// </para>
        /// </summary>
        public void Reset()
        {
            _currentCommandIndex = -1;
            _tool = _initialTool;
            CurrentPosition = _initialPosition;
            CurrentOrientation = _initialOrientation;
            EstimatedTimeSeconds = 0;
        }

        /// <summary>
        /// Executes a single command while keeping position and orientation state consistent.
        /// All execution paths (batch, single, step-by-step) share this method. The built-in move
        /// commands are normalized to one pose-sweep plan; custom commands keep the public
        /// <see cref="IToolpathCommand.Execute"/> path.
        /// </summary>
        private void ExecuteCore(IToolpathCommand command)
        {
            // Tool changes only affect the executor state
            if (command is ToolChange toolChange)
            {
                _tool = toolChange.NewTool;
                return;
            }

            Vector3 positionBefore = CurrentPosition;

            if (TryCreateMovePlan(command, out MovePlan plan))
            {
                ApplyMove(plan);
                AccumulateMoveTime(plan, positionBefore, CurrentPosition);
                return;
            }

            // Custom command: execute through the public interface. The position is only committed
            // when Execute returns, the orientation is kept and no time is estimated.
            Vector3 position = CurrentPosition;
            command.Execute(_simulator, _tool, ref position);
            CurrentPosition = position;
        }

        /// <summary>
        /// Normalizes the built-in move commands into a single pose-sweep plan. Returns false for
        /// custom commands, which are executed through <see cref="IToolpathCommand.Execute"/>.
        /// </summary>
        private bool TryCreateMovePlan(IToolpathCommand command, out MovePlan plan)
        {
            switch (command)
            {
                case G1Move5Axis g1Move5Axis:
                    plan = new MovePlan(CurrentPosition, g1Move5Axis.Target,
                        CurrentOrientation, g1Move5Axis.Orientation,
                        EffectiveFeedRate(g1Move5Axis.FeedRate), cutsMaterial: true);
                    return true;
                case G0Move5Axis g0Move5Axis:
                    plan = new MovePlan(CurrentPosition, g0Move5Axis.Target,
                        CurrentOrientation, g0Move5Axis.Orientation,
                        RapidFeedRate, cutsMaterial: false);
                    return true;
                case G1Move g1Move:
                    // A normal G1 keeps the current orientation: after a 5-axis move it must not
                    // silently fall back to the default 3-axis pose. For the default orientation
                    // this is identical to CutLinear.
                    plan = new MovePlan(CurrentPosition, g1Move.Target,
                        CurrentOrientation, CurrentOrientation,
                        EffectiveFeedRate(g1Move.FeedRate), cutsMaterial: true);
                    return true;
                case G0Move g0Move:
                    plan = new MovePlan(CurrentPosition, g0Move.Target,
                        CurrentOrientation, CurrentOrientation,
                        RapidFeedRate, cutsMaterial: false);
                    return true;
                default:
                    plan = default;
                    return false;
            }
        }

        /// <summary>
        /// Applies a normalized move: cutting moves sweep the tool to the new pose, rapid moves
        /// only update the pose.
        /// </summary>
        private void ApplyMove(in MovePlan plan)
        {
            if (plan.CutsMaterial)
            {
                _simulator.CutLinearWithOrientation(
                    plan.Start, plan.End, _tool, plan.StartOrientation, plan.EndOrientation);
            }

            CurrentPosition = plan.End;
            CurrentOrientation = plan.EndOrientation;
        }

        /// <summary>
        /// Effective feed rate for the time estimate: a missing (non-positive) G1 feed falls back
        /// to <see cref="RapidFeedRate"/>.
        /// </summary>
        private float EffectiveFeedRate(float feedRate)
        {
            return feedRate > 0f ? feedRate : RapidFeedRate;
        }

        private void AccumulateMoveTime(in MovePlan plan, Vector3 from, Vector3 to)
        {
            // Mirrors the previous "feed > 0" guard, which also skipped NaN.
            if (!(plan.FeedRate > 0f)) return;

            float distance = Vector3.Distance(from, to);
            if (distance <= 0f) return;

            EstimatedTimeSeconds += distance / plan.FeedRate * 60.0; // mm / (mm/min) -> minutes -> seconds
        }

        /// <summary>
        /// Normalized move: start / end pose, effective feed rate for the time estimate and whether
        /// the move removes material (G1) or is a rapid positioning move (G0).
        /// </summary>
        private readonly struct MovePlan
        {
            public MovePlan(Vector3 start, Vector3 end,
                ToolOrientation startOrientation, ToolOrientation endOrientation,
                float feedRate, bool cutsMaterial)
            {
                Start = start;
                End = end;
                StartOrientation = startOrientation;
                EndOrientation = endOrientation;
                FeedRate = feedRate;
                CutsMaterial = cutsMaterial;
            }

            public Vector3 Start { get; }
            public Vector3 End { get; }
            public ToolOrientation StartOrientation { get; }
            public ToolOrientation EndOrientation { get; }
            public float FeedRate { get; }
            public bool CutsMaterial { get; }
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
