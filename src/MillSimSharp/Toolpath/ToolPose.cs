using System;
using System.Numerics;
using MillSimSharp.Simulation;

namespace MillSimSharp.Toolpath
{
    /// <summary>
    /// Represents the pose of a cutting tool.
    /// <para>
    /// <see cref="Position"/> is always the <b>physical tool tip</b>, independent of tool type.
    /// For ball end mills the cutting ball center is derived as
    /// <c>Position + AxisTowardSpindle * tool.BallCenterOffsetFromTip</c>
    /// (see <see cref="GetCuttingCenter"/>).
    /// </para>
    /// </summary>
    public readonly struct ToolPose
    {
        /// <summary>
        /// Physical tool tip position in world coordinates (millimeters).
        /// </summary>
        public Vector3 Position { get; }

        /// <summary>
        /// Tool orientation (spindle axis rotation).
        /// </summary>
        public ToolOrientation Orientation { get; }

        /// <summary>
        /// Creates a tool pose from a physical tool tip position and orientation.
        /// </summary>
        /// <param name="position">Physical tool tip position in world coordinates.</param>
        /// <param name="orientation">Tool orientation.</param>
        public ToolPose(Vector3 position, ToolOrientation orientation)
        {
            Position = position;
            Orientation = orientation;
        }

        /// <summary>
        /// Gets the cutting axis direction (spindle -> physical tool tip).
        /// </summary>
        public Vector3 GetCuttingAxisDirection()
        {
            return Orientation.GetCuttingAxisDirection();
        }

        /// <summary>
        /// Gets the axis direction from the physical tool tip toward the spindle (tip -> spindle).
        /// </summary>
        public Vector3 GetAxisTowardSpindle()
        {
            return Orientation.GetAxisTowardSpindle();
        }

        /// <summary>
        /// Gets the center of the cutting geometry for the given tool.
        /// This is the cutting ball center for ball end mills and the physical tip otherwise.
        /// </summary>
        /// <param name="tool">Cutting tool.</param>
        /// <returns>Cutting center in world coordinates.</returns>
        public Vector3 GetCuttingCenter(Tool tool)
        {
            if (tool == null) throw new ArgumentNullException(nameof(tool));
            return Position + GetAxisTowardSpindle() * tool.BallCenterOffsetFromTip;
        }

        /// <inheritdoc />
        public override string ToString()
        {
            return $"Pos=({Position.X:F3}, {Position.Y:F3}, {Position.Z:F3}) {Orientation}";
        }
    }
}
