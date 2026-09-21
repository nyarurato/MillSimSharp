using System.Numerics;

namespace MillSimSharp.Simulation
{
    /// <summary>
    /// Interface for cutting simulators.
    /// Allows ToolpathExecutor to work with both VoxelGrid-based and SDF-based simulators.
    /// 
    /// <para><b>Tool reference point:</b></para>
    /// <para>
    /// All position parameters represent the coordinates of the physical tool tip, independent of the
    /// tool type. For ball end mills the cutting ball center is derived internally as
    /// <c>tip + AxisTowardSpindle * radius</c>.
    /// </para>
    /// </summary>
    public interface ICutterSimulator
    {
        /// <summary>
        /// Gets the sampling settings used for pose interpolation (linear/angular step limits).
        /// </summary>
        SimulationSettings Settings { get; }

        /// <summary>
        /// Performs a linear cut from start to end using the specified tool.
        /// </summary>
        /// <param name="start">Tool tip position at start.</param>
        /// <param name="end">Tool tip position at end.</param>
        /// <param name="tool">Cutting tool to use.</param>
        void CutLinear(Vector3 start, Vector3 end, Tool tool);

        /// <summary>
        /// Performs a point cut (drilling/plunging) at the specified position.
        /// </summary>
        /// <param name="position">Tool tip position.</param>
        /// <param name="tool">Cutting tool to use.</param>
        void CutPoint(Vector3 position, Tool tool);

        /// <summary>
        /// Performs a linear cut with specified tool orientation (for 5-axis machining).
        /// </summary>
        /// <param name="start">Tool tip position at start.</param>
        /// <param name="end">Tool tip position at end.</param>
        /// <param name="tool">Cutting tool to use.</param>
        /// <param name="startOrientation">Tool orientation at start.</param>
        /// <param name="endOrientation">Tool orientation at end.</param>
        void CutLinearWithOrientation(Vector3 start, Vector3 end, Tool tool,
            Toolpath.ToolOrientation startOrientation, Toolpath.ToolOrientation endOrientation);
    }
}
