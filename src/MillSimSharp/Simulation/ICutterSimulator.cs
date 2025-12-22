using System.Numerics;

namespace MillSimSharp.Simulation
{
    /// <summary>
    /// Interface for cutting simulators.
    /// Allows ToolpathExecutor to work with both VoxelGrid-based and SDF-based simulators.
    /// 
    /// <para><b>座標系の基準：</b></para>
    /// <para>
    /// すべての位置パラメータは工具先端（ツールチップ）の座標を表します。
    /// これは、エンドミルの切削刃の中心点、またはボールエンドミルの球の中心点です。
    /// </para>
    /// </summary>
    public interface ICutterSimulator
    {
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
