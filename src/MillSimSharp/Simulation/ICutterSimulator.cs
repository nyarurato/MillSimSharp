using System.Numerics;

namespace MillSimSharp.Simulation
{
    /// <summary>
    /// Interface for cutting simulators.
    /// Allows ToolpathExecutor to work with both VoxelGrid-based and SDF-based simulators.
    /// </summary>
    public interface ICutterSimulator
    {
        /// <summary>
        /// Performs a linear cut from start to end using the specified tool.
        /// </summary>
        void CutLinear(Vector3 start, Vector3 end, Tool tool);

        /// <summary>
        /// Performs a point cut (drilling/plunging) at the specified position.
        /// </summary>
        void CutPoint(Vector3 position, Tool tool);
    }
}
