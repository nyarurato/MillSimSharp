using System;
using System.Numerics;
using MillSimSharp.Geometry;
using MillSimSharp.Toolpath;

namespace MillSimSharp.Simulation
{
    /// <summary>
    /// Shared planning for tool sweeps: pose sampling (linear / angular steps and quaternion slerp),
    /// the exact swept-solid shortcut for straight moves perpendicular to the tool axis, and the
    /// solid predicates handed to the backends.
    /// <para>
    /// The voxel and SDF simulators keep their own removal primitives
    /// (<c>VoxelGrid.RemoveVoxelsInRegion</c> / <c>SDFGrid.CarveRegion</c>, edit batching and CSG
    /// repair); this class only computes what to remove.
    /// </para>
    /// </summary>
    internal static class ToolSweepPlanner
    {
        /// <summary>
        /// Removes the cutting solid for a point cut (drilling / plunging) at the physical tip.
        /// </summary>
        /// <param name="geometry">Cutting geometry (tool-local, origin = physical tip).</param>
        /// <param name="tip">Physical tool tip position.</param>
        /// <param name="removeSolidInRegion">Backend removal primitive.</param>
        public static void ExecutePointCut(
            IToolGeometry geometry,
            Vector3 tip,
            Action<BoundingBox, Func<Vector3, float>> removeSolidInRegion)
        {
            var (bounds, signedDistance) = BuildToolSolid(geometry, tip, Vector3.UnitZ);
            removeSolidInRegion(bounds, signedDistance);
        }

        /// <summary>
        /// Plans and applies a linear move with orientation: an exact swept solid for straight moves
        /// perpendicular to the tool axis, otherwise the union of sampled poses
        /// (shortest-rotation slerp). Rotation-only moves are swept as well.
        /// </summary>
        /// <param name="geometry">Cutting geometry (tool-local, origin = physical tip).</param>
        /// <param name="start">Physical tool tip position at start.</param>
        /// <param name="end">Physical tool tip position at end.</param>
        /// <param name="startOrientation">Tool orientation at start.</param>
        /// <param name="endOrientation">Tool orientation at end.</param>
        /// <param name="settings">Pose sampling settings.</param>
        /// <param name="removeSolidInRegion">Backend removal primitive.</param>
        public static void ExecuteLinearMove(
            IToolGeometry geometry,
            Vector3 start,
            Vector3 end,
            ToolOrientation startOrientation,
            ToolOrientation endOrientation,
            SimulationSettings settings,
            Action<BoundingBox, Func<Vector3, float>> removeSolidInRegion)
        {
            Vector3 delta = end - start;
            float distance = delta.Length();
            float angularDistance = ToolOrientation.AngularDistanceDegrees(startOrientation, endOrientation);
            int steps = settings.ComputeSteps(distance, angularDistance, ToolPoseMath.GetRotationSweepRadius(geometry));

            Quaternion qStart = startOrientation.GetQuaternion();
            Quaternion qEnd = endOrientation.GetQuaternion();

            if (distance > 1e-6f &&
                Quaternion.Dot(qStart, qEnd) >= 1f - 1e-6f &&
                MathF.Abs(Vector3.Dot(Vector3.Transform(Vector3.UnitZ, qStart), delta)) <= 1e-5f * distance)
            {
                // Exact swept solid for a straight move that is perpendicular to the tool axis.
                Vector3 axisTowardSpindle = Vector3.Transform(Vector3.UnitZ, qStart);
                var (bounds, signedDistance) = BuildSweptToolSolid(geometry, start, end, axisTowardSpindle);
                removeSolidInRegion(bounds, signedDistance);
                return;
            }

            for (int i = 0; i <= steps; i++)
            {
                float t = i / (float)steps;
                Vector3 position = Vector3.Lerp(start, end, t);
                Quaternion q = Quaternion.Slerp(qStart, qEnd, t);
                Vector3 axisTowardSpindle = Vector3.Transform(Vector3.UnitZ, q);

                var (bounds, signedDistance) = BuildToolSolid(geometry, position, axisTowardSpindle);
                removeSolidInRegion(bounds, signedDistance);
            }
        }

        /// <summary>
        /// World bounds and signed-distance predicate of a single tool pose.
        /// </summary>
        private static (BoundingBox Bounds, Func<Vector3, float> SignedDistance) BuildToolSolid(
            IToolGeometry geometry, Vector3 tip, Vector3 axisTowardSpindle)
        {
            BoundingBox worldBounds = ToolPoseMath.GetWorldBounds(geometry, tip, axisTowardSpindle);
            return (worldBounds,
                point => geometry.SignedDistance(ToolPoseMath.ToLocalPoint(point, tip, axisTowardSpindle)));
        }

        /// <summary>
        /// World bounds and signed-distance predicate of the exact swept solid of a straight move
        /// perpendicular to the tool axis (flat move: the tool profile is swept along the motion).
        /// </summary>
        private static (BoundingBox Bounds, Func<Vector3, float> SignedDistance) BuildSweptToolSolid(
            IToolGeometry geometry, Vector3 start, Vector3 end, Vector3 axisTowardSpindle)
        {
            Vector3 motion = end - start;
            float pathLength = motion.Length();
            Vector3 motionDirection = motion / pathLength;

            BoundingBox startBounds = ToolPoseMath.GetWorldBounds(geometry, start, axisTowardSpindle);
            BoundingBox endBounds = ToolPoseMath.GetWorldBounds(geometry, end, axisTowardSpindle);
            var worldBounds = new BoundingBox(
                Vector3.Min(startBounds.Min, endBounds.Min),
                Vector3.Max(startBounds.Max, endBounds.Max));

            Func<Vector3, float> signedDistance = point =>
            {
                Vector3 relative = point - start;
                float axial = Vector3.Dot(relative, axisTowardSpindle);
                Vector3 perpendicular = relative - axisTowardSpindle * axial;
                float along = Vector3.Dot(perpendicular, motionDirection);
                float clamped = Math.Clamp(along, 0f, pathLength);
                float radial = (perpendicular - motionDirection * clamped).Length();
                return geometry.SignedDistance(new Vector3(radial, 0f, axial));
            };

            return (worldBounds, signedDistance);
        }
    }
}
