using System.Numerics;
using MillSimSharp.Geometry;

namespace MillSimSharp.Simulation
{
    /// <summary>
    /// Cutting solid of a tool, expressed in tool-local coordinates:
    /// <list type="bullet">
    /// <item>origin = physical tool tip</item>
    /// <item>+Z = tool axis direction toward the spindle</item>
    /// <item>X / Y = radial directions</item>
    /// </list>
    /// <para>
    /// <b>Axisymmetric limitation:</b> the simulators evaluate this solid from the tool-local
    /// coordinates <c>(radialDistance, 0, axialDistance)</c>, so a geometry must be a solid of
    /// revolution around the local +Z axis. Custom geometries with azimuthal features (for
    /// example an elliptical cross-section) cannot be represented: the azimuthal component is
    /// discarded. All builtin geometries (flat, ball, bull-nose, taper) are solids of revolution.
    /// </para>
    /// <para>
    /// <b>Bounds contract:</b> <see cref="LocalBounds"/> must contain the whole solid. It may
    /// extend to negative local Z (below the physical tip) for special parts.
    /// </para>
    /// </summary>
    public interface IToolGeometry
    {
        /// <summary>
        /// Signed distance from the tool solid in millimeters (negative = inside the tool,
        /// positive = outside, zero = surface).
        /// </summary>
        /// <param name="localPoint">Point in tool-local coordinates.</param>
        /// <returns>Signed distance in millimeters.</returns>
        float SignedDistance(Vector3 localPoint);

        /// <summary>
        /// Local-space bounding box of the cutting solid in millimeters.
        /// </summary>
        BoundingBox LocalBounds { get; }

        /// <summary>
        /// Distance from the physical tip to the cutting center along the tool axis (mm).
        /// Zero for flat tools; equal to the radius for ball end mills.
        /// </summary>
        float CuttingCenterOffset { get; }

        /// <summary>
        /// Conservative distance from the physical tip (the rotation pivot) to the farthest point
        /// of the cutting solid in millimeters. Used to bound the path of any tool point during an
        /// orientation change (adaptive pose sampling), so it must not underestimate the solid.
        /// </summary>
        float RotationSweepRadius { get; }
    }
}
