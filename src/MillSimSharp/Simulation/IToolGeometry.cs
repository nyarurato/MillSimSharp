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
    }
}
