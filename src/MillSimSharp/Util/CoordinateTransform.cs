using System.Numerics;
using MillSimSharp.Geometry;

namespace MillSimSharp.Util
{
    /// <summary>
    /// Utility for coordinate system conversions.
    /// </summary>
    public static class CoordinateTransform
    {
        /// <summary>
        /// Converts machine coordinates to work coordinates.
        /// </summary>
        /// <param name="machineCoords">Coordinates in machine coordinate system.</param>
        /// <param name="workOrigin">Work coordinate system origin in machine coordinates.</param>
        /// <returns>Coordinates in work coordinate system.</returns>
        public static Vector3 MachineToWork(Vector3 machineCoords, Vector3 workOrigin)
        {
            return machineCoords - workOrigin;
        }

        /// <summary>
        /// Converts work coordinates to machine coordinates.
        /// </summary>
        /// <param name="workCoords">Coordinates in work coordinate system.</param>
        /// <param name="workOrigin">Work coordinate system origin in machine coordinates.</param>
        /// <returns>Coordinates in machine coordinate system.</returns>
        public static Vector3 WorkToMachine(Vector3 workCoords, Vector3 workOrigin)
        {
            return workCoords + workOrigin;
        }

        /// <summary>
        /// Translates a bounding box by an offset.
        /// </summary>
        /// <param name="bbox">Original bounding box.</param>
        /// <param name="offset">Translation offset.</param>
        /// <returns>Translated bounding box.</returns>
        public static BoundingBox TranslateBoundingBox(BoundingBox bbox, Vector3 offset)
        {
            return new BoundingBox(bbox.Min + offset, bbox.Max + offset);
        }
    }
}
