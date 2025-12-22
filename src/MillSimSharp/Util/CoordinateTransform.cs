using System;
using System.Numerics;
using MillSimSharp.Geometry;
using MillSimSharp.Toolpath;

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

        /// <summary>
        /// Transforms a point by rotation and translation for 5-axis machining.
        /// </summary>
        /// <param name="point">Point to transform.</param>
        /// <param name="orientation">Tool orientation.</param>
        /// <param name="origin">Rotation center.</param>
        /// <returns>Transformed point.</returns>
        public static Vector3 TransformPoint(Vector3 point, ToolOrientation orientation, Vector3 origin)
        {
            // Translate to origin
            Vector3 translated = point - origin;

            // Apply rotation
            Matrix4x4 rotation = orientation.GetRotationMatrix();
            Vector3 rotated = Vector3.Transform(translated, rotation);

            // Translate back
            return rotated + origin;
        }

        /// <summary>
        /// Computes the transformation matrix for 5-axis positioning.
        /// </summary>
        /// <param name="position">Tool position.</param>
        /// <param name="orientation">Tool orientation.</param>
        /// <returns>4x4 transformation matrix.</returns>
        public static Matrix4x4 Get5AxisTransform(Vector3 position, ToolOrientation orientation)
        {
            Matrix4x4 rotation = orientation.GetRotationMatrix();
            Matrix4x4 translation = Matrix4x4.CreateTranslation(position);
            return rotation * translation;
        }

        /// <summary>
        /// Converts tool tip position and orientation to machine coordinates.
        /// </summary>
        /// <param name="toolTipPosition">Tool tip position in work coordinates.</param>
        /// <param name="orientation">Tool orientation.</param>
        /// <param name="toolLength">Length of the tool.</param>
        /// <returns>Spindle position in machine coordinates.</returns>
        public static Vector3 ToolTipToSpindlePosition(Vector3 toolTipPosition, ToolOrientation orientation, float toolLength)
        {
            // Get tool direction vector
            Vector3 toolDirection = orientation.GetToolDirection();
            
            // Move along tool direction by tool length
            return toolTipPosition - toolDirection * toolLength;
        }

        /// <summary>
        /// Interpolates between two orientations.
        /// </summary>
        /// <param name="start">Starting orientation.</param>
        /// <param name="end">Ending orientation.</param>
        /// <param name="t">Interpolation parameter (0 to 1).</param>
        /// <returns>Interpolated orientation.</returns>
        public static ToolOrientation InterpolateOrientation(ToolOrientation start, ToolOrientation end, float t)
        {
            t = Math.Clamp(t, 0f, 1f);
            
            return new ToolOrientation(
                Lerp(start.A, end.A, t),
                Lerp(start.B, end.B, t),
                Lerp(start.C, end.C, t)
            );
        }

        private static float Lerp(float a, float b, float t)
        {
            return a + (b - a) * t;
        }
    }
}
