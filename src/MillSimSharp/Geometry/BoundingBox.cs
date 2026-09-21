using System;
using System.Numerics;

namespace MillSimSharp.Geometry
{
    /// <summary>
    /// Represents an axis-aligned bounding box defining the work area.
    /// </summary>
    public class BoundingBox
    {
        /// <summary>
        /// Minimum corner of the bounding box.
        /// </summary>
        public Vector3 Min { get; }

        /// <summary>
        /// Maximum corner of the bounding box.
        /// </summary>
        public Vector3 Max { get; }

        /// <summary>
        /// Size of the bounding box (Max - Min).
        /// </summary>
        public Vector3 Size => Max - Min;

        /// <summary>
        /// Center point of the bounding box.
        /// </summary>
        public Vector3 Center => (Min + Max) / 2.0f;

        /// <summary>
        /// Creates a bounding box from minimum and maximum corners.
        /// Both corners must contain only finite coordinates.
        /// </summary>
        public BoundingBox(Vector3 min, Vector3 max)
        {
            if (!IsFinite(min) || !IsFinite(max))
                throw new ArgumentException("Bounds must contain only finite coordinates.");

            if (max.X < min.X || max.Y < min.Y || max.Z < min.Z)
                throw new ArgumentException("Max must be greater than or equal to Min in all dimensions.");

            Min = min;
            Max = max;
        }

        /// <summary>
        /// Creates a bounding box from center point and size. Both must contain only finite
        /// coordinates and the size must be non-negative.
        /// </summary>
        public static BoundingBox FromCenterAndSize(Vector3 center, Vector3 size)
        {
            if (!IsFinite(center) || !IsFinite(size))
                throw new ArgumentException("Center and size must contain only finite coordinates.");

            if (size.X < 0 || size.Y < 0 || size.Z < 0)
                throw new ArgumentException("Size must be non-negative in all dimensions.");

            Vector3 halfSize = size / 2.0f;
            return new BoundingBox(center - halfSize, center + halfSize);
        }

        private static bool IsFinite(Vector3 value)
        {
            return float.IsFinite(value.X) && float.IsFinite(value.Y) && float.IsFinite(value.Z);
        }

        /// <summary>
        /// Checks if a point is inside the bounding box.
        /// </summary>
        public bool Contains(Vector3 point)
        {
            return point.X >= Min.X && point.X <= Max.X &&
                   point.Y >= Min.Y && point.Y <= Max.Y &&
                   point.Z >= Min.Z && point.Z <= Max.Z;
        }

        /// <summary>
        /// Expands the bounding box to include the given point.
        /// </summary>
        public BoundingBox ExpandToInclude(Vector3 point)
        {
            return new BoundingBox(
                new Vector3(
                    Math.Min(Min.X, point.X),
                    Math.Min(Min.Y, point.Y),
                    Math.Min(Min.Z, point.Z)
                ),
                new Vector3(
                    Math.Max(Max.X, point.X),
                    Math.Max(Max.Y, point.Y),
                    Math.Max(Max.Z, point.Z)
                )
            );
        }

        /// <summary>
        /// Returns a string representation of the bounding box.
        /// </summary>
        /// <returns></returns>
        public override string ToString()
        {
            return $"BoundingBox[Min={Min}, Max={Max}, Size={Size}]";
        }
    }
}
