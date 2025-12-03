using System;

namespace MillSimSharp.Simulation
{
    public enum ToolType
    {
        Flat,
        Ball,
        Bull
    }

    /// <summary>
    /// Abstract base class for all cutting tools.
    /// </summary>
    public abstract class Tool
    {
        /// <summary>
        /// Tool diameter in millimeters.
        /// </summary>
        public float Diameter { get; }

        /// <summary>
        /// Cutting length in millimeters.
        /// </summary>
        public float Length { get; }

        /// <summary>
        /// Type of the tool.
        /// </summary>
        public ToolType Type { get; }

        protected Tool(float diameter, float length, ToolType type)
        {
            if (diameter <= 0) throw new ArgumentException("Diameter must be positive", nameof(diameter));
            if (length <= 0) throw new ArgumentException("Length must be positive", nameof(length));

            Diameter = diameter;
            Length = length;
            Type = type;
        }

        /// <summary>
        /// Gets the radius of the tool at a specific height from the tip.
        /// </summary>
        /// <param name="heightFromTip">Height from the tool tip (0 is the tip).</param>
        /// <returns>Radius at the given height.</returns>
        public abstract float GetRadiusAtHeight(float heightFromTip);
    }
}
