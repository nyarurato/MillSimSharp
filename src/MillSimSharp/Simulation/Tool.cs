using System;

namespace MillSimSharp.Simulation
{
    /// <summary>
    /// Type of cutting tool.
    /// </summary>
    public enum ToolType
    {
        /// <summary>
        /// Flat end mill.
        /// </summary>
        Flat,
        /// <summary>
        /// Ball end mill.
        /// </summary>
        Ball
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

        /// <summary>
        /// Protected constructor for Tool.
        /// </summary>
        /// <param name="diameter"></param>
        /// <param name="length"></param>
        /// <param name="type"></param>
        /// <exception cref="ArgumentException"></exception>
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
