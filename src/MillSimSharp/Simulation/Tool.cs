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
        Ball,
        /// <summary>
        /// Bull nose (toroidal) end mill.
        /// </summary>
        BullNose,
        /// <summary>
        /// Tapered end mill.
        /// </summary>
        Taper
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
        /// Distance from the physical tool tip to the center of the cutting ball, measured along
        /// the tool axis (tip -> spindle). Zero for flat tools; equal to the radius for ball end mills.
        /// </summary>
        public abstract float BallCenterOffsetFromTip { get; }

        /// <summary>
        /// Gets the cutting solid geometry of this tool in tool-local coordinates
        /// (origin = physical tip, +Z = toward the spindle).
        /// </summary>
        /// <returns>Cutting geometry used by the simulators.</returns>
        public abstract IToolGeometry GetCuttingGeometry();

        /// <summary>
        /// Gets the shank geometry for collision checks in tool-local coordinates
        /// (origin = physical tip), or null when the shank is not modelled.
        /// </summary>
        /// <returns>Shank geometry or null.</returns>
        public virtual IToolGeometry? GetShankGeometry()
        {
            return null;
        }

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
    }
}
