using System;

namespace MillSimSharp.Simulation
{
    /// <summary>
    /// Represents an end mill (Flat or Ball).
    /// </summary>
    public class EndMill : Tool
    {
        /// <summary>
        /// Creates a new EndMill tool.
        /// </summary>
        /// <param name="diameter"></param>
        /// <param name="length"></param>
        /// <param name="isBallEnd"></param>
        public EndMill(float diameter, float length, bool isBallEnd = false)
            : base(diameter, length, isBallEnd ? ToolType.Ball : ToolType.Flat)
        {
        }

        /// <summary>
        /// Gets the distance from the physical tool tip to the cutting ball center.
        /// Equal to the radius for ball end mills, zero for flat end mills.
        /// </summary>
        public override float BallCenterOffsetFromTip => Type == ToolType.Ball ? Diameter / 2.0f : 0f;

        /// <summary>
        /// Gets the cutting solid geometry for this end mill.
        /// </summary>
        public override IToolGeometry GetCuttingGeometry()
        {
            return Type == ToolType.Ball
                ? new BallEndMillGeometry(Diameter / 2.0f, Length)
                : new FlatEndMillGeometry(Diameter / 2.0f, Length);
        }
    }
}
