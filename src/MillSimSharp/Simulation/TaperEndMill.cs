using System;

namespace MillSimSharp.Simulation
{
    /// <summary>
    /// Tapered end mill whose radius grows linearly from the tip over the cutting length.
    /// </summary>
    public class TaperEndMill : Tool
    {
        /// <summary>
        /// Taper (half) angle in degrees.
        /// </summary>
        public float TaperAngleDegrees { get; }

        /// <summary>
        /// Creates a tapered end mill.
        /// </summary>
        /// <param name="tipDiameter">Diameter at the physical tip in millimeters.</param>
        /// <param name="length">Cutting length in millimeters.</param>
        /// <param name="taperAngleDegrees">Taper angle in degrees.</param>
        public TaperEndMill(float tipDiameter, float length, float taperAngleDegrees)
            : base(tipDiameter, length, ToolType.Taper)
        {
            if (taperAngleDegrees < 0 || taperAngleDegrees >= 90)
                throw new ArgumentException("Taper angle must be in [0, 90) degrees.", nameof(taperAngleDegrees));

            TaperAngleDegrees = taperAngleDegrees;
        }

        /// <inheritdoc />
        public override float BallCenterOffsetFromTip => 0f;

        /// <inheritdoc />
        public override IToolGeometry GetCuttingGeometry()
        {
            return new TaperedEndMillGeometry(Diameter / 2.0f, TaperAngleDegrees, Length);
        }
    }
}
