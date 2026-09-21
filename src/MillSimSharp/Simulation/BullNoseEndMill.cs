using System;

namespace MillSimSharp.Simulation
{
    /// <summary>
    /// Bull nose (toroidal) end mill with a flat bottom of radius
    /// <c>Diameter / 2 - CornerRadius</c>.
    /// </summary>
    public class BullNoseEndMill : Tool
    {
        /// <summary>
        /// Corner radius in millimeters.
        /// </summary>
        public float CornerRadius { get; }

        /// <summary>
        /// Creates a bull nose end mill.
        /// </summary>
        /// <param name="diameter">Tool diameter in millimeters.</param>
        /// <param name="length">Cutting length in millimeters.</param>
        /// <param name="cornerRadius">Corner radius in millimeters.</param>
        public BullNoseEndMill(float diameter, float length, float cornerRadius)
            : base(diameter, length, ToolType.BullNose)
        {
            if (!float.IsFinite(cornerRadius) || cornerRadius <= 0 || cornerRadius > Diameter / 2.0f)
                throw new ArgumentException("Corner radius must be a finite positive number and not exceed the tool radius.", nameof(cornerRadius));

            CornerRadius = cornerRadius;
        }

        /// <inheritdoc />
        public override float BallCenterOffsetFromTip => 0f;

        /// <inheritdoc />
        public override IToolGeometry GetCuttingGeometry()
        {
            return new BullNoseEndMillGeometry(Diameter / 2.0f, CornerRadius, Length);
        }
    }
}
