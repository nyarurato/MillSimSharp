using System;

namespace MillSimSharp.Simulation
{
    /// <summary>
    /// Represents an end mill (Flat or Ball).
    /// </summary>
    public class EndMill : Tool
    {
        public EndMill(float diameter, float length, bool isBallEnd = false)
            : base(diameter, length, isBallEnd ? ToolType.Ball : ToolType.Flat)
        {
        }

        public override float GetRadiusAtHeight(float heightFromTip)
        {
            if (heightFromTip < 0 || heightFromTip > Length)
            {
                return 0; // Outside cutting length
            }

            float radius = Diameter / 2.0f;

            if (Type == ToolType.Ball)
            {
                // Ball end logic: radius varies in the spherical part
                if (heightFromTip < radius)
                {
                    // Calculate radius of the circle slice at this height
                    // Using Pythagorean theorem: r^2 + (R-h)^2 = R^2
                    // r = sqrt(R^2 - (R-h)^2)
                    float R = radius;
                    float h = heightFromTip;
                    float distFromCenter = R - h;
                    return MathF.Sqrt(R * R - distFromCenter * distFromCenter);
                }
            }

            return radius;
        }
    }
}
