using System;
using System.Numerics;

namespace MillSimSharp.Toolpath
{
    /// <summary>
    /// Represents the orientation of the tool in 5-axis machining.
    /// 
    /// <para><b>Coordinate system:</b></para>
    /// <list type="bullet">
    /// <item>XYZ coordinates: position of the physical tool tip (independent of the tool type)</item>
    /// <item>Default pose: the tool axis points along the negative Z-axis (0, 0, -1)</item>
    /// <item>Rotation center: the tool rotates about its physical tip</item>
    /// <item>Axis directions: <see cref="GetCuttingAxisDirection"/> (spindle → tip) and <see cref="GetAxisTowardSpindle"/> (tip → spindle)</item>
    /// </list>
    /// 
    /// <para><b>Rotation angle definition (right-handed coordinate system):</b></para>
    /// <list type="bullet">
    /// <item>A-axis: rotation around the X-axis (+ direction rotates Y toward Z)</item>
    /// <item>B-axis: rotation around the Y-axis (+ direction rotates Z toward X)</item>
    /// <item>C-axis: rotation around the Z-axis (+ direction rotates X toward Y)</item>
    /// <item>Rotation order: C → B → A (ZYX Euler angles)</item>
    /// </list>
    /// 
    /// <para><b>Relation to machine configuration:</b></para>
    /// <para>
    /// This struct expresses the "tool direction" independently of the concrete machine configuration
    /// (head rotary / table rotary). Conversion to a specific machine is done in post-processing.
    /// </para>
    /// </summary>
    public struct ToolOrientation
    {
        /// <summary>
        /// Rotation around X-axis (A-axis) in degrees.
        /// Positive rotation: Y-axis toward Z-axis (right-hand rule).
        /// </summary>
        public float A { get; set; }

        /// <summary>
        /// Rotation around Y-axis (B-axis) in degrees.
        /// Positive rotation: Z-axis toward X-axis (right-hand rule).
        /// </summary>
        public float B { get; set; }

        /// <summary>
        /// Rotation around Z-axis (C-axis) in degrees.
        /// Positive rotation: X-axis toward Y-axis (right-hand rule).
        /// </summary>
        public float C { get; set; }

        /// <summary>
        /// Creates a new tool orientation.
        /// </summary>
        /// <param name="a_deg">A-axis rotation in degrees.</param>
        /// <param name="b_deg">B-axis rotation in degrees.</param>
        /// <param name="c_deg">C-axis rotation in degrees.</param>
        public ToolOrientation(float a_deg = 0, float b_deg = 0, float c_deg = 0)
        {
            A = a_deg;
            B = b_deg;
            C = c_deg;
        }

        /// <summary>
        /// Gets the cutting axis direction (spindle -> physical tool tip) for the current orientation.
        /// <para>
        /// The default tool direction (A=B=C=0) is the negative Z-axis (0, 0, -1), which represents
        /// the tool pointing downward (toward the workpiece).
        /// </para>
        /// </summary>
        /// <returns>Normalized direction vector pointing from the spindle toward the tool tip.</returns>
        public Vector3 GetCuttingAxisDirection()
        {
            // Convert degrees to radians
            float aRad = A * MathF.PI / 180f;
            float bRad = B * MathF.PI / 180f;
            float cRad = C * MathF.PI / 180f;

            // Default tool direction: along negative Z-axis (0, 0, -1) - tool pointing downward
            // Rotation order: C (Z-axis), B (Y-axis), A (X-axis) = ZYX Euler angles

            // Rotation matrices
            var rotX = Matrix4x4.CreateRotationX(aRad);
            var rotY = Matrix4x4.CreateRotationY(bRad);
            var rotZ = Matrix4x4.CreateRotationZ(cRad);

            // Combined rotation
            var rotation = rotZ * rotY * rotX;

            // Apply to default tool direction
            var defaultDirection = new Vector3(0, 0, -1);
            var direction = Vector3.Transform(defaultDirection, rotation);

            return Vector3.Normalize(direction);
        }

        /// <summary>
        /// Gets the axis direction from the physical tool tip toward the spindle (tip -> spindle).
        /// This is the direction in which the tool body extends from the tip.
        /// </summary>
        /// <returns>Normalized direction vector pointing from the tool tip toward the spindle.</returns>
        public Vector3 GetAxisTowardSpindle()
        {
            return -GetCuttingAxisDirection();
        }

        /// <summary>
        /// Gets the quaternion representation of this orientation (rotation order C -> B -> A).
        /// </summary>
        /// <returns>Normalized quaternion.</returns>
        public Quaternion GetQuaternion()
        {
            float aRad = A * MathF.PI / 180f;
            float bRad = B * MathF.PI / 180f;
            float cRad = C * MathF.PI / 180f;

            var rotX = Quaternion.CreateFromAxisAngle(Vector3.UnitX, aRad);
            var rotY = Quaternion.CreateFromAxisAngle(Vector3.UnitY, bRad);
            var rotZ = Quaternion.CreateFromAxisAngle(Vector3.UnitZ, cRad);

            // Hamilton composition: q = qx * qy * qz applies the C, then B, then A rotation,
            // matching the row-vector matrix order rotZ * rotY * rotX used by GetRotationMatrix.
            return Quaternion.Normalize(rotX * rotY * rotZ);
        }

        /// <summary>
        /// Computes the shortest angular distance between two orientations in degrees.
        /// </summary>
        /// <param name="start">Start orientation.</param>
        /// <param name="end">End orientation.</param>
        /// <returns>Angular distance in degrees (0 to 180).</returns>
        public static float AngularDistanceDegrees(ToolOrientation start, ToolOrientation end)
        {
            Quaternion q0 = start.GetQuaternion();
            Quaternion q1 = end.GetQuaternion();

            float dot = MathF.Abs(Quaternion.Dot(q0, q1));
            if (dot > 1f) dot = 1f;

            float angleRad = 2f * MathF.Acos(dot);
            return angleRad * 180f / MathF.PI;
        }

        /// <summary>
        /// Creates an orientation from a quaternion (useful for interpolated results).
        /// </summary>
        /// <param name="q">Rotation quaternion.</param>
        /// <returns>Equivalent orientation as Euler angles.</returns>
        public static ToolOrientation FromQuaternion(Quaternion q)
        {
            if (q.LengthSquared() < 1e-12f) return Default;
            q = Quaternion.Normalize(q);

            // Row-vector matrix M = Rz(c) * Ry(b) * Rx(a); its transpose is Rx(a) Ry(b) Rz(c).
            var m = Matrix4x4.CreateFromQuaternion(q);

            float b = MathF.Asin(Math.Clamp(m.M31, -1f, 1f));
            float a = MathF.Atan2(-m.M32, m.M33);
            float c = MathF.Atan2(-m.M21, m.M11);

            const float radiansToDegrees = 180f / MathF.PI;
            return new ToolOrientation(a * radiansToDegrees, b * radiansToDegrees, c * radiansToDegrees);
        }

        /// <summary>
        /// Creates an orientation from the direction pointing from the physical tool tip toward the
        /// spindle (tip -> spindle), as provided by IJK-style pose input.
        /// <para>
        /// The result is the <b>shortest rotation</b> from the default spindle axis (+Z) to the
        /// given direction, expressed in the existing ZYX (C -> B -> A) Euler convention. The roll
        /// component C is generally non-zero for general directions (the shortest rotation is
        /// roll-free about the target axis). Any finite non-zero vector is accepted and normalized
        /// internally; near-antipodal directions keep their axis direction. Machine-specific Euler
        /// conventions, rotary unwind and tool-axis roll stay outside the core, and roll does not
        /// change material removal because tool cutting geometries are solids of revolution. For the
        /// exact antipodal direction (0, 0, -1), where the shortest rotation is not unique, the
        /// rotation is pinned to 180 degrees around X (A = +/-180, B = C = 0).
        /// </para>
        /// </summary>
        /// <param name="axisTowardSpindle">Direction from the tip toward the spindle. Any finite
        /// non-zero vector; it is normalized internally.</param>
        /// <returns>Orientation whose tool axis points along the given direction.</returns>
        /// <exception cref="ArgumentException">Thrown when the axis is zero or not finite.</exception>
        public static ToolOrientation FromAxisTowardSpindle(Vector3 axisTowardSpindle)
        {
            if (!float.IsFinite(axisTowardSpindle.X) ||
                !float.IsFinite(axisTowardSpindle.Y) ||
                !float.IsFinite(axisTowardSpindle.Z))
            {
                throw new ArgumentException("Tool axis must be finite.", nameof(axisTowardSpindle));
            }

            // Work in double precision: the shortest rotation for a near-antipodal axis has
            // components at the 1e-8 scale, which float rounding would collapse (for example
            // u.Z = -0.999999995 rounds to -1 and would lose the axis direction).
            double x = axisTowardSpindle.X;
            double y = axisTowardSpindle.Y;
            double z = axisTowardSpindle.Z;

            double length = Math.Sqrt(x * x + y * y + z * z);
            if (!(length > 0.0))
            {
                throw new ArgumentException("Tool axis must be a non-zero vector.", nameof(axisTowardSpindle));
            }

            double ux = x / length;
            double uy = y / length;
            double uz = z / length;

            // Shortest rotation from +Z to u: axis Z x u = (-uy, ux, 0), angle acos(u.Z); the
            // quaternion is normalize((Z x u, 1 + u.Z)).
            double crossX = -uy;
            double crossY = ux;
            double w = 1.0 + uz;

            double norm = Math.Sqrt(crossX * crossX + crossY * crossY + w * w);
            if (!(norm > 0.0))
            {
                // Exact antipodal: the shortest rotation is not unique; pin it to 180 degrees around X.
                return new ToolOrientation(180, 0, 0);
            }

            var q = new Quaternion(
                (float)(crossX / norm),
                (float)(crossY / norm),
                0f,
                (float)(w / norm));
            return FromQuaternion(q);
        }

        /// <summary>
        /// Interpolates between two orientations with quaternion spherical linear interpolation
        /// (shortest rotation).
        /// </summary>
        /// <param name="start">Start orientation.</param>
        /// <param name="end">End orientation.</param>
        /// <param name="t">Interpolation parameter (0 to 1).</param>
        /// <returns>Interpolated orientation.</returns>
        public static ToolOrientation Slerp(ToolOrientation start, ToolOrientation end, float t)
        {
            Quaternion q = Quaternion.Slerp(start.GetQuaternion(), end.GetQuaternion(), Math.Clamp(t, 0f, 1f));
            return FromQuaternion(q);
        }

        /// <summary>
        /// Gets the rotation matrix for this orientation.
        /// </summary>
        /// <returns>4x4 rotation matrix.</returns>
        public Matrix4x4 GetRotationMatrix()
        {
            float aRad = A * MathF.PI / 180f;
            float bRad = B * MathF.PI / 180f;
            float cRad = C * MathF.PI / 180f;

            var rotX = Matrix4x4.CreateRotationX(aRad);
            var rotY = Matrix4x4.CreateRotationY(bRad);
            var rotZ = Matrix4x4.CreateRotationZ(cRad);

            return rotZ * rotY * rotX;
        }

        /// <summary>
        /// Returns a human-readable description of the orientation.
        /// </summary>
        public override string ToString()
        {
            return $"A:{A:F3}° B:{B:F3}° C:{C:F3}°";
        }

        /// <summary>
        /// Default orientation (no rotation).
        /// </summary>
        public static ToolOrientation Default => new ToolOrientation(0, 0, 0);

        /// <summary>
        /// Checks if this is the default orientation.
        /// </summary>
        public bool IsDefault => A == 0 && B == 0 && C == 0;
    }
}
