using System;
using System.Numerics;

namespace MillSimSharp.Toolpath
{
    /// <summary>
    /// Represents the orientation of the tool in 5-axis machining.
    /// 
    /// <para><b>座標系の基準：</b></para>
    /// <list type="bullet">
    /// <item>XYZ座標：工具先端（Physical Tip）の位置を表します（工具種別に依存しません）</item>
    /// <item>デフォルト姿勢：工具軸はZ軸負方向（0, 0, -1）に向いています</item>
    /// <item>回転中心：工具先端を中心に回転します</item>
    /// <item>軸方向：<see cref="GetCuttingAxisDirection"/>（spindle → tip）と <see cref="GetAxisTowardSpindle"/>（tip → spindle）</item>
    /// </list>
    /// 
    /// <para><b>回転角度の定義（右手座標系）：</b></para>
    /// <list type="bullet">
    /// <item>A軸：X軸周りの回転（+方向はY→Zへの回転）</item>
    /// <item>B軸：Y軸周りの回転（+方向はZ→Xへの回転）</item>
    /// <item>C軸：Z軸周りの回転（+方向はX→Yへの回転）</item>
    /// <item>回転順序：C → B → A（オイラー角ZYX順）</item>
    /// </list>
    /// 
    /// <para><b>機械構成との関係：</b></para>
    /// <para>
    /// このクラスは工作機械の具体的な構成（ヘッド回転型／テーブル回転型）とは
    /// 独立した「工具方向」を表現します。実際の機械への変換は後処理で行います。
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
        /// デフォルトの工具方向（A=B=C=0）は、Z軸負方向（0, 0, -1）です。
        /// これは、工具が下向き（ワークに向かって）に配置された状態を表します。
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
