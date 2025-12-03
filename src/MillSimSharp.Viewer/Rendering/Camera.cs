using OpenTK.Mathematics;

namespace MillSimSharp.Viewer.Rendering
{
    /// <summary>
    /// Orbit camera for 3D viewing.
    /// </summary>
    public class Camera
    {
        public OpenTK.Mathematics.Vector3 Target { get; set; } = OpenTK.Mathematics.Vector3.Zero;
        public float Distance { get; set; } = 150.0f;
        public float Yaw { get; set; } = 45.0f;      // Rotation around Z axis (degrees) (azimuth)
        public float Pitch { get; set; } = 30.0f;    // Elevation angle from XY plane (degrees)

        private const float MinDistance = 10.0f;
        private const float MaxDistance = 500.0f;
        private const float MinPitch = -89.0f;
        private const float MaxPitch = 89.0f;

        public Matrix4 GetViewMatrix()
        {
            // Calculate camera position from spherical coordinates
            float yawRad = MathHelper.DegreesToRadians(Yaw);
            float pitchRad = MathHelper.DegreesToRadians(Pitch);

            // Recompute using Z-up convention (Z is vertical axis)
            OpenTK.Mathematics.Vector3 position = new OpenTK.Mathematics.Vector3(
                Distance * MathF.Cos(pitchRad) * MathF.Cos(yawRad), // X
                Distance * MathF.Cos(pitchRad) * MathF.Sin(yawRad), // Y
                Distance * MathF.Sin(pitchRad)                      // Z
            );

            position += Target;

            return Matrix4.LookAt(position, Target, OpenTK.Mathematics.Vector3.UnitZ);
        }

        public void ProcessMouseMove(float deltaX, float deltaY, float sensitivity = 0.2f)
        {
            Yaw += deltaX * sensitivity;
            // Flip vertical direction relative to previous behavior (user requested inverted up/down)
            Pitch += deltaY * sensitivity;

            // Clamp pitch to prevent flipping
            Pitch = Math.Clamp(Pitch, MinPitch, MaxPitch);
        }

        public void ProcessMouseWheel(float delta, float sensitivity = 5.0f)
        {
            Distance -= delta * sensitivity;
            Distance = Math.Clamp(Distance, MinDistance, MaxDistance);
        }
    }
}
