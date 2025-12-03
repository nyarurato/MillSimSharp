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
        public float Yaw { get; set; } = 45.0f;      // Rotation around Y axis (degrees)
        public float Pitch { get; set; } = 30.0f;    // Rotation around X axis (degrees)

        private const float MinDistance = 10.0f;
        private const float MaxDistance = 500.0f;
        private const float MinPitch = -89.0f;
        private const float MaxPitch = 89.0f;

        public Matrix4 GetViewMatrix()
        {
            // Calculate camera position from spherical coordinates
            float yawRad = MathHelper.DegreesToRadians(Yaw);
            float pitchRad = MathHelper.DegreesToRadians(Pitch);

            OpenTK.Mathematics.Vector3 position = new OpenTK.Mathematics.Vector3(
                Distance * MathF.Cos(pitchRad) * MathF.Cos(yawRad),
                Distance * MathF.Sin(pitchRad),
                Distance * MathF.Cos(pitchRad) * MathF.Sin(yawRad)
            );

            position += Target;

            return Matrix4.LookAt(position, Target, OpenTK.Mathematics.Vector3.UnitY);
        }

        public void ProcessMouseMove(float deltaX, float deltaY, float sensitivity = 0.2f)
        {
            Yaw += deltaX * sensitivity;
            Pitch -= deltaY * sensitivity; // Inverted for natural feel

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
