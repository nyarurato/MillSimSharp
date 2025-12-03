#version 330 core

layout(location = 0) in vec3 aPosition;      // Cube vertex position
layout(location = 1) in vec3 aNormal;        // Cube vertex normal
layout(location = 2) in vec3 aInstancePos;   // Voxel world position (instanced)
layout(location = 3) in vec3 aInstanceColor; // Voxel color (instanced)

uniform mat4 uView;
uniform mat4 uProjection;
uniform float uVoxelSize;

out vec3 vColor;
out vec3 vNormal;
out vec3 vFragPos;

void main()
{
    // Scale cube to voxel size and translate to instance position
    vec3 worldPos = aPosition * uVoxelSize + aInstancePos;
    
    gl_Position = uProjection * uView * vec4(worldPos, 1.0);
    
    vColor = aInstanceColor;
    vNormal = aNormal; // No rotation, so normal doesn't need transformation
    vFragPos = worldPos;
}
