#version 330 core

layout(location = 0) in vec3 aPosition;
layout(location = 1) in vec3 aNormal;

uniform mat4 uView;
uniform mat4 uProjection;

out vec3 vNormal;
out vec3 vFragPos;

void main() {
    vec3 worldPos = aPosition;
    gl_Position = uProjection * uView * vec4(worldPos, 1.0);
    vNormal = aNormal;
    vFragPos = worldPos;
}
