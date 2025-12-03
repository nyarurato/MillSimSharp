#version 330 core

in vec3 vNormal;
in vec3 vFragPos;

uniform vec3 uLightDir;

out vec4 FragColor;

void main() {
    float ambient = 0.2;
    vec3 norm = normalize(vNormal);
    vec3 lightDir = normalize(uLightDir);
    float diff = max(dot(norm, lightDir), 0.0);
    vec3 baseColor = vec3(0.8, 0.8, 0.8);
    vec3 color = (ambient + diff * 0.8) * baseColor;
    FragColor = vec4(color, 1.0);
}
