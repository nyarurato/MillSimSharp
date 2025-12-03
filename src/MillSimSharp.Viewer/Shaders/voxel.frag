#version 330 core

in vec3 vColor;
in vec3 vNormal;
in vec3 vFragPos;

uniform vec3 uLightDir;

out vec4 FragColor;

void main()
{
    // Ambient lighting
    float ambient = 0.3;
    
    // Diffuse lighting
    vec3 norm = normalize(vNormal);
    vec3 lightDir = normalize(uLightDir);
    float diff = max(dot(norm, lightDir), 0.0);
    
    // Combine lighting
    vec3 result = (ambient + diff * 0.7) * vColor;
    
    FragColor = vec4(result, 1.0);
}
