#version 450

layout(location = 0) in  vec3 fragNormal;
layout(location = 1) in  vec3 fragColor;
layout(location = 0) out vec4 outColor;

// Sun: fixed directional light (normalised in world space).
const vec3  kLightDir = normalize(vec3(1.0, 1.5, 1.0));
const float kAmbient  = 0.08;

void main()
{
    vec3  n       = normalize(fragNormal);
    float diffuse = max(dot(n, kLightDir), 0.0);
    vec3  color   = fragColor * (kAmbient + diffuse);
    outColor = vec4(color, 1.0);
}
