#version 450

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec3 inNormal;
layout(location = 2) in vec3 inColor;
layout(location = 3) in vec2 inUV;

layout(push_constant) uniform PC {
    mat4 mvp;
    mat3 normalMat;  // 48 bytes (3 × vec4-aligned columns)
    vec3 sunDir;     // 16 bytes (vec3 aligned to 16 in std430)
} pc;

layout(location = 0) out vec3 fragNormal;
layout(location = 1) out vec3 fragColor;
layout(location = 2) out vec2 fragUV;

void main()
{
    gl_Position = pc.mvp * vec4(inPosition, 1.0);
    fragNormal  = pc.normalMat * inNormal;
    fragColor   = inColor;
    fragUV      = inUV;
}
