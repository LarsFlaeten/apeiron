#version 450

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec3 inNormal;
layout(location = 2) in vec3 inColor;
layout(location = 3) in vec2 inUV;

layout(push_constant) uniform PC {
    mat4 mvp;
    vec4 normalCol0;   // rotation matrix columns 0 & 1;
    vec4 normalCol1;   // column 2 reconstructed in shader as cross(col0, col1)
    vec4 sunDirPad;    // xyz = sun direction  (w unused)
    vec4 viewDirPad;   // xyz = view direction (w unused)
} pc;

layout(location = 0) out vec3 fragNormal;
layout(location = 1) out vec3 fragColor;
layout(location = 2) out vec2 fragUV;

void main()
{
    gl_Position = pc.mvp * vec4(inPosition, 1.0);

    // Reconstruct rotation matrix; normalize to strip uniform scale factor.
    vec3 c0 = normalize(pc.normalCol0.xyz);
    vec3 c1 = normalize(pc.normalCol1.xyz);
    mat3 normalMat = mat3(c0, c1, cross(c0, c1));

    fragNormal = normalMat * inNormal;
    fragColor  = inColor;
    fragUV     = inUV;
}
