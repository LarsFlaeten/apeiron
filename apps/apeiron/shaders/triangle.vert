#version 450

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec3 inColor;

layout(push_constant) uniform PC {
    float angle;
} pc;

layout(location = 0) out vec3 fragColor;

void main()
{
    float c = cos(pc.angle);
    float s = sin(pc.angle);
    // Column-major: mat2(col0, col1)
    mat2  rot = mat2(c, s, -s, c);

    gl_Position = vec4(rot * inPosition.xy, inPosition.z, 1.0);
    fragColor   = inColor;
}
