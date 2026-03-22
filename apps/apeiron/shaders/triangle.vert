#version 450

// Triangle vertices hardcoded — no vertex buffer needed.
const vec2 kPositions[3] = vec2[](
    vec2( 0.0, -0.5),
    vec2( 0.5,  0.5),
    vec2(-0.5,  0.5)
);

const vec3 kColors[3] = vec3[](
    vec3(1.0,  0.15, 0.15),   // red
    vec3(0.15, 1.0,  0.15),   // green
    vec3(0.15, 0.15, 1.0)     // blue
);

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

    gl_Position = vec4(rot * kPositions[gl_VertexIndex], 0.0, 1.0);
    fragColor   = kColors[gl_VertexIndex];
}
