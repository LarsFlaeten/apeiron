#version 450

layout(vertices = 3) out;

layout(location = 0) in  vec3 tcNormal[];
layout(location = 1) in  vec3 tcColor[];
layout(location = 2) in  vec2 tcUV[];

layout(location = 0) out vec3 teNormal[];
layout(location = 1) out vec3 teColor[];
layout(location = 2) out vec2 teUV[];

layout(push_constant) uniform PC {
    mat4 mvp;
    vec4 normalCol0;   // w = displaceScale (0 = no displacement)
    vec4 normalCol1;
    vec4 sunDirPad;
    vec4 viewDirPad;
} pc;

// Tessellation level for one edge given its two clip-space vertices.
// Proportional to NDC edge length; one level per 0.002 NDC units.
float edgeLevel(vec4 p0, vec4 p1)
{
    if (p0.w <= 0.0 && p1.w <= 0.0) return 1.0;
    vec2 ndc0 = p0.xy / max(p0.w, 1e-4);
    vec2 ndc1 = p1.xy / max(p1.w, 1e-4);
    return clamp(length(ndc1 - ndc0) / 0.002, 1.0, 64.0);
}

void main()
{
    teNormal[gl_InvocationID] = tcNormal[gl_InvocationID];
    teColor [gl_InvocationID] = tcColor [gl_InvocationID];
    teUV    [gl_InvocationID] = tcUV    [gl_InvocationID];
    gl_out[gl_InvocationID].gl_Position = gl_in[gl_InvocationID].gl_Position;

    if (gl_InvocationID == 0) {
        // No displacement → no tessellation needed.
        if (pc.normalCol0.w == 0.0) {
            gl_TessLevelOuter[0] = gl_TessLevelOuter[1] =
            gl_TessLevelOuter[2] = gl_TessLevelInner[0] = 1.0;
            return;
        }

        vec4 p[3];
        for (int i = 0; i < 3; ++i)
            p[i] = pc.mvp * gl_in[i].gl_Position;

        // Outer level: one per edge (index = opposite vertex).
        gl_TessLevelOuter[0] = edgeLevel(p[1], p[2]);
        gl_TessLevelOuter[1] = edgeLevel(p[2], p[0]);
        gl_TessLevelOuter[2] = edgeLevel(p[0], p[1]);
        // Inner level: average of outer.
        gl_TessLevelInner[0] = (gl_TessLevelOuter[0] +
                                gl_TessLevelOuter[1] +
                                gl_TessLevelOuter[2]) / 3.0;
    }
}
