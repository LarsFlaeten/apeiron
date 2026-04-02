#version 450

layout(location = 0) in  vec3 fragNormal;
layout(location = 1) in  vec3 fragColor;
layout(location = 2) in  vec3 fragPos;
layout(location = 0) out vec4 outColor;

layout(push_constant) uniform PC {
    mat4  mvp;
    mat4  modelMat;
    vec4  sunDir;    // xyz = dir toward sun, w = isEmissive
    vec4  baseColor; // w = emissive intensity
} pc;

const float C_NEAR = 0.1;
const float C_FAR  = 1.0e9;

void main()
{
    // Logarithmic depth.
    float z_eye  = 1.0 / gl_FragCoord.w;
    gl_FragDepth = log2(z_eye / C_NEAR) / log2(C_FAR / C_NEAR);

    vec3 color = fragColor;

    if (pc.sunDir.w > 0.5) {
        // Emissive: output at given intensity (HDR).
        float intensity = max(pc.baseColor.w, 1.0);
        outColor = vec4(color * intensity, 1.0);
    } else {
        // Simple Lambert + ambient.
        vec3 N = normalize(fragNormal);
        vec3 L = normalize(pc.sunDir.xyz);
        float diff    = max(dot(N, L), 0.0);
        float ambient = 0.08;
        outColor = vec4(color * (ambient + diff * 0.92), 1.0);
    }
}
