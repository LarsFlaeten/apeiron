#version 450

layout(location = 0) in  vec3 fragNormal;
layout(location = 1) in  vec3 fragColor;
layout(location = 0) out vec4 outColor;

layout(push_constant) uniform PC {
    mat4 mvp;
    mat3 normalMat;
    vec3 sunDir;
} pc;

// Must match Camera near/far.
const float C_NEAR = 0.1;
const float C_FAR  = 1.0e9;

void main()
{
    // Logarithmic depth.
    float z_eye  = 1.0 / gl_FragCoord.w;
    gl_FragDepth = log2(z_eye / C_NEAR) / log2(C_FAR / C_NEAR);

    // Phong diffuse — sun direction comes from the CPU each draw call.
    vec3  n       = normalize(fragNormal);
    float diffuse = max(dot(n, normalize(pc.sunDir)), 0.0);
    vec3  color   = fragColor * (0.08 + diffuse);
    outColor = vec4(color, 1.0);
}
