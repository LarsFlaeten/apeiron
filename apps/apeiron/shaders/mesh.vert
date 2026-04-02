#version 450

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec3 inNormal;
layout(location = 2) in vec3 inColor;
layout(location = 3) in vec2 inUV;

layout(location = 0) out vec3 fragNormal;   // world-space
layout(location = 1) out vec3 fragColor;
layout(location = 2) out vec3 fragPos;      // world-space (for specular)
layout(location = 3) out vec2 fragUV;

layout(push_constant) uniform PC {
    mat4  mvp;
    mat4  modelMat;   // for normal transform (no non-uniform scale assumed)
    vec4  sunDir;     // xyz = direction, w = isEmissive (1.0) or lit (0.0)
    vec4  baseColor;  // rgba multiplier (baked-in material colour)
} pc;

void main()
{
    gl_Position = pc.mvp * vec4(inPosition, 1.0);

    // Logarithmic depth — matches mesh.frag and is compatible with planet pipeline.
    float C_NEAR = 1e-3;
    float C_FAR  = 1.0e9;
    float z_eye  = 1.0 / gl_Position.w;  // approximate; corrected in frag
    gl_Position.z = log2(z_eye / C_NEAR) / log2(C_FAR / C_NEAR) * gl_Position.w;

    mat3 normalMat = transpose(inverse(mat3(pc.modelMat)));
    fragNormal = normalize(normalMat * inNormal);
    fragColor  = inColor * pc.baseColor.rgb;
    fragPos    = vec3(pc.modelMat * vec4(inPosition, 1.0));
    fragUV     = inUV;
}
