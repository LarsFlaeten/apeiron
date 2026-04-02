#version 450

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec3 inNormal;   // unused, but matches Vertex layout
layout(location = 2) in vec3 inColor;    // material colour baked at load time
layout(location = 3) in vec2 inUV;

layout(location = 0) out vec3 fragColor;
layout(location = 1) out vec2 fragUV;

layout(push_constant) uniform PC {
    mat4  mvp;
    mat4  modelMat;
    vec4  sunDir;     // w = isEmissive (always 1 for plumes)
    vec4  baseColor;  // w = emissive intensity / throttle scale
} pc;

void main()
{
    gl_Position = pc.mvp * vec4(inPosition, 1.0);

    // Logarithmic depth — matches mesh pipeline.
    const float C_NEAR = 1e-3;
    const float C_FAR  = 1.0e9;
    float z_eye = 1.0 / gl_Position.w;
    gl_Position.z = log2(z_eye / C_NEAR) / log2(C_FAR / C_NEAR) * gl_Position.w;

    fragColor = inColor * pc.baseColor.rgb;
    fragUV    = inUV;
}
