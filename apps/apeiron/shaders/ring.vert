#version 450

// Same push-constant layout as triangle.vert / triangle.tese so the ring
// pipeline can share the same VkPipelineLayout.
layout(push_constant) uniform PC {
    mat4  mvp;
    vec4  normalCol0;    // xyz = normal matrix col 0, w = unused for rings
    vec4  normalCol1;    // xyz = normal matrix col 1, w = unused
    vec4  sunDirPad;     // xyz = sun direction (world space), w = unused
    vec4  viewDirPad;    // xyz = view direction,            w = lightIntensity
} pc;

layout(location = 0) in vec3 inPos;
layout(location = 1) in vec3 inNormal;   // always (0,0,1) in ring-local space
layout(location = 2) in vec3 inColor;    // unused
layout(location = 3) in vec2 inUV;       // u = 0 (inner edge) … 1 (outer edge)

layout(location = 0) out vec2  fragUV;
layout(location = 1) out vec3  fragNormal;
layout(location = 2) out float fragLightIntensity;
layout(location = 3) out vec3  fragSunDir;
layout(location = 4) out vec3  fragViewDir;

void main()
{
    gl_Position = pc.mvp * vec4(inPos, 1.0);
    fragUV = inUV;

    // Reconstruct the 3×3 normal matrix from the two packed columns
    // (identical to what triangle.tese does).
    vec3 c0 = normalize(pc.normalCol0.xyz);
    vec3 c1 = normalize(pc.normalCol1.xyz);
    mat3 normalMat = mat3(c0, c1, cross(c0, c1));
    fragNormal = normalMat * inNormal;

    fragLightIntensity = pc.viewDirPad.w;
    fragSunDir         = pc.sunDirPad.xyz;
    fragViewDir        = pc.viewDirPad.xyz;
}
