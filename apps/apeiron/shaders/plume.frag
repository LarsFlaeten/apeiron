#version 450

layout(location = 0) in  vec3 fragColor;
layout(location = 1) in  vec2 fragUV;
layout(location = 0) out vec4 outColor;

layout(push_constant) uniform PC {
    mat4  mvp;
    mat4  modelMat;
    vec4  sunDir;     // w = isEmissive (always 1 for plumes)
    vec4  baseColor;  // w = emissive intensity
} pc;

const float C_NEAR = 1e-3;
const float C_FAR  = 1.0e9;

void main()
{
    // Logarithmic depth.
    float z_eye  = 1.0 / gl_FragCoord.w;
    gl_FragDepth = log2(z_eye / C_NEAR) / log2(C_FAR / C_NEAR);

    // v = 0 at base (nozzle exit), v = 1 at tip.
    float v = fragUV.y;

    // u = 0..1 across the quad width; remap to [-1, 1] for radial distance.
    float u = fragUV.x;
    float radial = abs(u * 2.0 - 1.0);   // 0 at centre, 1 at edge

    // Alpha: full at base centre, fades both axially toward tip and radially toward edges.
    float axialFade  = 1.0 - v;               // linear fade along length
    float radialFade = 1.0 - radial * radial; // smooth radial falloff
    float alpha = axialFade * radialFade;
    alpha = clamp(alpha, 0.0, 1.0);

    // Colour: white-hot at base, cooling toward tip.
    // fragColor carries the material tint baked in Blender (type-specific).
    vec3 hotCore  = vec3(1.0, 1.0, 1.0);
    vec3 coolTip  = fragColor;
    vec3 color    = mix(hotCore, coolTip, v);

    float intensity = max(pc.baseColor.w, 1.0);
    outColor = vec4(color * intensity, alpha);
}
