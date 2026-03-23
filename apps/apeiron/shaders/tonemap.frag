#version 450

layout(location = 0) in  vec2 fragUV;
layout(location = 0) out vec4 outColor;

layout(set = 0, binding = 0) uniform sampler2D hdrTex;

layout(push_constant) uniform PC {
    float exposure;
} pc;

// ACES filmic tonemapping approximation (Krzysztof Narkowicz, 2015).
vec3 aces(vec3 x) {
    const float a = 2.51, b = 0.03, c = 2.43, d = 0.59, e = 0.14;
    return clamp((x * (a * x + b)) / (x * (c * x + d) + e), 0.0, 1.0);
}

void main()
{
    vec3 hdr     = texture(hdrTex, fragUV).rgb;
    vec3 exposed = hdr * pc.exposure;
    outColor     = vec4(aces(exposed), 1.0);
}
