#version 450

layout(location = 0) in  vec2 fragUV;
layout(location = 0) out vec4 outColor;

layout(set = 0, binding = 0) uniform sampler2D hdrTex;

layout(push_constant) uniform PC {
    float threshold;
} pc;

void main()
{
    vec3  color = texture(hdrTex, fragUV).rgb;
    float lum   = dot(color, vec3(0.2126, 0.7152, 0.0722));
    // Soft extraction: keep only the portion above the threshold.
    float bright = max(lum - pc.threshold, 0.0) / max(lum, 1e-4);
    outColor = vec4(color * bright, 1.0);
}
