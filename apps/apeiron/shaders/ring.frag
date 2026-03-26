#version 450

// Binding 0: the ring texture.
// RGB = ring colour, Alpha = ring opacity at this radial position.
layout(set = 0, binding = 0) uniform sampler2D ringTex;

layout(location = 0) in vec2  fragUV;
layout(location = 1) in vec3  fragNormal;
layout(location = 2) in float fragLightIntensity;
layout(location = 3) in vec3  fragSunDir;

layout(location = 0) out vec4 outColor;

void main()
{
    vec4 ring = texture(ringTex, fragUV);
    if (ring.a < 0.01) discard;

    // Rings are lit from both faces (sunlight passes through the thin disc),
    // so use abs() of the dot product.
    float NdotL  = abs(dot(normalize(fragNormal), normalize(fragSunDir)));
    float diffuse = max(NdotL, 0.05);   // small ambient so shadowed side isn't black

    outColor = vec4(ring.rgb * diffuse * fragLightIntensity, ring.a);
}
