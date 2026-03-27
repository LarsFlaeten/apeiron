#version 450

// Binding 0: the ring texture.
// RGB = ring colour, Alpha = ring opacity at this radial position.
layout(set = 0, binding = 0) uniform sampler2D ringTex;

layout(location = 0) in vec2  fragUV;
layout(location = 1) in vec3  fragNormal;
layout(location = 2) in float fragLightIntensity;
layout(location = 3) in vec3  fragSunDir;
layout(location = 4) in vec3  fragViewDir;

layout(location = 0) out vec4 outColor;

void main()
{
    vec4 ring = texture(ringTex, fragUV);
    if (ring.a < 0.01) discard;

    vec3 N = normalize(fragNormal);
    vec3 L = normalize(fragSunDir);
    vec3 V = normalize(fragViewDir);

    // Lommel-Seeliger scattering law — physically appropriate for particulate rings.
    // Ring particles (ice/rock) are individually lit regardless of the ring plane
    // orientation to the sun, unlike a Lambertian surface which goes dark edge-on.
    //
    //   I ∝  μ₀ / (μ₀ + μ)
    //
    // where μ₀ = sin(solar elevation) = |N·L|
    //       μ  = sin(observer elevation) = |N·V|
    //
    // As sun approaches edge-on (μ₀→0) the ratio stays finite because
    // the denominator also shrinks, so the rings remain visible.
    float mu0 = abs(dot(N, L));
    float mu  = abs(dot(N, V));
    float ls  = mu0 / (mu0 + mu + 0.001);   // small ε avoids 0/0

    // Normalise: at face-on (mu0=mu=1) ls=0.499 → scale by 2 so peak ≈ 1.
    float diffuse = ls * 2.0;

    outColor = vec4(ring.rgb * diffuse * fragLightIntensity, ring.a);
}
