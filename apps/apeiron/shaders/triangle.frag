#version 450

layout(location = 0) in  vec3 fragNormal;
layout(location = 1) in  vec3 fragColor;
layout(location = 0) out vec4 outColor;

// Must match Camera near/far.  Will be pushed as a constant once we finalise
// the physical scale.  log2(FAR/NEAR) ≈ 43.25 at these values.
const float C_NEAR = 0.1;
const float C_FAR  = 1.0e9;

// Sun: fixed directional light (normalised in world space).
const vec3  kLightDir = normalize(vec3(1.0, 1.5, 1.0));
const float kAmbient  = 0.08;

void main()
{
    // Logarithmic depth — remaps [near, far] non-linearly so precision is
    // distributed as equal ratios rather than equal distances.
    // gl_FragCoord.w = 1 / w_clip = 1 / z_eye (perspective-correct).
    float z_eye      = 1.0 / gl_FragCoord.w;
    gl_FragDepth     = log2(z_eye / C_NEAR) / log2(C_FAR / C_NEAR);

    // Phong diffuse
    vec3  n       = normalize(fragNormal);
    float diffuse = max(dot(n, kLightDir), 0.0);
    vec3  color   = fragColor * (kAmbient + diffuse);
    outColor = vec4(color, 1.0);
}
