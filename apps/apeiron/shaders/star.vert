#version 450

layout(location = 0) in vec3 inDirection;
layout(location = 1) in vec3 inColor;

layout(location = 0) out vec3 fragColor;

layout(push_constant) uniform PC {
    mat4 vpRot;  // VP with translation zeroed — stars appear at infinity
} pc;

void main()
{
    // Project direction as if infinitely far away.
    vec4 clip = pc.vpRot * vec4(inDirection, 0.0);
    // Force depth to the far plane so stars are always behind geometry.
    gl_Position = clip.xyww;

    // Scale point size slightly with brightness so the brightest stars
    // are marginally larger.  Clamped to avoid device-limit issues.
    float lum = dot(inColor, vec3(0.2126, 0.7152, 0.0722));
    gl_PointSize = clamp(0.5 + lum * 0.3, 1.0, 3.0);

    fragColor = inColor;
}
