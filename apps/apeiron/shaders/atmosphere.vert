#version 450

// Atmosphere shell vertex shader.
//
// The shell mesh is a unit sphere (radius 1.0) in local space.  The model
// matrix scales it to atmosphereRadius_km so it envelops the planet.
//
// fragLocalPos carries the raw local-space vertex position to the fragment
// shader, which uses it to reconstruct the view-ray direction.  The actual
// ray-sphere intersection is performed analytically in the fragment shader
// using the camera position (also in local space) from push constants.

layout(push_constant) uniform PC {
    mat4 mvp;            // full model-view-projection (64 bytes)
    vec4 cameraLocalGr;  // xyz = camera pos in local space, w = groundRatio (Rg/Ra)
    vec4 sunDirLight;    // xyz = sun direction in local space (normalised), w = lightIntensity
    vec4 rayleighBeta;   // xyz = β_R (per unit), w = H_R (per unit)
    vec4 miePack;        // x = β_M scat, y = β_M ext, z = H_M (per unit), w = mieG
} pc;

layout(location = 0) in vec3 inPos;
layout(location = 1) in vec3 inNormal;   // unused
layout(location = 2) in vec3 inColor;    // unused
layout(location = 3) in vec2 inUV;       // unused

// Local-space position of the vertex (= point on the unit sphere).
// The fragment shader uses this to compute the view-ray direction.
layout(location = 0) out vec3 fragLocalPos;

void main() {
    gl_Position  = pc.mvp * vec4(inPos, 1.0);
    fragLocalPos = inPos;
}
