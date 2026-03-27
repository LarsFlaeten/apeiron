#version 450

// Atmosphere shell fragment shader — single-scattering with precomputed
// transmittance LUT.
//
// All geometry is in local space where the atmosphere sphere has radius 1.0
// and the ground sphere has radius groundRatio = Rg/Ra.
//
// Output blend equation (src=One, dst=SrcAlpha):
//   result = outColor.rgb  +  background.rgb * outColor.a
//          = L_inscatter   +  background      * transmittance
// This correctly composites in-scattered sunlight on top of whatever is
// behind the atmosphere (planet surface or space).

layout(set = 0, binding = 0) uniform sampler2D u_transmittance;

layout(push_constant) uniform PC {
    mat4 mvp;
    vec4 cameraLocalGr;  // xyz = camera (local), w = groundRatio (Rg/Ra)
    vec4 sunDirLight;    // xyz = sun direction (local, normalised), w = lightIntensity
    vec4 rayleighBeta;   // xyz = β_R (per unit, per wavelength), w = H_R (local)
    vec4 miePack;        // x = β_M scat, y = β_M ext, z = H_M (local), w = mieG
} pc;

layout(location = 0) in  vec3 fragLocalPos;
layout(location = 0) out vec4 outColor;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

// Ray–sphere intersection.  Returns (t_near, t_far); both negative = no hit.
vec2 raySphere(vec3 ro, vec3 rd, float radius) {
    float b    = dot(ro, rd);
    float c    = dot(ro, ro) - radius * radius;
    float disc = b * b - c;
    if (disc < 0.0) return vec2(-1.0);
    float sq = sqrt(disc);
    return vec2(-b - sq, -b + sq);
}

// Sample the precomputed transmittance LUT at (radius r, cos-zenith mu).
// r is in local units [groundRatio, 1.0], mu in [-1, +1].
vec3 sampleT(float r, float mu) {
    float Rg = pc.cameraLocalGr.w;
    float u  = clamp((r - Rg) / (1.0 - Rg), 0.0, 1.0);
    float v  = clamp(mu * 0.5 + 0.5,         0.0, 1.0);
    return texture(u_transmittance, vec2(u, v)).rgb;
}

// Rayleigh phase function.
float phaseR(float cosT) {
    const float inv4pi = 1.0 / (4.0 * 3.14159265);
    return (3.0 * inv4pi * 0.25) * (1.0 + cosT * cosT);
}

// Henyey-Greenstein Mie phase function.
float phaseM(float cosT, float g) {
    float g2  = g * g;
    float denom = pow(1.0 + g2 - 2.0 * g * cosT, 1.5);
    return (3.0 * (1.0 - g2) * (1.0 + cosT * cosT)) /
           (8.0 * 3.14159265 * (2.0 + g2) * max(denom, 1e-6));
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

void main() {
    vec3  camLocal  = pc.cameraLocalGr.xyz;
    float Rg        = pc.cameraLocalGr.w;   // ground sphere radius (local)
    float Ra        = 1.0;                   // atmosphere sphere radius (local)

    vec3  rayDir    = normalize(fragLocalPos - camLocal);
    vec3  sunDir    = pc.sunDirLight.xyz;

    // ---- Ray-sphere intersections ----
    vec2  atmHit    = raySphere(camLocal, rayDir, Ra);
    vec2  gndHit    = raySphere(camLocal, rayDir, Rg);

    // No atmosphere intersection at all → discard.
    if (atmHit.y < 0.0) { outColor = vec4(0.0, 0.0, 0.0, 1.0); return; }

    // Integration bounds: start at atmosphere entry (or 0 if inside),
    // end at ground surface or atmosphere exit.
    float tStart = max(0.0, atmHit.x);
    float tEnd   = (gndHit.x > 0.0 && gndHit.x < atmHit.y)
                   ? gndHit.x : atmHit.y;
    if (tEnd <= tStart) { outColor = vec4(0.0, 0.0, 0.0, 1.0); return; }

    // ---- Single-scattering integration along view ray ----
    const int N  = 16;
    float     dt = (tEnd - tStart) / float(N);

    vec3  accumR    = vec3(0.0);    // in-scattered Rayleigh light
    vec3  accumM    = vec3(0.0);    // in-scattered Mie light
    float odR       = 0.0;          // accumulated Rayleigh optical depth (view)
    float odM       = 0.0;          // accumulated Mie optical depth (view)

    for (int i = 0; i < N; ++i) {
        float t = tStart + (float(i) + 0.5) * dt;
        vec3  P = camLocal + t * rayDir;
        float r = length(P);
        float h = max(0.0, r - Rg);

        float rhoR = exp(-h / pc.rayleighBeta.w);
        float rhoM = exp(-h / pc.miePack.z);

        // Accumulate view-ray optical depth so far.
        odR += rhoR * dt;
        odM += rhoM * dt;

        // Transmittance from camera to sample point.
        vec3 T_cam_P = exp(-(pc.rayleighBeta.xyz * odR + vec3(pc.miePack.y) * odM));

        // Transmittance from sample point toward the sun.
        // Only lit when sun is above the local horizon.
        float mu_s    = dot(normalize(P), sunDir);
        float r_clamped = clamp(r, Rg, Ra);
        vec3  T_P_sun = (mu_s > -0.1)
                        ? sampleT(r_clamped, mu_s)
                        : vec3(0.0);

        // Accumulate in-scattered light for Rayleigh and Mie.
        vec3 T = T_cam_P * T_P_sun;
        accumR += pc.rayleighBeta.xyz * rhoR * T * dt;
        accumM += vec3(pc.miePack.x)  * rhoM * T * dt;
    }

    // ---- Phase functions and solar irradiance ----
    float cosTheta = dot(rayDir, sunDir);
    vec3 L = accumR * phaseR(cosTheta)
           + accumM * phaseM(cosTheta, pc.miePack.w);
    L *= pc.sunDirLight.w;   // lightIntensity = 1/d² in AU

    // ---- Transmittance along the full view segment ----
    vec3  totalT = exp(-(pc.rayleighBeta.xyz * odR + vec3(pc.miePack.y) * odM));
    float T_luma = dot(totalT, vec3(0.2126, 0.7152, 0.0722));  // perceptual average

    // Output: rgb = in-scattered radiance, a = transmittance (for blending).
    // Blend: src=One, dst=SrcAlpha  →  result = L + background * T_luma
    outColor = vec4(L, T_luma);
}
