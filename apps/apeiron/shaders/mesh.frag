#version 450

layout(location = 0) in  vec3 fragNormal;
layout(location = 1) in  vec3 fragColor;
layout(location = 2) in  vec3 fragPos;
layout(location = 3) in  vec2 fragUV;
layout(location = 0) out vec4 outColor;

// ---- Material descriptor set (set 0) ----
layout(set = 0, binding = 0) uniform MaterialUBO {
    vec4  baseColor;   // RGB base colour factor
    float metallic;
    float roughness;
    float _pad0;
    float _pad1;
} mat;

layout(set = 0, binding = 1) uniform sampler2D albedoTex;

// ---- Push constants ----
layout(push_constant) uniform PC {
    mat4  mvp;
    mat4  modelMat;
    vec4  sunDir;    // xyz = dir toward sun, w = isEmissive
    vec4  baseColor; // w = emissive intensity
} pc;

// C_NEAR is 1 m in km — covers ship close-up views (camera at ~30 m).
const float C_NEAR = 1e-3;
const float C_FAR  = 1.0e9;

// ---------------------------------------------------------------------------
// GGX / Trowbridge-Reitz BRDF helpers
// ---------------------------------------------------------------------------

const float PI = 3.14159265358979;

// Normal distribution function (GGX / Trowbridge-Reitz).
float D_GGX(float NdotH, float roughness) {
    float a  = roughness * roughness;
    float a2 = a * a;
    float d  = (NdotH * NdotH) * (a2 - 1.0) + 1.0;
    return a2 / (PI * d * d);
}

// Geometry term — Smith GGX with Schlick approximation.
float G_Smith(float NdotV, float NdotL, float roughness) {
    float r = roughness + 1.0;
    float k = (r * r) / 8.0;
    float gv = NdotV / (NdotV * (1.0 - k) + k);
    float gl = NdotL / (NdotL * (1.0 - k) + k);
    return gv * gl;
}

// Fresnel — Schlick approximation.
vec3 F_Schlick(float cosTheta, vec3 F0) {
    return F0 + (1.0 - F0) * pow(clamp(1.0 - cosTheta, 0.0, 1.0), 5.0);
}

// ---------------------------------------------------------------------------
void main()
{
    // Logarithmic depth.
    float z_eye  = 1.0 / gl_FragCoord.w;
    gl_FragDepth = log2(z_eye / C_NEAR) / log2(C_FAR / C_NEAR);

    if (pc.sunDir.w > 0.5) {
        // Emissive — plume nodes and emissive materials bypass PBR.
        float intensity = max(pc.baseColor.w, 1.0);
        outColor = vec4(fragColor * intensity, 1.0);
        return;
    }

    // ---- GGX PBR ----
    // Sample albedo texture and combine with material base colour factor
    // and the vertex colour baked in at load time.
    vec3 texColor = texture(albedoTex, fragUV).rgb;
    vec3 albedo   = texColor * mat.baseColor.rgb * fragColor;

    float metallic  = mat.metallic;
    float roughness = max(mat.roughness, 0.04);  // clamp to avoid singularity

    vec3 N = normalize(fragNormal);
    vec3 L = normalize(pc.sunDir.xyz);
    vec3 V = normalize(-fragPos);   // approximate view dir (camera near origin in render space)
    vec3 H = normalize(L + V);

    float NdotL = max(dot(N, L), 0.0);
    float NdotV = max(dot(N, V), 0.0001);
    float NdotH = max(dot(N, H), 0.0);
    float HdotV = max(dot(H, V), 0.0);

    // F0: dialectric = 0.04, metallic = albedo colour.
    vec3 F0 = mix(vec3(0.04), albedo, metallic);

    vec3 F   = F_Schlick(HdotV, F0);
    float D  = D_GGX(NdotH, roughness);
    float G  = G_Smith(NdotV, NdotL, roughness);

    // Specular.
    vec3 specular = (D * G * F) / max(4.0 * NdotV * NdotL, 0.001);

    // Diffuse: metals have no diffuse component.
    vec3 kD = (1.0 - F) * (1.0 - metallic);
    vec3 diffuse = kD * albedo / PI;

    // Direct lighting.
    float ambient = 0.05;
    vec3 color = (diffuse + specular) * NdotL + albedo * ambient;

    outColor = vec4(color, 1.0);
}
