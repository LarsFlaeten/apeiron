#version 450

layout(location = 0) in  vec3 fragNormal;
layout(location = 1) in  vec3 fragColor;
layout(location = 2) in  vec2 fragUV;
layout(location = 0) out vec4 outColor;

layout(set = 0, binding = 0) uniform sampler2D diffuseTex;
layout(set = 0, binding = 1) uniform sampler2D specularTex;
layout(set = 0, binding = 2) uniform sampler2D normalTex;
layout(set = 0, binding = 3) uniform sampler2D cloudsTex;

layout(push_constant) uniform PC {
    mat4 mvp;
    vec4 normalCol0;
    vec4 normalCol1;
    vec4 sunDirPad;   // xyz = sunDir,  w = isEmissive (1.0) or lit (0.0)
    vec4 viewDirPad;  // xyz = viewDir, w = lightIntensity (1/d² in AU)
} pc;

const float C_NEAR = 0.1;
const float C_FAR  = 1.0e9;
const float PI     = 3.14159265359;

void main()
{
    // Logarithmic depth.
    float z_eye  = 1.0 / gl_FragCoord.w;
    gl_FragDepth = log2(z_eye / C_NEAR) / log2(C_FAR / C_NEAR);

    // Emissive bodies (the sun): output bright HDR colour, skip all lighting.
    if (pc.sunDirPad.w > 0.5) {
        vec3 diffColor = texture(diffuseTex, fragUV).rgb * fragColor;
        outColor = vec4(diffColor * 50.0, 1.0);
        return;
    }

    vec3 L = normalize(pc.sunDirPad.xyz);
    vec3 V = normalize(pc.viewDirPad.xyz);
    float lightIntensity = pc.viewDirPad.w;

    // Reconstruct rotation matrix (same as vertex shader).
    vec3 c0 = normalize(pc.normalCol0.xyz);
    vec3 c1 = normalize(pc.normalCol1.xyz);
    mat3 normalMat = mat3(c0, c1, cross(c0, c1));

    // TBN for equirectangular sphere (poles at ±Z, u=longitude, v=latitude).
    float phi = fragUV.x * 2.0 * PI;
    vec3 N_geom = normalize(fragNormal);
    vec3 T      = normalize(normalMat * vec3(-sin(phi), cos(phi), 0.0));
    vec3 B      = normalize(cross(N_geom, T));
    mat3 TBN    = mat3(T, B, N_geom);

    // Perturb normal from normal map (tangent-space, OpenGL convention).
    vec3 normalSample = texture(normalTex, fragUV).rgb * 2.0 - 1.0;
    vec3 N = normalize(TBN * normalSample);

    // Diffuse (Lambertian) — use geometric normal so the terminator matches
    // the cloud layer (normal map detail is reserved for specular only).
    float diff      = max(dot(N_geom, L), 0.0);
    vec3  diffColor = texture(diffuseTex, fragUV).rgb * fragColor;
    vec3  color     = diffColor * (0.002 + diff * lightIntensity);

    // Specular (Blinn-Phong), fade near terminator, scaled by light intensity.
    float specMask  = texture(specularTex, fragUV).r;
    float specAtten = smoothstep(0.0, 0.1, dot(N_geom, L));
    if (specMask > 0.01 && specAtten > 0.0) {
        vec3  H    = normalize(L + V);
        float spec = pow(max(dot(N, H), 0.0), 60.0) * specMask * specAtten;
        color += vec3(spec * 0.6 * lightIntensity);
    }

    // Cloud layer (R channel = coverage; lit by geometric normal).
    float cloudCover = texture(cloudsTex, fragUV).r;
    if (cloudCover > 0.01) {
        float cloudDiff  = max(dot(N_geom, L), 0.0);
        vec3  cloudColor = vec3(0.002 + cloudDiff * lightIntensity);
        color = mix(color, cloudColor, cloudCover);
    }

    outColor = vec4(color, 1.0);
}
