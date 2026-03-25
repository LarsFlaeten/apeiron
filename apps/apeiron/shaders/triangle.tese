#version 450

layout(triangles, equal_spacing, ccw) in;

layout(location = 0) in  vec3 teNormal[];
layout(location = 1) in  vec3 teColor[];
layout(location = 2) in  vec2 teUV[];

layout(location = 0) out vec3 fragNormal;
layout(location = 1) out vec3 fragColor;
layout(location = 2) out vec2 fragUV;

layout(set = 0, binding = 4) uniform sampler2D heightTex;

layout(push_constant) uniform PC {
    mat4 mvp;
    vec4 normalCol0;   // w = displaceScale (fraction of sphere radius)
    vec4 normalCol1;
    vec4 sunDirPad;
    vec4 viewDirPad;
} pc;

void main()
{
    vec3 b = gl_TessCoord;

    // Barycentric interpolation of object-space attributes.
    vec3 pos    = b.x * gl_in[0].gl_Position.xyz
                + b.y * gl_in[1].gl_Position.xyz
                + b.z * gl_in[2].gl_Position.xyz;
    vec3 normal = b.x * teNormal[0]
                + b.y * teNormal[1]
                + b.z * teNormal[2];
    vec2 uv     = b.x * teUV[0]
                + b.y * teUV[1]
                + b.z * teUV[2];
    vec3 color  = b.x * teColor[0]
                + b.y * teColor[1]
                + b.z * teColor[2];

    // Displace vertex outward along the surface normal.
    // displaceScale is a fraction of the unit-sphere radius (≡ fraction of radiusKm).
    float height     = texture(heightTex, uv).r;
    float dispScale  = pc.normalCol0.w;
    pos += normalize(normal) * height * dispScale;

    gl_Position = pc.mvp * vec4(pos, 1.0);

    // Transform the mesh normal by the planet's orientation matrix.
    vec3 c0 = normalize(pc.normalCol0.xyz);
    vec3 c1 = normalize(pc.normalCol1.xyz);
    mat3 normalMat = mat3(c0, c1, cross(c0, c1));
    fragNormal = normalMat * normalize(normal);
    fragColor  = color;
    fragUV     = uv;
}
