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

    float dispScale = pc.normalCol0.w;
    float h         = textureLod(heightTex, uv, 0).r;
    vec3  N         = normalize(normal);

    // Longitude/latitude tangents from the unit-sphere direction (pre-displacement).
    // T_lon follows longitude; T_lat follows latitude.
    vec3  sph    = normalize(pos);
    float cosLat = length(sph.xy);
    vec3  T_lon  = (cosLat > 0.001)
                   ? normalize(vec3(-sph.y, sph.x, 0.0))
                   : vec3(1.0, 0.0, 0.0);
    vec3  T_lat  = cross(T_lon, N);

    // Displace vertex outward along the surface normal.
    // displaceScale is a fraction of the unit-sphere radius (≡ fraction of radiusKm).
    pos += N * h * dispScale;
    gl_Position = pc.mvp * vec4(pos, 1.0);

    // Compute the geometric normal of the displaced surface from the
    // heightmap gradient (finite differences along lon/lat directions).
    const float PI  = 3.14159265;
    const float eps = 1.0 / 8192.0;          // one texel of an 8 K heightmap
    float hU    = textureLod(heightTex, uv + vec2(eps, 0.0), 0).r;
    float hV    = textureLod(heightTex, uv + vec2(0.0, eps), 0).r;
    float arcU  = 2.0 * PI * eps * max(cosLat, 0.001);
    float arcV  = PI * eps;
    vec3  displN = normalize(N
        - T_lon * ((hU - h) * dispScale / arcU)
        - T_lat * ((hV - h) * dispScale / arcV));

    // Transform the displaced normal by the planet's orientation matrix.
    vec3 c0 = normalize(pc.normalCol0.xyz);
    vec3 c1 = normalize(pc.normalCol1.xyz);
    mat3 normalMat = mat3(c0, c1, cross(c0, c1));
    fragNormal = normalMat * displN;
    fragColor  = color;
    fragUV     = uv;
}
