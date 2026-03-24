#version 450

layout(location = 0) in  vec2 fragUV;
layout(location = 0) out vec4 outColor;

layout(set = 0, binding = 0) uniform sampler2D srcTex;

layout(push_constant) uniform PC {
    vec2 texelDir;   // (1/w, 0) for horizontal, (0, 1/h) for vertical
} pc;

// 9-tap Gaussian using the bilinear linear-sampling trick (5 fetches).
// Weights/offsets for sigma ≈ 2.
void main()
{
    const float w[5] = float[](0.2270270270, 0.1945945946,
                                0.1216216216, 0.0540540541, 0.0162162162);
    const float o[5] = float[](0.0, 1.3846153846,
                                3.2307692308, 5.0769230769, 6.9230769231);

    vec3 result = texture(srcTex, fragUV).rgb * w[0];
    for (int i = 1; i < 5; ++i) {
        vec2 off = pc.texelDir * o[i];
        result += texture(srcTex, fragUV + off).rgb * w[i];
        result += texture(srcTex, fragUV - off).rgb * w[i];
    }
    outColor = vec4(result, 1.0);
}
