#version 450

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec3 inNormal;
layout(location = 2) in vec3 inColor;
layout(location = 3) in vec2 inUV;

layout(location = 0) out vec3 tcNormal;
layout(location = 1) out vec3 tcColor;
layout(location = 2) out vec2 tcUV;

void main()
{
    // Pass raw object-space data to the tessellation control shader.
    // MVP is applied in the tessellation evaluation shader after displacement.
    gl_Position = vec4(inPosition, 1.0);
    tcNormal    = inNormal;
    tcColor     = inColor;
    tcUV        = inUV;
}
