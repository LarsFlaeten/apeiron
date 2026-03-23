#version 450

// Fullscreen triangle with no vertex buffer.
// Vertices 0-2 cover the entire clip space.
layout(location = 0) out vec2 fragUV;

void main()
{
    fragUV      = vec2((gl_VertexIndex << 1) & 2, gl_VertexIndex & 2);
    gl_Position = vec4(fragUV * 2.0 - 1.0, 0.0, 1.0);
}
