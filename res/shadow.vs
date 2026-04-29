#version 330

out vec4 FragColor;

in vec4 vertexPosition;

uniform mat4 matProjection;
out vec3 vViewPos;
out vec3 vPos;
uniform mat4 mvp;

uniform vec3 lightPos;

void main()
{
    float depth = length(vertexPosition.xyz - lightPos) / 100.0;
    FragColor = vec4(vec3(depth), 1.0);
    gl_Position = mvp * vertexPosition;
}