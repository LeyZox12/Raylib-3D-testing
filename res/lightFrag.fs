#version 330
#extension GL_OES_standard_derivatives : enable

out vec4 finalColor;
in vec3 vViewPos;
in vec3 vPos;
in vec3 faceNormal;

uniform sampler2D shadowMap;
uniform mat4 lightMatrix;

uniform vec3 camPos;
uniform vec3 lightPos;

out vec3 fragNormal;

void main() {
    vec3 lightDir = normalize(lightPos - camPos);
    float light = max(0.1, dot(normalize(lightPos-vPos), normalize(faceNormal)));    
    vec4 lightSpacePos = lightMatrix * vec4(vPos, 1.0);
    vec3 projCoords = lightSpacePos.xyz / lightSpacePos.w;
    projCoords = projCoords * 0.5 + 0.5;
    float closestDepth = texture(shadowMap, projCoords.xy).r;
    float currentDepth = projCoords.z;
    float shadow = currentDepth > closestDepth ? 1.0 : 1.0;
    finalColor = vec4(vec3(light) * shadow, 1.0);

}   