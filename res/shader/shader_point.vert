
#version 430 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aIntensity;
 
uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;
 
out vec3 ourColor;
void main()
{
    gl_Position = projection * view * model * vec4(aPos, 1.0);
    ourColor = vec3(0.5f, 1.0f, 1.0f);
}
