#version 400 core

layout(location = 0) out vec4 color;

in vec2			v_TexCoord;

uniform vec4 u_Colour;

void main()
{
	color = u_Colour;
};