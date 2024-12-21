#version 400 core

layout(location = 0) out vec4 color;

in vec2			v_TexCoord;

uniform vec4 u_Colour;

void main()
{
	float xval = v_TexCoord.x;
	float yval = v_TexCoord.y;
	float distance = (xval - 0.5f) * (xval - 0.5f) + (yval - 0.5f) * (yval - 0.5f);
	if(distance < 0.25f)
		color = u_Colour;
	else
		discard;
};