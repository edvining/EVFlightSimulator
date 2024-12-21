#version 330 core

layout(location = 0) out vec4 color;

in vec2 v_TexCoord; // Texture coordinates in local quad space (-0.5 to 0.5)

uniform vec4 u_Colour;      // Circle color

void main()
{
	float xval = v_TexCoord.x;
	float yval = v_TexCoord.y;
	float distance = (xval - 0.5f) * (xval - 0.5f) + (yval - 0.5f) * (yval - 0.5f);
	if(distance < 0.25f)
		color = u_Colour * 1-(distance/0.25f);
	else
		discard;
};