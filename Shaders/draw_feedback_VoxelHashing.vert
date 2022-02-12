#version 130

    in vec3 position;
    in vec3 color;
    in vec3 normal;
    uniform mat4 MVP;
    uniform mat4 pose;
    uniform float threshold;
    uniform int colorType;

    uniform float materialShininess;
    uniform vec4 materialAmbient;
    uniform vec4 materialDiffuse;
    uniform vec4 materialSpecular;
    uniform vec4 lightAmbient;
    uniform vec4 lightDiffuse;
    uniform vec4 lightSpecular;


out vec4 vColor;

#include "color.glsl"

void main()
{
    /*if(position.w > threshold)
    {*/
        if(colorType == 4)
        {
            vColor = vec4(decodeColor(color.z), 1.0);
        }
        else if(colorType == 3)
        {
            vColor = vec4(decodeColor(color.y), 1.0);
        }
        else if(colorType == 2)
        {
            vColor = vec4(-normal.xyz,1.0);
        }
        else if(colorType == 1)
        {
            vColor = vec4(decodeColor(color.x), 1.0);
        }
        else
        {
            // use Phong shading

            vec4 material = materialDiffuse;
	    
            vec3 eyeDir = normalize(position.xyz);
	    vec4 light_dir_vec4 = vec4(eyeDir,1.0);
	    vec3 light_dir = light_dir_vec4.xyz; 
            vec3 R = normalize(reflect(-normalize(light_dir), normal.xyz));

            vec4 res = lightAmbient  * materialAmbient                                                       // Ambient
                + lightDiffuse  * material * max(dot(normal.xyz, -normalize(light_dir)), 0.0)                  // Diffuse
                + lightSpecular * materialSpecular * pow(max(dot(R, eyeDir), 0.0f), materialShininess); // Specular

            vColor = clamp(res, 0.0, 1.0);
            //vColor = vec4((vec3(.5f, .5f, .5f) * abs(dot(normal.xyz, vec3(1.0, 1.0, 1.0)))) + vec3(0.1f, 0.1f, 0.1f), 1.0f);
        }
	    gl_Position = MVP * pose * vec4(position.xyz, 1.0);
}
