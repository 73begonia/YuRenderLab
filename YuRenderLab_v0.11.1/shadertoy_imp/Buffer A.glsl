// ======================== Buffer A ========================
// Diffuse IBL using spherical harmonics
//
// Shadertoy channel setup:
//   iChannel0 = Buffer A  (self, filter: nearest, wrap: clamp)
//   iChannel1 = Cubemap   (e.g. "Uffizi Gallery", filter: mipmap, wrap: clamp)

const float c1 = 0.429043;
const float c2 = 0.511664;
const float c3 = 0.743125;
const float c4 = 0.886227;
const float c5 = 0.247708;

const float Y00 = 0.282095;
const float Y1n = 0.488603;
const float Y2n = 1.092548;
const float Y20 = 0.315392;
const float Y22 = 0.546274;

vec3 getRadiance(vec3 dir)
{
    vec3 col = inv_gamma(texture(iChannel1, dir).rgb);
    col += 0.5 * pow(col, vec3(2.0));
    return col;
}

void mainImage(out vec4 fragColor, in vec2 fragCoord)
{
    vec3 currentColour = texture(iChannel1, vec3(1,1,1)).rgb;

    bool cubemapChangedFlag = texelFetch(iChannel0, ivec2(4.5, 4.5), 0).rgb != currentColour;

    bool run = iFrame == 0 || cubemapChangedFlag;

    if (run)
    {
        vec4 col = vec4(0);

        vec2 texCoord = fragCoord.xy / iResolution.xy;
        vec2 thetaphi = ((texCoord * 2.0) - vec2(1.0)) * vec2(PI, HALF_PI);
        vec3 rayDir = vec3( cos(thetaphi.y) * cos(thetaphi.x),
                           -sin(thetaphi.y),
                            cos(thetaphi.y) * sin(thetaphi.x));
        
        col = vec4(getRadiance(rayDir), 1.0);
        col.x = max(col.x, 1e-5);
        col.y = max(col.y, 1e-5);
        col.z = max(col.z, 1e-5);

        if (fragCoord.x < 4.0 && fragCoord.y < 4.0)
        {
            vec3 L00  = vec3(0);
            vec3 L1_1 = vec3(0);
            vec3 L10  = vec3(0);
            vec3 L11  = vec3(0);

            vec3 L2_2 = vec3(0);
            vec3 L2_1 = vec3(0);
            vec3 L20  = vec3(0);
            vec3 L21  = vec3(0);
            vec3 L22  = vec3(0);

            float phi = PI * (3.0 - sqrt(5.0));

            float sampleCount;
            if (run)
            {
                sampleCount = iResolution.x < 2000.0 ? SH_SAMPLE_COUNT : SH_LOW_SAMPLE_COUNT;
            }
            else
            {
                sampleCount = 1.0;
            }

            for (float i = float(ZERO); i < sampleCount; i++)
            {
                float y = 1.0 - (i / sampleCount) * 2.0;
                float radius = sqrt(1.0 - y * y);

                float theta = phi * i;

                float x = cos(theta) * radius;
                float z = sin(theta) * radius;

                vec3 dir = normalize(vec3(x, y, z));

                vec3 radiance = getRadiance(dir);

                L00  += radiance * Y00;
                L1_1 += radiance * Y1n * dir.y;
                L10  += radiance * Y1n * dir.z;
                L11  += radiance * Y1n * dir.x;
                L2_2 += radiance * Y2n * dir.x * dir.y;
                L2_1 += radiance * Y2n * dir.y * dir.z;
                L20  += radiance * Y20 * (3.0 * pow(dir.z, 2.0) - 1.0);
                L21  += radiance * Y2n * dir.x * dir.z;
                L22  += radiance * Y22 * (pow(dir.x, 2.0) - pow(dir.y, 2.0));
            }

            float factor = 4.0 * PI / sampleCount;

            L00  *= factor;
            L1_1 *= factor;
            L10  *= factor;
            L11  *= factor;
            L2_2 *= factor;
            L2_1 *= factor;
            L20  *= factor;
            L21  *= factor;
            L22  *= factor;

            int idxM = int(fragCoord.y-0.5);

            if(fragCoord.x == 0.5){
                mat4 redMatrix;
                redMatrix[0] = vec4(c1*L22.r, c1*L2_2.r, c1*L21.r, c2*L11.r);
                redMatrix[1] = vec4(c1*L2_2.r, -c1*L22.r, c1*L2_1.r, c2*L1_1.r);
                redMatrix[2] = vec4(c1*L21.r, c1*L2_1.r, c3*L20.r, c2*L10.r);
                redMatrix[3] = vec4(c2*L11.r, c2*L1_1.r, c2*L10.r, c4*L00.r-c5*L20.r);
                col = redMatrix[idxM];
            }

            if(fragCoord.x == 1.5){
                mat4 grnMatrix;
                grnMatrix[0] = vec4(c1*L22.g, c1*L2_2.g, c1*L21.g, c2*L11.g);
                grnMatrix[1] = vec4(c1*L2_2.g, -c1*L22.g, c1*L2_1.g, c2*L1_1.g);
                grnMatrix[2] = vec4(c1*L21.g, c1*L2_1.g, c3*L20.g, c2*L10.g);
                grnMatrix[3] = vec4(c2*L11.g, c2*L1_1.g, c2*L10.g, c4*L00.g-c5*L20.g);
                col = grnMatrix[idxM];
            }

            if(fragCoord.x == 2.5){
                mat4 bluMatrix;
                bluMatrix[0] = vec4(c1*L22.b, c1*L2_2.b, c1*L21.b, c2*L11.b);
                bluMatrix[1] = vec4(c1*L2_2.b, -c1*L22.b, c1*L2_1.b, c2*L1_1.b);
                bluMatrix[2] = vec4(c1*L21.b, c1*L2_1.b, c3*L20.b, c2*L10.b);
                bluMatrix[3] = vec4(c2*L11.b, c2*L1_1.b, c2*L10.b, c4*L00.b-c5*L20.b);
                col = bluMatrix[idxM];
            }
        }

        if (fragCoord.x == 4.5 && fragCoord.y == 4.5)
        {
            col = vec4(texture(iChannel1, vec3(1,1,1)).rgb, 1.0);
        }

        if (fragCoord.x == 5.5 && fragCoord.y == 5.5)
        {
            col = vec4(iResolution.xy, 0.0, 1.0);
        }

        fragColor = col;
    }
    else
    {
        fragColor = texelFetch(iChannel0, ivec2(fragCoord.xy), 0);
    }
}
