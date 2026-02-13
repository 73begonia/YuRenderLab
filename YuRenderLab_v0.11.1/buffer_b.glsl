// ======================== Buffer B ========================
// Specular IBL: prefiltered environment maps + BRDF integration
// iChannel0 = BufferA (SH + environment map)
// iChannel1 = BufferB (self)

#iChannel0 "file://buffer_a.glsl"
#iChannel1 "self"
#include "common.glsl"

vec3 getEnvironment(vec3 rayDir, float level)
{
    vec2 texCoord = vec2((atan(rayDir.z, rayDir.x) / TWO_PI) + 0.5, acos(rayDir.y) / PI);
    return texture(iChannel0, texCoord, level).rgb;
}

float radicalInverse(uint bits) {
    bits = (bits << 16u) | (bits >> 16u);
    bits = ((bits & 0x55555555u) << 1u) | ((bits & 0xAAAAAAAAu) >> 1u);
    bits = ((bits & 0x33333333u) << 2u) | ((bits & 0xCCCCCCCCu) >> 2u);
    bits = ((bits & 0x0F0F0F0Fu) << 4u) | ((bits & 0xF0F0F0F0u) >> 4u);
    bits = ((bits & 0x00FF00FFu) << 8u) | ((bits & 0xFF00FF00u) >> 8u);
    return float(bits) * 2.3283064365386963e-10; // / 0x100000000
}

vec2 hammersley(int i, int N)
{
    return vec2(float(i)/float(N), radicalInverse(uint(i)));
}

vec3 rotateToNormal(vec3 L, vec3 N)
{
    vec3 tangent;
    vec3 bitangent;

    pixarONB(N, tangent, bitangent);

    tangent = normalize(tangent);
    bitangent = normalize(bitangent);

    return normalize(tangent * L.x + bitangent * L.y + N * L.z);
}

vec3 importanceSampleGGX(vec2 Xi, vec3 N, float roughness)
{
    float a = roughness*roughness;

    float cosTheta = sqrt((1.0 - Xi.x) / (1.0 + (a * a - 1.0) * Xi.x));
    float sinTheta = sqrt(1.0 - cosTheta * cosTheta);
    float phi = Xi.y * 2.0 * PI;

    vec3 L = normalize(vec3(cos(phi) * sinTheta, sin(phi) * sinTheta, cosTheta));

    return rotateToNormal(L, N);
}

float distribution(float NdotH, float roughness)
{
    float a2 = roughness * roughness;
    return a2 / (PI * pow(NdotH * NdotH * (a2 - 1.0) + 1.0, 2.0));
}

vec3 getPreFilteredColour(vec3 N, float roughness, int sampleCount)
{
    vec3 R = N;
    vec3 V = R;

    float totalWeight = 0.0;
    vec3 prefilteredColor = vec3(0.0);

    for (int i = ZERO; i < sampleCount; i++)
    {
        vec2 Xi = hammersley(i, sampleCount);
        vec3 H = importanceSampleGGX(Xi, N, roughness);
        vec3 L = normalize(reflect(-V, H));

        float NdotL = dot_c(N, L);

        if (NdotL > 0.0)
        {
            float level = 0.0;

        #if ENV_FILTERING == 1
            float NdotH = dot_c(N, H);
            float VdotH = dot_c(V, H);

            float pdf = distribution(NdotH, roughness*roughness) * NdotH / (4.0 * VdotH);

            float omegaS = 1.0 / (float(sampleCount) * pdf);

            float envMapSize = 512.0;
            float omegaP = 4.0 * PI / (6.0 * envMapSize * envMapSize);
            float mipBias = 1.0;
            level = max(0.5 * log2(omegaS / omegaP) + mipBias, 0.0);
        #endif

            prefilteredColor += getEnvironment(L, level) * NdotL;
            totalWeight      += NdotL;
        }
    }
    prefilteredColor = prefilteredColor / totalWeight;

    return prefilteredColor;
}

float geometry(float cosTheta, float k)
{
    return (cosTheta) / (cosTheta * (1.0 - k) + k);
}

float smithShadowing(float NdotV, float NdotL, float roughness)
{
    float k = (roughness * roughness) / 2.0;
    return geometry(NdotV, k) * geometry(NdotL, k);
}

vec2 integrateBRDF(float NdotV, float roughness, int sampleCount)
{
    vec3 N = vec3(0.0, 0.0, 1.0);

    vec3 V = normalize(vec3(sqrt(1.0 - NdotV * NdotV), 0.0, NdotV));

    vec2 result = vec2(0);

    for (int i = ZERO; i < sampleCount; i++)
    {
        vec2 Xi = hammersley(i, sampleCount);
        vec3 H = importanceSampleGGX(Xi, N, roughness);
        vec3 L = normalize(reflect(-V, H));

        float NdotL = dot_c(N, L);
        float NdotH = dot_c(N, H);
        float VdotH = dot_c(V, H);

        if (NdotL > 0.0)
        {
            float G = smithShadowing(NdotV, NdotL, roughness);
            float S = (G * VdotH) / (NdotH * NdotV);

            float F = pow(1.0 - VdotH, 5.0);

            result.x += F * S;
            result.y += S;
        }
    }
    return result / float(sampleCount);
}

void mainImage(out vec4 fragColor, in vec2 fragCoord)
{
    vec3 currentColour = texelFetch(iChannel0, ivec2(4.5, 4.5), 0).rgb;

    bool cubemapChangedFlag = texelFetch(iChannel1, ivec2(4.5, 4.5), 0).rgb != currentColour;

    bool run = iFrame == 0 || iFrame == 10 || cubemapChangedFlag;

    if (run)
    {
        int sampleCount;
        if (run)
        {
            sampleCount = iResolution.x < 2000.0 ? SAMPLE_COUNT : LOW_SAMPLE_COUNT;
        }
        else
        {
            sampleCount = 1;
        }

        vec3 col = vec3(0);
        float factor = 1.0/2.0;
        float roughness = 0.0;

        if (fragCoord.y < 0.5 * iResolution.y)
        {
            if (fragCoord.x > 0.5 * iResolution.x)
            {
                factor = 1.0/4.0;
                roughness = 0.25;
            }
            if (fragCoord.x > 0.75 * iResolution.x)
            {
                factor = 1.0/8.0;
                roughness = 0.5;
            }
            if (fragCoord.x > 0.875 * iResolution.x)
            {
                factor = 1.0/16.0;
                roughness = 0.75;
            }
            if (fragCoord.x > 0.9375 * iResolution.x)
            {
                factor = 1.0/32.0;
                roughness = 1.0;
            }

            vec2 texCoord = fragCoord.xy / (iResolution.xy * factor);
            vec2 thetaphi = ((texCoord * 2.0) - vec2(1.0)) * vec2(PI, HALF_PI);
            vec3 rayDir = vec3( cos(thetaphi.y) * cos(thetaphi.x),
                               -sin(thetaphi.y),
                                cos(thetaphi.y) * sin(thetaphi.x));
            
            if (fragCoord.x < 0.5 * iResolution.x)
            {
                col = getEnvironment(rayDir, 0.0);
            }
            else
            {
                col = getPreFilteredColour(rayDir, roughness, sampleCount);
            }
        }
        else
        {
            if (fragCoord.x < 0.5 * iResolution.x)
            {
                if (iFrame == 0 || iFrame == 10)
                {
                    vec2 texCoord = vec2(2.0*fragCoord.x/iResolution.x,
                                         2.0*(fragCoord.y/iResolution.y - 0.5));
                    vec2 c = integrateBRDF(texCoord.x, texCoord.y,
                    iResolution.x < 2000.0 ? BRDF_SAMPLE_COUNT : BRDF_LOW_SAMPLE_COUNT);
                    col = vec3(c.x, c.y, 0.0);
                }
                else
                {
                    col = texture(iChannel1, fragCoord.xy / iResolution.xy).rgb;
                }
            }
        }
        if (fragCoord.x == 4.5 && fragCoord.y == 4.5)
        {
            col = texelFetch(iChannel0, ivec2(4.5, 4.5), 0).rgb;
        }

        if (fragCoord.x == 5.5 && fragCoord.y == 5.5)
        {
            col = vec3(iResolution.xy, 0.0);
        }

        fragColor = vec4(col, 1.0);
    }
    else
    {
        fragColor = texture(iChannel1, fragCoord.xy / iResolution.xy);
    }
}