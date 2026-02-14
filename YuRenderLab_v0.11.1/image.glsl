// ======================== Image (Main Pass) ========================
// iChannel0 = BufferD (camera state)
// iChannel1 = BufferA (SH matrices + environment map)
// iChannel2 = BufferB (prefiltered specular + BRDF integration)

#iChannel0 "file://buffer_d.glsl"
#iChannel1 "file://buffer_a.glsl"
#iChannel2 "file://buffer_b.glsl"
#include "common.glsl"

// Dynamic object positions (set in mainImage, read in sceneSDF)
vec3 g_spherePos;
vec3 g_tallboxPos;
vec4 g_sphereQuat;
vec4 g_tallboxQuat;
vec3 g_sphereSize;
vec3 g_tallboxSize;

// PBR + IBL rendering of a Cornell box scene.
// No direct lights, no shadows, no normal maps.
// Diffuse IBL via spherical harmonics (BufferA).
// Specular IBL via split-sum prefiltered env map + BRDF LUT (BufferB).

#define TONEMAPPING

const float AMBIENT_STRENGTH = 1.0;
const float EXPOSURE = 1.0;

// Raymarching
const float EPSILON = 1e-3;
const int MAX_MARCH_STEPS = 256;
const float MAX_DIST = 1e3;

// ----------------------- SDF Primitives -----------------------

float sdBox(vec3 p, vec3 b)
{
    vec3 q = abs(p) - b;
    return length(max(q, 0.0)) + min(max(q.x, max(q.y, q.z)), 0.0);
}

float sdSphere(vec3 p, float r)
{
    return length(p) - r;
}

// ----------------------- Cornell Box Scene -----------------------
// Box interior: X in [-2.5, 2.5], Y in [0, 5], Z in [0, 5]
// Open face at Z = 0 (camera side)

const float WALL_THICK = 0.1;

// Material IDs
#define MAT_NONE     0
#define MAT_FLOOR    1
#define MAT_CEILING  2
#define MAT_BACK     3
#define MAT_LEFT     4
#define MAT_RIGHT    5
#define MAT_SPHERE   6
#define MAT_TALL     7

struct SceneHit {
    float d;
    int matId;
};

SceneHit opUnion(SceneHit a, SceneHit b) {
    if (a.d < b.d) return a;
    else return b;
}

SceneHit sceneSDF(vec3 p)
{
    SceneHit h;
    h.d = MAX_DIST;
    h.matId = MAT_NONE;
    float d;

    // Floor: y = 0
    d = sdBox(p - vec3(0.0, -WALL_THICK * 0.5, 2.5),
              vec3(2.5 + WALL_THICK, WALL_THICK * 0.5, 2.5 + WALL_THICK));
    h = opUnion(h, SceneHit(d, MAT_FLOOR));

    // Ceiling: y = 5
    d = sdBox(p - vec3(0.0, 5.0 + WALL_THICK * 0.5, 2.5),
              vec3(2.5 + WALL_THICK, WALL_THICK * 0.5, 2.5 + WALL_THICK));
    h = opUnion(h, SceneHit(d, MAT_CEILING));

    // Back wall: z = 5
    d = sdBox(p - vec3(0.0, 2.5, 5.0 + WALL_THICK * 0.5),
              vec3(2.5 + WALL_THICK, 2.5 + WALL_THICK, WALL_THICK * 0.5));
    h = opUnion(h, SceneHit(d, MAT_BACK));

    // Left wall (red): x = -2.5
    d = sdBox(p - vec3(-2.5 - WALL_THICK * 0.5, 2.5, 2.5),
              vec3(WALL_THICK * 0.5, 2.5 + WALL_THICK, 2.5 + WALL_THICK));
    h = opUnion(h, SceneHit(d, MAT_LEFT));

    // Right wall (green): x = 2.5
    d = sdBox(p - vec3(2.5 + WALL_THICK * 0.5, 2.5, 2.5),
              vec3(WALL_THICK * 0.5, 2.5 + WALL_THICK, 2.5 + WALL_THICK));
    h = opUnion(h, SceneHit(d, MAT_RIGHT));

    // Sphere (dynamic position and size from Buffer D)
    {
        d = sdSphere(p - g_spherePos, g_sphereSize.x);
        h = opUnion(h, SceneHit(d, MAT_SPHERE));
    }

    // Tall box (dynamic position, rotation and size from Buffer D)
    {
        vec3 q = p - g_tallboxPos;
        mat3 rotMat = quatToMat(g_tallboxQuat);
        q = transpose(rotMat) * q;  // inverse rotation for SDF sampling
        d = sdBox(q, g_tallboxSize);
        h = opUnion(h, SceneHit(d, MAT_TALL));
    }

    return h;
}

// Raymarching
float marchScene(vec3 ro, vec3 rd, out int matId)
{
    float t = 0.01;
    matId = MAT_NONE;
    for (int i = 0; i < MAX_MARCH_STEPS; i++)
    {
        SceneHit h = sceneSDF(ro + rd * t);
        if (h.d < EPSILON) {
            matId = h.matId;
            return t;
        }
        t += h.d;
        if (t > MAX_DIST) break;
    }
    return MAX_DIST;
}

// Normal via tetrahedron technique
vec3 getNormal(vec3 p)
{
    vec3 n = vec3(0.0);
    for (int i = 0; i < 4; i++) {
        vec3 e = 0.5773 * (2.0 * vec3((((i+3)>>1)&1), ((i>>1)&1), (i&1)) - 1.0);
        n += e * sceneSDF(p + e * EPSILON).d;
    }
    return normalize(n);
}

// ----------------------- Material -----------------------

vec3 getAlbedo(int matId) {
    if (matId == MAT_LEFT)  return vec3(0.14, 0.45, 0.091);  // Green
    if (matId == MAT_RIGHT) return vec3(0.63, 0.065, 0.05);  // Red
    return vec3(0.73);  // White for all others
}

float getRoughness(int matId) {
    float roughness = 0.8;
    if (matId == MAT_SPHERE) roughness = 0.25;
    if (matId == MAT_TALL)   roughness = 0.6;
    // Clamp to avoid extreme values (0 causes NaN, 1 causes atlas sampling issues)
    return clamp(roughness, 0.05, 0.999);
}

float getMetalness(int matId) {
    if (matId == MAT_SPHERE) return 1.0;  // Metallic sphere
    if (matId == MAT_TALL)   return 0.0;
    return 0.0;  // Dielectric
}

// ----------------------- Camera -----------------------

vec3 rayDirection(float fieldOfView, vec2 fragCoord) {
    vec2 xy = fragCoord - iResolution.xy / 2.0;
    float z = (0.5 * iResolution.y) / tan(radians(fieldOfView) / 2.0);
    return normalize(vec3(xy, -z));
}

mat3 lookAt(vec3 camera, vec3 at, vec3 up) {
    vec3 zaxis = normalize(at - camera);
    vec3 xaxis = normalize(cross(zaxis, up));
    vec3 yaxis = cross(xaxis, zaxis);
    return mat3(xaxis, yaxis, -zaxis);
}

// ----------------------- PBR -----------------------

vec3 fresnelSchlickRoughness(float cosTheta, vec3 F0, float roughness) {
    return F0 + (max(vec3(1.0 - roughness), F0) - F0) * pow(1.0 - cosTheta, 5.0);
}

// ----------------------- IBL -----------------------

vec3 getSHIrradiance(vec3 normal)
{
    vec4 n = vec4(normal, 1.0);

    mat4 redMatrix = mat4(
        texelFetch(iChannel1, ivec2(0,0), 0),
        texelFetch(iChannel1, ivec2(0,1), 0),
        texelFetch(iChannel1, ivec2(0,2), 0),
        texelFetch(iChannel1, ivec2(0,3), 0));

    mat4 grnMatrix = mat4(
        texelFetch(iChannel1, ivec2(1,0), 0),
        texelFetch(iChannel1, ivec2(1,1), 0),
        texelFetch(iChannel1, ivec2(1,2), 0),
        texelFetch(iChannel1, ivec2(1,3), 0));

    mat4 bluMatrix = mat4(
        texelFetch(iChannel1, ivec2(2,0), 0),
        texelFetch(iChannel1, ivec2(2,1), 0),
        texelFetch(iChannel1, ivec2(2,2), 0),
        texelFetch(iChannel1, ivec2(2,3), 0));

    float r = dot(n, redMatrix * n);
    float g = dot(n, grnMatrix * n);
    float b = dot(n, bluMatrix * n);

    return vec3(r, g, b);
}

// Get environment colour from equirectangular projection in BufferA
vec3 getEnvironment(vec3 rayDir, vec2 scaleSize)
{
    vec2 texCoord = vec2((atan(rayDir.z, rayDir.x) / TWO_PI) + 0.5, acos(rayDir.y) / PI);
    texCoord.x = clamp(texCoord.x, 1e-3, 0.999);
    texCoord *= scaleSize;
    return texture(iChannel1, texCoord).rgb;
}

// Get interpolated prefiltered environment for roughness from BufferB atlas
vec3 getEnvironment(vec3 rayDir, float roughness, vec2 scaleSize)
{
    float level1 = floor(1.0 + roughness * 5.0);
    level1 = max(1.0, level1);
    float level2 = min(level1 + 1.0, 5.0);

    float size1 = 1.0 / pow(2.0, level1);
    float size2 = 1.0 / pow(2.0, level2);

    float offset1 = 0.0;
    float i;
    for (i = 1.0; i < level1; i++) {
        offset1 += 1.0 / pow(2.0, i);
    }
    float offset2 = offset1 + 1.0 / pow(2.0, i);

    vec2 tc = vec2((atan(rayDir.z, rayDir.x) / TWO_PI) + 0.5, acos(rayDir.y) / PI);
    vec2 texCoord1 = tc;
    vec2 texCoord2 = tc;

    float f = fract(roughness * 5.0);

    texCoord1.x = clamp(texCoord1.x, level1 * 0.005, 1.0 - level1 * 0.005);
    texCoord2.x = clamp(texCoord2.x, level2 * 0.005, 1.0 - level2 * 0.005);

    texCoord1 = vec2(offset1, 0.0) + size1 * texCoord1;
    texCoord2 = vec2(offset2, 0.0) + size2 * texCoord2;

    texCoord1 *= scaleSize;
    texCoord2 *= scaleSize;

    return mix(texture(iChannel2, texCoord1).rgb, texture(iChannel2, texCoord2).rgb, f);
}

vec2 getBRDFIntegrationMap(vec2 coord, vec2 scaleSize)
{
    coord = clamp(coord, 1e-5, 0.99);
    vec2 texCoord = vec2(coord.x / 2.0, coord.y / 2.0 + 0.5);
    texCoord *= scaleSize;
    return texture(iChannel2, texCoord).rg;
}

// ACES tone mapping
vec3 ACESFilm(vec3 x) {
    return clamp((x * (2.51 * x + 0.03)) / (x * (2.43 * x + 0.59) + 0.14), 0.0, 1.0);
}

// ----------------------- Main -----------------------

void mainImage(out vec4 fragColor, in vec2 fragCoord)
{
    // Read atlas size from Buffer B for resolution scaling
    vec2 atlasSize = texelFetch(iChannel2, ivec2(5, 5), 0).rg;
    vec2 scaleSize = 1.0 / (iResolution.xy / atlasSize);

    // Camera from BufferD
    vec3 camPos = ld(iChannel0, 3, ROW_CAMERA).xyz;
    vec3 camDir = ld(iChannel0, 2, ROW_CAMERA).xyz;
    if (length(camDir) < 0.001) camDir = vec3(0.0, 0.0, 1.0);

    // Load dynamic object positions (fallback to defaults on first frame)
    g_spherePos = ld(iChannel0, OBJ_SPHERE_ID, ROW_OBJECTS).xyz;
    g_tallboxPos = ld(iChannel0, OBJ_TALLBOX_ID, ROW_OBJECTS).xyz;
    if (length(g_spherePos) < 0.001 && length(g_tallboxPos) < 0.001) {
        g_spherePos = SPHERE_INIT_POS;
        g_tallboxPos = TALLBOX_INIT_POS;
    }

    // Load dynamic rotations and sizes (fallback to defaults on first frame)
    g_sphereQuat = ld(iChannel0, OBJ_SPHERE_ID, ROW_ROTATIONS);
    g_tallboxQuat = ld(iChannel0, OBJ_TALLBOX_ID, ROW_ROTATIONS);
    g_sphereSize = ld(iChannel0, OBJ_SPHERE_ID, ROW_SIZES).xyz;
    g_tallboxSize = ld(iChannel0, OBJ_TALLBOX_ID, ROW_SIZES).xyz;
    if (length(g_sphereQuat) < 0.001) g_sphereQuat = quatId();
    if (length(g_tallboxQuat) < 0.001) g_tallboxQuat = quatAxis(vec3(0.0, 1.0, 0.0), -TALLBOX_ROT_ANGLE);
    if (length(g_sphereSize) < 0.001) g_sphereSize = vec3(SPHERE_RADIUS);
    if (length(g_tallboxSize) < 0.001) g_tallboxSize = TALLBOX_HALFSIZE;

    vec3 rd = rayDirection(CAM_FOV, fragCoord);
    vec3 up = vec3(0.0, 1.0, 0.0);
    mat3 viewMatrix = lookAt(camPos, camPos + camDir, up);
    rd = normalize(viewMatrix * rd);

    // Raymarch
    int matId;
    float t = marchScene(camPos, rd, matId);

    vec3 col;

    if (t < MAX_DIST)
    {
        vec3 p = camPos + rd * t;
        vec3 n = getNormal(p);

        vec3 albedo = getAlbedo(matId);
        float roughness = getRoughness(matId);
        float metalness = getMetalness(matId);

        // F0 for dielectrics (IOR 1.5 -> 4% reflectance)
        float IOR = 1.5;
        vec3 F0 = vec3(pow(IOR - 1.0, 2.0) / pow(IOR + 1.0, 2.0));
        F0 = mix(F0, albedo, metalness);

        // BRDF integration LUT
        vec2 envBRDF = getBRDFIntegrationMap(vec2(dot_c(n, -rd), roughness), scaleSize);
        vec3 energyCompensation = 1.0 + F0 * (1.0 / max(envBRDF.y, 0.001) - 1.0);

        // Diffuse IBL (spherical harmonics)
        vec3 F = fresnelSchlickRoughness(dot_c(n, -rd), F0, roughness);
        vec3 kD = (1.0 - F) * (1.0 - metalness);
        vec3 irradiance = AMBIENT_STRENGTH * getSHIrradiance(n);
        vec3 diffuse = irradiance * albedo / PI;

        // Specular IBL (prefiltered environment map)
        vec3 R = reflect(rd, n);
        vec3 prefilteredColor = getEnvironment(R, roughness, scaleSize);
        vec3 specular = prefilteredColor * mix(envBRDF.xxx, envBRDF.yyy, F0);
        specular *= energyCompensation;

        // Combine
        col = EXPOSURE * (kD * diffuse + specular);

        #ifdef TONEMAPPING
            col = ACESFilm(col);
        #endif
    }
    else
    {
        // Sky / environment background
        col = getEnvironment(rd, scaleSize);
    }

    col = gamma(col);

    // ---- Gizmo overlay (post-tonemap) ----
    vec4 stateData = ld(iChannel0, 0, ROW_STATE);
    int selId = int(stateData.y);
    float transMode = stateData.z;
    int actPart = int(stateData.w);

    if (selId >= 0 && selId < OBJ_COUNT) {
        vec3 objPos = ld(iChannel0, selId, ROW_OBJECTS).xyz;
        vec4 objQuat = ld(iChannel0, selId, ROW_ROTATIONS);
        if (length(objQuat) < 0.001) objQuat = quatId();
        float pxScalar = 1.0 / iResolution.y;

        Ray gizmoRay = Ray(camPos, rd);
        GizmoHit gh = traceGizmo(gizmoRay, objPos, camPos, actPart, transMode, objQuat, pxScalar);

        if (gh.t < INF) {
            bool occluded = (t < MAX_DIST && t < gh.t);
            float alpha = occluded ? gh.alpha * 0.2 : gh.alpha;
            vec3 gc = occluded ? gh.col * GZ_OCCLUDED_BRIGHT : gh.col;
            col = mix(col, gc, alpha);
        }
    }

    fragColor = vec4(col, 1.0);
}