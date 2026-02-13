const float PI = 3.1415926535;
const float TWO_PI = 2.0 * PI;
const float HALF_PI = 0.5 * PI;

#define GAMMA 2.2
#define INV_GAMMA (1.0/GAMMA)

#define ZERO (min(iFrame, 0))

#define SH_SAMPLE_COUNT 4096.0
#define SH_LOW_SAMPLE_COUNT 1024.0

#define SAMPLE_COUNT 1024
#define LOW_SAMPLE_COUNT 128

#define BRDF_SAMPLE_COUNT 1024
#define BRDF_LOW_SAMPLE_COUNT 128

#define ENV_FILTERING 0

const float minDot = 1e-5;

float dot_c(vec3 a, vec3 b)
{
    return max(dot(a, b), minDot);
}

float sat(float x)
{
    return clamp(x, 0.0, 1.0);
}

vec3 sat(vec3 x)
{
    return clamp(x, vec3(0), vec3(1));
}

float modulo(float m, float n)
{
    return mod(mod(m, n) + n, n);
}

vec3 gamma(vec3 col)
{
    return pow(col, vec3(INV_GAMMA));
}

vec3 inv_gamma(vec3 col)
{
    return pow(col, vec3(GAMMA));
}

void pixarONB(vec3 n, out vec3 b1, out vec3 b2)
{
    float sign_ = n.z >= 0.0 ? 1.0 : -1.0;
    float a = -1.0 / (sign_ + n.z);
    float b = n.x * n.y * a;
    b1 = vec3(1.0 + sign_ * n.x * n.x * a, sign_ * b, -sign_ * n.x);
    b2 = vec3(b, sign_ + n.y * n.y * a, -n.y);
}

// ----------------------- Camera configuration -----------------------

#define CAM_FOV 60.0
#define CAM_INIT_POS vec3(0.0, 2.5, -3.5)
#define CAM_INIT_DIR vec3(0.0, 0.0, 1.0)
#define CAM_SENS_X 10.0
#define CAM_SENS_Y 5.0
#define CAM_SPEED 0.05
#define CAM_SPRINT 3.0

// ----------------------- Data layout -----------------------

#define ROW_CAMERA 0

// ----------------------- Key codes -----------------------

#define KEY_W 87
#define KEY_A 65
#define KEY_S 83
#define KEY_D 68
#define KEY_SPACE 32
#define KEY_CTRL 17
#define KEY_SHIFT 16

// ----------------------- Helper functions -----------------------

vec4 ld(sampler2D b, int x, int y) { return texelFetch(b, ivec2(x, y), 0); }

bool keyDown(sampler2D kb, int k) { return texelFetch(kb, ivec2(k, 0), 0).x > 0.0; }

vec3 angles2dir(vec2 a) {
    float cp = cos(a.x), sp = sin(a.x);
    float cy = cos(a.y), sy = sin(a.y);
    return vec3(cp * sy, sp, cp * cy);
}