const float PI = 3.1415926535;
const float TWO_PI = 2.0 * PI;
const float HALF_PI = 0.5 * PI;
#define TAU TWO_PI

#define GAMMA 2.2
#define INV_GAMMA (1.0/GAMMA)

#define ZERO (min(iFrame, 0))
#define INF 1e10

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
#define CAM_FOCUS_DIST 2.0

// ----------------------- Data layout -----------------------

#define ROW_CAMERA 0
#define ROW_STATE 1
#define ROW_KEYS 2
#define ROW_OBJECTS 3
#define ROW_DRAG 4
#define ROW_ROTATIONS 5
#define ROW_SIZES 6
#define ROW_DRAG_ROT 7
#define ROW_UNDO 8
#define ROW_UNDO_ROT 9
#define ROW_CLIPBOARD 10
#define ROW_CLIP_ROT 11
#define ROW_MATERIAL 12
#define ROW_PANEL 13

// ----------------------- Interaction modes -----------------------

#define MODE_NONE 0.0
#define MODE_CAMERA 1.0
#define MODE_TRANSFORM 2.0
#define MODE_SLIDER 3.0
#define MODE_PICKER_SV 4.0
#define MODE_PICKER_H 5.0

// ----------------------- Key codes -----------------------

#define KEY_W 87
#define KEY_A 65
#define KEY_S 83
#define KEY_D 68
#define KEY_Q 81
#define KEY_E 69
#define KEY_R 82
#define KEY_F 70
#define KEY_Z 90
#define KEY_C 67
#define KEY_V 86
#define KEY_SPACE 32
#define KEY_CTRL 17
#define KEY_SHIFT 16
#define KEY_ALT 18

// ----------------------- Change flags -----------------------

#define CHG_NONE 0u
#define CHG_CAMERA 1u
#define CHG_SELECTION 2u
#define CHG_OBJECT 4u
#define CHG_DRAG 8u
#define CHG_UNDO 16u
#define CHG_UNDO_EXEC 32u
#define CHG_CLIPBOARD 64u
#define CHG_MATERIAL 128u
#define CHG_PANEL 256u

// ----------------------- Helper functions -----------------------

vec4 ld(sampler2D b, int x, int y) { return texelFetch(b, ivec2(x, y), 0); }

bool keyDown(sampler2D kb, int k) { return texelFetch(kb, ivec2(k, 0), 0).x > 0.0; }

vec3 angles2dir(vec2 a) {
    float cp = cos(a.x), sp = sin(a.x);
    float cy = cos(a.y), sy = sin(a.y);
    return vec3(cp * sy, sp, cp * cy);
}

// ----------------------- Input State -----------------------

struct InpState {
    vec2 mpos;
    bool mdown;
    bool mpressed;
    bool mreleased;
    bool kshift;
    bool kalt;
    bool kctrl;
};

InpState gatherInp(sampler2D kb, vec4 mouse, float prevDown) {
    InpState s;
    s.mpos = mouse.xy;
    s.mdown = mouse.z > 0.0;
    s.mpressed = s.mdown && prevDown < 0.5;
    s.mreleased = !s.mdown && prevDown > 0.5;
    s.kshift = keyDown(kb, KEY_SHIFT);
    s.kalt = keyDown(kb, KEY_ALT);
    s.kctrl = keyDown(kb, KEY_CTRL);
    return s;
}

// =====================================================================
// Gizmo System (Phase 1: Translation)
// =====================================================================

// ----------------------- Transform constants -----------------------

#define TRANS_TRANSLATE 0.0
#define TRANS_ROTATE 1.0
#define TRANS_SCALE 2.0

#define PART_NONE -1
#define PART_CENTER 0
#define PART_X 1
#define PART_Y 2
#define PART_Z 3

// ----------------------- Gizmo sizing -----------------------

#define GZ_SIZE 0.8
#define GZ_CYL_RAD 0.015
#define GZ_CONE_LEN 0.05
#define GZ_CONE_RAD 0.04
#define GZ_CENTER_RAD 0.045
#define GZ_RING_RAD 0.7
#define GZ_RING_TUBE 0.02
#define GZ_PICK_RAD 0.08
#define GZ_MIN_SCALE 0.3
#define GZ_MAX_SCALE 2.0
#define GZ_SCALE_FACTOR 0.2
#define GZ_OCCLUDED_BRIGHT 0.35

// ----------------------- Snapping -----------------------

#define TF_SNAP_TRANS 0.25
#define TF_SNAP_ROT (PI / 12.0)
#define TF_SNAP_SCALE 0.1
#define TF_SCALE_MIN 0.1
#define TF_SCALE_MAX 5.0
#define TF_ROT_SENS 3.0
#define TF_SCALE_SENS 2.0

// ----------------------- Gizmo colors -----------------------

const vec3 AXIS_COLS[3] = vec3[3](
    vec3(0.9, 0.15, 0.15),   // X - Red
    vec3(0.15, 0.85, 0.15),  // Y - Green
    vec3(0.2, 0.35, 0.9)     // Z - Blue
);
#define AXIS_COL_CENTER vec3(0.92, 0.92, 0.92)
#define AXIS_COL_HL vec3(1.0, 0.85, 0.1)

// ----------------------- Scene objects (dynamic) -----------------------

#define OBJ_SPHERE_ID 0
#define OBJ_TALLBOX_ID 1
#define OBJ_COUNT 2

#define SPHERE_RADIUS 0.75
#define SPHERE_INIT_POS vec3(-0.8, 0.75, 1.8)
#define TALLBOX_HALFSIZE vec3(0.7, 1.5, 0.7)
#define TALLBOX_INIT_POS vec3(0.9, 1.5, 3.2)
#define TALLBOX_ROT_ANGLE (-0.30)

// ----------------------- Gizmo data structures -----------------------

struct Ray { vec3 ro; vec3 rd; };

struct GizmoHit {
    float t;
    float alpha;
    vec3 col;
    int part;
};

// =====================================================================
// Quaternion Math
// =====================================================================

vec4 quatId() { return vec4(0.0, 0.0, 0.0, 1.0); }

vec4 quatMul(vec4 a, vec4 b) {
    return vec4(
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z
    );
}

vec4 quatAxis(vec3 axis, float angle) {
    float ha = angle * 0.5;
    return vec4(axis * sin(ha), cos(ha));
}

vec3 quatRot(vec4 q, vec3 v) {
    vec3 t = 2.0 * cross(q.xyz, v);
    return v + q.w * t + cross(q.xyz, t);
}

vec4 quatConj(vec4 q) {
    return vec4(-q.xyz, q.w);
}

mat3 quatToMat(vec4 q) {
    float x = q.x, y = q.y, z = q.z, w = q.w;
    float x2 = x + x, y2 = y + y, z2 = z + z;
    float xx = x * x2, xy = x * y2, xz = x * z2;
    float yy = y * y2, yz = y * z2, zz = z * z2;
    float wx = w * x2, wy = w * y2, wz = w * z2;
    return mat3(
        1.0 - (yy + zz), xy + wz, xz - wy,
        xy - wz, 1.0 - (xx + zz), yz + wx,
        xz + wy, yz - wx, 1.0 - (xx + yy)
    );
}

vec4 matToQuat(mat3 m) {
    float tr = m[0][0] + m[1][1] + m[2][2];
    vec4 q;
    if (tr > 0.0) {
        float s = sqrt(tr + 1.0) * 2.0;
        q = vec4((m[1][2] - m[2][1]) / s, (m[2][0] - m[0][2]) / s, (m[0][1] - m[1][0]) / s, 0.25 * s);
    } else if (m[0][0] > m[1][1] && m[0][0] > m[2][2]) {
        float s = sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0;
        q = vec4(0.25 * s, (m[0][1] + m[1][0]) / s, (m[2][0] + m[0][2]) / s, (m[1][2] - m[2][1]) / s);
    } else if (m[1][1] > m[2][2]) {
        float s = sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0;
        q = vec4((m[0][1] + m[1][0]) / s, 0.25 * s, (m[1][2] + m[2][1]) / s, (m[2][0] - m[0][2]) / s);
    } else {
        float s = sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0;
        q = vec4((m[2][0] + m[0][2]) / s, (m[1][2] + m[2][1]) / s, 0.25 * s, (m[0][1] - m[1][0]) / s);
    }
    return normalize(q);
}

vec4 applyWorldRot(vec4 q, float rx, float ry, float rz) {
    vec4 qx = quatAxis(vec3(1.0, 0.0, 0.0), rx);
    vec4 qy = quatAxis(vec3(0.0, 1.0, 0.0), ry);
    vec4 qz = quatAxis(vec3(0.0, 0.0, 1.0), rz);
    return normalize(quatMul(qz, quatMul(qy, quatMul(qx, q))));
}

// ----------------------- Interaction Result -----------------------

struct IxResult {
    uint flags;
    vec3 camPos;
    vec3 camDir;
    vec2 angles;
    vec2 dragAngles;
    vec2 dragStart;
    float imode;
    // Gizmo fields
    int selId;
    float transMode;
    int actPart;
    int targetId;
    vec3 objPos;
    vec3 dragVal;
    vec3 dragPlane;
    // Rotation / Scale fields
    float dragAngle;
    vec4 dragQuat;
    vec4 objQuat;
    vec3 objSiz;
    // Undo fields
    vec3 undoPos;
    vec3 undoSiz;
    vec4 undoQuat;
    // Clipboard fields
    vec3 clipPos;
    vec3 clipSiz;
    vec4 clipQuat;
    // Material / panel fields
    vec3 hsv;
    float roughness;
    float metalness;
    float exposure;
    float sunAngle;
    float sliderDragStart;  // drag origin value
    int sliderId;           // which slider is being dragged
};

IxResult initIxResult() {
    IxResult r;
    r.flags = CHG_NONE;
    r.camPos = vec3(0.0);
    r.camDir = vec3(0.0);
    r.angles = vec2(0.0);
    r.dragAngles = vec2(0.0);
    r.dragStart = vec2(0.0);
    r.imode = MODE_NONE;
    r.selId = -1;
    r.transMode = TRANS_TRANSLATE;
    r.actPart = PART_NONE;
    r.targetId = -1;
    r.objPos = vec3(0.0);
    r.dragVal = vec3(0.0);
    r.dragPlane = vec3(0.0);
    r.dragAngle = 0.0;
    r.dragQuat = quatId();
    r.objQuat = quatId();
    r.objSiz = vec3(1.0);
    r.undoPos = vec3(0.0);
    r.undoSiz = vec3(0.0);
    r.undoQuat = quatId();
    r.clipPos = vec3(0.0);
    r.clipSiz = vec3(0.0);
    r.clipQuat = quatId();
    r.hsv = vec3(0.0);
    r.roughness = 0.5;
    r.metalness = 0.0;
    r.exposure = 1.0;
    r.sunAngle = 0.0;
    r.sliderDragStart = 0.0;
    r.sliderId = -1;
    return r;
}

// =====================================================================
// HSV Conversion
// =====================================================================

vec3 hsv2rgb(vec3 c) {
    float h = c.x * 6.0, s = c.y, v = c.z;
    float C = v * s;
    float X = C * (1.0 - abs(mod(h, 2.0) - 1.0));
    float m = v - C;
    vec3 rgb = h < 1.0 ? vec3(C, X, 0.0) : h < 2.0 ? vec3(X, C, 0.0) :
               h < 3.0 ? vec3(0.0, C, X) : h < 4.0 ? vec3(0.0, X, C) :
               h < 5.0 ? vec3(X, 0.0, C) : vec3(C, 0.0, X);
    return rgb + m;
}

vec3 rgb2hsv(vec3 c) {
    float cmax = max(c.r, max(c.g, c.b));
    float cmin = min(c.r, min(c.g, c.b));
    float d = cmax - cmin;
    float h = 0.0;
    if (d > 0.0001) {
        if (cmax == c.r) h = mod((c.g - c.b) / d, 6.0) / 6.0;
        else if (cmax == c.g) h = ((c.b - c.r) / d + 2.0) / 6.0;
        else h = ((c.r - c.g) / d + 4.0) / 6.0;
    }
    return vec3(h, cmax > 0.0001 ? d / cmax : 0.0, cmax);
}

float snap(float v, float s) { return round(v / s) * s; }
vec3 snap3(vec3 v, float s) { return vec3(snap(v.x, s), snap(v.y, s), snap(v.z, s)); }

vec3 axisVec(int a) {
    return a == 0 ? vec3(1, 0, 0) : (a == 1 ? vec3(0, 1, 0) : vec3(0, 0, 1));
}

vec3 axisCol(int a) { return AXIS_COLS[clamp(a, 0, 2)]; }

float gizmoScl(vec3 c, vec3 cam) {
    return clamp(length(c - cam) * GZ_SCALE_FACTOR, GZ_MIN_SCALE, GZ_MAX_SCALE);
}

vec3 applyShade(vec3 col, vec3 n) {
    return col * (0.4 + max(dot(n, normalize(vec3(0.5, 0.8, -0.3))), 0.0) * 0.6);
}

// =====================================================================
// Ray Helpers
// =====================================================================

Ray createPickRay(vec2 mousePos, vec2 res, vec3 camPos, vec3 camDir) {
    vec2 xy = mousePos - res / 2.0;
    float z = (0.5 * res.y) / tan(radians(CAM_FOV) / 2.0);
    vec3 rdLocal = normalize(vec3(xy, -z));

    vec3 up = vec3(0.0, 1.0, 0.0);
    vec3 zaxis = normalize(camDir);
    vec3 xaxis = normalize(cross(zaxis, up));
    if (length(cross(zaxis, up)) < 0.001) xaxis = vec3(1.0, 0.0, 0.0);
    vec3 yaxis = cross(xaxis, zaxis);
    mat3 viewMat = mat3(xaxis, yaxis, -zaxis);

    return Ray(camPos, normalize(viewMat * rdLocal));
}

vec3 rayPlane(vec3 ro, vec3 rd, vec3 pp, vec3 pn) {
    float d = dot(rd, pn);
    if (abs(d) < 0.0001) return pp;
    return ro + rd * max(dot(pp - ro, pn) / d, 0.0);
}

vec3 projectAxis(Ray r, vec3 pp, vec3 axis, vec3 camDir) {
    vec3 pn = normalize(cross(axis, cross(camDir, axis)));
    if (length(pn) < 0.001) pn = -camDir;
    return rayPlane(r.ro, r.rd, pp, pn);
}

float calcRotAngle(vec3 ro, vec3 rd, vec3 c, vec3 axis) {
    vec3 hit = rayPlane(ro, rd, c, axis) - c;
    if (length(hit) < 0.001) return 0.0;
    hit = normalize(hit);
    vec3 up = abs(axis.y) < 0.99 ? vec3(0.0, 1.0, 0.0) : vec3(1.0, 0.0, 0.0);
    vec3 rt = normalize(cross(up, axis));
    return atan(dot(hit, cross(axis, rt)), dot(hit, rt));
}

float angleDiff(float cur, float start) {
    float d = cur - start;
    return d > PI ? d - TAU : (d < -PI ? d + TAU : d);
}

// =====================================================================
// Analytical Intersection (for object selection)
// =====================================================================

bool intersectSphere(Ray r, vec3 c, float rad, out float t) {
    vec3 oc = r.ro - c;
    float b = dot(oc, r.rd);
    float det = b * b - (dot(oc, oc) - rad * rad);
    if (det < 0.0) return false;
    t = -b - sqrt(det);
    if (t < 0.001) { t = -b + sqrt(det); if (t < 0.001) return false; }
    return true;
}

bool intersectBox(Ray r, vec3 c, vec3 hs, out float t) {
    vec3 inv_rd = 1.0 / r.rd;
    vec3 t1 = (c - hs - r.ro) * inv_rd;
    vec3 t2 = (c + hs - r.ro) * inv_rd;
    vec3 tmin = min(t1, t2);
    vec3 tmax = max(t1, t2);
    float enter = max(max(tmin.x, tmin.y), tmin.z);
    float texit = min(min(tmax.x, tmax.y), tmax.z);
    if (enter > texit || texit < 0.0) return false;
    t = enter > 0.0 ? enter : texit;
    if (t < 0.001) return false;
    return true;
}

bool intersectRotBox(Ray r, vec3 c, vec3 hs, vec4 rot, out float t, out vec3 n) {
    mat3 m = quatToMat(rot);
    mat3 inv = transpose(m);
    vec3 localRo = inv * (r.ro - c);
    vec3 localRd = inv * r.rd;
    vec3 inv_rd = 1.0 / localRd;
    vec3 t1 = (-hs - localRo) * inv_rd;
    vec3 t2 = ( hs - localRo) * inv_rd;
    vec3 tmin = min(t1, t2);
    vec3 tmax = max(t1, t2);
    float enter = max(max(tmin.x, tmin.y), tmin.z);
    float texit = min(min(tmax.x, tmax.y), tmax.z);
    if (enter > texit || texit < 0.0) return false;
    t = enter > 0.0 ? enter : texit;
    if (t < 0.001) return false;
    vec3 lp = localRo + localRd * t;
    vec3 d = abs(lp) - hs;
    vec3 localN = d.x > d.y && d.x > d.z ? vec3(sign(lp.x), 0.0, 0.0) :
                  d.y > d.z ? vec3(0.0, sign(lp.y), 0.0) : vec3(0.0, 0.0, sign(lp.z));
    n = m * localN;
    return true;
}

// =====================================================================
// Segment Distance (for anti-aliased line rendering)
// =====================================================================

void getOrthoBasis(vec3 n, out vec3 u, out vec3 v) {
    if (abs(n.y) < 0.999) {
        u = normalize(cross(n, vec3(0.0, 1.0, 0.0)));
    } else {
        u = normalize(cross(n, vec3(1.0, 0.0, 0.0)));
    }
    v = cross(n, u);
}

void getArcQuad(vec3 cam, vec3 c, int ax, out float sa, out float sb) {
    vec3 v = normalize(cam - c);
    vec3 pa, pb;
    if (ax == 0) { pa = vec3(0.0, 1.0, 0.0); pb = vec3(0.0, 0.0, 1.0); }
    else if (ax == 1) { pa = vec3(1.0, 0.0, 0.0); pb = vec3(0.0, 0.0, 1.0); }
    else { pa = vec3(1.0, 0.0, 0.0); pb = vec3(0.0, 1.0, 0.0); }
    sa = sign(dot(v, pa));
    sb = sign(dot(v, pb));
    if (sa == 0.0) sa = 1.0;
    if (sb == 0.0) sb = 1.0;
}

float lineSegDist(vec3 ro, vec3 rd, vec3 p0, vec3 p1, out float rayT) {
    vec3 v = p1 - p0;
    vec3 w = ro - p0;
    float a2 = dot(rd, rd);
    float b2 = dot(rd, v);
    float c2 = dot(v, v);
    float d2 = dot(rd, w);
    float e2 = dot(v, w);
    float den = a2 * c2 - b2 * b2;
    float s = den < 0.0001 ? e2 / c2 : (a2 * e2 - b2 * d2) / den;
    float t = den < 0.0001 ? 0.0 : (b2 * e2 - c2 * d2) / den;
    s = clamp(s, 0.0, 1.0);
    rayT = max(t, 0.0);
    return length(ro + rd * rayT - (p0 + v * s));
}

void minGizmoSegment(vec3 ro, vec3 rd, vec3 p0, vec3 p1, int part,
                     inout float minD, inout float minT, inout int bestPart) {
    float t;
    float d = lineSegDist(ro, rd, p0, p1, t);
    if (t > 0.0 && d < minD) {
        minD = d;
        minT = t;
        bestPart = part;
    }
}

// =====================================================================
// Gizmo Geometry Intersection (cone, cylinder, disk)
// =====================================================================

bool hitCylinder(vec3 ro, vec3 rd, vec3 base, vec3 ax, float rad, float ht, out float t, out vec3 n) {
    vec3 ab = ax * ht;
    vec3 ao = ro - base;
    vec3 aoxab = cross(ao, ab);
    vec3 vxab = cross(rd, ab);
    float ab2 = dot(ab, ab);
    float a = dot(vxab, vxab);
    float b = 2.0 * dot(vxab, aoxab);
    float c = dot(aoxab, aoxab) - rad * rad * ab2;
    float det = b * b - 4.0 * a * c;
    if (det < 0.0) return false;
    t = (-b - sqrt(det)) / (2.0 * a);
    if (t < 0.001) { t = (-b + sqrt(det)) / (2.0 * a); if (t < 0.001) return false; }
    float h = dot(ro + rd * t - base, ax);
    if (h < 0.0 || h > ht) return false;
    n = normalize(ro + rd * t - base - ax * h);
    return true;
}

bool hitCone(vec3 ro, vec3 rd, vec3 apex, vec3 ax, float ang, float ht, out float t, out vec3 n) {
    vec3 co = ro - apex;
    float ca = cos(ang);
    float sa = sin(ang);
    float c2 = ca * ca;
    float s2 = sa * sa;
    float dv = dot(rd, ax);
    float cv = dot(co, ax);
    float a = dv * dv * c2 - dot(rd, rd) * s2;
    float b = 2.0 * (dv * cv * c2 - dot(rd, co) * s2);
    float c = cv * cv * c2 - dot(co, co) * s2;
    float det = b * b - 4.0 * a * c;
    if (det < 0.0) return false;
    float sq = sqrt(det);
    t = INF;
    for (int i = 0; i < 2; i++) {
        float ti = (-b + (i == 0 ? -sq : sq)) / (2.0 * a);
        if (ti < 0.001) continue;
        float h = dot(ro + rd * ti - apex, ax);
        if (h > 0.0 && h < ht && ti < t) {
            t = ti;
            vec3 on = apex + ax * h;
            n = normalize(normalize(ro + rd * t - on) * ca - ax * sa);
        }
    }
    return t < INF;
}

bool hitDisk(vec3 ro, vec3 rd, vec3 c, vec3 n, float rad, out float t) {
    float d = dot(rd, n);
    if (abs(d) < 0.0001) return false;
    t = dot(c - ro, n) / d;
    return t > 0.001 && length(ro + rd * t - c) <= rad;
}

// =====================================================================
// Gizmo Picking (analytical geometry)
// =====================================================================

int pickAxisGizmo(Ray r, vec3 c, float sc, mat3 axT, out float ht) {
    int part = PART_NONE;
    ht = INF;
    float sz = GZ_SIZE * sc;
    float cr = GZ_CENTER_RAD * sc;
    float pr = GZ_PICK_RAD * sc;

    // Center sphere (enlarged picking radius)
    vec3 oc = r.ro - c;
    float b = dot(oc, r.rd);
    float det = b * b - (dot(oc, oc) - cr * cr * 3.24);
    if (det >= 0.0) {
        float t = -b - sqrt(det);
        if (t > 0.001 && t < ht) { ht = t; part = PART_CENTER; }
    }

    // Axis lines
    for (int ax = 0; ax < 3; ax++) {
        vec3 dir = axT * axisVec(ax);
        vec3 st = c + dir * cr;
        vec3 en = c + dir * sz;
        float rt;
        float dist = lineSegDist(r.ro, r.rd, st, en, rt);
        if (dist < pr && rt < ht) { ht = rt; part = PART_X + ax; }
    }
    return part;
}

int pickGizmo(Ray r, vec3 center, float gscale, out float ht) {
    return pickAxisGizmo(r, center, gscale, mat3(1.0), ht);
}

int pickRotGizmo(Ray r, vec3 c, vec3 cam, float sc, int actPart, out float ht) {
    int part = PART_NONE;
    ht = INF;
    float rr = GZ_RING_RAD * sc;
    float pr = GZ_PICK_RAD * sc;

    for (int ax = 0; ax < 3; ax++) {
        vec3 n = axisVec(ax);
        float d = dot(r.rd, n);
        if (abs(d) < 0.0001) continue;
        float pt = dot(c - r.ro, n) / d;
        if (pt < 0.001) continue;
        vec3 hp = r.ro + r.rd * pt;
        float dist = abs(length(hp - c) - rr);
        if (dist > pr) continue;
        if (actPart == PART_X + ax) {
            if (pt < ht) { ht = pt; part = PART_X + ax; }
            continue;
        }
        vec3 th = hp - c;
        if (length(th) > 0.001) {
            th = normalize(th);
            float sa, sb;
            getArcQuad(cam, c, ax, sa, sb);
            vec3 pa, pb;
            if (ax == 0) { pa = vec3(0.0, 1.0, 0.0); pb = vec3(0.0, 0.0, 1.0); }
            else if (ax == 1) { pa = vec3(1.0, 0.0, 0.0); pb = vec3(0.0, 0.0, 1.0); }
            else { pa = vec3(1.0, 0.0, 0.0); pb = vec3(0.0, 1.0, 0.0); }
            if (dot(th, pa) * sa > -0.1 && dot(th, pb) * sb > -0.1) {
                if (pt < ht) { ht = pt; part = PART_X + ax; }
            }
        }
    }
    return part;
}

int pickGizmoFull(Ray r, vec3 center, vec3 camPos, float gscale, float transMode, int actPart, vec4 objQuat, out float ht) {
    if (transMode == TRANS_ROTATE) {
        return pickRotGizmo(r, center, camPos, gscale, actPart, ht);
    }
    mat3 axT = transMode == TRANS_SCALE ? quatToMat(objQuat) : mat3(1.0);
    return pickAxisGizmo(r, center, gscale, axT, ht);
}

// =====================================================================
// Gizmo Rendering (analytical geometry)
// =====================================================================

GizmoHit traceAxisGizmo(Ray r, vec3 c, int actPart, float sc, mat3 axT, bool useBox, float pxScalar) {
    GizmoHit g;
    g.t = INF; g.part = PART_NONE; g.alpha = 0.0; g.col = vec3(0.0);

    float sz = GZ_SIZE * sc;
    float cr = GZ_CENTER_RAD * sc;
    float cnl = GZ_CONE_LEN * sc;
    float cnr = GZ_CONE_RAD * sc;
    float th = GZ_CYL_RAD * sc;

    float t; vec3 n;

    // Center sphere
    vec3 oc = r.ro - c;
    float b = dot(oc, r.rd);
    float det = b * b - (dot(oc, oc) - cr * cr);
    if (det >= 0.0) {
        t = -b - sqrt(det);
        if (t > 0.001 && t < g.t) {
            g.t = t; g.part = PART_CENTER; g.alpha = 1.0;
            n = normalize(r.ro + r.rd * t - c);
            g.col = applyShade(actPart == PART_CENTER ? AXIS_COL_HL : AXIS_COL_CENTER, n);
        }
    }

    // Axis arrows (cone tips or box tips + base disks)
    for (int ax = 0; ax < 3; ax++) {
        vec3 dir = axT * axisVec(ax);
        vec3 col = (actPart == PART_X + ax) ? AXIS_COL_HL : axisCol(ax);

        if (useBox) {
            // Scale mode: box tips
            float cs = cnr * 0.7;
            vec3 cc = c + dir * (sz - cs);
            float bt; vec3 bn;
            if (intersectRotBox(r, cc, vec3(cs), matToQuat(axT), bt, bn) && bt < g.t) {
                g.t = bt; g.part = PART_X + ax; g.alpha = 1.0;
                g.col = applyShade(col, bn);
            }
        } else {
            // Translate mode: cone tips
            vec3 apex = c + dir * sz;
            float coneAng = atan(cnr / cnl);
            if (hitCone(r.ro, r.rd, apex, -dir, coneAng, cnl, t, n) && t < g.t) {
                g.t = t; g.part = PART_X + ax; g.alpha = 1.0;
                g.col = applyShade(col, n);
            }
            // Cone base disk
            vec3 coneBase = c + dir * (sz - cnl);
            if (hitDisk(r.ro, r.rd, coneBase, -dir, cnr, t) && t < g.t) {
                g.t = t; g.part = PART_X + ax; g.alpha = 1.0;
                g.col = applyShade(col, -dir);
            }
        }
    }

    // Anti-aliased axis lines
    float bestD = INF;
    float bestT = INF;
    int bestPart = PART_NONE;
    for (int ax = 0; ax < 3; ax++) {
        vec3 dir = axT * axisVec(ax);
        vec3 pStart = c + dir * cr;
        vec3 pEnd = c + dir * (sz - (useBox ? cnr * 1.4 : cnl));
        minGizmoSegment(r.ro, r.rd, pStart, pEnd, PART_X + ax, bestD, bestT, bestPart);
    }
    if (bestT < INF && bestT < g.t) {
        float px = bestT * pxScalar;
        if (bestD < th + px) {
            g.t = bestT;
            g.part = bestPart;
            g.alpha = smoothstep(th + px, th - px, bestD);
            vec3 col = (actPart == bestPart) ? AXIS_COL_HL : axisCol(bestPart - PART_X);
            g.col = applyShade(col, -r.rd);
        }
    }

    return g;
}

GizmoHit traceRotGizmo(Ray r, vec3 c, vec3 cam, int actPart, float sc, float pxScalar) {
    GizmoHit g;
    g.t = INF; g.part = PART_NONE; g.alpha = 0.0; g.col = vec3(0.0);
    float rr = GZ_RING_RAD * sc;
    float th = GZ_RING_TUBE * sc;
    const int SEGS = 64;
    float bestD = INF;
    float bestT = INF;
    int bestPart = PART_NONE;

    for (int ax = 0; ax < 3; ax++) {
        vec3 n = axisVec(ax);
        vec3 u, v;
        getOrthoBasis(n, u, v);
        float sa, sb;
        getArcQuad(cam, c, ax, sa, sb);
        vec3 pa, pb;
        if (ax == 0) { pa = vec3(0.0, 1.0, 0.0); pb = vec3(0.0, 0.0, 1.0); }
        else if (ax == 1) { pa = vec3(1.0, 0.0, 0.0); pb = vec3(0.0, 0.0, 1.0); }
        else { pa = vec3(1.0, 0.0, 0.0); pb = vec3(0.0, 1.0, 0.0); }
        vec3 pPrev = c + u * rr;
        int currentPart = PART_X + ax;
        bool isAct = (actPart == currentPart);
        for (int i = 1; i <= SEGS; i++) {
            float ang = float(i) * TAU / float(SEGS);
            vec3 pNext = c + (u * cos(ang) + v * sin(ang)) * rr;
            vec3 midDir = normalize((pPrev + pNext) * 0.5 - c);
            if (isAct || (dot(midDir, pa) * sa > -0.1 && dot(midDir, pb) * sb > -0.1)) {
                minGizmoSegment(r.ro, r.rd, pPrev, pNext, currentPart, bestD, bestT, bestPart);
            }
            pPrev = pNext;
        }
    }

    if (bestT < INF) {
        float px = bestT * pxScalar;
        if (bestD < th + px) {
            g.t = bestT;
            g.part = bestPart;
            g.alpha = smoothstep(th + px, th - px, bestD);
            bool isAct = (actPart == bestPart);
            vec3 baseCol = isAct ? AXIS_COL_HL : axisCol(bestPart - PART_X);
            g.col = applyShade(baseCol, -r.rd);
        }
    }
    return g;
}

GizmoHit traceGizmo(Ray r, vec3 center, vec3 camPos, int actPart, float transMode, vec4 objQuat, float pxScalar) {
    float gsc = gizmoScl(center, camPos);
    if (transMode == TRANS_ROTATE) {
        return traceRotGizmo(r, center, camPos, actPart, gsc, pxScalar);
    }
    return traceAxisGizmo(r, center, actPart, gsc,
                          transMode == TRANS_SCALE ? quatToMat(objQuat) : mat3(1.0),
                          transMode == TRANS_SCALE,
                          pxScalar);
}

// =====================================================================
// UI System — MagicaCSG Style
// =====================================================================

// ----------------------- Color scheme -----------------------

#define UI_COL_PANEL    vec3(0.14, 0.14, 0.16)
#define UI_COL_BTN      vec3(0.22, 0.22, 0.25)
#define UI_COL_BTN_SEL  vec3(0.35, 0.20, 0.20)
#define UI_COL_ACCENT   vec3(1.0, 0.42, 0.42)
#define UI_COL_ICON     vec3(0.68, 0.68, 0.72)
#define UI_COL_ICON_SEL vec3(1.0, 0.75, 0.60)
#define UI_COL_BORDER   vec3(0.08, 0.08, 0.10)
#define UI_COL_SEPARATOR vec3(0.20, 0.20, 0.22)

// ----------------------- Layout -----------------------

#define UI_TOOLBAR_W 44.0
#define UI_BTN_SIZE  32.0
#define UI_MARGIN    6.0
#define UI_GAP       4.0
#define UI_TOP_PAD   6.0
#define UI_CORNER_R  4.0
#define UI_ICON_LINE 1.4

// ----------------------- Actions -----------------------

#define ACT_NONE      -1
#define ACT_TRANSLATE  0
#define ACT_ROTATE     1
#define ACT_SCALE      2
#define ACT_FOCUS      3
#define ACT_COUNT      4

// ----------------------- 2D SDF primitives -----------------------

float sdBox2D(vec2 p, vec2 b) {
    vec2 d = abs(p) - b;
    return length(max(d, vec2(0.0))) + min(max(d.x, d.y), 0.0);
}

float sdRoundBox2D(vec2 p, vec2 b, float r) {
    return sdBox2D(p, b - vec2(r)) - r;
}

float sdSegment2D(vec2 p, vec2 a, vec2 b) {
    vec2 pa = p - a, ba = b - a;
    float h = clamp(dot(pa, ba) / dot(ba, ba), 0.0, 1.0);
    return length(pa - ba * h);
}

// ----------------------- Toolbar layout -----------------------

float toolbarHeight() {
    return UI_TOP_PAD + UI_MARGIN * 2.0
         + float(ACT_COUNT) * UI_BTN_SIZE
         + float(ACT_COUNT - 1) * UI_GAP;
}

vec2 getToolBtnPos(int idx, vec2 res) {
    return vec2(
        UI_MARGIN + UI_BTN_SIZE * 0.5,
        res.y - UI_TOP_PAD - UI_MARGIN - UI_BTN_SIZE * 0.5
              - float(idx) * (UI_BTN_SIZE + UI_GAP)
    );
}

int checkToolbarClick(vec2 mp, vec2 res) {
    for (int i = 0; i < ACT_COUNT; i++) {
        vec2 pos = getToolBtnPos(i, res);
        vec2 d = abs(mp - pos) - vec2(UI_BTN_SIZE * 0.5);
        if (max(d.x, d.y) < 0.0) return i;
    }
    return ACT_NONE;
}

float getToolbarAlpha(vec2 fc, vec2 res) {
    float tbH = toolbarHeight();
    vec2 center = vec2(UI_TOOLBAR_W * 0.5,
                       res.y - tbH * 0.5 - UI_TOP_PAD * 0.5);
    float d = sdRoundBox2D(fc - center,
                           vec2(UI_TOOLBAR_W * 0.5, tbH * 0.5 + UI_TOP_PAD * 0.5),
                           UI_CORNER_R);
    return d < 0.0 ? 1.0 : 0.0;
}

// ----------------------- SDF vector icons -----------------------

float iconTranslateSDF(vec2 p) {
    float d = 1e5;
    float arm = 10.0;
    float tip = 3.5;
    // Cross
    d = min(d, sdSegment2D(p, vec2(-arm, 0.0), vec2(arm, 0.0)));
    d = min(d, sdSegment2D(p, vec2(0.0, -arm), vec2(0.0, arm)));
    // Right arrow
    d = min(d, sdSegment2D(p, vec2(arm, 0.0), vec2(arm - tip, tip)));
    d = min(d, sdSegment2D(p, vec2(arm, 0.0), vec2(arm - tip, -tip)));
    // Left arrow
    d = min(d, sdSegment2D(p, vec2(-arm, 0.0), vec2(-arm + tip, tip)));
    d = min(d, sdSegment2D(p, vec2(-arm, 0.0), vec2(-arm + tip, -tip)));
    // Up arrow
    d = min(d, sdSegment2D(p, vec2(0.0, arm), vec2(-tip, arm - tip)));
    d = min(d, sdSegment2D(p, vec2(0.0, arm), vec2(tip, arm - tip)));
    // Down arrow
    d = min(d, sdSegment2D(p, vec2(0.0, -arm), vec2(-tip, -arm + tip)));
    d = min(d, sdSegment2D(p, vec2(0.0, -arm), vec2(tip, -arm + tip)));
    return d;
}

float iconRotateSDF(vec2 p) {
    float d = 1e5;
    float r = 9.0;
    // Arc: draw ring but clip a gap near angle 0
    float ringD = abs(length(p) - r);
    float a = atan(p.y, p.x); // [-PI, PI]
    float gap = 0.5;
    if (abs(a) < gap) ringD = 1e5;
    d = min(d, ringD);
    // Arrow at upper gap end (angle = gap, arc arriving counterclockwise)
    vec2 ep = r * vec2(cos(gap), sin(gap));
    vec2 tang = vec2(-sin(gap), cos(gap));
    vec2 norm = normalize(ep);
    d = min(d, sdSegment2D(p, ep, ep + tang * 4.0 + norm * 2.8));
    d = min(d, sdSegment2D(p, ep, ep + tang * 4.0 - norm * 2.8));
    return d;
}

float iconScaleSDF(vec2 p) {
    float d = 1e5;
    // Diagonal line
    d = min(d, sdSegment2D(p, vec2(-5.0, -5.0), vec2(5.0, 5.0)));
    // Small square at bottom-left (outline)
    d = min(d, abs(sdBox2D(p - vec2(-7.0, -7.0), vec2(2.5))));
    // Larger square at top-right (outline)
    d = min(d, abs(sdBox2D(p - vec2(6.5, 6.5), vec2(4.0))));
    return d;
}

float iconFocusSDF(vec2 p) {
    float d = 1e5;
    float r = 7.5;
    float cgap = 3.0;
    // Circle
    d = min(d, abs(length(p) - r));
    // Crosshair with center gap
    d = min(d, sdSegment2D(p, vec2(-r - 3.0, 0.0), vec2(-cgap, 0.0)));
    d = min(d, sdSegment2D(p, vec2(cgap, 0.0), vec2(r + 3.0, 0.0)));
    d = min(d, sdSegment2D(p, vec2(0.0, -r - 3.0), vec2(0.0, -cgap)));
    d = min(d, sdSegment2D(p, vec2(0.0, cgap), vec2(0.0, r + 3.0)));
    return d;
}

float getIconSDF(int action, vec2 p) {
    if (action == ACT_TRANSLATE) return iconTranslateSDF(p);
    if (action == ACT_ROTATE)   return iconRotateSDF(p);
    if (action == ACT_SCALE)    return iconScaleSDF(p);
    if (action == ACT_FOCUS)    return iconFocusSDF(p);
    return 1e5;
}

// ----------------------- Button rendering -----------------------

vec4 drawToolBtn(vec2 fc, int idx, float transMode, bool hasSel, vec2 res) {
    vec2 pos = getToolBtnPos(idx, res);
    vec2 lp = fc - pos;
    float hs = UI_BTN_SIZE * 0.5;

    if (abs(lp.x) > hs + 1.0 || abs(lp.y) > hs + 1.0) return vec4(0.0);

    // Selection state
    bool sel = false;
    if (hasSel) {
        if (idx == ACT_TRANSLATE) sel = (transMode == TRANS_TRANSLATE);
        if (idx == ACT_ROTATE)    sel = (transMode == TRANS_ROTATE);
        if (idx == ACT_SCALE)     sel = (transMode == TRANS_SCALE);
    }

    vec4 result = vec4(0.0);

    // Rounded rect background
    float btnD = sdRoundBox2D(lp, vec2(hs), UI_CORNER_R);
    if (btnD < 0.0) {
        vec3 bg = sel ? UI_COL_BTN_SEL : UI_COL_BTN;
        bg += vec3(0.015) * (lp.y / hs); // subtle top-bright gradient
        result = vec4(bg, 1.0);
    }

    // Anti-aliased border
    float bAlpha = 1.0 - smoothstep(-1.2, 0.0, btnD);
    if (bAlpha > 0.0 && btnD > -2.0) {
        vec3 bc = sel ? UI_COL_ACCENT * 0.55 : UI_COL_BORDER;
        result = vec4(mix(result.rgb, bc, bAlpha), max(result.a, bAlpha));
    }

    // Icon
    float iconD = getIconSDF(idx, lp);
    float iconA = 1.0 - smoothstep(UI_ICON_LINE - 0.6, UI_ICON_LINE + 0.6, iconD);
    if (iconA > 0.005) {
        vec3 ic = sel ? UI_COL_ACCENT : UI_COL_ICON;
        result.rgb = mix(result.rgb, ic, iconA);
        result.a = max(result.a, iconA);
    }

    // Left accent bar for selected state
    if (sel) {
        float barD = sdRoundBox2D(lp - vec2(-hs + 1.5, 0.0), vec2(1.5, hs * 0.55), 1.0);
        float barA = 1.0 - smoothstep(-0.5, 0.5, barD);
        result.rgb = mix(result.rgb, UI_COL_ACCENT, barA);
    }

    return result;
}

// ----------------------- Full toolbar rendering -----------------------

vec4 renderToolbar(vec2 fc, vec2 res, float transMode, bool hasSel) {
    float tbH = toolbarHeight();
    vec2 center = vec2(UI_TOOLBAR_W * 0.5,
                       res.y - tbH * 0.5 - UI_TOP_PAD * 0.5);
    float panelD = sdRoundBox2D(fc - center,
                                vec2(UI_TOOLBAR_W * 0.5, tbH * 0.5 + UI_TOP_PAD * 0.5),
                                UI_CORNER_R);

    vec4 result = vec4(0.0);

    // Panel background
    if (panelD < 0.0) {
        float edge = 1.0 - smoothstep(-2.0, 0.0, panelD);
        vec3 bg = mix(UI_COL_PANEL, UI_COL_BORDER, edge * 0.6);
        result = vec4(bg, 0.92);
    }

    // Buttons
    for (int i = 0; i < ACT_COUNT; i++) {
        vec4 btn = drawToolBtn(fc, i, transMode, hasSel, res);
        if (btn.a > 0.005) {
            result.rgb = mix(result.rgb, btn.rgb, btn.a);
            result.a = max(result.a, btn.a);
        }
    }

    return result;
}

// =====================================================================
// Axis Indicator (bottom-left corner)
// =====================================================================

#define AXIS_IND_CENTER vec2(50.0, 50.0)
#define AXIS_IND_SIZE   30.0
#define AXIS_IND_LINE_W 2.0
#define AXIS_IND_LABEL_OFF 8.0

float lineDist2D(vec2 p, vec2 a, vec2 b) {
    vec2 pa = p - a, ba = b - a;
    return length(pa - ba * clamp(dot(pa, ba) / dot(ba, ba), 0.0, 1.0));
}

float drawAxisChar(vec2 p, int c) {
    if (p.x < 0.0 || p.y < 0.0 || p.x > 5.0 || p.y > 5.0) return 0.0;
    int x = int(p.x);
    int y = int(p.y);
    if (x > 4 || y > 4) return 0.0;
    // X=0, Y=1, Z=2 bitmap patterns (5x5 each)
    int pat[15];
    pat[0]=0x11; pat[1]=0x0A; pat[2]=0x04; pat[3]=0x0A; pat[4]=0x11;
    pat[5]=0x11; pat[6]=0x0A; pat[7]=0x04; pat[8]=0x04; pat[9]=0x04;
    pat[10]=0x1F; pat[11]=0x02; pat[12]=0x04; pat[13]=0x08; pat[14]=0x1F;
    return ((pat[c * 5 + (4 - y)] >> (4 - x)) & 1) == 1 ? 1.0 : 0.0;
}

vec4 renderAxisIndicator(vec2 fc, vec3 camDir) {
    if (length(camDir) < 0.001) camDir = vec3(0.0, 0.0, 1.0);
    vec3 fwd = normalize(camDir);
    vec3 rt = normalize(cross(fwd, vec3(0.0, 1.0, 0.0)));
    if (length(cross(fwd, vec3(0.0, 1.0, 0.0))) < 0.001) rt = vec3(1.0, 0.0, 0.0);
    vec3 up = cross(rt, fwd);

    vec2 sp[3];
    float dp[3];
    for (int i = 0; i < 3; i++) {
        vec3 ax = axisVec(i);
        sp[i] = vec2(dot(ax, rt), dot(ax, up));
        dp[i] = dot(ax, fwd);
    }
    // Sort back-to-front
    int ord[3];
    ord[0] = 0; ord[1] = 1; ord[2] = 2;
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2 - i; j++) {
            if (dp[ord[j]] > dp[ord[j + 1]]) {
                int t = ord[j]; ord[j] = ord[j + 1]; ord[j + 1] = t;
            }
        }
    }

    vec4 r = vec4(0.0);
    // Subtle background disc
    float bgD = length(fc - AXIS_IND_CENTER) - AXIS_IND_SIZE - 8.0;
    if (bgD < 0.0) {
        r = vec4(UI_COL_PANEL, 0.45 * smoothstep(0.0, -5.0, bgD));
    }
    // Axis lines and labels
    for (int k = 0; k < 3; k++) {
        int i = ord[k];
        vec2 ep = AXIS_IND_CENTER + sp[i] * AXIS_IND_SIZE;
        float d = lineDist2D(fc, AXIS_IND_CENTER, ep);
        if (d < AXIS_IND_LINE_W) {
            r = vec4(axisCol(i), smoothstep(AXIS_IND_LINE_W, AXIS_IND_LINE_W * 0.5, d));
        }
        if (length(sp[i]) > 0.001) {
            vec2 lc = ep + normalize(sp[i]) * AXIS_IND_LABEL_OFF;
            if (drawAxisChar(fc - lc + vec2(2.0), i) > 0.0) {
                r = vec4(axisCol(i), 1.0);
            }
        }
    }
    return r;
}

// =====================================================================
// Number Rendering (bitmap digit font)
// =====================================================================

float sampleDigit(int n, vec2 uv) {
    if (uv.x < 0.0 || uv.y < 0.0 || uv.x >= 1.0 || uv.y >= 1.0) return 0.0;
    int font[10];
    font[0]=0x75557; font[1]=0x22222; font[2]=0x74717; font[3]=0x74747; font[4]=0x11574;
    font[5]=0x71747; font[6]=0x71757; font[7]=0x74444; font[8]=0x75757; font[9]=0x75747;
    ivec2 ip = ivec2(floor(uv * vec2(4.0, 5.0)));
    return float((font[n] >> (ip.x + ip.y * 4)) & 1);
}

// Render a signed float value. Returns alpha for digit pixels.
float printFloat(vec2 p, float val, float charW, int decimals) {
    bool neg = val < 0.0;
    float av = abs(val);
    int intPart = int(floor(av));
    int numIntDigits = 1;
    { int tmp = intPart; for (int i = 0; i < 6; i++) { tmp /= 10; if (tmp > 0) numIntDigits++; } }
    int totalChars = (neg ? 1 : 0) + numIntDigits + 1 + decimals;
    float cx = p.x / charW;
    float cy = p.y / (charW * 1.4);
    if (cy < 0.0 || cy >= 1.0 || cx < 0.0 || cx >= float(totalChars)) return 0.0;
    int ci = int(floor(cx));
    vec2 cuv = vec2(fract(cx), cy);
    int pos = 0;
    if (neg) {
        if (ci == 0) {
            if (cuv.y > 0.35 && cuv.y < 0.55 && cuv.x > 0.1 && cuv.x < 0.8) return 1.0;
            return 0.0;
        }
        pos = 1;
    }
    if (ci >= pos && ci < pos + numIntDigits) {
        int did = ci - pos;
        int dv = intPart;
        for (int i = 0; i < numIntDigits - 1 - did; i++) dv /= 10;
        dv = dv - (dv / 10) * 10;
        return sampleDigit(dv, cuv);
    }
    pos += numIntDigits;
    if (ci == pos) {
        if (cuv.x > 0.3 && cuv.x < 0.7 && cuv.y < 0.2) return 1.0;
        return 0.0;
    }
    pos++;
    if (ci >= pos && ci < pos + decimals) {
        int did = ci - pos;
        float frac = av - floor(av);
        for (int i = 0; i <= did; i++) frac *= 10.0;
        int dv = int(floor(frac)) - int(floor(frac / 10.0)) * 10;
        return sampleDigit(dv, cuv);
    }
    return 0.0;
}

// =====================================================================
// Bitmap Label Rendering
// =====================================================================

#define LBL_POS  0
#define LBL_SIZ  1
#define LBL_EXP  2
#define LBL_SUN  3
#define LBL_RGH  4
#define LBL_MET  5
#define LBL_X    6
#define LBL_Y    7
#define LBL_Z    8
#define LBL_COL  9

// 4x5 bitmap font for needed characters
float charBitmap(int ch, vec2 uv) {
    if (uv.x < 0.0 || uv.y < 0.0 || uv.x >= 1.0 || uv.y >= 1.0) return 0.0;
    int x = int(uv.x * 4.0);
    int y = int(uv.y * 5.0);
    if (x > 3 || y > 4) return 0.0;
    int pat = 0;
    // P=0 o=1 s=2 S=3 i=4 z=5 E=6 x=7 p=8 R=9 g=10 h=11 M=12 e=13 t=14 C=15 l=16 u=17 n=18
    if(ch== 0){int r[5];r[0]=0xE;r[1]=0x9;r[2]=0xE;r[3]=0x8;r[4]=0x8;pat=r[4-y];}
    if(ch== 1){int r[5];r[0]=0x0;r[1]=0x6;r[2]=0x9;r[3]=0x9;r[4]=0x6;pat=r[4-y];}
    if(ch== 2){int r[5];r[0]=0x0;r[1]=0x7;r[2]=0x8;r[3]=0x1;r[4]=0xE;pat=r[4-y];}
    if(ch== 3){int r[5];r[0]=0x7;r[1]=0x8;r[2]=0x6;r[3]=0x1;r[4]=0xE;pat=r[4-y];}
    if(ch== 4){int r[5];r[0]=0x4;r[1]=0x0;r[2]=0x4;r[3]=0x4;r[4]=0x4;pat=r[4-y];}
    if(ch== 5){int r[5];r[0]=0x0;r[1]=0xF;r[2]=0x2;r[3]=0x4;r[4]=0xF;pat=r[4-y];}
    if(ch== 6){int r[5];r[0]=0xF;r[1]=0x8;r[2]=0xE;r[3]=0x8;r[4]=0xF;pat=r[4-y];}
    if(ch== 7){int r[5];r[0]=0x0;r[1]=0x9;r[2]=0x6;r[3]=0x6;r[4]=0x9;pat=r[4-y];}
    if(ch== 8){int r[5];r[0]=0x0;r[1]=0xE;r[2]=0x9;r[3]=0xE;r[4]=0x8;pat=r[4-y];}
    if(ch== 9){int r[5];r[0]=0xE;r[1]=0x9;r[2]=0xE;r[3]=0xA;r[4]=0x9;pat=r[4-y];}
    if(ch==10){int r[5];r[0]=0x0;r[1]=0x7;r[2]=0x9;r[3]=0x7;r[4]=0x1;pat=r[4-y];}
    if(ch==11){int r[5];r[0]=0x8;r[1]=0x8;r[2]=0xE;r[3]=0x9;r[4]=0x9;pat=r[4-y];}
    if(ch==12){int r[5];r[0]=0x9;r[1]=0xF;r[2]=0xF;r[3]=0x9;r[4]=0x9;pat=r[4-y];}
    if(ch==13){int r[5];r[0]=0x0;r[1]=0x6;r[2]=0x9;r[3]=0xF;r[4]=0x6;pat=r[4-y];}
    if(ch==14){int r[5];r[0]=0x4;r[1]=0xE;r[2]=0x4;r[3]=0x4;r[4]=0x3;pat=r[4-y];}
    if(ch==15){int r[5];r[0]=0x7;r[1]=0x8;r[2]=0x8;r[3]=0x8;r[4]=0x7;pat=r[4-y];}
    if(ch==16){int r[5];r[0]=0x4;r[1]=0x4;r[2]=0x4;r[3]=0x4;r[4]=0x6;pat=r[4-y];}
    if(ch==17){int r[5];r[0]=0x0;r[1]=0x9;r[2]=0x9;r[3]=0x9;r[4]=0x7;pat=r[4-y];}
    if(ch==18){int r[5];r[0]=0x0;r[1]=0xE;r[2]=0x9;r[3]=0x9;r[4]=0x9;pat=r[4-y];}
    return float((pat >> (3 - x)) & 1);
}

float drawLabel(vec2 p, float charW, int label) {
    int c0=-1,c1=-1,c2=-1;
    if(label==LBL_POS){c0=0;c1=1;c2=2;}
    if(label==LBL_SIZ){c0=3;c1=4;c2=5;}
    if(label==LBL_EXP){c0=6;c1=7;c2=8;}
    if(label==LBL_SUN){c0=3;c1=17;c2=18;}
    if(label==LBL_RGH){c0=9;c1=10;c2=11;}
    if(label==LBL_MET){c0=12;c1=13;c2=14;}
    if(label==LBL_X){c0=7;c1=-1;c2=-1;}
    if(label==LBL_Y){return drawAxisChar(p/charW,1);}
    if(label==LBL_Z){return drawAxisChar(p/charW,2);}
    if(label==LBL_COL){c0=15;c1=1;c2=16;}
    float a=0.0;
    float cw=charW, ch=charW*1.25;
    if(c0>=0) a=max(a,charBitmap(c0,vec2(p.x/cw,p.y/ch)));
    if(c1>=0) a=max(a,charBitmap(c1,vec2((p.x-cw*1.1)/cw,p.y/ch)));
    if(c2>=0) a=max(a,charBitmap(c2,vec2((p.x-cw*2.2)/cw,p.y/ch)));
    return a;
}

// =====================================================================
// Property Panel (right side)
// =====================================================================

#define PANEL_W        200.0
#define PANEL_MARGIN   8.0
#define PANEL_GAP      3.0
#define PANEL_ROW_H    18.0
#define PANEL_LABEL_W  32.0
#define PANEL_CHAR_W   5.0
#define PANEL_SECTION_H 22.0
#define PANEL_CORNER_R 4.0

#define SLIDER_NONE     -1
#define SLIDER_EXPOSURE  0
#define SLIDER_SUN       1
#define SLIDER_ROUGHNESS 2
#define SLIDER_METALNESS 3

vec2 panelOrigin(vec2 res) {
    return vec2(res.x - PANEL_W - PANEL_MARGIN, res.y - PANEL_MARGIN);
}

float panelHeight(bool hasSel) {
    float h = PANEL_SECTION_H + 2.0 * PANEL_ROW_H + PANEL_GAP * 3.0;
    if (hasSel) {
        h += PANEL_SECTION_H + 3.0 * (PANEL_ROW_H + PANEL_GAP);
        h += PANEL_SECTION_H + 3.0 * (PANEL_ROW_H + PANEL_GAP);
        h += PANEL_SECTION_H + 2.0 * PANEL_ROW_H + PANEL_GAP * 3.0;
    }
    return h;
}

bool insidePanel(vec2 fc, vec2 res, bool hasSel) {
    vec2 org = panelOrigin(res);
    float ph = panelHeight(hasSel);
    return fc.x >= org.x && fc.x <= org.x + PANEL_W &&
           fc.y <= org.y && fc.y >= org.y - ph;
}

// ---- Slider ----

struct SliderRect {
    vec2 pos; vec2 size;
    float minV, maxV, val;
    int id;
};

SliderRect getSliderRect(int sid, vec2 org, float yOff, float val, float mn, float mx) {
    SliderRect s;
    float sx = org.x + PANEL_LABEL_W + PANEL_MARGIN + 2.0;
    float sw = PANEL_W - PANEL_LABEL_W - PANEL_MARGIN * 3.0 - 2.0;
    s.pos = vec2(sx + sw * 0.5, org.y - yOff);
    s.size = vec2(sw * 0.5, PANEL_ROW_H * 0.5 - 2.0);
    s.minV = mn; s.maxV = mx; s.val = val; s.id = sid;
    return s;
}

int checkSliderClick(vec2 mp, SliderRect s) {
    vec2 d = abs(mp - s.pos) - s.size;
    return max(d.x, d.y) < 0.0 ? s.id : SLIDER_NONE;
}

float sliderDragValue(vec2 mp, SliderRect s) {
    float t = clamp((mp.x - (s.pos.x - s.size.x)) / (s.size.x * 2.0), 0.0, 1.0);
    return s.minV + t * (s.maxV - s.minV);
}

vec4 drawSlider(vec2 fc, SliderRect s) {
    vec2 lp = fc - s.pos;
    if (abs(lp.x) > s.size.x + 1.0 || abs(lp.y) > s.size.y + 1.0) return vec4(0.0);
    float d = sdRoundBox2D(lp, s.size, 3.0);
    if (d > 0.0) return vec4(0.0);
    vec4 result = vec4(vec3(0.12, 0.12, 0.14), 1.0);
    float t = clamp((s.val - s.minV) / (s.maxV - s.minV), 0.0, 1.0);
    float fillX = -s.size.x + t * s.size.x * 2.0;
    if (lp.x < fillX) result.rgb = UI_COL_ACCENT * 0.65;
    float thumbD = abs(lp.x - fillX);
    if (thumbD < 3.0 && abs(lp.y) < s.size.y) {
        result.rgb = mix(result.rgb, vec3(0.95), smoothstep(3.0, 1.5, thumbD));
    }
    float edge = 1.0 - smoothstep(-1.5, 0.0, d);
    result.rgb = mix(result.rgb, UI_COL_BORDER, edge * 0.5);
    return result;
}

vec4 drawSectionHeader(vec2 fc, vec2 org, float yOff, int label) {
    vec2 lp = fc - vec2(org.x + PANEL_MARGIN, org.y - yOff);
    if (lp.x < 0.0 || lp.x > PANEL_W - PANEL_MARGIN * 2.0 ||
        lp.y > 0.0 || lp.y < -PANEL_SECTION_H) return vec4(0.0);
    vec4 result = vec4(0.0);
    if (lp.y > -2.0) result = vec4(UI_COL_SEPARATOR, 0.8);
    float la = drawLabel(vec2(lp.x, lp.y + PANEL_SECTION_H - 5.0), PANEL_CHAR_W, label);
    if (la > 0.0) result = vec4(UI_COL_ICON * 1.2, 1.0);
    return result;
}

vec4 drawValueRow(vec2 fc, vec2 org, float yOff, int label, float val) {
    vec2 lp = fc - vec2(org.x + PANEL_MARGIN, org.y - yOff);
    if (lp.x < 0.0 || lp.x > PANEL_W - PANEL_MARGIN * 2.0 ||
        lp.y > 0.0 || lp.y < -PANEL_ROW_H) return vec4(0.0);
    vec4 result = vec4(0.0);
    float la = drawLabel(vec2(lp.x, lp.y + PANEL_ROW_H - 5.0), PANEL_CHAR_W, label);
    if (la > 0.0) {
        vec3 lc = UI_COL_ICON;
        if(label==LBL_X) lc=AXIS_COLS[0];
        if(label==LBL_Y) lc=AXIS_COLS[1];
        if(label==LBL_Z) lc=AXIS_COLS[2];
        result = vec4(lc, 1.0);
    }
    float na = printFloat(vec2(lp.x - PANEL_LABEL_W, lp.y + PANEL_ROW_H - 5.0), val, PANEL_CHAR_W, 2);
    if (na > 0.0) result = vec4(vec3(0.82, 0.82, 0.84), 1.0);
    return result;
}

vec4 drawSliderRow(vec2 fc, vec2 org, float yOff, int label, SliderRect s) {
    vec4 result = vec4(0.0);
    vec2 lp = fc - vec2(org.x + PANEL_MARGIN, org.y - yOff);
    if (lp.x >= 0.0 && lp.x < PANEL_LABEL_W && lp.y <= 0.0 && lp.y >= -PANEL_ROW_H) {
        float la = drawLabel(vec2(lp.x, lp.y + PANEL_ROW_H - 5.0), PANEL_CHAR_W, label);
        if (la > 0.0) result = vec4(UI_COL_ICON, 1.0);
    }
    vec4 sl = drawSlider(fc, s);
    if (sl.a > result.a) result = sl;
    float numP = printFloat(vec2(fc.x - (s.pos.x + s.size.x + 4.0), fc.y - (s.pos.y - 3.5)), s.val, PANEL_CHAR_W - 1.0, 2);
    if (numP > 0.0) result = vec4(vec3(0.72), 1.0);
    return result;
}

// ---- Full panel ----

vec4 renderPropertyPanel(vec2 fc, vec2 res, int selId, float transMode,
                         vec3 objPos, vec3 objSiz, float exposure, float sunAngle,
                         float roughness, float metalness) {
    bool hasSel = (selId >= 0);
    vec2 org = panelOrigin(res);
    float ph = panelHeight(hasSel);
    if (fc.x < org.x - 2.0 || fc.x > org.x + PANEL_W + 2.0 ||
        fc.y > org.y + 2.0 || fc.y < org.y - ph - 2.0) return vec4(0.0);

    vec2 pc = vec2(org.x + PANEL_W * 0.5, org.y - ph * 0.5);
    float bgD = sdRoundBox2D(fc - pc, vec2(PANEL_W * 0.5, ph * 0.5), PANEL_CORNER_R);
    vec4 result = vec4(0.0);
    if (bgD < 0.0) {
        float edge = 1.0 - smoothstep(-2.0, 0.0, bgD);
        result = vec4(mix(UI_COL_PANEL, UI_COL_BORDER, edge * 0.5), 0.92);
    } else return vec4(0.0);

    float y = PANEL_GAP;

    // Scene section
    vec4 hdr = drawSectionHeader(fc, org, y, LBL_EXP);
    if(hdr.a>0.005){result.rgb=mix(result.rgb,hdr.rgb,hdr.a);result.a=max(result.a,hdr.a);}
    y += PANEL_SECTION_H;

    SliderRect sExp = getSliderRect(SLIDER_EXPOSURE, org, y + PANEL_ROW_H*0.5, exposure, 0.1, 5.0);
    vec4 sr = drawSliderRow(fc, org, y, LBL_EXP, sExp);
    if(sr.a>0.005){result.rgb=mix(result.rgb,sr.rgb,sr.a);result.a=max(result.a,sr.a);}
    y += PANEL_ROW_H + PANEL_GAP;

    SliderRect sSun = getSliderRect(SLIDER_SUN, org, y + PANEL_ROW_H*0.5, sunAngle, 0.0, 6.283);
    sr = drawSliderRow(fc, org, y, LBL_SUN, sSun);
    if(sr.a>0.005){result.rgb=mix(result.rgb,sr.rgb,sr.a);result.a=max(result.a,sr.a);}
    y += PANEL_ROW_H + PANEL_GAP;

    if (hasSel) {
        // Position
        hdr = drawSectionHeader(fc, org, y, LBL_POS);
        if(hdr.a>0.005){result.rgb=mix(result.rgb,hdr.rgb,hdr.a);result.a=max(result.a,hdr.a);}
        y += PANEL_SECTION_H;
        for (int i = 0; i < 3; i++) {
            float v=(i==0)?objPos.x:(i==1?objPos.y:objPos.z);
            vec4 vr = drawValueRow(fc, org, y, LBL_X+i, v);
            if(vr.a>0.005){result.rgb=mix(result.rgb,vr.rgb,vr.a);result.a=max(result.a,vr.a);}
            y += PANEL_ROW_H + PANEL_GAP;
        }
        // Size
        hdr = drawSectionHeader(fc, org, y, LBL_SIZ);
        if(hdr.a>0.005){result.rgb=mix(result.rgb,hdr.rgb,hdr.a);result.a=max(result.a,hdr.a);}
        y += PANEL_SECTION_H;
        for (int i = 0; i < 3; i++) {
            float v=(i==0)?objSiz.x:(i==1?objSiz.y:objSiz.z);
            vec4 vr = drawValueRow(fc, org, y, LBL_X+i, v);
            if(vr.a>0.005){result.rgb=mix(result.rgb,vr.rgb,vr.a);result.a=max(result.a,vr.a);}
            y += PANEL_ROW_H + PANEL_GAP;
        }
        // Material
        hdr = drawSectionHeader(fc, org, y, LBL_COL);
        if(hdr.a>0.005){result.rgb=mix(result.rgb,hdr.rgb,hdr.a);result.a=max(result.a,hdr.a);}
        y += PANEL_SECTION_H;
        SliderRect sRgh = getSliderRect(SLIDER_ROUGHNESS, org, y+PANEL_ROW_H*0.5, roughness, 0.05, 1.0);
        sr = drawSliderRow(fc, org, y, LBL_RGH, sRgh);
        if(sr.a>0.005){result.rgb=mix(result.rgb,sr.rgb,sr.a);result.a=max(result.a,sr.a);}
        y += PANEL_ROW_H + PANEL_GAP;
        SliderRect sMet = getSliderRect(SLIDER_METALNESS, org, y+PANEL_ROW_H*0.5, metalness, 0.0, 1.0);
        sr = drawSliderRow(fc, org, y, LBL_MET, sMet);
        if(sr.a>0.005){result.rgb=mix(result.rgb,sr.rgb,sr.a);result.a=max(result.a,sr.a);}
    }
    return result;
}

int checkPanelSliderClick(vec2 mp, vec2 res, int selId,
                          float exposure, float sunAngle, float roughness, float metalness) {
    bool hasSel = (selId >= 0);
    vec2 org = panelOrigin(res);
    float y = PANEL_GAP + PANEL_SECTION_H;
    SliderRect sExp = getSliderRect(SLIDER_EXPOSURE, org, y+PANEL_ROW_H*0.5, exposure, 0.1, 5.0);
    if(checkSliderClick(mp,sExp)!=SLIDER_NONE) return SLIDER_EXPOSURE;
    y += PANEL_ROW_H + PANEL_GAP;
    SliderRect sSun = getSliderRect(SLIDER_SUN, org, y+PANEL_ROW_H*0.5, sunAngle, 0.0, 6.283);
    if(checkSliderClick(mp,sSun)!=SLIDER_NONE) return SLIDER_SUN;
    y += PANEL_ROW_H + PANEL_GAP;
    if (hasSel) {
        y += PANEL_SECTION_H + 3.0*(PANEL_ROW_H+PANEL_GAP);
        y += PANEL_SECTION_H + 3.0*(PANEL_ROW_H+PANEL_GAP);
        y += PANEL_SECTION_H;
        SliderRect sRgh = getSliderRect(SLIDER_ROUGHNESS, org, y+PANEL_ROW_H*0.5, roughness, 0.05, 1.0);
        if(checkSliderClick(mp,sRgh)!=SLIDER_NONE) return SLIDER_ROUGHNESS;
        y += PANEL_ROW_H + PANEL_GAP;
        SliderRect sMet = getSliderRect(SLIDER_METALNESS, org, y+PANEL_ROW_H*0.5, metalness, 0.0, 1.0);
        if(checkSliderClick(mp,sMet)!=SLIDER_NONE) return SLIDER_METALNESS;
    }
    return SLIDER_NONE;
}

SliderRect getSliderById(int sid, vec2 res, int selId,
                         float exposure, float sunAngle, float roughness, float metalness) {
    vec2 org = panelOrigin(res);
    float y = PANEL_GAP + PANEL_SECTION_H;
    if(sid==SLIDER_EXPOSURE) return getSliderRect(sid,org,y+PANEL_ROW_H*0.5,exposure,0.1,5.0);
    y += PANEL_ROW_H + PANEL_GAP;
    if(sid==SLIDER_SUN) return getSliderRect(sid,org,y+PANEL_ROW_H*0.5,sunAngle,0.0,6.283);
    if(selId>=0) {
        y += PANEL_ROW_H + PANEL_GAP;
        y += PANEL_SECTION_H + 3.0*(PANEL_ROW_H+PANEL_GAP);
        y += PANEL_SECTION_H + 3.0*(PANEL_ROW_H+PANEL_GAP);
        y += PANEL_SECTION_H;
        if(sid==SLIDER_ROUGHNESS) return getSliderRect(sid,org,y+PANEL_ROW_H*0.5,roughness,0.05,1.0);
        y += PANEL_ROW_H + PANEL_GAP;
        if(sid==SLIDER_METALNESS) return getSliderRect(sid,org,y+PANEL_ROW_H*0.5,metalness,0.0,1.0);
    }
    return getSliderRect(-1, org, 0.0, 0.0, 0.0, 1.0);
}

// =====================================================================
// HSV Color Picker (MagicaCSG style)
// =====================================================================

#define PICKER_SV_SIZE 120.0
#define PICKER_H_WIDTH 16.0
#define PICKER_GAP_C 6.0
#define PICKER_MARGIN_C 8.0

vec2 pickerOrigin(vec2 res) {
    float tbH = toolbarHeight();
    return vec2(PICKER_MARGIN_C, res.y - tbH - UI_TOP_PAD - PICKER_MARGIN_C - PICKER_SV_SIZE);
}

int checkPickerClick(vec2 mp, vec2 res) {
    vec2 org = pickerOrigin(res);
    vec2 p = mp - org;
    if(p.x>=0.0&&p.x<=PICKER_SV_SIZE&&p.y>=0.0&&p.y<=PICKER_SV_SIZE) return 1;
    float hst = PICKER_SV_SIZE + PICKER_GAP_C;
    if(p.x>=hst&&p.x<=hst+PICKER_H_WIDTH&&p.y>=0.0&&p.y<=PICKER_SV_SIZE) return 2;
    return 0;
}

vec3 getPickerHSV(vec2 mp, vec2 res, vec3 cur, float mode) {
    vec2 org = pickerOrigin(res);
    vec2 p = mp - org;
    vec3 hsv = cur;
    if(mode==MODE_PICKER_SV){hsv.y=clamp(p.x/PICKER_SV_SIZE,0.0,1.0);hsv.z=clamp(p.y/PICKER_SV_SIZE,0.0,1.0);}
    else if(mode==MODE_PICKER_H){hsv.x=clamp(p.y/PICKER_SV_SIZE,0.0,1.0);}
    return hsv;
}

bool insidePicker(vec2 fc, vec2 res) {
    vec2 org = pickerOrigin(res);
    float tw = PICKER_SV_SIZE + PICKER_GAP_C + PICKER_H_WIDTH;
    vec2 p = fc - org;
    return p.x >= -PICKER_MARGIN_C && p.x <= tw + PICKER_MARGIN_C &&
           p.y >= -PICKER_MARGIN_C - 14.0 && p.y <= PICKER_SV_SIZE + PICKER_MARGIN_C;
}

vec4 renderPicker(vec2 fc, vec2 res, vec3 hsv, bool show) {
    if (!show) return vec4(0.0);
    vec2 org = pickerOrigin(res);
    float tw = PICKER_SV_SIZE + PICKER_GAP_C + PICKER_H_WIDTH;
    vec2 p = fc - org;
    if(p.x<-PICKER_MARGIN_C-2.0||p.x>tw+PICKER_MARGIN_C+2.0||
       p.y<-PICKER_MARGIN_C-14.0||p.y>PICKER_SV_SIZE+PICKER_MARGIN_C+2.0)
        return vec4(0.0);

    vec4 r = vec4(0.0);
    vec2 bgC = vec2(tw*0.5, PICKER_SV_SIZE*0.5);
    float bgD = sdRoundBox2D(p-bgC, vec2(tw*0.5+PICKER_MARGIN_C, PICKER_SV_SIZE*0.5+PICKER_MARGIN_C), PANEL_CORNER_R);
    if(bgD<0.0){float e=1.0-smoothstep(-2.0,0.0,bgD);r=vec4(mix(UI_COL_PANEL,UI_COL_BORDER,e*0.5),0.92);}

    // SV square
    if(p.x>=0.0&&p.x<=PICKER_SV_SIZE&&p.y>=0.0&&p.y<=PICKER_SV_SIZE){
        vec2 sv=clamp(p/PICKER_SV_SIZE,0.0,1.0);
        r=vec4(hsv2rgb(vec3(hsv.x,sv.x,sv.y)),1.0);
        float bd=sdBox2D(p-vec2(PICKER_SV_SIZE*0.5),vec2(PICKER_SV_SIZE*0.5));
        if(bd>-1.0) r.rgb=mix(r.rgb,UI_COL_BORDER,smoothstep(-1.0,0.0,bd));
    }
    // SV cursor
    vec2 svm=vec2(hsv.y,hsv.z)*PICKER_SV_SIZE;
    float svD=length(p-svm);
    if(svD<5.0){
        float ring=smoothstep(5.0,3.5,svD)-smoothstep(3.5,2.0,svD);
        r=vec4(mix(r.rgb,vec3(1.0),ring*0.9),max(r.a,ring));
        if(svD<2.0) r=vec4(hsv2rgb(hsv),1.0);
    }
    // Hue bar
    float hst=PICKER_SV_SIZE+PICKER_GAP_C;
    if(p.x>=hst&&p.x<=hst+PICKER_H_WIDTH&&p.y>=0.0&&p.y<=PICKER_SV_SIZE){
        float hc=clamp(p.y/PICKER_SV_SIZE,0.0,1.0);
        r=vec4(hsv2rgb(vec3(hc,1.0,1.0)),1.0);
        float hbd=sdBox2D(p-vec2(hst+PICKER_H_WIDTH*0.5,PICKER_SV_SIZE*0.5),vec2(PICKER_H_WIDTH*0.5,PICKER_SV_SIZE*0.5));
        if(hbd>-1.0) r.rgb=mix(r.rgb,UI_COL_BORDER,smoothstep(-1.0,0.0,hbd));
    }
    // Hue cursor
    float hmy=hsv.x*PICKER_SV_SIZE;
    if(abs(p.y-hmy)<2.5&&p.x>=hst-3.0&&p.x<=hst+PICKER_H_WIDTH+3.0){
        float hy=1.0-smoothstep(1.0,2.5,abs(p.y-hmy));
        bool ib=(p.x>=hst&&p.x<=hst+PICKER_H_WIDTH);
        if(!ib)r=vec4(mix(r.rgb,vec3(1.0),hy*0.9),max(r.a,hy));
        else r.rgb=mix(r.rgb,vec3(1.0),hy*0.3);
    }
    // Color preview swatch
    vec2 pvPos=vec2(PICKER_SV_SIZE*0.5,-PICKER_MARGIN_C*0.5);
    float pvD=sdRoundBox2D(p-pvPos,vec2(PICKER_SV_SIZE*0.5,4.0),2.0);
    if(pvD<0.0){r=vec4(hsv2rgb(hsv),1.0);if(pvD>-1.0)r.rgb=mix(r.rgb,UI_COL_BORDER,smoothstep(-1.0,0.0,pvD));}

    return r;
}

// =====================================================================
// Material Preview Sphere
// =====================================================================

vec4 renderMatPreview(vec2 fc, vec2 res, vec3 albedo, float roughness, float metalness) {
    vec2 org = pickerOrigin(res);
    float cy = org.y - PICKER_MARGIN_C - 20.0;
    float cx = org.x + PICKER_SV_SIZE * 0.5;
    float radius = 16.0;
    vec2 p = fc - vec2(cx, cy);
    float d = length(p);
    if (d > radius + 1.0) return vec4(0.0);
    vec3 n = vec3(p / radius, sqrt(max(0.0, 1.0 - dot(p,p) / (radius*radius))));
    if (d > radius) return vec4(0.0);
    vec3 ld = normalize(vec3(0.5, 0.8, -0.5));
    float ndl = max(dot(n, ld), 0.0);
    vec3 vd = vec3(0.0, 0.0, 1.0);
    vec3 h = normalize(ld + vd);
    float ndh = max(dot(n, h), 0.0);
    float ndv = max(dot(n, vd), 0.0);
    float r2 = roughness * roughness;
    float spec = pow(ndh, mix(256.0, 4.0, r2));
    vec3 F0 = mix(vec3(0.04), albedo, metalness);
    vec3 F = F0 + (1.0 - F0) * pow(1.0 - ndv, 5.0);
    vec3 kD = (1.0 - F) * (1.0 - metalness);
    vec3 ambient = vec3(0.15) * albedo;
    vec3 col = ambient + kD * albedo * ndl + F * spec;
    col = col / (col + vec3(1.0));
    col = pow(col, vec3(INV_GAMMA));
    float aa = smoothstep(radius, radius - 1.0, d);
    return vec4(col, aa);
}