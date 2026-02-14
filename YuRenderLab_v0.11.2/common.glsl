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

// ----------------------- Interaction modes -----------------------

#define MODE_NONE 0.0
#define MODE_CAMERA 1.0
#define MODE_TRANSFORM 2.0

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
    return r;
}

// =====================================================================
// Gizmo Utility Functions
// =====================================================================

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