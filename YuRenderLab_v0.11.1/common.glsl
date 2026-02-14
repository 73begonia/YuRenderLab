const float PI = 3.1415926535;
const float TWO_PI = 2.0 * PI;
const float HALF_PI = 0.5 * PI;

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
#define GZ_PICK_RAD 0.08
#define GZ_MIN_SCALE 0.3
#define GZ_MAX_SCALE 2.0
#define GZ_SCALE_FACTOR 0.2
#define GZ_OCCLUDED_BRIGHT 0.35

// ----------------------- Snapping -----------------------

#define TF_SNAP_TRANS 0.25

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

// =====================================================================
// Segment Distance (for anti-aliased line rendering)
// =====================================================================

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

// =====================================================================
// Gizmo Rendering (analytical geometry)
// =====================================================================

GizmoHit traceAxisGizmo(Ray r, vec3 c, int actPart, float sc, float pxScalar) {
    GizmoHit g;
    g.t = INF; g.part = PART_NONE; g.alpha = 0.0; g.col = vec3(0.0);

    mat3 axT = mat3(1.0);
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

    // Axis arrows (cone tips + base disks)
    for (int ax = 0; ax < 3; ax++) {
        vec3 dir = axT * axisVec(ax);
        vec3 col = (actPart == PART_X + ax) ? AXIS_COL_HL : axisCol(ax);

        // Cone tip
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

    // Anti-aliased axis lines
    float bestD = INF;
    float bestT = INF;
    int bestPart = PART_NONE;
    for (int ax = 0; ax < 3; ax++) {
        vec3 dir = axT * axisVec(ax);
        vec3 pStart = c + dir * cr;
        vec3 pEnd = c + dir * (sz - cnl);
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

GizmoHit traceGizmo(Ray r, vec3 center, vec3 camPos, int actPart, float pxScalar) {
    float gsc = gizmoScl(center, camPos);
    return traceAxisGizmo(r, center, actPart, gsc, pxScalar);
}