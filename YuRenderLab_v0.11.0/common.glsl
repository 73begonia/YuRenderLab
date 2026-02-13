// ============================================================================
// YuRenderLab v0.11 - Common (shared include)
// Adapted from Shadertoy Common tab for VS Code ShaderToy extension
// Usage: #include "common.glsl"
// ============================================================================

//=============================================================================
// UE 风格变换 Gizmo（混合方案：场景SDF + 交互解析几何）
//=============================================================================

#define PI 3.14159265359
#define TAU (2.0 * PI)
#define INF 1e10

//=============================================================================
// 光线步进配置
//=============================================================================

#define MAX_STEPS 256
#define MAX_DIST 100.0
#define SURF_DIST 0.002
#define SHADOW_STEPS 256
#define SHADOW_SOFTNESS 16.0

// 模式定义
#define SHADOW_MODE_TRADITIONAL 0  // 传统方法：最快，但在尖角处有条纹
#define SHADOW_MODE_IMPROVED    1  // 改进版(Aaltonen)：三角测量，消除条纹
#define SHADOW_MODE_INNER       2  // 终极版(IQ/Nurof3n)：允许穿入物体，物理更准确
#define CURRENT_SHADOW_MODE 2 

//=============================================================================
// 配置常量
//=============================================================================

#define CAM_FOV 60.0
#define CAM_NEAR 0.1
#define CAM_FAR 100.0
#define CAM_INIT_POS vec3(0.0, 0.0, -5.0)
#define CAM_SENS_X 10.0
#define CAM_SENS_Y 5.0
#define CAM_SPEED 0.05
#define CAM_SPRINT 3.0
#define CAM_FOCUS_DIST 2.5

#define TF_ROT_SENS 3.0
#define TF_SCALE_SENS 2.0
#define TF_SCALE_MIN 0.1
#define TF_SCALE_MAX 5.0
#define TF_SNAP_TRANS 0.25
#define TF_SNAP_ROT (PI / 12.0)
#define TF_SNAP_SCALE 0.1

#define GZ_SIZE 0.8
#define GZ_CYL_RAD 0.015
#define GZ_CONE_LEN 0.05
#define GZ_CONE_RAD 0.04
#define GZ_CENTER_RAD 0.045
#define GZ_PICK_RAD 0.08
#define GZ_RING_RAD 0.7
#define GZ_RING_TUBE 0.02
#define GZ_MIN_SCALE 0.3
#define GZ_MAX_SCALE 2.0
#define GZ_SCALE_FACTOR 0.2
#define GZ_OCCLUDED_BRIGHT 0.35

#define OUTLINE_WIDTH 2.0
#define OUTLINE_COL vec3(1.0, 0.9, 0.2)
#define OUTLINE_COL_OCC vec3(0.5, 0.5, 0.5)

#define AXIS_IND_CENTER vec2(50.0, 50.0)
#define AXIS_IND_SIZE 30.0
#define AXIS_IND_LINE_W 2.0
#define AXIS_IND_LABEL_OFF 8.0

#define LT_WIRE_THICK 0.008
#define LT_PT_RING_RAD 0.3
#define LT_SPOT_HEIGHT 0.6
#define LT_SPOT_RAD 0.35
#define LT_SPOT_INNER 0.8
#define LT_SPOT_OUTER 1.0
#define LT_AREA_OUTER 0.5
#define LT_AREA_INNER 0.3
#define LT_PT_INTENSITY 3.0
#define LT_SPOT_INTENSITY 4.0
#define LT_AREA_INTENSITY 2.5
#define LT_AMBIENT 0.0
#define LT_BASE_RANGE 3.0

#define UI_BTN_SIZE 40.0
#define UI_MARGIN 10.0
#define UI_TOP 30.0
#define UI_GAP 4.0
#define UI_PX_SIZE 2.0
#define UI_BG_COL vec3(0.24, 0.17, 0.17)
#define UI_BG_COL_LT vec3(0.29, 0.21, 0.21)
#define UI_BORDER_COL vec3(0.55, 0.35, 0.25)
#define UI_HL_COL vec3(0.7, 0.5, 0.3)

#define PICKER_SV_SIZE 120.0
#define PICKER_H_WIDTH 20.0
#define PICKER_GAP 8.0
#define PICKER_PX_SIZE 2.0
#define PICKER_TITLE_H 10.0

#define PX_BLUE vec3(0.3, 0.5, 0.8)
#define PX_RED vec3(0.7, 0.25, 0.25)
#define PX_CYAN vec3(0.3, 0.7, 0.7)
#define PX_YELLOW vec3(0.8, 0.7, 0.3)
#define PX_GREEN vec3(0.3, 0.6, 0.35)
#define PX_WHITE vec3(0.85, 0.82, 0.78)
#define PX_ORANGE vec3(0.8, 0.45, 0.2)

const vec3 AXIS_COLS[3] = vec3[3](
    vec3(0.9, 0.15, 0.15),
    vec3(0.15, 0.85, 0.15),
    vec3(0.2, 0.35, 0.9)
);
#define AXIS_COL_CENTER vec3(0.92, 0.92, 0.92)
#define AXIS_COL_HL vec3(1.0, 0.85, 0.1)

//=============================================================================
// 枚举常量
//=============================================================================

#define MAX_OBJECTS 10

#define OBJ_NONE 0
#define OBJ_SPHERE 1
#define OBJ_BOX 2
#define OBJ_POINT_LIGHT 3
#define OBJ_SPOT_LIGHT 4
#define OBJ_AREA_LIGHT 5

#define ROW_CAMERA 0
#define ROW_SELECT 1
#define ROW_OBJECTS 2
#define ROW_COLORS 10
#define ROW_ROTATIONS 11
#define ROW_DRAG 12
#define ROW_DRAG_ROT 13
#define ROW_KEYS 14
#define ROW_CLIPBOARD 15
#define ROW_CLIP_ROT 16
#define ROW_UNDO 17
#define ROW_UNDO_ROT 18
#define ROW_PICKER 19
#define ROW_UI 20

#define MODE_NONE 0.0
#define MODE_CAMERA 1.0
#define MODE_TRANSFORM 2.0
#define MODE_ROLL 3.0
#define MODE_PICKER_SV 4.0
#define MODE_PICKER_H 5.0
#define MODE_PICKER_DRAG 6.0

#define TRANS_TRANSLATE 0.0
#define TRANS_ROTATE 1.0
#define TRANS_SCALE 2.0

#define PART_NONE -1
#define PART_CENTER 0
#define PART_X 1
#define PART_Y 2
#define PART_Z 3

#define ID_NONE -1
#define ID_WALL_RIGHT -2
#define ID_WALL_LEFT -3
#define ID_WALL_BACK -4
#define ID_FLOOR -5
#define ID_CEILING -6

#define KEY_W 87
#define KEY_A 65
#define KEY_S 83
#define KEY_D 68
#define KEY_E 69
#define KEY_R 82
#define KEY_F 70
#define KEY_C 67
#define KEY_V 86
#define KEY_Z 90
#define KEY_SPACE 32
#define KEY_CTRL 17
#define KEY_SHIFT 16
#define KEY_ALT 18
#define KEY_DELETE 46

#define ACT_NONE 0
#define ACT_TRANSLATE 1
#define ACT_ROTATE 2
#define ACT_SCALE 3
#define ACT_FOCUS 4
#define ACT_DELETE 5
#define ACT_SPHERE 6
#define ACT_BOX 7
#define ACT_PT_LIGHT 8
#define ACT_SPOT_LIGHT 9
#define ACT_AREA_LIGHT 10

#define CHG_NONE 0u
#define CHG_CAMERA 1u
#define CHG_SELECTION 2u
#define CHG_OBJECT 4u
#define CHG_DRAG 8u
#define CHG_PICKER 16u
#define CHG_UNDO 32u
#define CHG_CLIPBOARD 64u
#define CHG_ADD 128u
#define CHG_DELETE 256u
#define CHG_UNDO_EXEC 512u
#define CHG_UI 1024u

//=============================================================================
// 数据结构
//=============================================================================

struct Ray { vec3 ro; vec3 rd; };
struct Hit { float t; vec3 p; vec3 n; vec3 col; int id; };
struct GizmoHit { float t; float alpha; vec3 col; int part; };
struct WireHit { float t; vec3 col; float alpha; };
struct Obj { int typ; vec3 pos; vec3 siz; vec3 col; vec4 quat; };

struct GizmoCtx {
    vec3 center;
    vec3 camPos;
    float transMode;
    float gscale;
    int actPart;
    vec4 quat;
    bool isLit;
};

struct Btn {
    vec2 pos;
    vec2 siz;
    int action;
    bool sel;
};

struct SDFResult {
    float d;
    int id;
    vec3 col;
};

//=============================================================================
// 工具函数
//=============================================================================

vec4 ld(sampler2D b, int x, int y) { return texelFetch(b, ivec2(x, y), 0); }

// [VS Code 适配] 键盘纹理布局差异:
// Shadertoy.com: row 0 = 按键保持状态 (hold)
// VS Code ShaderToy 扩展: row 1 = 按键保持状态 (hold), row 0 = 事件
// 因此将采样行从 0 改为 1
bool keyDown(int k, sampler2D kb) { return texelFetch(kb, ivec2(k, 0), 0).x > 0.0; }

float snap(float v, float s) { return round(v / s) * s; }
vec3 snap3(vec3 v, float s) { return vec3(snap(v.x, s), snap(v.y, s), snap(v.z, s)); }

vec3 axisVec(int a) { return a == 0 ?  vec3(1.0, 0.0, 0.0) : (a == 1 ? vec3(0.0, 1.0, 0.0) : vec3(0.0, 0.0, 1.0)); }
vec3 axisCol(int a) { return AXIS_COLS[clamp(a, 0, 2)]; }

bool isLightTyp(int t) { return t == OBJ_POINT_LIGHT || t == OBJ_SPOT_LIGHT || t == OBJ_AREA_LIGHT; }

float gizmoScl(vec3 c, vec3 cam) {
    return clamp(length(c - cam) * GZ_SCALE_FACTOR, GZ_MIN_SCALE, GZ_MAX_SCALE);
}

vec3 randCol(int seed) {
    return vec3(
        fract(sin(float(seed) * 12.9898) * 43758.5453),
        fract(sin(float(seed) * 78.233) * 43758.5453),
        fract(sin(float(seed) * 45.164) * 43758.5453)
    ) * 0.5 + 0.4;
}

//=============================================================================
// 四元数
//=============================================================================

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

//=============================================================================
// 颜色转换
//=============================================================================

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

//=============================================================================
// 射线和变换
//=============================================================================

vec3 angles2dir(vec2 a) {
    float cp = cos(a.x), sp = sin(a.x);
    float cy = cos(a.y), sy = sin(a.y);
    return vec3(cp * sy, sp, cp * cy);
}

Ray createRay(vec2 uv, vec3 camPos, vec3 camDir, float fov, float asp) {
    vec3 up = vec3(0.0, 1.0, 0.0);
    vec3 rt = normalize(cross(camDir, up));
    if (length(cross(camDir, up)) < 0.001) {
        rt = vec3(1.0, 0.0, 0.0);
    }
    vec3 cu = cross(rt, camDir);
    float h = tan(fov * 0.5);
    vec2 p = (uv * 2.0 - 1.0) * vec2(asp * h, h);
    return Ray(camPos, normalize(camDir + rt * p.x + cu * p.y));
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

//=============================================================================
// 物体加载
//=============================================================================

int getObjCount(sampler2D buf) {
    return int(ld(buf, 0, ROW_OBJECTS).x);
}

Obj loadObj(sampler2D buf, int id) {
    Obj o;
    vec4 d1 = ld(buf, id * 2 + 1, ROW_OBJECTS);
    vec4 d2 = ld(buf, id * 2 + 2, ROW_OBJECTS);
    o.typ = int(d1.x);
    o.pos = d1.yzw;
    o.siz = d2.xyz;
    o.col = ld(buf, id, ROW_COLORS).xyz;
    o.quat = ld(buf, id, ROW_ROTATIONS);
    return o;
}

void loadObjQuick(sampler2D buf, int id, out int typ, out vec3 pos, out vec3 siz) {
    vec4 d1 = ld(buf, id * 2 + 1, ROW_OBJECTS);
    vec4 d2 = ld(buf, id * 2 + 2, ROW_OBJECTS);
    typ = int(d1.x);
    pos = d1.yzw;
    siz = d2.xyz;
}

//=============================================================================
// SDF 基础图元（用于场景渲染）
//=============================================================================

float sdSphere(vec3 p, float r) {
    return length(p) - r;
}

float sdBox(vec3 p, vec3 b) {
    vec3 q = abs(p) - b;
    return length(max(q, 0.0)) + min(max(q.x, max(q.y, q.z)), 0.0);
}

vec3 opRotate(vec3 p, vec4 q) {
    return quatRot(quatConj(q), p);
}

//=============================================================================
// 场景SDF（用于渲染）
//=============================================================================

float sdWallsOnly(vec3 p) {
    float d = INF;
    d = min(d, sdBox(p - vec3(2.0, 0.0, 0.0), vec3(0.1, 2.1, 2.1)));
    d = min(d, sdBox(p - vec3(-2.0, 0.0, 0.0), vec3(0.1, 2.1, 2.1)));
    d = min(d, sdBox(p - vec3(0.0, 0.0, 2.0), vec3(2.1, 2.1, 0.1)));
    d = min(d, sdBox(p - vec3(0.0, -2.0, 0.0), vec3(2.1, 0.1, 2.1)));
    d = min(d, sdBox(p - vec3(0.0, 2.0, 0.0), vec3(2.1, 0.1, 2.1)));
    return d;
}

SDFResult sdScene(vec3 p, sampler2D buf, int cnt, bool incLights) {
    SDFResult res;
    res.d = INF;
    res.id = ID_NONE;
    res.col = vec3(0.0);
    
    float dRight = sdBox(p - vec3(2.0, 0.0, 0.0), vec3(0.1, 2.1, 2.1));
    if (dRight < res.d) { res.d = dRight; res.id = ID_WALL_RIGHT; res.col = vec3(0.8, 0.2, 0.2); }
    
    float dLeft = sdBox(p - vec3(-2.0, 0.0, 0.0), vec3(0.1, 2.1, 2.1));
    if (dLeft < res.d) { res.d = dLeft; res.id = ID_WALL_LEFT; res.col = vec3(0.2, 0.8, 0.2); }
    
    float dBack = sdBox(p - vec3(0.0, 0.0, 2.0), vec3(2.1, 2.1, 0.1));
    if (dBack < res.d) { res.d = dBack; res.id = ID_WALL_BACK; res.col = vec3(0.9); }
    
    float dFloor = sdBox(p - vec3(0.0, -2.0, 0.0), vec3(2.1, 0.1, 2.1));
    if (dFloor < res.d) { res.d = dFloor; res.id = ID_FLOOR; res.col = vec3(0.9); }
    
    float dCeil = sdBox(p - vec3(0.0, 2.0, 0.0), vec3(2.1, 0.1, 2.1));
    if (dCeil < res.d) { res.d = dCeil; res.id = ID_CEILING; res.col = vec3(0.9); }
    
    for (int i = 0; i < MAX_OBJECTS && i < cnt; i++) {
        int typ; vec3 pos, siz;
        loadObjQuick(buf, i, typ, pos, siz);
        
        if (typ == OBJ_NONE) continue;
        if (!incLights && isLightTyp(typ)) continue;
        
        float d = INF;
        Obj o = loadObj(buf, i);
        
        if (typ == OBJ_SPHERE) {
            d = sdSphere(p - o.pos, o.siz.x);
        } else if (typ == OBJ_BOX) {
            vec3 lp = opRotate(p - o.pos, o.quat);
            d = sdBox(lp, o.siz);
        } else if (isLightTyp(typ)) {
            float rad = typ == OBJ_POINT_LIGHT ? LT_PT_RING_RAD * o.siz.x * 1.5 : 
                        typ == OBJ_SPOT_LIGHT ? max(LT_SPOT_HEIGHT, LT_SPOT_RAD) * o.siz.x : 
                        LT_AREA_OUTER * o.siz.x * 1.2;
            d = sdSphere(p - o.pos, rad);
        }
        
        if (d < res.d) {
            res.d = d;
            res.id = i;
            res.col = o.col;
        }
    }
    
    return res;
}

// 仅场景物体SDF（不含光源，用于阴影）
float sdSceneShadow(vec3 p, sampler2D buf, int cnt) {
    float d = sdWallsOnly(p);
    
    for (int i = 0; i < MAX_OBJECTS && i < cnt; i++) {
        int typ; vec3 pos, siz;
        loadObjQuick(buf, i, typ, pos, siz);
        
        if (typ == OBJ_NONE || isLightTyp(typ)) continue;
        
        Obj o = loadObj(buf, i);
        
        if (typ == OBJ_SPHERE) {
            d = min(d, sdSphere(p - o.pos, o.siz.x));
        } else if (typ == OBJ_BOX) {
            vec3 lp = opRotate(p - o.pos, o.quat);
            d = min(d, sdBox(lp, o.siz));
        }
    }
    
    return d;
}

//=============================================================================
// 光线步进（场景渲染）
//=============================================================================

vec3 calcNormal(vec3 p, sampler2D buf, int cnt) {
    vec2 e = vec2(1.0,-1.0)*0.5773*0.0005;
    return normalize(
        e.xyy * sdScene(p + e.xyy, buf, cnt, false).d +
        e.yyx * sdScene(p + e.yyx, buf, cnt, false).d +
        e.yxy * sdScene(p + e.yxy, buf, cnt, false).d +
        e.xxx * sdScene(p + e.xxx, buf, cnt, false).d
    );
}

Hit rayMarch(Ray r, sampler2D buf, int cnt, bool incLights) {
    Hit h;
    h.t = 0.0;
    h.id = ID_NONE;
    h.p = vec3(0.0);
    h.n = vec3(0.0);
    h.col = vec3(0.0);
    
    float t = 0.0;
    for (int i = 0; i < MAX_STEPS; i++) {
        vec3 p = r.ro + r.rd * t;
        SDFResult res = sdScene(p, buf, cnt, incLights);
        
        if (res.d < SURF_DIST) {
            h.t = t;
            h.p = p;
            h.id = res.id;
            h.col = res.col;
            h.n = calcNormal(p, buf, cnt);
            return h;
        }
        
        t += res.d;
        if (t > MAX_DIST) break;
    }
    
    h.t = INF;
    return h;
}

// ============================================================================
// 软阴影算法集合 (Soft Shadows)
// ============================================================================

float softShadow(vec3 ro, vec3 rd, float mint, float maxt, sampler2D buf, int cnt) {
    float res = 1.0;
    float t = mint;
    float ph = 1e10;

    for (int i = 0; i < SHADOW_STEPS; i++) {
        vec3 p = ro + rd * t;
        float h = sdSceneShadow(p, buf, cnt);

#if CURRENT_SHADOW_MODE == SHADOW_MODE_TRADITIONAL
        if (h < 0.001) return 0.0;
        res = min(res, SHADOW_SOFTNESS * h / t);
        t += h;
        
#elif CURRENT_SHADOW_MODE == SHADOW_MODE_IMPROVED
        if (h < 0.001) return 0.0;
        float y = (i == 0) ? 0.0 : h * h / (2.0 * ph);
        float d = sqrt(max(0.0, h * h - y * y));
        res = min(res, SHADOW_SOFTNESS * d / max(0.0, t - y));
        ph = h;
        t += h;
        
#else
        res = min(res, SHADOW_SOFTNESS * h / t);
        t += clamp(h, 0.005, 0.50);
        if (res < -1.0 || t > maxt) break;
#endif

        if (t > maxt) break;
    }
    
#if CURRENT_SHADOW_MODE == SHADOW_MODE_INNER
    res = max(res, -1.0);
    return 0.25 * (1.0 + res) * (1.0 + res) * (2.0 - res);
#else
    res = clamp(res, 0.0, 1.0);
    return res * res * (3.0 - 2.0 * res);
#endif
}

//=============================================================================
// 解析几何求交（用于拾取和轮廓检测）
//=============================================================================

Hit hitSphere(Ray r, vec3 c, float rad, vec3 col, int id) {
    Hit h;
    h.t = INF;
    h.id = ID_NONE;
    h.p = vec3(0.0);
    h.n = vec3(0.0);
    h.col = vec3(0.0);
    
    vec3 oc = r.ro - c;
    float b = dot(oc, r.rd);
    float det = b * b - (dot(oc, oc) - rad * rad);
    if (det < 0.0) return h;
    float t = -b - sqrt(det);
    if (t < 0.001) { t = -b + sqrt(det); if (t < 0.001) return h; }
    h.t = t;
    h.p = r.ro + r.rd * t;
    h.n = normalize(h.p - c);
    h.col = col;
    h.id = id;
    return h;
}

Hit hitBox(Ray r, vec3 c, vec3 hs, vec3 col, int id) {
    Hit h;
    h.t = INF;
    h.id = ID_NONE;
    h.p = vec3(0.0);
    h.n = vec3(0.0);
    h.col = vec3(0.0);
    
    vec3 inv = 1.0 / r.rd;
    vec3 t1 = (c - hs - r.ro) * inv;
    vec3 t2 = (c + hs - r.ro) * inv;
    vec3 tmin = min(t1, t2);
    vec3 tmax = max(t1, t2);
    float enter = max(max(tmin.x, tmin.y), tmin.z);
    float texit = min(min(tmax.x, tmax.y), tmax.z);
    if (enter > texit || texit < 0.0) return h;
    float t = enter > 0.0 ? enter : texit;
    if (t < 0.001) return h;
    h.t = t;
    h.p = r.ro + r.rd * t;
    h.col = col;
    h.id = id;
    vec3 lp = h.p - c;
    vec3 d = abs(lp) - hs;
    h.n = d.x > d.y && d.x > d.z ? vec3(sign(lp.x), 0.0, 0.0) :
          d.y > d.z ? vec3(0.0, sign(lp.y), 0.0) : vec3(0.0, 0.0, sign(lp.z));
    return h;
}

Hit hitRotBox(Ray r, vec3 c, vec3 hs, vec4 rot, vec3 col, int id) {
    mat3 m = quatToMat(rot);
    mat3 inv = transpose(m);
    Hit h = hitBox(Ray(inv * (r.ro - c), inv * r.rd), vec3(0.0), hs, col, id);
    if (h.t < INF) {
        h.p = m * h.p + c;
        h.n = m * h.n;
    }
    return h;
}

Hit hitObj(Ray r, Obj o, int id) {
    if (o.typ == OBJ_SPHERE) return hitSphere(r, o.pos, o.siz.x, o.col, id);
    if (o.typ == OBJ_BOX) return hitRotBox(r, o.pos, o.siz, o.quat, o.col, id);
    if (isLightTyp(o.typ)) {
        float rad = o.typ == OBJ_POINT_LIGHT ? LT_PT_RING_RAD * o.siz.x * 1.5 :  
                    o.typ == OBJ_SPOT_LIGHT ? max(LT_SPOT_HEIGHT, LT_SPOT_RAD) * o.siz.x : 
                    LT_AREA_OUTER * o.siz.x * 1.2;
        Hit h = hitSphere(r, o.pos, rad, o.col, id);
        if (h.t < INF) h.n = -r.rd;
        return h;
    }
    Hit h;
    h.t = INF;
    h.id = ID_NONE;
    return h;
}

Hit traceSceneAnalytic(Ray r, sampler2D buf, int cnt, bool incLights) {
    Hit best;
    best.t = INF;
    best.id = ID_NONE;
    best.p = vec3(0.0);
    best.n = vec3(0.0);
    best.col = vec3(0.0);
    
    Hit wh;
    wh = hitBox(r, vec3(2.0, 0.0, 0.0), vec3(0.1, 2.1, 2.1), vec3(0.8, 0.2, 0.2), ID_WALL_RIGHT);
    if (wh.t < best.t) best = wh;
    wh = hitBox(r, vec3(-2.0, 0.0, 0.0), vec3(0.1, 2.1, 2.1), vec3(0.2, 0.8, 0.2), ID_WALL_LEFT);
    if (wh.t < best.t) best = wh;
    wh = hitBox(r, vec3(0.0, 0.0, 2.0), vec3(2.1, 2.1, 0.1), vec3(0.9), ID_WALL_BACK);
    if (wh.t < best.t) best = wh;
    wh = hitBox(r, vec3(0.0, -2.0, 0.0), vec3(2.1, 0.1, 2.1), vec3(0.9), ID_FLOOR);
    if (wh.t < best.t) best = wh;
    wh = hitBox(r, vec3(0.0, 2.0, 0.0), vec3(2.1, 0.1, 2.1), vec3(0.9), ID_CEILING);
    if (wh.t < best.t) best = wh;
    
    for (int i = 0; i < MAX_OBJECTS && i < cnt; i++) {
        int typ; vec3 pos, siz;
        loadObjQuick(buf, i, typ, pos, siz);
        
        if (typ == OBJ_NONE) continue;
        if (!incLights && isLightTyp(typ)) continue;
        
        Obj o = loadObj(buf, i);
        Hit h = hitObj(r, o, i);
        if (h.t < best.t) best = h;
    }
    
    return best;
}

//=============================================================================
// 光源线框（解析几何）
//=============================================================================

float lineSegDist(vec3 ro, vec3 rd, vec3 p0, vec3 p1, out float rayT) {
    vec3 v = p1 - p0;
    vec3 w = ro - p0;
    float a = dot(rd, rd);
    float b = dot(rd, v);
    float c = dot(v, v);
    float d = dot(rd, w);
    float e = dot(v, w);
    float den = a * c - b * b;
    float s = den < 0.0001 ? e / c : (a * e - b * d) / den;
    float t = den < 0.0001 ? 0.0 : (b * e - c * d) / den;
    s = clamp(s, 0.0, 1.0);
    rayT = max(t, 0.0);
    return length(ro + rd * rayT - (p0 + v * s));
}

void minSegment(vec3 ro, vec3 rd, vec3 p0, vec3 p1, inout float minD, inout float minT) {
    float t;
    float d = lineSegDist(ro, rd, p0, p1, t);
    if (t > 0.0 && d < minD) {
        minD = d;
        minT = t;
    }
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

void checkSeg(vec3 ro, vec3 rd, vec3 p0, vec3 p1, float th, float pxScalar, vec3 col, inout WireHit best) {
    float t;
    float d = lineSegDist(ro, rd, p0, p1, t);
    if (t > 0.0 && t < best.t) {
        float px = t * pxScalar;
        if (d < th + px) {
            best.t = t;
            best.col = col;
            best.alpha = smoothstep(th + px, th - px, d);
        }
    }
}

void getOrthoBasis(vec3 n, out vec3 u, out vec3 v) {
    if (abs(n.y) < 0.999) {
        u = normalize(cross(n, vec3(0.0, 1.0, 0.0)));
    } else {
        u = normalize(cross(n, vec3(1.0, 0.0, 0.0)));
    }
    v = cross(n, u);
}

WireHit tracePointWire(vec3 ro, vec3 rd, vec3 c, float rad, vec4 rot, vec3 col, float th, float pxScalar) {
    WireHit w;
    w.t = INF; w.alpha = 0.0; w.col = col;
    mat3 m = quatToMat(rot);
    float bestD = INF;
    float bestT = INF;
    for (int ax = 0; ax < 3; ax++) {
        vec3 normal = m * axisVec(ax);
        vec3 u, v;
        getOrthoBasis(normal, u, v);
        const int SEGS = 24; 
        vec3 pPrev = c + u * rad;
        for (int i = 1; i <= SEGS; i++) {
            float ang = float(i) * TAU / float(SEGS);
            vec3 pNext = c + (u * cos(ang) + v * sin(ang)) * rad;
            minSegment(ro, rd, pPrev, pNext, bestD, bestT);
            pPrev = pNext;
        }
    }
    if (bestT < INF) {
        float px = bestT * pxScalar;
        if (bestD < th + px) {
            w.t = bestT;
            w.alpha = smoothstep(th + px, th - px, bestD);
        }
    }
    return w;
}

WireHit traceSpotWire(vec3 ro, vec3 rd, vec3 apex, float ht, float rad, vec4 rot, vec3 col, float th, float pxScalar) {
    WireHit w;
    w.t = INF; w.alpha = 0.0; w.col = col;
    mat3 m = quatToMat(rot);
    vec3 dir = m * vec3(0.0, -1.0, 0.0);
    vec3 baseCenter = apex + dir * ht;
    vec3 u, v;
    getOrthoBasis(dir, u, v);
    float bestD = INF;
    float bestT = INF;
    for (int i = 0; i < 4; i++) {
        float a = float(i) * PI * 0.5;
        vec3 pRim = baseCenter + (u * cos(a) + v * sin(a)) * rad;
        minSegment(ro, rd, apex, pRim, bestD, bestT);
    }
    const int SEGS = 32;
    vec3 pPrev = baseCenter + u * rad;
    for (int i = 1; i <= SEGS; i++) {
        float ang = float(i) * TAU / float(SEGS);
        vec3 pNext = baseCenter + (u * cos(ang) + v * sin(ang)) * rad;
        minSegment(ro, rd, pPrev, pNext, bestD, bestT);
        pPrev = pNext;
    }
    if (bestT < INF) {
        float px = bestT * pxScalar;
        if (bestD < th + px) {
            w.t = bestT;
            w.alpha = smoothstep(th + px, th - px, bestD);
        }
    }
    return w;
}

WireHit traceAreaWire(vec3 ro, vec3 rd, vec3 c, float outer, float inner, vec4 rot, vec3 col, float th, float pxScalar) {
    WireHit w;
    w.t = INF; w.alpha = 0.0; w.col = col;
    mat3 m = quatToMat(rot);
    vec3 rt = m * vec3(1.0, 0.0, 0.0);
    vec3 up = m * vec3(0.0, 1.0, 0.0);
    vec3 oc[4], ic[4];
    oc[0] = c + (-rt - up) * outer;
    oc[1] = c + (rt - up) * outer;
    oc[2] = c + (rt + up) * outer;
    oc[3] = c + (-rt + up) * outer;
    ic[0] = c + (-rt - up) * inner;
    ic[1] = c + (rt - up) * inner;
    ic[2] = c + (rt + up) * inner;
    ic[3] = c + (-rt + up) * inner;
    float bestD = INF;
    float bestT = INF;
    for (int i = 0; i < 4; i++) {
        int n = (i + 1) % 4;
        minSegment(ro, rd, oc[i], oc[n], bestD, bestT);
        minSegment(ro, rd, ic[i], ic[n], bestD, bestT);
        minSegment(ro, rd, oc[i], ic[i], bestD, bestT);
    }
    if (bestT < INF) {
        float px = bestT * pxScalar;
        if (bestD < th + px) {
            w.t = bestT;
            w.alpha = smoothstep(th + px, th - px, bestD);
        }
    }
    return w;
}

WireHit traceLightWire(Ray r, Obj o, float pxScalar) {
    float s = o.siz.x;
    float th = LT_WIRE_THICK * s;
    if (o.typ == OBJ_POINT_LIGHT) {
        return tracePointWire(r.ro, r.rd, o.pos, LT_PT_RING_RAD * s, o.quat, o.col, th, pxScalar);
    } else if (o.typ == OBJ_SPOT_LIGHT) {
        return traceSpotWire(r.ro, r.rd, o.pos, LT_SPOT_HEIGHT * s, LT_SPOT_RAD * s, o.quat, o.col, th, pxScalar);
    } else if (o.typ == OBJ_AREA_LIGHT) {
        return traceAreaWire(r.ro, r.rd, o.pos, LT_AREA_OUTER * s, LT_AREA_INNER * s, o.quat, o.col, th, pxScalar);
    }
    WireHit w; w.t = INF; w.alpha = 0.0; return w;
}

//=============================================================================
// Gizmo 拾取（解析几何）
//=============================================================================

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

int pickAxisGizmo(Ray r, vec3 c, float sc, mat3 axT, out float ht) {
    int part = PART_NONE;
    ht = INF;
    float sz = GZ_SIZE * sc;
    float cr = GZ_CENTER_RAD * sc;
    float pr = GZ_PICK_RAD * sc;

    vec3 oc = r.ro - c;
    float b = dot(oc, r.rd);
    float det = b * b - (dot(oc, oc) - cr * cr * 3.24);
    if (det >= 0.0) {
        float t = -b - sqrt(det);
        if (t > 0.001 && t < ht) { ht = t; part = PART_CENTER; }
    }

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

int pickGizmo(Ray r, GizmoCtx ctx, out float ht) {
    if (ctx.isLit && ctx.transMode == TRANS_SCALE) {
        vec3 oc = r.ro - ctx.center;
        float b = dot(oc, r.rd);
        float rad = GZ_CENTER_RAD * ctx.gscale * 2.0;
        float det = b * b - (dot(oc, oc) - rad * rad);
        if (det >= 0.0) {
            ht = -b - sqrt(det);
            if (ht > 0.001) return PART_CENTER;
        }
        ht = INF;
        return PART_NONE;
    }
    if (ctx.transMode == TRANS_ROTATE) {
        return pickRotGizmo(r, ctx.center, ctx.camPos, ctx.gscale, ctx.actPart, ht);
    }
    mat3 axT = ctx.transMode == TRANS_SCALE ? quatToMat(ctx.quat) : mat3(1.0);
    return pickAxisGizmo(r, ctx.center, ctx.gscale, axT, ht);
}

//=============================================================================
// Gizmo 渲染（解析几何）
//=============================================================================

vec3 applyShade(vec3 col, vec3 n) {
    return col * (0.4 + max(dot(n, normalize(vec3(0.5, 0.8, -0.3))), 0.0) * 0.6);
}

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

GizmoHit traceLightScaleGizmo(Ray r, vec3 c, int actPart, float sc) {
    GizmoHit g;
    g.t = INF;
    g.part = PART_NONE;
    g.alpha = 1.0;
    g.col = vec3(0.0);
    float rad = GZ_CENTER_RAD * sc * 1.5;
    vec3 oc = r.ro - c;
    float b = dot(oc, r.rd);
    float det = b * b - (dot(oc, oc) - rad * rad);
    if (det >= 0.0) {
        float t = -b - sqrt(det);
        if (t > 0.001) {
            g.t = t;
            g.part = PART_CENTER;
            vec3 n = normalize(r.ro + r.rd * t - c);
            g.col = applyShade(actPart == PART_CENTER ? AXIS_COL_HL : AXIS_COL_CENTER, n);
        }
    }
    return g;
}

GizmoHit traceAxisGizmo(Ray r, vec3 c, int actPart, float sc, mat3 axT, bool useBox, float pxScalar) {
    GizmoHit g;
    g.t = INF; g.part = PART_NONE; g.alpha = 0.0; g.col = vec3(0.0);
    float sz = GZ_SIZE * sc;
    float cr = GZ_CENTER_RAD * sc;
    float cnl = GZ_CONE_LEN * sc;
    float cnr = GZ_CONE_RAD * sc;
    float th = GZ_CYL_RAD * sc;

    float t; vec3 n;
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

    for (int ax = 0; ax < 3; ax++) {
        vec3 dir = axT * axisVec(ax);
        vec3 col = (actPart == PART_X + ax) ? AXIS_COL_HL : axisCol(ax);
        if (useBox) {
            float cs = cnr * 0.7;
            vec3 cc = c + dir * (sz - cs);
            Hit bh = hitRotBox(r, cc, vec3(cs), matToQuat(axT), col, 0);
            if (bh.t < g.t) {
                g.t = bh.t; g.part = PART_X + ax; g.alpha = 1.0;
                g.col = applyShade(col, bh.n);
            }
        } else {
            vec3 apex = c + dir * sz;
            float coneAng = atan(cnr / cnl);
            if (hitCone(r.ro, r.rd, apex, -dir, coneAng, cnl, t, n) && t < g.t) {
                g.t = t; g.part = PART_X + ax; g.alpha = 1.0;
                g.col = applyShade(col, n);
            }
            vec3 coneBase = c + dir * (sz - cnl);
            if (hitDisk(r.ro, r.rd, coneBase, -dir, cnr, t) && t < g.t) {
                g.t = t; g.part = PART_X + ax; g.alpha = 1.0;
                g.col = applyShade(col, -dir);
            }
        }
    }

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

GizmoHit traceGizmo(Ray r, GizmoCtx ctx, float pxScalar) {
    if (ctx.isLit && ctx.transMode == TRANS_SCALE) {
        return traceLightScaleGizmo(r, ctx.center, ctx.actPart, ctx.gscale);
    }
    if (ctx.transMode == TRANS_ROTATE) {
        return traceRotGizmo(r, ctx.center, ctx.camPos, ctx.actPart, ctx.gscale, pxScalar);
    }
    return traceAxisGizmo(r, ctx.center, ctx.actPart, ctx.gscale,
                          ctx.transMode == TRANS_SCALE ? quatToMat(ctx.quat) : mat3(1.0),
                          ctx.transMode == TRANS_SCALE,
                          pxScalar);
}

//=============================================================================
// 光照
//=============================================================================

float distAtten(float d, float range) {
    float a = 1.0 / max(d * d, 0.0001);
    float w = pow(clamp(1.0 - pow(d / range, 4.0), 0.0, 1.0), 2.0);
    return a * w;
}

vec3 calcSpec(vec3 N, vec3 L, vec3 V, vec3 lc, float rough) {
    vec3 H = normalize(L + V);
    float nh = max(dot(N, H), 0.0);
    float nv = max(dot(N, V), 0.0);
    float a = rough * rough;
    float a2 = a * a;
    float dn = nh * nh * (a2 - 1.0) + 1.0;
    float D = a2 / (PI * dn * dn);
    vec3 F0 = vec3(0.04);
    vec3 F = F0 + (1.0 - F0) * pow(1.0 - nv, 5.0);
    return lc * D * F * 0.5;
}

vec3 calcPointLight(vec3 hp, vec3 n, vec3 vd, vec3 sc, Obj lt, sampler2D buf, int cnt) {
    if (sdSceneShadow(lt.pos, buf, cnt) < 0.1) return vec3(0.0);
    vec3 lv = lt.pos - hp;
    float d = length(lv);
    vec3 L = normalize(lv);
    float range = LT_BASE_RANGE * lt.siz.x;
    if (d > range) return vec3(0.0);
    float shadow = softShadow(hp + n * 0.02, L, 0.02, d - 0.02, buf, cnt);
    if (shadow < 0.01) return vec3(0.0);
    float a = distAtten(d, range) * LT_PT_INTENSITY;
    float nl = max(dot(n, L), 0.0);
    return (sc * lt.col * nl + calcSpec(n, L, vd, lt.col, 0.4)) * a * shadow;
}

vec3 calcSpotLight(vec3 hp, vec3 n, vec3 vd, vec3 sc, Obj lt, sampler2D buf, int cnt) {
    if (sdSceneShadow(lt.pos, buf, cnt) < 0.12) return vec3(0.0);
    vec3 lv = lt.pos - hp;
    float d = length(lv);
    vec3 L = normalize(lv);
    float range = LT_BASE_RANGE * lt.siz.x * 1.5;
    if (d > range) return vec3(0.0);
    vec3 sd = quatRot(lt.quat, vec3(0.0, -1.0, 0.0));
    float ca = dot(-L, sd);
    float oc = cos(LT_SPOT_OUTER * 0.5);
    float ic = cos(LT_SPOT_INNER * 0.5);
    if (ca < oc) return vec3(0.0);
    float shadow = softShadow(hp + n * 0.02, L, 0.02, d - 0.02, buf, cnt);
    if (shadow < 0.01) return vec3(0.0);
    float se = smoothstep(oc, ic, ca);
    float a = distAtten(d, range) * se * LT_SPOT_INTENSITY;
    float nl = max(dot(n, L), 0.0);
    return (sc * lt.col * nl + calcSpec(n, L, vd, lt.col, 0.3)) * a * shadow;
}

vec3 calcAreaLight(vec3 hp, vec3 n, vec3 vd, vec3 sc, Obj lt, sampler2D buf, int cnt) {
    if (sdSceneShadow(lt.pos, buf, cnt) < 0.45) return vec3(0.0);
    float s = lt.siz.x;
    float as = LT_AREA_OUTER * s * 0.9;
    mat3 m = quatToMat(lt.quat);
    vec3 ln = m * vec3(0.0, 0.0, -1.0);
    vec3 lr = m * vec3(1.0, 0.0, 0.0);
    vec3 lu = m * vec3(0.0, 1.0, 0.0);
    vec3 tp = hp - lt.pos;
    float x = clamp(dot(tp, lr), -as, as);
    float y = clamp(dot(tp, lu), -as, as);
    vec3 cp = lt.pos + lr * x + lu * y;
    vec3 lv = cp - hp;
    float d = length(lv);
    vec3 L = normalize(lv);
    if (dot(n, L) < 0.0 || dot(ln, -L) < 0.0) return vec3(0.0);
    float shadow = softShadow(hp + n * 0.02, L, 0.02, d - 0.02, buf, cnt);
    if (shadow < 0.01) return vec3(0.0);
    float range = LT_BASE_RANGE * s * 1.5;
    float a = distAtten(d, range) * LT_AREA_INTENSITY;
    float nl = max(dot(n, L), 0.0);
    float lf = max(dot(ln, -L), 0.0);
    vec3 spec = calcSpec(n, L, vd, lt.col, 0.5);
    return (sc * lt.col * nl * lf + spec) * a * shadow;
}

vec3 calcLighting(Hit h, vec3 camPos, sampler2D buf, int cnt) {
    if (h.id == ID_NONE) return vec3(0.02, 0.02, 0.05);
    vec3 vd = normalize(camPos - h.p);
    vec3 total = h.col * LT_AMBIENT;
    for (int i = 0; i < MAX_OBJECTS && i < cnt; i++) {
        int typ; vec3 pos, siz;
        loadObjQuick(buf, i, typ, pos, siz);
        if (!isLightTyp(typ)) continue;
        Obj lt = loadObj(buf, i);
        if (lt.typ == OBJ_POINT_LIGHT) total += calcPointLight(h.p, h.n, vd, h.col, lt, buf, cnt);
        else if (lt.typ == OBJ_SPOT_LIGHT) total += calcSpotLight(h.p, h.n, vd, h.col, lt, buf, cnt);
        else if (lt.typ == OBJ_AREA_LIGHT) total += calcAreaLight(h.p, h.n, vd, h.col, lt, buf, cnt);
    }
    return total;
}

//=============================================================================
// 轮廓检测（解析几何）
//=============================================================================

const vec2 OUTLINE_OFFS[4] = vec2[4](vec2(0.0, 1.0), vec2(1.0, 0.0), vec2(0.0, -1.0), vec2(-1.0, 0.0));

vec2 detectOutline(vec2 fc, vec2 res, vec3 camPos, vec3 camDir, int selId, Hit sceneHit, sampler2D buf, int cnt) {
    if (selId < 0) return vec2(0.0);
    int typ; vec3 pos, siz;
    loadObjQuick(buf, selId, typ, pos, siz);
    if (isLightTyp(typ)) return vec2(0.0);
    vec2 uv = fc / res;
    vec2 ps = OUTLINE_WIDTH / res;
    float asp = res.x / res.y;
    float fov = radians(CAM_FOV);
    Ray r = createRay(uv, camPos, camDir, fov, asp);
    Obj sel = loadObj(buf, selId);
    Hit selHit = hitObj(r, sel, selId);
    if (selHit.id == selId) return vec2(0.0);
    float nearDist = INF;
    bool hasNeighbor = false;
    for (int i = 0; i < 4; i++) {
        Ray nr = createRay(uv + OUTLINE_OFFS[i] * ps, camPos, camDir, fov, asp);
        Hit nh = hitObj(nr, sel, selId);
        if (nh.id == selId) {
            hasNeighbor = true;
            nearDist = min(nearDist, nh.t);
        }
    }
    if (!hasNeighbor) return vec2(0.0);
    bool occluded = sceneHit.id != ID_NONE && sceneHit.id != selId && sceneHit.t < nearDist;
    return vec2(1.0, occluded ? 1.0 : 0.0);
}

//=============================================================================
// UI 系统
//=============================================================================

float sdBox2D(vec2 p, vec2 b) {
    vec2 d = abs(p) - b;
    return length(max(d, vec2(0.0))) + min(max(d.x, d.y), 0.0);
}

float drawCorner(vec2 p, vec2 sz, float ps) {
    vec2 hs = sz * 0.5;
    vec2 ap = abs(p);
    vec2 cd = hs - ap;
    if (cd.x > ps * 3.0 || cd.y > ps * 3.0 || cd.x < 0.0 || cd.y < 0.0) return 0.0;
    vec2 lp = floor(cd / ps);
    if ((lp.x == 0.0 && lp.y <= 2.0) || (lp.y == 0.0 && lp.x <= 2.0)) return 1.0;
    return 0.0;
}

vec2 getPickerDefaultPos(vec2 res) {
    float tw = PICKER_SV_SIZE + PICKER_GAP + PICKER_H_WIDTH;
    return vec2(res.x - UI_MARGIN - tw,
                res.y - UI_MARGIN - UI_BTN_SIZE - UI_GAP * 10.0 - PICKER_SV_SIZE - PICKER_TITLE_H - UI_TOP);
}

Btn getAddBtn(int idx, int lastClick, vec2 res) {
    Btn b;
    b.siz = vec2(UI_BTN_SIZE);
    b.pos = vec2(UI_MARGIN + UI_BTN_SIZE * 0.5 + float(idx) * (UI_BTN_SIZE + UI_GAP),
                 res.y - UI_MARGIN - UI_BTN_SIZE * 0.5 - UI_TOP);
    b.sel = false;
    if (idx == 0) { b.action = ACT_SPHERE; b.sel = (lastClick == ACT_SPHERE); }
    else if (idx == 1) { b.action = ACT_BOX; b.sel = (lastClick == ACT_BOX); }
    else if (idx == 2) { b.action = ACT_PT_LIGHT; b.sel = (lastClick == ACT_PT_LIGHT); }
    else if (idx == 3) { b.action = ACT_SPOT_LIGHT; b.sel = (lastClick == ACT_SPOT_LIGHT); }
    else { b.action = ACT_AREA_LIGHT; b.sel = (lastClick == ACT_AREA_LIGHT); }
    return b;
}

Btn getToolBtn(int idx, float transMode, bool hasSel, int lastClick, vec2 res) {
    Btn b;
    b.siz = vec2(UI_BTN_SIZE);
    float re = res.x - UI_MARGIN - UI_BTN_SIZE * 0.5;
    b.pos = vec2(re - float(4 - idx) * (UI_BTN_SIZE + UI_GAP),
                 res.y - UI_MARGIN - UI_BTN_SIZE * 0.5 - UI_TOP);
    b.sel = false;
    if (idx == 0) { b.action = ACT_TRANSLATE; b.sel = hasSel && (transMode == TRANS_TRANSLATE || lastClick == ACT_TRANSLATE); }
    else if (idx == 1) { b.action = ACT_ROTATE; b.sel = hasSel && (transMode == TRANS_ROTATE || lastClick == ACT_ROTATE); }
    else if (idx == 2) { b.action = ACT_SCALE; b.sel = hasSel && (transMode == TRANS_SCALE || lastClick == ACT_SCALE); }
    else if (idx == 3) { b.action = ACT_FOCUS; b.sel = (lastClick == ACT_FOCUS); }
    else { b.action = ACT_DELETE; b.sel = (lastClick == ACT_DELETE); }
    return b;
}

bool insideBtn(vec2 p, Btn b) {
    vec2 d = abs(p - b.pos) - b.siz * 0.5;
    return max(d.x, d.y) < 0.0;
}

int checkUIClick(vec2 mp, vec2 res, float transMode, bool hasSel) {
    for (int i = 0; i < 5; i++) {
        Btn b = getAddBtn(i, ACT_NONE, res);
        if (insideBtn(mp, b)) return b.action;
    }
    if (hasSel) {
        for (int i = 0; i < 5; i++) {
            Btn b = getToolBtn(i, transMode, hasSel, ACT_NONE, res);
            if (insideBtn(mp, b)) return b.action;
        }
    }
    return ACT_NONE;
}

int checkPickerClick(vec2 mp, vec2 bp) {
    vec2 p = mp - bp;
    float tw = PICKER_SV_SIZE + PICKER_GAP + PICKER_H_WIDTH;
    if (p.x >= -PICKER_PX_SIZE * 3.0 && p.x <= tw + PICKER_PX_SIZE * 3.0 &&
        p.y >= PICKER_SV_SIZE && p.y <= PICKER_SV_SIZE + PICKER_TITLE_H + PICKER_PX_SIZE * 4.0)
        return 3;
    if (p.x >= 0.0 && p.x <= PICKER_SV_SIZE && p.y >= 0.0 && p.y <= PICKER_SV_SIZE)
        return 1;
    float hs = PICKER_SV_SIZE + PICKER_GAP;
    if (p.x >= hs && p.x <= hs + PICKER_H_WIDTH && p.y >= 0.0 && p.y <= PICKER_SV_SIZE)
        return 2;
    return 0;
}

vec3 getPickerHSV(vec2 mp, vec2 bp, vec3 cur, float md) {
    vec2 p = mp - bp;
    vec3 hsv = cur;
    if (md == MODE_PICKER_SV) {
        hsv.y = clamp(p.x / PICKER_SV_SIZE, 0.0, 1.0);
        hsv.z = clamp(p.y / PICKER_SV_SIZE, 0.0, 1.0);
    } else if (md == MODE_PICKER_H) {
        hsv.x = clamp(p.y / PICKER_SV_SIZE, 0.0, 1.0);
    }
    return hsv;
}

//=============================================================================
// 像素艺术图标
//=============================================================================

vec4 iconTranslate(vec2 p, bool isSel) {
    float ps = UI_PX_SIZE;
    vec2 pc = floor(p / ps + vec2(8.0));
    vec3 c = vec3(0.0);
    float a = 0.0;
    if ((pc.y == 7.0 || pc.y == 8.0) && pc.x >= 2.0 && pc.x <= 13.0) { c = PX_BLUE; a = 1.0; }
    if ((pc.x == 7.0 || pc.x == 8.0) && pc.y >= 2.0 && pc.y <= 13.0) { c = PX_BLUE; a = 1.0; }
    if (pc.x == 14.0 && (pc.y == 7.0 || pc.y == 8.0)) { c = PX_RED; a = 1.0; }
    if (pc.x == 13.0 && (pc.y == 6.0 || pc.y == 9.0)) { c = PX_RED; a = 1.0; }
    if (pc.x == 1.0  && (pc.y == 7.0 || pc.y == 8.0)) { c = PX_RED; a = 1.0; }
    if (pc.x == 2.0  && (pc.y == 6.0 || pc.y == 9.0)) { c = PX_RED; a = 1.0; }
    if (pc.y == 14.0 && (pc.x == 7.0 || pc.x == 8.0)) { c = PX_RED; a = 1.0; }
    if (pc.y == 13.0 && (pc.x == 6.0 || pc.x == 9.0)) { c = PX_RED; a = 1.0; }
    if (pc.y == 1.0  && (pc.x == 7.0 || pc.x == 8.0)) { c = PX_RED; a = 1.0; }
    if (pc.y == 2.0  && (pc.x == 6.0 || pc.x == 9.0)) { c = PX_RED; a = 1.0; }
    if (isSel) c = mix(c, PX_WHITE, 0.3);
    return vec4(c, a);
}

vec4 iconRotate(vec2 p, bool isSel) {
    float ps = UI_PX_SIZE;
    vec2 pc = floor(p / ps + vec2(8.0));
    vec3 c = vec3(0.0);
    float a = 0.0;
    vec2 ctr = vec2(7.5);
    float d = length(pc - ctr);
    if (d >= 5.0 && d <= 7.0) {
        vec2 dir = pc - ctr;
        float ang = atan(dir.y, dir.x);
        if (ang < PI * 0.75 || ang > -PI * 0.5) { c = PX_BLUE; a = 1.0; }
    }
    if (a > 0.0 && mod(pc.x + pc.y, 3.0) > 1.5) c = PX_RED;
    if (pc.x >= 10.0 && pc.x <= 12.0 && pc.y >= 10.0 && pc.y <= 12.0) { c = PX_RED; a = 1.0; }
    if (isSel) c = mix(c, PX_WHITE, 0.3);
    return vec4(c, a);
}

vec4 iconScale(vec2 p, bool isSel) {
    float ps = UI_PX_SIZE;
    vec2 pc = floor(p / ps + vec2(8.0));
    vec3 c = vec3(0.0);
    float a = 0.0;
    if (abs(pc.x - pc.y) <= 1.0 && pc.x >= 3.0 && pc.x <= 12.0) { c = PX_BLUE; a = 1.0; if (mod(pc.x, 3.0) > 1.5) c = PX_RED; }
    if (pc.x >= 2.0 && pc.x <= 5.0 && pc.y >= 2.0 && pc.y <= 5.0) { c = PX_RED; a = 1.0; }
    if (pc.x >= 10.0 && pc.x <= 13.0 && pc.y >= 10.0 && pc.y <= 13.0) { c = PX_ORANGE; a = 1.0; }
    if (isSel) c = mix(c, PX_WHITE, 0.3);
    return vec4(c, a);
}

vec4 iconFocus(vec2 p, bool isSel) {
    float ps = UI_PX_SIZE;
    vec2 pc = floor(p / ps + vec2(8.0));
    vec3 c = vec3(0.0);
    float a = 0.0;
    if (pc.x >= 4.0 && pc.x <= 5.0 && pc.y >= 3.0 && pc.y <= 12.0) { c = PX_BLUE; a = 1.0; }
    if (pc.y >= 11.0 && pc.y <= 12.0 && pc.x >= 4.0 && pc.x <= 11.0) { c = PX_BLUE; a = 1.0; }
    if (pc.y >= 7.0 && pc.y <= 8.0 && pc.x >= 4.0 && pc.x <= 9.0) { c = PX_RED; a = 1.0; }
    if (isSel) c = mix(c, PX_WHITE, 0.3);
    return vec4(c, a);
}

vec4 iconSphere(vec2 p, bool isSel) {
    float ps = UI_PX_SIZE;
    vec2 pc = floor(p / ps + vec2(8.0));
    vec3 c = vec3(0.0);
    float a = 0.0;
    vec2 ctr = vec2(7.5);
    float d = length(pc - ctr);
    if (d <= 6.0) { c = PX_ORANGE; a = 1.0; if (d <= 3.0 && pc.x < 7.0 && pc.y > 8.0) c = mix(c, PX_WHITE, 0.4); if (pc.x > 9.0 && pc.y < 6.0) c = c * 0.7; }
    if (d > 5.5 && d <= 6.5) { c = PX_ORANGE * 0.6; a = 1.0; }
    if (isSel) c = mix(c, PX_WHITE, 0.3);
    return vec4(c, a);
}

vec4 iconBox(vec2 p, bool isSel) {
    float ps = UI_PX_SIZE;
    vec2 pc = floor(p / ps + vec2(8.0));
    vec3 c = vec3(0.0);
    float a = 0.0;
    if (pc.x >= 4.0 && pc.x <= 11.0 && pc.y >= 9.0 && pc.y <= 13.0) { c = PX_BLUE; a = 1.0; }
    if (pc.x >= 4.0 && pc.x <= 9.0 && pc.y >= 3.0 && pc.y <= 9.0) { c = PX_BLUE * 0.7; a = 1.0; }
    if (pc.x >= 9.0 && pc.x <= 13.0 && pc.y >= 5.0 && pc.y <= 11.0) { c = PX_BLUE * 0.5; a = 1.0; }
    if (isSel) c = mix(c, PX_WHITE, 0.3);
    return vec4(c, a);
}

vec4 iconPointLight(vec2 p, bool isSel) {
    float ps = UI_PX_SIZE;
    vec2 pc = floor(p / ps + vec2(8.0));
    vec3 c = vec3(0.0);
    float a = 0.0;
    vec2 ctr = vec2(7.5);
    float d = length(pc - ctr);
    if (d <= 3.0) { c = PX_YELLOW; a = 1.0; }
    for (int i = 0; i < 8; i++) {
        float ang = float(i) * PI / 4.0;
        vec2 dir = vec2(cos(ang), sin(ang));
        for (float dd = 4.5; dd <= 6.5; dd += 1.0) {
            vec2 rp = ctr + dir * dd;
            if (length(pc - floor(rp)) < 1.0) { c = PX_YELLOW; a = 1.0; }
        }
    }
    if (isSel) c = mix(c, PX_WHITE, 0.3);
    return vec4(c, a);
}

vec4 iconSpotLight(vec2 p, bool isSel) {
    float ps = UI_PX_SIZE;
    vec2 pc = floor(p / ps + vec2(8.0));
    vec3 c = vec3(0.0);
    float a = 0.0;
    if (pc.y >= 11.0 && pc.y <= 13.0 && pc.x >= 6.0 && pc.x <= 9.0) { c = PX_YELLOW; a = 1.0; }
    float cw = (13.0 - pc.y) * 0.6;
    if (pc.y >= 2.0 && pc.y <= 11.0) {
        if (abs(pc.x - 7.5) <= cw && abs(abs(pc.x - 7.5) - cw) < 1.0) { c = PX_ORANGE; a = 1.0; }
    }
    if (pc.y == 2.0 && pc.x >= 3.0 && pc.x <= 12.0) { c = PX_ORANGE; a = 1.0; }
    if (isSel) c = mix(c, PX_WHITE, 0.3);
    return vec4(c, a);
}

vec4 iconAreaLight(vec2 p, bool isSel) {
    float ps = UI_PX_SIZE;
    vec2 pc = floor(p / ps + vec2(8.0));
    vec3 c = vec3(0.0);
    float a = 0.0;
    if (pc.x >= 2.0 && pc.x <= 13.0 && pc.y >= 6.0 && pc.y <= 13.0) {
        bool border = pc.x == 2.0 || pc.x == 13.0 || pc.y == 6.0 || pc.y == 13.0;
        c = border ? PX_CYAN * 0.7 : PX_CYAN;
        a = 1.0;
    }
    if (pc.y >= 2.0 && pc.y <= 5.0) {
        if (pc.x == 4.0 || pc.x == 7.0 || pc.x == 8.0 || pc.x == 11.0) { c = PX_CYAN * 0.5; a = 1.0; }
    }
    if (isSel) c = mix(c, PX_WHITE, 0.3);
    return vec4(c, a);
}

vec4 iconDelete(vec2 p, bool isSel) {
    float ps = UI_PX_SIZE;
    vec2 pc = floor(p / ps + vec2(8.0));
    vec3 c = vec3(0.0);
    float a = 0.0;
    float d1 = abs(pc.x - pc.y);
    float d2 = abs(pc.x - (15.0 - pc.y));
    if ((d1 <= 1.5 || d2 <= 1.5) && pc.x >= 3.0 && pc.x <= 12.0 && pc.y >= 3.0 && pc.y <= 12.0) { c = PX_RED; a = 1.0; }
    if (isSel) c = mix(c, PX_WHITE, 0.3);
    return vec4(c, a);
}

vec4 getIcon(int act, vec2 p, bool isSel) {
    if (act == ACT_TRANSLATE) return iconTranslate(p, isSel);
    if (act == ACT_ROTATE) return iconRotate(p, isSel);
    if (act == ACT_SCALE) return iconScale(p, isSel);
    if (act == ACT_FOCUS) return iconFocus(p, isSel);
    if (act == ACT_SPHERE) return iconSphere(p, isSel);
    if (act == ACT_BOX) return iconBox(p, isSel);
    if (act == ACT_PT_LIGHT) return iconPointLight(p, isSel);
    if (act == ACT_SPOT_LIGHT) return iconSpotLight(p, isSel);
    if (act == ACT_AREA_LIGHT) return iconAreaLight(p, isSel);
    if (act == ACT_DELETE) return iconDelete(p, isSel);
    return vec4(0.0);
}

vec4 drawBtn(vec2 fc, Btn b, float time) {
    vec2 p = fc - b.pos;
    float ps = UI_PX_SIZE;
    vec2 hs = b.siz * 0.5;
    if (abs(p.x) > hs.x + ps || abs(p.y) > hs.y + ps) return vec4(0.0);
    vec4 r = vec4(0.0);
    if (abs(p.x) < hs.x && abs(p.y) < hs.y) {
        r = vec4(b.sel ? UI_BG_COL_LT : UI_BG_COL, 1.0);
    }
    float ba = drawCorner(p, b.siz, ps);
    if (ba > 0.0) r = vec4(b.sel ? UI_HL_COL : UI_BORDER_COL, 1.0);
    vec2 pc = floor((p + hs) / ps);
    vec2 mp = floor(b.siz / ps);
    if ((pc.y == 0.0 || pc.y == mp.y - 1.0) && pc.x > 2.0 && pc.x < mp.x - 3.0) {
        if (mod(pc.x, 3.0) < 2.0) r = vec4(b.sel ? UI_HL_COL : UI_BORDER_COL, 1.0);
    }
    if ((pc.x == 0.0 || pc.x == mp.x - 1.0) && pc.y > 2.0 && pc.y < mp.y - 3.0) {
        if (mod(pc.y, 3.0) < 2.0) r = vec4(b.sel ? UI_HL_COL : UI_BORDER_COL, 1.0);
    }
    vec4 icon = getIcon(b.action, p, b.sel);
    if (icon.a > 0.0) r = vec4(icon.rgb, 1.0);
    if (b.sel && r.a > 0.0) r.rgb += vec3(0.05) * sin(time * 2.0);
    return r;
}

vec3 renderUI(vec2 fc, vec2 res, int selId, float transMode, float time, int lastClick) {
    vec4 ui = vec4(0.0);
    bool hasSel = selId >= 0;
    for (int i = 0; i < 5; i++) {
        Btn b = getAddBtn(i, lastClick, res);
        vec4 bc = drawBtn(fc, b, time);
        if (bc.a > ui.a) ui = bc;
    }
    if (hasSel) {
        for (int i = 0; i < 5; i++) {
            Btn b = getToolBtn(i, transMode, hasSel, lastClick, res);
            vec4 bc = drawBtn(fc, b, time);
            if (bc.a > ui.a) ui = bc;
        }
    }
    return ui.rgb * ui.a;
}

float getUIAlpha(vec2 fc, vec2 res, int selId, float transMode) {
    bool hasSel = selId >= 0;
    float ps = UI_PX_SIZE;
    for (int i = 0; i < 5; i++) {
        Btn b = getAddBtn(i, ACT_NONE, res);
        vec2 d = abs(fc - b.pos) - b.siz * 0.5 - vec2(ps);
        if (max(d.x, d.y) < 0.0) return 1.0;
    }
    if (hasSel) {
        for (int i = 0; i < 5; i++) {
            Btn b = getToolBtn(i, transMode, hasSel, ACT_NONE, res);
            vec2 d = abs(fc - b.pos) - b.siz * 0.5 - vec2(ps);
            if (max(d.x, d.y) < 0.0) return 1.0;
        }
    }
    return 0.0;
}

//=============================================================================
// 颜色选择器渲染
//=============================================================================

vec4 renderPicker(vec2 fc, vec2 bp, vec3 hsv, bool show) {
    if (!show) return vec4(0.0);
    float ps = PICKER_PX_SIZE;
    vec2 p = fc - bp;
    float tw = PICKER_SV_SIZE + PICKER_GAP + PICKER_H_WIDTH;
    float th = PICKER_SV_SIZE;
    if (p.x < -ps * 4.0 || p.x > tw + ps * 4.0 || p.y < -ps * 8.0 || p.y > th + PICKER_TITLE_H + ps * 6.0)
        return vec4(0.0);
    vec4 r = vec4(0.0);
    float bgd = sdBox2D(p - vec2(tw * 0.5, th * 0.5 + PICKER_TITLE_H * 0.3),
                        vec2(tw * 0.5 + ps * 3.0, th * 0.5 + ps * 5.0 + PICKER_TITLE_H * 0.5));
    if (bgd < 0.0) {
        r = vec4(UI_BG_COL, 0.95);
        if (bgd > -ps * 2.0) {
            vec2 pc = floor((p + vec2(ps * 3.0, ps * 5.0)) / ps);
            if (mod(pc.x + pc.y, 3.0) < 2.0) r.rgb = UI_BORDER_COL;
        }
    }
    if (p.y >= th && p.y <= th + PICKER_TITLE_H + ps * 2.0 && p.x >= -ps * 2.0 && p.x <= tw + ps * 2.0) {
        r = vec4(UI_BORDER_COL * 0.8, 1.0);
        vec2 tp = floor((p - vec2(0.0, th)) / ps);
        if (mod(tp.x, 4.0) < 2.0 && tp.y >= 2.0 && tp.y <= 5.0) r.rgb = UI_HL_COL;
    }
    if (p.x >= 0.0 && p.x <= PICKER_SV_SIZE && p.y >= 0.0 && p.y <= PICKER_SV_SIZE) {
        vec2 sv = clamp(p / PICKER_SV_SIZE, 0.0, 1.0);
        r = vec4(hsv2rgb(vec3(hsv.x, sv.x, sv.y)), 1.0);
        float bd = sdBox2D(p - vec2(PICKER_SV_SIZE * 0.5), vec2(PICKER_SV_SIZE * 0.5));
        if (bd > -ps) r.rgb = UI_BORDER_COL;
    }
    vec2 svm = vec2(hsv.y, hsv.z) * PICKER_SV_SIZE;
    vec2 md = p - svm;
    if ((abs(md.x) < ps && abs(md.y) < ps * 5.0) || (abs(md.y) < ps && abs(md.x) < ps * 5.0)) r = vec4(PX_WHITE, 1.0);
    if ((abs(md.x) < ps * 0.5 && abs(md.y) < ps * 4.0) || (abs(md.y) < ps * 0.5 && abs(md.x) < ps * 4.0)) r = vec4(0.0, 0.0, 0.0, 1.0);
    if (abs(md.x) < ps && abs(md.y) < ps) r = vec4(hsv2rgb(hsv), 1.0);
    float hst = PICKER_SV_SIZE + PICKER_GAP;
    if (p.x >= hst && p.x <= hst + PICKER_H_WIDTH && p.y >= 0.0 && p.y <= PICKER_SV_SIZE) {
        float hc = clamp(p.y / PICKER_SV_SIZE, 0.0, 1.0);
        r = vec4(hsv2rgb(vec3(hc, 1.0, 1.0)), 1.0);
        float hbd = sdBox2D(p - vec2(hst + PICKER_H_WIDTH * 0.5, PICKER_SV_SIZE * 0.5),
                            vec2(PICKER_H_WIDTH * 0.5, PICKER_SV_SIZE * 0.5));
        if (hbd > -ps) r.rgb = UI_BORDER_COL;
    }
    float hmy = hsv.x * PICKER_SV_SIZE;
    if (abs(p.y - hmy) < ps * 2.0) {
        if (p.x >= hst - ps * 3.0 && p.x < hst) r = vec4(PX_WHITE, 1.0);
        if (p.x > hst + PICKER_H_WIDTH && p.x <= hst + PICKER_H_WIDTH + ps * 3.0) r = vec4(PX_WHITE, 1.0);
    }
    if (abs(p.y - hmy) < ps) {
        if (p.x >= hst - ps * 2.0 && p.x < hst - ps) r = vec4(0.0, 0.0, 0.0, 1.0);
        if (p.x > hst + PICKER_H_WIDTH + ps && p.x <= hst + PICKER_H_WIDTH + ps * 2.0) r = vec4(0.0, 0.0, 0.0, 1.0);
    }
    vec2 pvc = vec2(PICKER_SV_SIZE * 0.5, -ps * 5.0);
    float pvd = sdBox2D(p - pvc, vec2(PICKER_SV_SIZE * 0.5, ps * 3.0));
    if (pvd < 0.0) {
        r = vec4(hsv2rgb(hsv), 1.0);
        if (pvd > -ps) r.rgb = UI_BORDER_COL;
    }
    return r;
}

//=============================================================================
// 坐标轴指示器
//=============================================================================

float drawChar(vec2 p, int c) {
    if (p.x < 0.0 || p.y < 0.0 || p.x > 5.0 || p.y > 5.0) return 0.0;
    int x = int(p.x);
    int y = int(p.y);
    if (x > 4 || y > 4) return 0.0;
    int pat[15];
    pat[0] = 0x11; pat[1] = 0x0A; pat[2] = 0x04; pat[3] = 0x0A; pat[4] = 0x11;
    pat[5] = 0x11; pat[6] = 0x0A; pat[7] = 0x04; pat[8] = 0x04; pat[9] = 0x04;
    pat[10] = 0x1F; pat[11] = 0x02; pat[12] = 0x04; pat[13] = 0x08; pat[14] = 0x1F;
    return ((pat[c * 5 + (4 - y)] >> (4 - x)) & 1) == 1 ? 1.0 : 0.0;
}

float lineDist2D(vec2 p, vec2 a, vec2 b) {
    vec2 pa = p - a;
    vec2 ba = b - a;
    return length(pa - ba * clamp(dot(pa, ba) / dot(ba, ba), 0.0, 1.0));
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
    int ord[3];
    ord[0] = 0; ord[1] = 1; ord[2] = 2;
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2 - i; j++) {
            if (dp[ord[j]] > dp[ord[j + 1]]) {
                int t = ord[j];
                ord[j] = ord[j + 1];
                ord[j + 1] = t;
            }
        }
    }
    vec4 r = vec4(0.0);
    for (int k = 0; k < 3; k++) {
        int i = ord[k];
        vec2 ep = AXIS_IND_CENTER + sp[i] * AXIS_IND_SIZE;
        float d = lineDist2D(fc, AXIS_IND_CENTER, ep);
        if (d < AXIS_IND_LINE_W) {
            r = vec4(axisCol(i), smoothstep(AXIS_IND_LINE_W, AXIS_IND_LINE_W * 0.5, d));
        }
        if (length(sp[i]) > 0.001) {
            vec2 lc = ep + normalize(sp[i]) * AXIS_IND_LABEL_OFF;
            if (drawChar(fc - lc + vec2(2.0), i) > 0.0) {
                r = vec4(axisCol(i), 1.0);
            }
        }
    }
    return r;
}

//=============================================================================
// FPS 显示
//=============================================================================

float sampleDigit(int n, vec2 uv) {
    if (uv.x < 0.0 || uv.y < 0.0 || uv.x >= 1.0 || uv.y >= 1.0) return 0.0;
    int font[10];
    font[0] = 0x75557; font[1] = 0x22222; font[2] = 0x74717; font[3] = 0x74747; font[4] = 0x11574;
    font[5] = 0x71747; font[6] = 0x71757; font[7] = 0x74444; font[8] = 0x75757; font[9] = 0x75747;
    ivec2 ip = ivec2(floor(uv * vec2(4.0, 5.0)));
    return float((font[n] >> (ip.x + ip.y * 4)) & 1);
}

float printInt(vec2 uv, float val) {
    float md = 1.0 + ceil(log2(max(1.0, val)) / log2(10.0));
    float did = floor(uv.x);
    if (did >= 0.0 && did < md) {
        float dv = mod(floor(val / pow(10.0, md - 1.0 - did)), 10.0);
        return sampleDigit(int(dv), vec2(fract(uv.x), uv.y));
    }
    return 0.0;
}

vec3 renderFPS(vec2 fc, vec2 res, float fps) {
    vec2 uv = fc / res.y;
    float v = printInt((uv - vec2(0.01, 0.98)) * 64.0, fps);
    return vec3(v * 0.9, v * 0.95, v * 0.8);
}

//=============================================================================
// 色调映射 - ACES
//=============================================================================

const mat3 ACESInputMat = mat3(
    0.59719, 0.35458, 0.04823,
    0.07600, 0.90834, 0.01566,
    0.02840, 0.13383, 0.83777
);

const mat3 ACESOutputMat = mat3(
     1.60475, -0.53108, -0.07367,
    -0.10208,  1.10813, -0.00605,
    -0.00327, -0.07276,  1.07602
);

vec3 RRTAndODTFit(vec3 v) {
    vec3 a = v * (v + 0.0245786) - 0.000090537;
    vec3 b = v * (0.983729 * v + 0.4329510) + 0.238081;
    return a / b;
}

vec3 ACESFitted(vec3 color) {
    color = color * ACESInputMat;
    color = RRTAndODTFit(color);
    color = color * ACESOutputMat;
    color = clamp(color, 0.0, 1.0);
    return color;
}

float linear_srgb(float x) {
    return mix(1.055 * pow(x, 1.0/2.4) - 0.055, 12.92 * x, step(x, 0.0031308));
}

vec3 linear_srgb(vec3 x) {
    return mix(1.055 * pow(x, vec3(1.0/2.4)) - 0.055, 12.92 * x, step(x, vec3(0.0031308)));
}

float srgb_linear(float x) {
    return mix(pow((x + 0.055) / 1.055, 2.4), x / 12.92, step(x, 0.04045));
}

vec3 srgb_linear(vec3 x) {
    return mix(pow((x + 0.055) / 1.055, vec3(2.4)), x / 12.92, step(x, vec3(0.04045)));
}

//=============================================================================
// LPV 系统配置 (Light Propagation Volumes)
//=============================================================================

const ivec3 lpvsizei = ivec3(32);
const vec3 lpvsize = vec3(lpvsizei);

#define LPV_BOUNDS_MIN vec3(-2.6, -2.6, -2.6)
#define LPV_BOUNDS_MAX vec3( 2.6,  2.6,  2.6)
#define LPV_BOUNDS_SIZE (LPV_BOUNDS_MAX - LPV_BOUNDS_MIN)

float packfragcoord2(vec2 p, vec2 s) {
    return floor(p.y) * s.x + p.x;
}

vec2 unpackfragcoord2(float p, vec2 s) {
    float x = mod(p, s.x);
    float y = (p - x) / s.x + 0.5;
    return vec2(x, y);
}

ivec2 unpackfragcoord2(int p, ivec2 s) {
    int x = p % s.x;
    int y = (p - x) / s.x;
    return ivec2(x, y);
}

float packfragcoord3(vec3 p, vec3 s) {
    return floor(p.z) * s.x * s.y + floor(p.y) * s.x + p.x;
}

int packfragcoord3(ivec3 p, ivec3 s) {
    return p.z * s.x * s.y + p.y * s.x + p.x;
}

vec3 unpackfragcoord3(float p, vec3 s) {
    float x = mod(p, s.x);
    float y = mod((p - x) / s.x, s.y);
    float z = (p - x - floor(y) * s.x) / (s.x * s.y);
    return vec3(x, y + 0.5, z + 0.5);
}

vec4 sh_project(vec3 n) {
    return vec4(n, sqrt(1.0/3.0));
}

float sh_dot(vec4 a, vec4 b) {
    return max(dot(a, b), 0.0);
}

const float m3div4pi = 3.0 / (4.0 * PI);

float sh_flux(float d) {
    return d * m3div4pi;
}

float sh_shade(vec4 vL, vec4 vN) {
    return sh_flux(sh_dot(vL, vN));
}

#define SHSharpness 0.7
vec4 sh_irradiance_probe(vec4 v) {
    const float sh_c0 = (2.0 - SHSharpness) * 1.0;
    const float sh_c1 = SHSharpness * 2.0 / 3.0;
    return vec4(v.xyz * sh_c1, v.w * sh_c0);
}

float shade_probe(vec4 sh, vec4 shn) {
    return sh_shade(sh_irradiance_probe(sh), shn);
}

vec3 worldToVoxelNorm(vec3 p) {
    return (p - LPV_BOUNDS_MIN) / LPV_BOUNDS_SIZE;
}
