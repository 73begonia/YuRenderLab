// ============================================================================
// YuRenderLab v0.11 - Buffer C (LPV 光照传播核心)
// 原始通道映射:
//   iChannel0: Buffer A (Geometry - 遮挡/法线)
//   iChannel1: Buffer B (Albedo - 材质颜色)
//   iChannel2: Buffer C (Self - 上一帧数据)
//   iChannel3: Buffer D (Scene Objects - 光源数据)
// ============================================================================

#iChannel0 "file://buffer_a.glsl"
#iChannel0::MinFilter "Linear"
#iChannel0::MagFilter "Linear"

#iChannel1 "file://buffer_b.glsl"
#iChannel1::MinFilter "Linear"
#iChannel1::MagFilter "Linear"

#iChannel2 "self"
#iChannel2::MinFilter "Linear"
#iChannel2::MagFilter "Linear"

#iChannel3 "file://buffer_d.glsl"

#include "common.glsl"

// ============================================================================
// 1. 基础配置与辅助函数
// ============================================================================

#define USE_LPV_OCCLUSION 1
#define USE_LPV_BOUNCE 1
#define USE_TRIQUADRATIC_INTERPOLATION 1

// LPV 全局变量
float numvoxels;
float channel;
vec3 cmix;

// 临时存储
vec4 gv4[6];
vec4 gv[8];
float bc4[6];
float bc[8];

// 立体角常量
const float solid_angle_front = 0.4006696846462392 * m3div4pi;
const float solid_angle_side = 0.4234313544367392 * m3div4pi;

// -----------------------------------------------------------
// 读取函数
// [VS Code 适配] iChannelResolution[N].xy → iResolution.xy
// 所有 Buffer 与主画面共享同一分辨率
// -----------------------------------------------------------

// 从 Buffer A 获取几何体积数据（遮挡 & 法线SH）
vec4 fetch_gv(vec3 p) {
    if ((min(p.x, min(p.y, p.z)) < 0.5) || (max(p.x, max(p.y, p.z)) > (lpvsize.x - 0.5))) return vec4(0.0);
    float posidx = packfragcoord3(p, lpvsize);
    vec2 uv = unpackfragcoord2(posidx, iResolution.xy) / iResolution.xy;
    return texture(iChannel0, uv);
}

// 从 Buffer B 获取反照率数据（获取当前处理通道的颜色分量）
float fetch_av_comp(vec3 p) {
    if ((min(p.x, min(p.y, p.z)) < 0.5) || (max(p.x, max(p.y, p.z)) > (lpvsize.x - 0.5))) return 0.0;
    float posidx = packfragcoord3(p, lpvsize);
    vec2 uv = unpackfragcoord2(posidx, iResolution.xy) / iResolution.xy;
    // 只返回当前正在处理的通道 (R, G 或 B)
    return dot(texture(iChannel1, uv).rgb, cmix);
}

// 从 Buffer C 自身获取上一帧的 LPV 数据
vec4 fetch_lpv(vec3 p) {
    if ((min(p.x, min(p.y, p.z)) < 0.5) || (max(p.x, max(p.y, p.z)) > (lpvsize.x - 0.5))) return vec4(0.0);
    float posidx = packfragcoord3(p, lpvsize) + channel * numvoxels;
    vec2 uv = unpackfragcoord2(posidx, iResolution.xy) / iResolution.xy;
    return texture(iChannel2, uv);
}

// -----------------------------------------------------------
// 光照计算辅助 (复用 Common 逻辑的简化版)
// -----------------------------------------------------------

float distAtten_LPV(float d, float range) {
    float a = 1.0 / max(d * d, 0.01);
    float w = clamp(1.0 - pow(d / range, 4.0), 0.0, 1.0);
    return a * w * w;
}

// ============================================================================
// 2. 传播计算核心 (Propagation)
// ============================================================================

vec4 accum_face(vec4 shcoeffs, int i, int j, int dim, int face_dim, 
                vec3 p, vec3 offset, vec3 face_offset,
                vec4 gvcoeffs, vec4 gvrefcoeffs, float gvrefcolor) {
    if (i == j) return vec4(0.0);

    vec3 dirw = normalize(face_offset - offset);
    float solid_angle = (dim == face_dim) ? solid_angle_front : solid_angle_side;
    
    vec4 outdirsh = sh_project(dirw);
    vec4 indirsh = outdirsh;
    vec4 invindirsh = sh_project(-dirw);
    
    float influx = sh_dot(shcoeffs, indirsh) * solid_angle;
    
    #if USE_LPV_OCCLUSION
    float occluded = sh_dot(gvcoeffs, indirsh);
    #else
    float occluded = 0.0;
    #endif
    
    float outflux = influx * (1.0 - occluded);
    vec4 result = outdirsh * outflux; 
    
    #if USE_LPV_BOUNCE
    vec4 rvec = gvrefcoeffs;
    float reflected = outflux * sh_dot(rvec, invindirsh);
    if (reflected > 0.0) {
        result += rvec * (reflected * gvrefcolor);
    }
    #endif    
    
    return result;
}

vec4 sample_neighbors(int i, int dim, vec3 p, vec3 offset, vec4 gvcoeffs) {
    vec4 shcoeffs = fetch_lpv(p + offset);
    vec4 shsumcoeffs = vec4(0.0);
    vec3 e = vec3(-0.5, 0.5, 0.0);
    shsumcoeffs += accum_face(shcoeffs, i, 0, dim, 2, p, offset, e.zzx, gvcoeffs, gv4[0], bc4[0]);
    shsumcoeffs += accum_face(shcoeffs, i, 1, dim, 2, p, offset, e.zzy, gvcoeffs, gv4[1], bc4[1]);
    shsumcoeffs += accum_face(shcoeffs, i, 2, dim, 1, p, offset, e.zxz, gvcoeffs, gv4[2], bc4[2]);
    shsumcoeffs += accum_face(shcoeffs, i, 3, dim, 1, p, offset, e.zyz, gvcoeffs, gv4[3], bc4[3]);
    shsumcoeffs += accum_face(shcoeffs, i, 4, dim, 0, p, offset, e.xzz, gvcoeffs, gv4[4], bc4[4]);
    shsumcoeffs += accum_face(shcoeffs, i, 5, dim, 0, p, offset, e.yzz, gvcoeffs, gv4[5], bc4[5]);
    return shsumcoeffs;
}

// ============================================================================
// 3. 主函数
// ============================================================================

void mainImage(out vec4 fragColor, in vec2 fragCoord)
{
    // 计算索引和通道 (Buffer C 把 R,G,B 三个通道的数据平铺存储)
    float posidx = packfragcoord2(fragCoord.xy, iResolution.xy);
    numvoxels = lpvsize.x * lpvsize.y * lpvsize.z;
    channel = floor(posidx / numvoxels);
    posidx -= channel * numvoxels;
    
    // RGB 掩码
    cmix = vec3(float(channel == 0.0), float(channel == 1.0), float(channel == 2.0));
    
    if (posidx >= numvoxels) {
        fragColor = vec4(0.0);
        return;
    }

    vec3 pos = unpackfragcoord3(posidx, lpvsize);
    vec3 uvw = (pos - 0.5) / lpvsize;
    vec3 wpos = uvw * LPV_BOUNDS_SIZE + LPV_BOUNDS_MIN;
    float voxelSize = LPV_BOUNDS_SIZE.x / lpvsize.x;

    // ------------------------------------------------------------------------
    // Step 1: 光源注入 (Light Injection)
    // ------------------------------------------------------------------------
    
    vec4 injectedLightSH = vec4(0.0);
    
    vec4 gv_data = fetch_gv(pos);
    float geometryDensity = length(gv_data.xyz);
    
    if (geometryDensity > 0.01) {
        vec3 N = normalize(gv_data.xyz);
        float albedoComp = fetch_av_comp(pos);
        
        int objCnt = getObjCount(iChannel3);
        float totalDirectFlux = 0.0;
        
        for (int i = 0; i < MAX_OBJECTS && i < objCnt; i++) {
            int typ; vec3 lpos, lsiz;
            loadObjQuick(iChannel3, i, typ, lpos, lsiz);
            
            if (!isLightTyp(typ)) continue;
            
            Obj lt = loadObj(iChannel3, i);
            vec3 lCol = lt.col;
            float lightComp = dot(lCol, cmix);
            
            if (typ == OBJ_POINT_LIGHT) {
                vec3 L = lpos - wpos;
                float d = length(L);
                L = normalize(L);
                float range = LT_BASE_RANGE * lt.siz.x;
                
                if (d < range) {
                    float atten = distAtten_LPV(d, range) * LT_PT_INTENSITY;
                    float NdotL = max(dot(N, L), 0.0);
                    totalDirectFlux += lightComp * atten * NdotL;
                }
            }
            else if (typ == OBJ_SPOT_LIGHT) {
                vec3 L = lpos - wpos;
                float d = length(L);
                L = normalize(L);
                float range = LT_BASE_RANGE * lt.siz.x * 1.5;
                
                if (d < range) {
                    vec3 sd = quatRot(lt.quat, vec3(0.0, -1.0, 0.0));
                    float ca = dot(-L, sd);
                    float oc = cos(LT_SPOT_OUTER * 0.5);
                    float ic = cos(LT_SPOT_INNER * 0.5);
                    
                    if (ca > oc) {
                        float spotAtten = smoothstep(oc, ic, ca);
                        float atten = distAtten_LPV(d, range) * LT_SPOT_INTENSITY * spotAtten;
                        float NdotL = max(dot(N, L), 0.0);
                        totalDirectFlux += lightComp * atten * NdotL;
                    }
                }
            }
            else if (typ == OBJ_AREA_LIGHT) {
                vec3 L = lpos - wpos;
                float d = length(L);
                L = normalize(L);
                float range = LT_BASE_RANGE * lt.siz.x * 1.5;
                
                if (d < range) {
                    vec3 ln = quatRot(lt.quat, vec3(0.0, 0.0, -1.0));
                    float lf = max(dot(ln, -L), 0.0);
                    
                    if (lf > 0.0) {
                        float atten = distAtten_LPV(d, range) * LT_AREA_INTENSITY * lf;
                        float NdotL = max(dot(N, L), 0.0);
                        totalDirectFlux += lightComp * atten * NdotL;
                    }
                }
            }
        }
        
        float injectionMult = 4.0; 
        injectedLightSH = sh_project(N) * totalDirectFlux * albedoComp * injectionMult;
    }

    // ------------------------------------------------------------------------
    // Step 2: 光线传播 (Propagation)
    // ------------------------------------------------------------------------
    
    vec4 propagatedSH = vec4(0.0);
    vec3 e = vec3(-1.0, 1.0, 0.0);
    
    #if USE_LPV_OCCLUSION || USE_LPV_BOUNCE
    vec2 w = vec2(0.0, 1.0);
    gv[0] = fetch_gv(pos + w.xxx); gv[1] = fetch_gv(pos + w.xxy);
    gv[2] = fetch_gv(pos + w.xyx); gv[3] = fetch_gv(pos + w.xyy);
    gv[4] = fetch_gv(pos + w.yxx); gv[5] = fetch_gv(pos + w.yxy);
    gv[6] = fetch_gv(pos + w.yyx); gv[7] = fetch_gv(pos + w.yyy);

    #if USE_LPV_BOUNCE
    bc[0] = fetch_av_comp(pos + w.xxx); bc[1] = fetch_av_comp(pos + w.xxy);
    bc[2] = fetch_av_comp(pos + w.xyx); bc[3] = fetch_av_comp(pos + w.xyy);
    bc[4] = fetch_av_comp(pos + w.yxx); bc[5] = fetch_av_comp(pos + w.yxy);
    bc[6] = fetch_av_comp(pos + w.yyx); bc[7] = fetch_av_comp(pos + w.yyy);
    #endif    

    gv4[0] = (gv[0] + gv[1] + gv[2] + gv[3]) * 0.25;
    gv4[1] = (gv[4] + gv[5] + gv[6] + gv[7]) * 0.25;
    gv4[2] = (gv[0] + gv[4] + gv[1] + gv[5]) * 0.25;
    gv4[3] = (gv[2] + gv[6] + gv[3] + gv[7]) * 0.25;
    gv4[4] = (gv[0] + gv[2] + gv[4] + gv[6]) * 0.25;
    gv4[5] = (gv[1] + gv[3] + gv[5] + gv[7]) * 0.25;

    #if USE_LPV_BOUNCE
    bc4[0] = (bc[0] + bc[1] + bc[2] + bc[3]) * 0.25;
    bc4[1] = (bc[4] + bc[5] + bc[6] + bc[7]) * 0.25;
    bc4[2] = (bc[0] + bc[4] + bc[1] + bc[5]) * 0.25;
    bc4[3] = (bc[2] + bc[6] + bc[3] + bc[7]) * 0.25;
    bc4[4] = (bc[0] + bc[2] + bc[4] + bc[6]) * 0.25;
    bc4[5] = (bc[1] + bc[3] + bc[5] + bc[7]) * 0.25;
    #endif
    #endif 
    
    propagatedSH += sample_neighbors(0, 2, pos, e.zzx, gv4[0]);
    propagatedSH += sample_neighbors(1, 2, pos, e.zzy, gv4[1]);
    propagatedSH += sample_neighbors(2, 1, pos, e.zxz, gv4[2]);
    propagatedSH += sample_neighbors(3, 1, pos, e.zyz, gv4[3]);
    propagatedSH += sample_neighbors(4, 0, pos, e.xzz, gv4[4]);
    propagatedSH += sample_neighbors(5, 0, pos, e.yzz, gv4[5]);

    // ------------------------------------------------------------------------
    // Step 3: 合并结果
    // ------------------------------------------------------------------------
    fragColor = injectedLightSH + propagatedSH * 0.95; 
}
