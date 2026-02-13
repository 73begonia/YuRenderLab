// ============================================================================
// YuRenderLab v0.11 - Image (主渲染入口)
// 原始通道映射:
//   iChannel0: Buffer D (State)
//   iChannel1: Buffer C (LPV 光照结果)
// ============================================================================

#iChannel0 "file://buffer_d.glsl"

#iChannel1 "file://buffer_c.glsl"
#iChannel1::MinFilter "Linear"
#iChannel1::MagFilter "Linear"

#include "common.glsl"

// ============================================
// LPV 采样与计算函数
// ============================================

// 采样指定通道的 SH 数据
vec4 fetch_lpv_raw(ivec3 p, int ch, sampler2D lpvBuf, vec2 res) {
    p = clamp(p, ivec3(0), lpvsizei - 1);
    float numvoxels = lpvsize.x * lpvsize.y * lpvsize.z;
    float posidx = packfragcoord3(vec3(p), lpvsize) + float(ch) * numvoxels;
    vec2 uv = unpackfragcoord2(posidx, res) / res;
    return texture(lpvBuf, uv); 
}

// 获取某位置的 RGB 光照（使用 SH 重建）
vec3 fetch_lpv_rgb(ivec3 p, vec4 shn, sampler2D lpvBuf, vec2 res) {
    vec4 shr = fetch_lpv_raw(p, 0, lpvBuf, res);
    vec4 shg = fetch_lpv_raw(p, 1, lpvBuf, res);
    vec4 shb = fetch_lpv_raw(p, 2, lpvBuf, res);
    return vec3(
        shade_probe(shr, shn),
        shade_probe(shg, shn),
        shade_probe(shb, shn));
}

// 三线性插值采样 LPV
vec3 sample_lpv_trilin(vec3 wpos, vec4 shn, sampler2D lpvBuf, vec2 res) {
    vec3 uvw = worldToVoxelNorm(wpos);
    
    if (uvw.x < 0.0 || uvw.x > 1.0 || uvw.y < 0.0 || uvw.y > 1.0 || uvw.z < 0.0 || uvw.z > 1.0) 
        return vec3(0.0);
        
    vec3 pf = uvw * lpvsize - 0.5;
    ivec3 p = ivec3(floor(pf));
    vec3 w = fract(pf);
    
    ivec3 p111 = p + ivec3(1);
    
    vec3 c000 = fetch_lpv_rgb(ivec3(p.x,    p.y,    p.z),    shn, lpvBuf, res);
    vec3 c100 = fetch_lpv_rgb(ivec3(p111.x, p.y,    p.z),    shn, lpvBuf, res);
    vec3 c010 = fetch_lpv_rgb(ivec3(p.x,    p111.y, p.z),    shn, lpvBuf, res);
    vec3 c110 = fetch_lpv_rgb(ivec3(p111.x, p111.y, p.z),    shn, lpvBuf, res);
    vec3 c001 = fetch_lpv_rgb(ivec3(p.x,    p.y,    p111.z), shn, lpvBuf, res);
    vec3 c101 = fetch_lpv_rgb(ivec3(p111.x, p.y,    p111.z), shn, lpvBuf, res);
    vec3 c011 = fetch_lpv_rgb(ivec3(p.x,    p111.y, p111.z), shn, lpvBuf, res);
    vec3 c111 = fetch_lpv_rgb(ivec3(p111.x, p111.y, p111.z), shn, lpvBuf, res);

    return mix(
        mix(mix(c000, c100, w.x), mix(c010, c110, w.x), w.y),
        mix(mix(c001, c101, w.x), mix(c011, c111, w.x), w.y),
        w.z
    );
}

// 屏幕空间环境光遮蔽 (SSAO)
float calcSSAO(vec3 p, vec3 n, sampler2D buf, int cnt) {
    float occ = 0.0;
    float sca = 1.0;
    
    for(int i = 0; i < 5; i++) {
        float h = 0.05 + 0.12 * float(i) / 4.0;
        vec3 sp = p + n * h;
        float d = sdScene(sp, buf, cnt, false).d;
        occ += (h - d) * sca;
        sca *= 0.95;
    }
    
    return clamp(1.0 - 0.9 * occ, 0.0, 1.0);
}

// 结合直接光、LPV 间接光和 SSAO 的光照计算
vec3 calcLightingWithSSAOAndLPV(Hit h, vec3 camPos, sampler2D buf, int cnt, sampler2D lpvBuf, vec2 lpvRes, float ssao) {
    vec3 directLight = calcLighting(h, camPos, buf, cnt);
    
    if (h.id == ID_NONE) return directLight;

    float voxelWorldSize = (LPV_BOUNDS_SIZE.x / float(lpvsizei.x));
    float normalBias = voxelWorldSize * 0.5;
    
    vec4 shn = sh_project(h.n); 
    vec3 indirectLight = sample_lpv_trilin(h.p + h.n * normalBias, shn, lpvBuf, lpvRes);
    
    float aoFactor = mix(0.1, 1.0, ssao); 
    
    vec3 ambientComponent = indirectLight * h.col * 1.5 * aoFactor;
    vec3 directComponent = directLight * mix(0.6, 1.0, ssao);
    
    return directComponent + ambientComponent;
}

//=============================================================================
// 主渲染入口
//=============================================================================

vec4 renderScene(vec2 fc, vec2 res, float time, sampler2D buf, sampler2D lpvBuf, float fps) {
    vec3 camDir = ld(buf, 2, ROW_CAMERA).xyz;
    vec3 camPos = ld(buf, 3, ROW_CAMERA).xyz;
    if (length(camDir) < 0.001) camDir = vec3(0.0, 0.0, 1.0);

    vec4 selData = ld(buf, 0, ROW_SELECT);
    int selId = int(selData.x);
    float transMode = selData.z;
    int actPart = int(selData.w);

    int cnt = getObjCount(buf);
    vec3 pickerHSV = ld(buf, 0, ROW_PICKER).xyz;
    vec2 pickerPos = ld(buf, 1, ROW_PICKER).xy;
    int lastClick = int(ld(buf, 0, ROW_UI).x);

    float asp = res.x / res.y;
    float fov = radians(CAM_FOV);
    Ray ray = createRay(fc / res, camPos, camDir, fov, asp);

    // 1. SDF光线步进渲染场景
    Hit sceneHit = rayMarch(ray, buf, cnt, false);
    vec3 col = vec3(0.0);

    if (sceneHit.id != ID_NONE) {
        // 2. 计算 SSAO
        float occ = calcSSAO(sceneHit.p, sceneHit.n, buf, cnt);
        
        // 3. 计算综合光照 (Direct + LPV + SSAO)
        // [VS Code 适配] iChannelResolution[1].xy → iResolution.xy
        col = calcLightingWithSSAOAndLPV(sceneHit, camPos, buf, cnt, lpvBuf, iResolution.xy, occ);
    } else {
        col = vec3(0.02, 0.02, 0.05);
    }
    
    // 4. ACES 色调映射 + sRGB gamma
    col = linear_srgb(ACESFitted(col));

    // 5. 解析几何轮廓检测
    vec2 outlineInfo = detectOutline(fc, res, camPos, camDir, selId, sceneHit, buf, cnt);
    if (outlineInfo.x > 0.5) {
        col = outlineInfo.y > 0.5 ? OUTLINE_COL_OCC : OUTLINE_COL;
    }

    // 6. 解析几何光源线框
    float pxScalar = 1.0 / res.y; 
    for (int i = 0; i < MAX_OBJECTS && i < cnt; i++) {
        int typ; vec3 pos, siz;
        loadObjQuick(buf, i, typ, pos, siz);
        if (!isLightTyp(typ)) continue;

        Obj o = loadObj(buf, i);
        WireHit wh = traceLightWire(ray, o, pxScalar);
        if (wh.alpha > 0.0) {
            bool occ = sceneHit.id != ID_NONE && sceneHit.t < wh.t;
            vec3 wc = wh.col;
            if (selId == i) wc = AXIS_COL_HL;
            if (occ) wc = wc * GZ_OCCLUDED_BRIGHT;
            col = mix(col, wc, wh.alpha);
        }
    }

    // 7. 解析几何 Gizmo 渲染
    if (selId >= 0 && selId < cnt) {
        Obj selObj = loadObj(buf, selId);
        if (selObj.typ != OBJ_NONE) {
            float gsc = gizmoScl(selObj.pos, camPos);
            bool isLit = isLightTyp(selObj.typ);

            GizmoCtx ctx;
            ctx.center = selObj.pos;
            ctx.camPos = camPos;
            ctx.transMode = transMode;
            ctx.actPart = actPart;
            ctx.quat = selObj.quat;
            ctx.gscale = gsc;
            ctx.isLit = isLit;

            GizmoHit gh = traceGizmo(ray, ctx, pxScalar);
            if (gh.t < INF) {
                bool occ = sceneHit.id != ID_NONE && sceneHit.t < gh.t;
                float alpha = occ ? gh.alpha * 0.2 : gh.alpha; 
                vec3 gc = occ ? gh.col * 0.5 : gh.col; 
                col = mix(col, gc, alpha);
            }
        }
    }

    // 8. UI 渲染
    vec3 uiCol = renderUI(fc, res, selId, transMode, time, lastClick);
    float uiAlpha = getUIAlpha(fc, res, selId, transMode);
    if (uiAlpha > 0.0) col = mix(col, uiCol, uiAlpha);

    // 9. Picker
    vec4 pickerCol = renderPicker(fc, pickerPos, pickerHSV, selId >= 0);
    if (pickerCol.a > 0.0) col = mix(col, pickerCol.rgb, pickerCol.a);

    // 10. 坐标轴指示器
    vec4 axisInd = renderAxisIndicator(fc, camDir);
    if (axisInd.a > 0.0) col = mix(col, axisInd.rgb, axisInd.a);

    // 11. FPS
    vec3 fpsCol = renderFPS(fc, res, fps);
    if (fpsCol.r > 0.0 || fpsCol.g > 0.0 || fpsCol.b > 0.0) col = fpsCol;

    return vec4(col, 1.0);
}

void mainImage(out vec4 fragColor, in vec2 fragCoord) {
    // [VS Code 适配] iFrameRate 不可用，通过 iTimeDelta 计算 FPS
    float fps = 1.0 / max(iTimeDelta, 0.001);
    fragColor = renderScene(fragCoord, iResolution.xy, iTime, iChannel0, iChannel1, fps);
}
