// ============================================================================
// YuRenderLab v0.11 - Buffer A (几何体素生成 / Occlusion Volume)
// 原始通道映射:
//   iChannel0: Buffer D (场景数据)
// ============================================================================

#iChannel0 "file://buffer_d.glsl"

#include "common.glsl"

void mainImage( out vec4 fragColor, in vec2 fragCoord )
{
    int cnt = getObjCount(iChannel0);
    
    float posidx = packfragcoord2(fragCoord.xy, iResolution.xy);
    float totalVoxels = lpvsize.x * lpvsize.y * lpvsize.z;
    
    if (posidx >= totalVoxels) {
        fragColor = vec4(0.0);
        return;
    }
    
    vec3 voxelCoord = unpackfragcoord3(posidx, lpvsize);
    
    vec3 uvw = voxelCoord / lpvsize;
    vec3 wpos = uvw * LPV_BOUNDS_SIZE + LPV_BOUNDS_MIN;
    
    float voxelSize = LPV_BOUNDS_SIZE.x / lpvsize.x;
    float r = voxelSize * 0.87 * 1.3;
    
    float d = sdSceneShadow(wpos, iChannel0, cnt);
    
    if (d < r) {
        float opacity = clamp((1.0 - d / r) * 1.5, 0.0, 1.0);
        
        vec3 n = calcNormal(wpos, iChannel0, cnt);
        
        fragColor = sh_project(n) * opacity;
    } else {
        fragColor = vec4(0.0);
    }
    
}
