#iChannel0 "self"
#iKeyboard

#include "common.glsl"

void mainImage(out vec4 fragColor, in vec2 fragCoord) 
{
    ivec2 px = ivec2(fragCoord);

    if (px.y != ROW_CAMERA || px.x > 3)
    {
        fragColor = vec4(0.0);
        return;
    }

    if (iFrame == 0)
    {
        if (px.x == 0)      fragColor = vec4(0.0);
        else if (px.x == 1) fragColor = vec4(0.0, 0.0, 0.0, 0.0);
        else if (px.x == 2) fragColor = vec4(CAM_INIT_DIR, 0.0);
        else                fragColor = vec4(CAM_INIT_POS, 0.0);
        return;
    }

    vec4 slot0 = ld(iChannel0, 0, ROW_CAMERA);
    vec4 slot1 = ld(iChannel0, 1, ROW_CAMERA);
    vec3 camDir = ld(iChannel0, 2, ROW_CAMERA).xyz;
    vec3 camPos = ld(iChannel0, 3, ROW_CAMERA).xyz;

    float prevDown  = slot0.z;
    vec2 dragStart  = slot0.xy;
    vec2 angles     = slot1.xy;
    vec2 dragAngles = slot1.zw;

    bool mdown = iMouse.z > 0.0;
    bool mpressed = mdown && prevDown < 0.5;

    if (mpressed)
    {
        dragStart = iMouse.xy;
        dragAngles = angles;
    }

    if (mdown)
    {
        vec2 delta = iMouse.xy - dragStart;
        angles.y = dragAngles.y - delta.x / iResolution.x * CAM_SENS_X;
        angles.x = dragAngles.x + delta.y / iResolution.y * CAM_SENS_Y;
        angles.x = clamp(angles.x, -PI*0.49, PI*0.49);
    }

    camDir = angles2dir(angles);

    vec3 fwd = normalize(camDir);
    vec3 rt  = normalize(cross(fwd, vec3(0.0, 1.0, 0.0)));
    if (length(cross(fwd, vec3(0.0, 1.0, 0.0))) < 0.001)
        rt = vec3(1.0, 0.0, 0.0);
    
    float spd = CAM_SPEED;
    if (keyDown(iKeyboard, KEY_SHIFT)) spd *= CAM_SPRINT;

    if (keyDown(iKeyboard, KEY_W))     camPos += fwd * spd;
    if (keyDown(iKeyboard, KEY_S))     camPos -= fwd * spd;
    if (keyDown(iKeyboard, KEY_A))     camPos -= rt  * spd;
    if (keyDown(iKeyboard, KEY_D))     camPos += rt  * spd;
    if (keyDown(iKeyboard, KEY_SPACE)) camPos.y += spd;
    if (keyDown(iKeyboard, KEY_CTRL))  camPos.y -= spd;

    if (px.x == 0)      fragColor = vec4(dragStart, mdown ? 1.0 : 0.0, 0.0);
    else if (px.x == 1) fragColor = vec4(angles, dragAngles);
    else if (px.x == 2) fragColor = vec4(camDir, 0.0);
    else                fragColor = vec4(camPos, 0.0);
}

