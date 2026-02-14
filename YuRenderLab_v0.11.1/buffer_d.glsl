#iChannel0 "self"
#iKeyboard

#include "common.glsl"

//=============================================================================
// Scene picking (analytical intersection for object selection)
//=============================================================================

int pickSceneObject(Ray r, sampler2D buf) {
    float bestT = INF;
    int bestId = -1;

    // Test sphere
    vec3 spherePos = ld(buf, 0, ROW_OBJECTS).xyz;
    float ts;
    if (intersectSphere(r, spherePos, SPHERE_RADIUS, ts) && ts < bestT) {
        bestT = ts;
        bestId = OBJ_SPHERE_ID;
    }

    // Test tall box (rotated)
    vec3 tallboxPos = ld(buf, 1, ROW_OBJECTS).xyz;
    vec3 localRo = r.ro - tallboxPos;
    vec3 localRd = r.rd;
    float cRot = cos(TALLBOX_ROT_ANGLE), sRot = sin(TALLBOX_ROT_ANGLE);
    localRo.xz = mat2(cRot, sRot, -sRot, cRot) * localRo.xz;
    localRd.xz = mat2(cRot, sRot, -sRot, cRot) * localRd.xz;
    float tb;
    if (intersectBox(Ray(localRo, localRd), vec3(0.0), TALLBOX_HALFSIZE, tb) && tb < bestT) {
        bestT = tb;
        bestId = OBJ_TALLBOX_ID;
    }

    return bestId;
}

//=============================================================================
// Gizmo 交互处理
//=============================================================================

bool procGizmoInp(InpState inp, vec2 res, int selId, float md, float transMode, int actPart,
                  vec3 camPos, vec3 camDir, vec2 dragStart, sampler2D buf, inout IxResult r) {
    if (selId < 0) return false;

    vec3 objPos = ld(buf, selId, ROW_OBJECTS).xyz;
    float gsc = gizmoScl(objPos, camPos);

    // Currently dragging
    if (md == MODE_TRANSFORM) {
        if (inp.mdown) {
            Ray curRay = createPickRay(inp.mpos, res, camPos, camDir);

            vec3 dVal = ld(buf, 0, ROW_DRAG).xyz;
            vec3 dPlane = ld(buf, 1, ROW_DRAG).xyz;

            r.flags = r.flags | CHG_OBJECT;
            r.targetId = selId;

            // Translation logic
            vec3 newPos;
            if (actPart == PART_CENTER || actPart == PART_NONE) {
                // Move along camera view plane
                newPos = dVal + rayPlane(camPos, curRay.rd, dPlane, -camDir) - dPlane;
            } else {
                // Move along single axis
                vec3 ax = axisVec(actPart - PART_X);
                Ray startRay = createPickRay(dragStart, res, camPos, camDir);
                newPos = dVal + ax * dot(
                    projectAxis(curRay, dPlane, ax, camDir) -
                    projectAxis(startRay, dPlane, ax, camDir), ax);
            }
            r.objPos = inp.kctrl ? snap3(newPos, TF_SNAP_TRANS) : newPos;
            return true;
        }
        return false;
    }

    // Try to pick gizmo on mouse press
    if (inp.mpressed && !inp.kshift) {
        Ray pickRay = createPickRay(inp.mpos, res, camPos, camDir);

        float hitT;
        int hitPart = pickGizmo(pickRay, objPos, gsc, hitT);

        if (hitPart != PART_NONE) {
            r.flags = r.flags | CHG_SELECTION | CHG_CAMERA | CHG_DRAG;

            r.selId = selId;
            r.imode = MODE_TRANSFORM;
            r.transMode = transMode;
            r.actPart = hitPart;

            r.dragStart = inp.mpos;
            r.camPos = camPos;
            r.camDir = camDir;
            r.angles = ld(buf, 1, ROW_CAMERA).xy;
            r.dragAngles = ld(buf, 1, ROW_CAMERA).zw;

            r.dragPlane = objPos;
            r.dragVal = objPos;

            return true;
        }
    }

    return false;
}

bool procSceneInp(InpState inp, vec2 res, int selId, float transMode,
                  vec3 camPos, vec3 camDir, sampler2D buf, inout IxResult r) {
    if (!inp.mpressed) return false;
    if (inp.kshift) return false;

    Ray pickRay = createPickRay(inp.mpos, res, camPos, camDir);
    int hitId = pickSceneObject(pickRay, buf);

    if (hitId >= 0) {
        if (hitId == selId) {
            // Click on already selected object → start center drag
            vec3 objPos = ld(buf, selId, ROW_OBJECTS).xyz;

            r.flags = r.flags | CHG_SELECTION | CHG_CAMERA | CHG_DRAG;
            r.selId = selId;
            r.imode = MODE_TRANSFORM;
            r.transMode = transMode;
            r.actPart = PART_CENTER;

            r.dragStart = inp.mpos;
            r.camPos = camPos;
            r.camDir = camDir;
            r.angles = ld(buf, 1, ROW_CAMERA).xy;
            r.dragAngles = ld(buf, 1, ROW_CAMERA).zw;

            r.dragPlane = objPos;
            r.dragVal = objPos;
        } else {
            // Select new object
            r.flags = r.flags | CHG_SELECTION;
            r.selId = hitId;
            r.imode = MODE_NONE;
            r.transMode = transMode;
            r.actPart = PART_NONE;
        }
        return true;
    }

    return false;
}

//=============================================================================
// 相机交互处理
//=============================================================================

void procCameraInp(InpState inp, vec2 res, int selId, float md, float transMode,
                   vec2 angles, vec2 dragAngles, vec2 dragStart,
                   vec3 camPos, vec3 camDir, sampler2D kb, sampler2D buf, inout IxResult r) {

    // 当前处于相机旋转模式
    if (md == MODE_CAMERA) {
        if (inp.mdown) {
            vec2 delta = inp.mpos - dragStart;
            vec2 newAng;
            newAng.y = dragAngles.y - delta.x / res.x * CAM_SENS_X;
            newAng.x = clamp(dragAngles.x + delta.y / res.y * CAM_SENS_Y, -PI * 0.49, PI * 0.49);

            r.flags = r.flags | CHG_CAMERA;
            r.angles = newAng;
            r.camDir = angles2dir(newAng);
            r.camPos = camPos;
            r.dragStart = dragStart;
            r.dragAngles = dragAngles;
        }
        return;
    }

    // 按下鼠标 → 取消选择 + 开始相机旋转
    if (inp.mpressed) {
        r.flags = r.flags | CHG_SELECTION | CHG_CAMERA;
        r.selId = -1;
        r.imode = MODE_CAMERA;
        r.transMode = transMode;
        r.actPart = PART_NONE;
        r.dragStart = inp.mpos;
        r.dragAngles = angles;
        r.camPos = camPos;
        r.camDir = camDir;
        r.angles = angles;
    }
}

void procCameraMove(InpState inp, vec3 camPos, vec3 camDir, vec2 angles, sampler2D kb, sampler2D buf, inout IxResult r) {
    float spd = CAM_SPEED;
    if (inp.kshift) spd *= CAM_SPRINT;

    vec3 fwd = normalize(camDir);
    vec3 rt = normalize(cross(fwd, vec3(0.0, 1.0, 0.0)));
    if (length(cross(fwd, vec3(0.0, 1.0, 0.0))) < 0.001) {
        rt = vec3(1.0, 0.0, 0.0);
    }

    vec3 newPos = camPos;
    bool moved = false;

    if (keyDown(kb, KEY_W)) { newPos = newPos + fwd * spd; moved = true; }
    if (keyDown(kb, KEY_S)) { newPos = newPos - fwd * spd; moved = true; }
    if (keyDown(kb, KEY_A)) { newPos = newPos - rt * spd; moved = true; }
    if (keyDown(kb, KEY_D)) { newPos = newPos + rt * spd; moved = true; }
    if (keyDown(kb, KEY_SPACE)) { newPos.y = newPos.y + spd; moved = true; }
    if (keyDown(kb, KEY_CTRL)) { newPos.y = newPos.y - spd; moved = true; }

    if (moved) {
        r.flags = r.flags | CHG_CAMERA;
        r.camPos = newPos;
        r.camDir = camDir;
        r.angles = angles;
        r.dragStart = ld(buf, 0, ROW_CAMERA).xy;
        r.dragAngles = ld(buf, 1, ROW_CAMERA).zw;
    }
}

//=============================================================================
// 主交互处理
//=============================================================================

IxResult processInteraction(InpState inp, vec4 mouse, vec2 res, int frame, sampler2D buf, sampler2D kb) {
    IxResult r = initIxResult();

    vec4 mouseState = ld(buf, 0, ROW_CAMERA);
    vec4 anglesData = ld(buf, 1, ROW_CAMERA);
    vec3 camDir = ld(buf, 2, ROW_CAMERA).xyz;
    vec3 camPos = ld(buf, 3, ROW_CAMERA).xyz;
    vec4 stateData = ld(buf, 0, ROW_STATE);

    float md = stateData.x;     // interaction mode
    int selId = int(stateData.y);  // selected object ID
    float transMode = stateData.z; // transform mode
    int actPart = int(stateData.w); // active gizmo part
    vec2 angles = anglesData.xy;
    vec2 dragAngles = anglesData.zw;
    vec2 dragStart = mouseState.xy;

    if (length(camDir) < 0.001) {
        camDir = CAM_INIT_DIR;
    }

    // 键盘移动相机
    procCameraMove(inp, camPos, camDir, angles, kb, buf, r);

    if ((r.flags & CHG_CAMERA) != 0u) {
        camPos = r.camPos;
    }

    // 鼠标释放时退出当前模式（保持选择）
    if (inp.mreleased) {
        if (md == MODE_CAMERA) {
            r.flags = r.flags | CHG_CAMERA;
            r.camPos = camPos;
            r.camDir = camDir;
            r.angles = angles;
            r.dragStart = dragStart;
            r.dragAngles = angles;
        }
        r.flags = r.flags | CHG_SELECTION;
        r.selId = selId;        // keep selection
        r.imode = MODE_NONE;
        r.transMode = transMode; // keep transform mode
        r.actPart = PART_NONE;  // clear active part
        return r;
    }

    // 分层交互优先级：Gizmo > 场景选择 > 相机
    if (procGizmoInp(inp, res, selId, md, transMode, actPart, camPos, camDir, dragStart, buf, r)) return r;
    if (procSceneInp(inp, res, selId, transMode, camPos, camDir, buf, r)) return r;
    procCameraInp(inp, res, selId, md, transMode, angles, dragAngles, dragStart, camPos, camDir, kb, buf, r);

    return r;
}

//=============================================================================
// 状态初始化
//=============================================================================

vec4 initState(ivec2 px, vec2 res) {
    if (px.y == ROW_CAMERA) {
        if (px.x == 0) return vec4(0.0);
        if (px.x == 1) return vec4(0.0, 0.0, 0.0, 0.0);
        if (px.x == 2) return vec4(CAM_INIT_DIR, 0.0);
        if (px.x == 3) return vec4(CAM_INIT_POS, 0.0);
    }
    if (px.y == ROW_STATE && px.x == 0) {
        // vec4(imode, selId, transMode, actPart)
        return vec4(MODE_NONE, -1.0, TRANS_TRANSLATE, float(PART_NONE));
    }
    if (px.y == ROW_OBJECTS) {
        if (px.x == OBJ_SPHERE_ID) return vec4(SPHERE_INIT_POS, 0.0);
        if (px.x == OBJ_TALLBOX_ID) return vec4(TALLBOX_INIT_POS, 0.0);
    }
    if (px.y == ROW_DRAG) {
        return vec4(0.0);
    }
    return vec4(0.0);
}

//=============================================================================
// 应用交互结果
//=============================================================================

vec4 applyIxResult(ivec2 px, IxResult r, InpState inp, sampler2D buf) {

    if (px.y == ROW_CAMERA) {
        vec4 oldMouse = ld(buf, 0, ROW_CAMERA);
        vec4 oldAngs = ld(buf, 1, ROW_CAMERA);
        vec4 oldDir = ld(buf, 2, ROW_CAMERA);
        vec4 oldPos = ld(buf, 3, ROW_CAMERA);

        if (px.x == 0) {
            vec2 ds = (r.flags & CHG_CAMERA) != 0u ? r.dragStart : oldMouse.xy;
            return vec4(ds, inp.mdown ? 1.0 : 0.0, 0.0);
        }
        if (px.x == 1) {
            vec2 angs = (r.flags & CHG_CAMERA) != 0u ? r.angles : oldAngs.xy;
            vec2 dAngs = (r.flags & CHG_CAMERA) != 0u ? r.dragAngles : oldAngs.zw;
            return vec4(angs, dAngs);
        }
        if (px.x == 2) {
            vec3 dir = (r.flags & CHG_CAMERA) != 0u ? r.camDir : oldDir.xyz;
            return vec4(dir, 0.0);
        }
        if (px.x == 3) {
            vec3 pos = (r.flags & CHG_CAMERA) != 0u ? r.camPos : oldPos.xyz;
            return vec4(pos, 0.0);
        }
    }

    if (px.y == ROW_STATE && px.x == 0) {
        if ((r.flags & CHG_SELECTION) != 0u) {
            return vec4(r.imode, float(r.selId), r.transMode, float(r.actPart));
        }
        return ld(buf, 0, ROW_STATE);
    }

    if (px.y == ROW_OBJECTS) {
        if ((r.flags & CHG_OBJECT) != 0u && r.targetId >= 0 && px.x == r.targetId) {
            return vec4(r.objPos, 0.0);
        }
        return ld(buf, px.x, ROW_OBJECTS);
    }

    if (px.y == ROW_DRAG) {
        if ((r.flags & CHG_DRAG) != 0u) {
            if (px.x == 0) return vec4(r.dragVal, 0.0);
            if (px.x == 1) return vec4(r.dragPlane, 0.0);
        }
        return ld(buf, px.x, ROW_DRAG);
    }

    return vec4(0.0);
}

//=============================================================================
// 主入口
//=============================================================================

void mainImage(out vec4 fragColor, in vec2 fragCoord) {
    ivec2 px = ivec2(fragCoord);
    vec2 res = iResolution.xy;

    // 只处理相关行
    if (px.y > ROW_DRAG) {
        fragColor = vec4(0.0);
        return;
    }

    if (iFrame == 0) {
        fragColor = initState(px, res);
        return;
    }

    vec4 mouseState = ld(iChannel0, 0, ROW_CAMERA);
    float prevDown = mouseState.z;

    InpState inp = gatherInp(iKeyboard, iMouse, prevDown);

    IxResult r = processInteraction(inp, iMouse, res, iFrame, iChannel0, iKeyboard);

    fragColor = applyIxResult(px, r, inp, iChannel0);
}

