#iChannel0 "self"
#iKeyboard

#include "common.glsl"

//=============================================================================
// UI 交互处理 (MagicaCSG Style Toolbar)
//=============================================================================

bool procUIInp(InpState inp, vec2 res, int selId, float transMode,
               vec3 camPos, vec3 camDir, sampler2D buf, inout IxResult r) {
    if (!inp.mpressed) return false;

    int action = checkToolbarClick(inp.mpos, res);
    if (action == ACT_NONE) return false;

    if (action == ACT_TRANSLATE || action == ACT_ROTATE || action == ACT_SCALE) {
        float newMode = TRANS_TRANSLATE;
        if (action == ACT_ROTATE) newMode = TRANS_ROTATE;
        if (action == ACT_SCALE)  newMode = TRANS_SCALE;

        r.flags = r.flags | CHG_SELECTION;
        r.selId = selId;
        r.imode = MODE_NONE;
        r.transMode = newMode;
        r.actPart = PART_NONE;
        return true;
    }

    if (action == ACT_FOCUS && selId >= 0) {
        vec3 objPos = ld(buf, selId, ROW_OBJECTS).xyz;
        if (length(camDir) < 0.001) camDir = vec3(0.0, 0.0, 1.0);
        vec3 focusPos = objPos - normalize(camDir) * CAM_FOCUS_DIST;

        r.flags = r.flags | CHG_CAMERA;
        r.camPos = focusPos;
        r.camDir = camDir;
        r.angles = ld(buf, 1, ROW_CAMERA).xy;
        r.dragAngles = ld(buf, 1, ROW_CAMERA).zw;
        r.dragStart = ld(buf, 0, ROW_CAMERA).xy;
        return true;
    }

    // Click was on toolbar but no valid action → consume click to block passthrough
    return true;
}

//=============================================================================
// Property Panel & Slider 交互处理
//=============================================================================

bool procPanelInp(InpState inp, vec2 res, int selId, float md,
                  vec3 camPos, vec3 camDir, sampler2D buf, inout IxResult r) {
    // Load current material state
    vec4 matData0 = ld(buf, 0, ROW_MATERIAL);
    vec4 matData1 = ld(buf, 1, ROW_MATERIAL);
    float exposure  = matData0.x;
    float sunAngle  = matData0.y;
    float roughness = matData0.z;
    float metalness = matData0.w;
    vec3  hsv       = matData1.xyz;

    vec4 panelData = ld(buf, 0, ROW_PANEL);
    int curSlider = int(panelData.x);
    float sliderDragOrig = panelData.y;

    // Currently dragging a slider
    if (md == MODE_SLIDER) {
        if (inp.mdown && curSlider != SLIDER_NONE) {
            SliderRect sr = getSliderById(curSlider, res, selId, exposure, sunAngle, roughness, metalness);
            float newVal = sliderDragValue(inp.mpos, sr);
            r.flags = r.flags | CHG_MATERIAL | CHG_PANEL;
            r.exposure = exposure;
            r.sunAngle = sunAngle;
            r.roughness = roughness;
            r.metalness = metalness;
            r.hsv = hsv;
            r.sliderId = curSlider;
            r.sliderDragStart = sliderDragOrig;
            if (curSlider == SLIDER_EXPOSURE)  r.exposure  = newVal;
            if (curSlider == SLIDER_SUN)       r.sunAngle  = newVal;
            if (curSlider == SLIDER_ROUGHNESS) r.roughness = newVal;
            if (curSlider == SLIDER_METALNESS) r.metalness = newVal;
            return true;
        }
        return false;
    }

    // Try to click a slider on mouse press
    if (inp.mpressed) {
        bool hasSel = (selId >= 0);
        if (insidePanel(inp.mpos, res, hasSel)) {
            int hit = checkPanelSliderClick(inp.mpos, res, selId, exposure, sunAngle, roughness, metalness);
            if (hit != SLIDER_NONE) {
                SliderRect sr = getSliderById(hit, res, selId, exposure, sunAngle, roughness, metalness);
                r.flags = r.flags | CHG_SELECTION | CHG_PANEL | CHG_MATERIAL;
                r.selId = selId;
                r.imode = MODE_SLIDER;
                r.transMode = ld(buf, 0, ROW_STATE).z;
                r.actPart = PART_NONE;
                r.sliderId = hit;
                r.sliderDragStart = sr.val;
                r.exposure = exposure;
                r.sunAngle = sunAngle;
                r.roughness = roughness;
                r.metalness = metalness;
                r.hsv = hsv;
                return true;
            }
            // Click inside panel but not on slider → consume to block passthrough
            return true;
        }
    }
    return false;
}

//=============================================================================
// Color Picker 交互处理
//=============================================================================

bool procPickerInp(InpState inp, vec2 res, int selId, float md,
                   sampler2D buf, inout IxResult r) {
    if (selId < 0) return false;  // picker only shown when object selected

    vec4 matData0 = ld(buf, 0, ROW_MATERIAL);
    vec4 matData1 = ld(buf, 1, ROW_MATERIAL);
    float exposure  = matData0.x;
    float sunAngle  = matData0.y;
    float roughness = matData0.z;
    float metalness = matData0.w;
    vec3  hsv       = matData1.xyz;

    // Currently dragging in picker
    if (md == MODE_PICKER_SV || md == MODE_PICKER_H) {
        if (inp.mdown) {
            vec3 newHSV = getPickerHSV(inp.mpos, res, hsv, md == MODE_PICKER_SV ? MODE_PICKER_SV : MODE_PICKER_H);
            r.flags = r.flags | CHG_MATERIAL;
            r.hsv = newHSV;
            r.exposure = exposure;
            r.sunAngle = sunAngle;
            r.roughness = roughness;
            r.metalness = metalness;
            return true;
        }
        return false;
    }

    // Try to click picker on mouse press
    if (inp.mpressed && insidePicker(inp.mpos, res)) {
        int region = checkPickerClick(inp.mpos, res);
        if (region == 1) {
            // SV area click
            vec3 newHSV = getPickerHSV(inp.mpos, res, hsv, MODE_PICKER_SV);
            r.flags = r.flags | CHG_SELECTION | CHG_MATERIAL;
            r.selId = selId;
            r.imode = MODE_PICKER_SV;
            r.transMode = ld(buf, 0, ROW_STATE).z;
            r.actPart = PART_NONE;
            r.hsv = newHSV;
            r.exposure = exposure;
            r.sunAngle = sunAngle;
            r.roughness = roughness;
            r.metalness = metalness;
            return true;
        }
        if (region == 2) {
            // Hue bar click
            vec3 newHSV = getPickerHSV(inp.mpos, res, hsv, MODE_PICKER_H);
            r.flags = r.flags | CHG_SELECTION | CHG_MATERIAL;
            r.selId = selId;
            r.imode = MODE_PICKER_H;
            r.transMode = ld(buf, 0, ROW_STATE).z;
            r.actPart = PART_NONE;
            r.hsv = newHSV;
            r.exposure = exposure;
            r.sunAngle = sunAngle;
            r.roughness = roughness;
            r.metalness = metalness;
            return true;
        }
        // Click inside picker area but not in SV/H → consume
        return true;
    }
    return false;
}

//=============================================================================
// Scene picking (analytical intersection for object selection)
//=============================================================================

int pickSceneObject(Ray r, sampler2D buf) {
    float bestT = INF;
    int bestId = -1;

    // Test sphere (dynamic size)
    vec3 spherePos = ld(buf, 0, ROW_OBJECTS).xyz;
    vec3 sphereSize = ld(buf, OBJ_SPHERE_ID, ROW_SIZES).xyz;
    float sphereRad = (length(sphereSize) > 0.001) ? sphereSize.x : SPHERE_RADIUS;
    float ts;
    if (intersectSphere(r, spherePos, sphereRad, ts) && ts < bestT) {
        bestT = ts;
        bestId = OBJ_SPHERE_ID;
    }

    // Test tall box (dynamic rotation from quaternion)
    vec3 tallboxPos = ld(buf, 1, ROW_OBJECTS).xyz;
    vec4 tallboxQuat = ld(buf, OBJ_TALLBOX_ID, ROW_ROTATIONS);
    vec3 tallboxSize = ld(buf, OBJ_TALLBOX_ID, ROW_SIZES).xyz;
    if (length(tallboxQuat) < 0.001) tallboxQuat = quatAxis(vec3(0.0, 1.0, 0.0), -TALLBOX_ROT_ANGLE);
    if (length(tallboxSize) < 0.001) tallboxSize = TALLBOX_HALFSIZE;
    mat3 rotMat = quatToMat(tallboxQuat);
    mat3 invRot = transpose(rotMat);
    vec3 localRo = invRot * (r.ro - tallboxPos);
    vec3 localRd = invRot * r.rd;
    float tb;
    if (intersectBox(Ray(localRo, localRd), vec3(0.0), tallboxSize, tb) && tb < bestT) {
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
    vec4 objQuat = ld(buf, selId, ROW_ROTATIONS);
    vec3 objSiz = ld(buf, selId, ROW_SIZES).xyz;
    float gsc = gizmoScl(objPos, camPos);

    // Currently dragging
    if (md == MODE_TRANSFORM) {
        if (inp.mdown) {
            Ray curRay = createPickRay(inp.mpos, res, camPos, camDir);
            vec2 delta = inp.mpos - dragStart;

            vec3 dVal = ld(buf, 0, ROW_DRAG).xyz;
            vec3 dPlane = ld(buf, 1, ROW_DRAG).xyz;
            float dAngle = ld(buf, 2, ROW_DRAG).x;
            vec4 dQuat = ld(buf, 0, ROW_DRAG_ROT);

            r.flags = r.flags | CHG_OBJECT;
            r.targetId = selId;

            if (transMode == TRANS_TRANSLATE) {
                // Translation logic
                vec3 newPos;
                if (actPart == PART_CENTER || actPart == PART_NONE) {
                    newPos = dVal + rayPlane(camPos, curRay.rd, dPlane, -camDir) - dPlane;
                } else {
                    vec3 ax = axisVec(actPart - PART_X);
                    Ray startRay = createPickRay(dragStart, res, camPos, camDir);
                    newPos = dVal + ax * dot(
                        projectAxis(curRay, dPlane, ax, camDir) -
                        projectAxis(startRay, dPlane, ax, camDir), ax);
                }
                r.objPos = inp.kctrl ? snap3(newPos, TF_SNAP_TRANS) : newPos;
                r.objQuat = objQuat;
                r.objSiz = objSiz;
            }
            else if (transMode == TRANS_ROTATE) {
                r.objPos = objPos;
                r.objSiz = objSiz;

                if (actPart >= PART_X) {
                    int ax = actPart - PART_X;
                    float curAng = calcRotAngle(curRay.ro, curRay.rd, objPos, axisVec(ax));
                    float dAng = angleDiff(curAng, dAngle);
                    if (inp.kctrl) dAng = snap(dAng, TF_SNAP_ROT);
                    r.objQuat = normalize(quatMul(quatAxis(axisVec(ax), dAng), dQuat));
                } else {
                    // Center drag: screen-space rotation
                    float rx = delta.y / res.y * PI * TF_ROT_SENS;
                    float ry = delta.x / res.x * PI * TF_ROT_SENS;
                    if (inp.kctrl) { rx = snap(rx, TF_SNAP_ROT); ry = snap(ry, TF_SNAP_ROT); }
                    r.objQuat = applyWorldRot(dQuat, rx, ry, 0.0);
                }
            }
            else if (transMode == TRANS_SCALE) {
                r.objPos = objPos;
                r.objQuat = objQuat;

                vec3 newSiz;
                if (actPart == PART_CENTER || actPart == PART_NONE) {
                    // Uniform scale
                    newSiz = dVal * (1.0 + delta.y / res.y * TF_SCALE_SENS);
                } else {
                    // Per-axis scale
                    int ax = actPart - PART_X;
                    float d = (ax == 0) ? -delta.x / res.x * TF_SCALE_SENS : delta.y / res.y * TF_SCALE_SENS;
                    newSiz = dVal;
                    newSiz[ax] = dVal[ax] * (1.0 + d);
                }
                r.objSiz = clamp(inp.kctrl ? snap3(newSiz, TF_SNAP_SCALE) : newSiz,
                                 vec3(TF_SCALE_MIN), vec3(TF_SCALE_MAX));
            }
            return true;
        }
        return false;
    }

    // Try to pick gizmo on mouse press
    if (inp.mpressed && !inp.kshift) {
        Ray pickRay = createPickRay(inp.mpos, res, camPos, camDir);

        float hitT;
        int hitPart = pickGizmoFull(pickRay, objPos, camPos, gsc, transMode, actPart, objQuat, hitT);

        if (hitPart != PART_NONE) {
            r.flags = r.flags | CHG_SELECTION | CHG_CAMERA | CHG_DRAG | CHG_UNDO;

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
            r.dragVal = transMode == TRANS_SCALE ? objSiz : objPos;
            r.dragQuat = objQuat;

            if (transMode == TRANS_ROTATE && hitPart >= PART_X) {
                r.dragAngle = calcRotAngle(pickRay.ro, pickRay.rd, objPos, axisVec(hitPart - PART_X));
            }

            // Save undo state
            r.targetId = selId;
            r.undoPos = objPos;
            r.undoSiz = objSiz;
            r.undoQuat = objQuat;

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
            vec4 objQuat = ld(buf, selId, ROW_ROTATIONS);
            vec3 objSiz = ld(buf, selId, ROW_SIZES).xyz;

            r.flags = r.flags | CHG_SELECTION | CHG_CAMERA | CHG_DRAG | CHG_UNDO;
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
            r.dragVal = transMode == TRANS_SCALE ? objSiz : objPos;
            r.dragQuat = objQuat;

            // Save undo state
            r.targetId = selId;
            r.undoPos = objPos;
            r.undoSiz = objSiz;
            r.undoQuat = objQuat;
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

void procCameraMove(InpState inp, vec3 camPos, vec3 camDir, vec2 angles, int selId, sampler2D kb, sampler2D buf, inout IxResult r) {
    // When an object is selected, disable camera movement unless Shift is held
    // This prevents W/E/R keys from conflicting with gizmo mode switching
    bool canMove = (selId < 0) || inp.kshift;
    if (!canMove) return;

    float spd = CAM_SPEED;
    if (inp.kshift && selId < 0) spd *= CAM_SPRINT;

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
// 键盘快捷键处理 (Phase 3)
//=============================================================================

void procKeyboardInp(InpState inp, int selId, float transMode,
                     vec3 camPos, vec3 camDir, sampler2D buf, sampler2D kb, inout IxResult r) {
    vec4 prevKeys = ld(buf, 0, ROW_KEYS);
    vec4 prevKeys2 = ld(buf, 1, ROW_KEYS);

    bool kW = keyDown(kb, KEY_W);
    bool kE = keyDown(kb, KEY_E);
    bool kR = keyDown(kb, KEY_R);
    bool kF = keyDown(kb, KEY_F);
    bool kZ = keyDown(kb, KEY_Z);
    bool kC = keyDown(kb, KEY_C);
    bool kV = keyDown(kb, KEY_V);

    // "Just pressed" detection (current frame pressed, previous frame not)
    bool kWJust = kW && prevKeys.x < 0.5;
    bool kEJust = kE && prevKeys.y < 0.5;
    bool kRJust = kR && prevKeys.z < 0.5;
    bool kFJust = kF && prevKeys.w < 0.5;
    bool kZJust = kZ && prevKeys2.x < 0.5;
    bool kCJust = kC && prevKeys2.y < 0.5;
    bool kVJust = kV && prevKeys2.z < 0.5;

    // W/E/R: Switch gizmo mode (only when object selected, no modifiers)
    if (selId >= 0 && !inp.kshift && !inp.kalt && !inp.kctrl) {
        if (kWJust) {
            r.flags = r.flags | CHG_SELECTION;
            r.selId = selId;
            r.imode = MODE_NONE;
            r.transMode = TRANS_TRANSLATE;
            r.actPart = PART_NONE;
        }
        if (kEJust) {
            r.flags = r.flags | CHG_SELECTION;
            r.selId = selId;
            r.imode = MODE_NONE;
            r.transMode = TRANS_ROTATE;
            r.actPart = PART_NONE;
        }
        if (kRJust) {
            r.flags = r.flags | CHG_SELECTION;
            r.selId = selId;
            r.imode = MODE_NONE;
            r.transMode = TRANS_SCALE;
            r.actPart = PART_NONE;
        }
    }

    // F: Focus camera on selected object
    if (kFJust && selId >= 0 && !inp.kctrl) {
        vec3 objPos = ld(buf, selId, ROW_OBJECTS).xyz;
        if (length(camDir) < 0.001) camDir = vec3(0.0, 0.0, 1.0);

        vec3 focusPos = objPos - normalize(camDir) * CAM_FOCUS_DIST;

        r.flags = r.flags | CHG_CAMERA;
        r.camPos = focusPos;
        r.camDir = camDir;
        r.angles = ld(buf, 1, ROW_CAMERA).xy;
        r.dragAngles = ld(buf, 1, ROW_CAMERA).zw;
        r.dragStart = ld(buf, 0, ROW_CAMERA).xy;
    }

    // Ctrl+Z: Undo last transform
    if (inp.kctrl && kZJust) {
        vec4 undoData = ld(buf, 0, ROW_UNDO);
        int undoId = int(undoData.x);
        if (undoId >= 0 && undoId < OBJ_COUNT) {
            r.flags = r.flags | CHG_UNDO_EXEC | CHG_OBJECT;
            r.targetId = undoId;
            r.objPos = ld(buf, 1, ROW_UNDO).xyz;
            r.objSiz = ld(buf, 2, ROW_UNDO).xyz;
            r.objQuat = ld(buf, 0, ROW_UNDO_ROT);
        }
    }

    // Ctrl+C: Copy selected object's transform to clipboard
    if (inp.kctrl && kCJust && selId >= 0 && selId < OBJ_COUNT) {
        r.flags = r.flags | CHG_CLIPBOARD;
        r.clipPos = ld(buf, selId, ROW_OBJECTS).xyz;
        r.clipSiz = ld(buf, selId, ROW_SIZES).xyz;
        r.clipQuat = ld(buf, selId, ROW_ROTATIONS);
    }

    // Ctrl+V: Paste clipboard transform onto selected object
    if (inp.kctrl && kVJust && selId >= 0 && selId < OBJ_COUNT) {
        vec4 clipData = ld(buf, 0, ROW_CLIPBOARD);
        // Check clipboard is not empty (clipPos stored with valid flag in .w)
        if (clipData.w > 0.5) {
            // Save undo before paste
            r.flags = r.flags | CHG_UNDO | CHG_OBJECT;
            r.targetId = selId;
            r.undoPos = ld(buf, selId, ROW_OBJECTS).xyz;
            r.undoSiz = ld(buf, selId, ROW_SIZES).xyz;
            r.undoQuat = ld(buf, selId, ROW_ROTATIONS);
            // Apply clipboard transform
            r.objPos = clipData.xyz;
            r.objSiz = ld(buf, 1, ROW_CLIPBOARD).xyz;
            r.objQuat = ld(buf, 0, ROW_CLIP_ROT);
        }
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

    // 键盘快捷键处理 (W/E/R/F)
    procKeyboardInp(inp, selId, transMode, camPos, camDir, buf, kb, r);

    // 键盘移动相机
    procCameraMove(inp, camPos, camDir, angles, selId, kb, buf, r);

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
        if (md == MODE_SLIDER || md == MODE_PICKER_SV || md == MODE_PICKER_H) {
            // Keep material state on release
            vec4 matData0 = ld(buf, 0, ROW_MATERIAL);
            vec4 matData1 = ld(buf, 1, ROW_MATERIAL);
            r.flags = r.flags | CHG_MATERIAL;
            r.exposure = matData0.x;
            r.sunAngle = matData0.y;
            r.roughness = matData0.z;
            r.metalness = matData0.w;
            r.hsv = matData1.xyz;
        }
        r.flags = r.flags | CHG_SELECTION;
        r.selId = selId;        // keep selection
        r.imode = MODE_NONE;
        r.transMode = transMode; // keep transform mode
        r.actPart = PART_NONE;  // clear active part
        return r;
    }

    // UI 工具栏点击 (最高优先级)
    if (procUIInp(inp, res, selId, transMode, camPos, camDir, buf, r)) return r;

    // 阻止工具栏区域的鼠标穿透到 Gizmo/场景/相机
    bool onToolbar = (inp.mpressed && getToolbarAlpha(inp.mpos, res) > 0.5);

    // Property Panel / Slider 交互 (高优先级)
    if (procPanelInp(inp, res, selId, md, camPos, camDir, buf, r)) return r;

    // Color Picker 交互
    if (procPickerInp(inp, res, selId, md, buf, r)) return r;

    // 阻止面板/拾色器区域穿透
    bool hasSel = (selId >= 0);
    bool onPanel = (inp.mpressed && insidePanel(inp.mpos, res, hasSel));
    bool onPicker = (inp.mpressed && hasSel && insidePicker(inp.mpos, res));

    // 分层交互优先级：Gizmo > 场景选择 > 相机
    if (!onToolbar && !onPanel && !onPicker) {
        if (procGizmoInp(inp, res, selId, md, transMode, actPart, camPos, camDir, dragStart, buf, r)) return r;
        if (procSceneInp(inp, res, selId, transMode, camPos, camDir, buf, r)) return r;
    }
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
    if (px.y == ROW_KEYS) {
        return vec4(0.0);
    }
    if (px.y == ROW_OBJECTS) {
        if (px.x == OBJ_SPHERE_ID) return vec4(SPHERE_INIT_POS, 0.0);
        if (px.x == OBJ_TALLBOX_ID) return vec4(TALLBOX_INIT_POS, 0.0);
    }
    if (px.y == ROW_DRAG) {
        return vec4(0.0);
    }
    if (px.y == ROW_ROTATIONS) {
        if (px.x == OBJ_SPHERE_ID) return quatId();
        if (px.x == OBJ_TALLBOX_ID) return quatAxis(vec3(0.0, 1.0, 0.0), -TALLBOX_ROT_ANGLE);
    }
    if (px.y == ROW_SIZES) {
        if (px.x == OBJ_SPHERE_ID) return vec4(vec3(SPHERE_RADIUS), 0.0);
        if (px.x == OBJ_TALLBOX_ID) return vec4(TALLBOX_HALFSIZE, 0.0);
    }
    if (px.y == ROW_DRAG_ROT) {
        return quatId();
    }
    if (px.y == ROW_UNDO) {
        if (px.x == 0) return vec4(-1.0, 0.0, 0.0, 0.0); // undoId = -1 (no undo)
        return vec4(0.0);
    }
    if (px.y == ROW_UNDO_ROT) {
        return quatId();
    }
    if (px.y == ROW_CLIPBOARD) {
        return vec4(0.0); // .w = 0.0 means empty clipboard
    }
    if (px.y == ROW_CLIP_ROT) {
        return quatId();
    }
    if (px.y == ROW_MATERIAL) {
        // x=exposure, y=sunAngle, z=roughness, w=metalness
        if (px.x == 0) return vec4(1.0, 0.0, 0.5, 0.0);
        // x=h, y=s, z=v  (default white)
        if (px.x == 1) return vec4(0.0, 0.0, 1.0, 0.0);
        return vec4(0.0);
    }
    if (px.y == ROW_PANEL) {
        // x=sliderId (-1=none), y=sliderDragStart
        if (px.x == 0) return vec4(-1.0, 0.0, 0.0, 0.0);
        return vec4(0.0);
    }
    return vec4(0.0);
}

//=============================================================================
// 应用交互结果
//=============================================================================

vec4 applyIxResult(ivec2 px, IxResult r, InpState inp, sampler2D buf, sampler2D kb) {

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

    if (px.y == ROW_KEYS) {
        // Store current key states for "just pressed" detection next frame
        if (px.x == 0) {
            return vec4(
                keyDown(kb, KEY_W) ? 1.0 : 0.0,
                keyDown(kb, KEY_E) ? 1.0 : 0.0,
                keyDown(kb, KEY_R) ? 1.0 : 0.0,
                keyDown(kb, KEY_F) ? 1.0 : 0.0
            );
        }
        if (px.x == 1) {
            return vec4(
                keyDown(kb, KEY_Z) ? 1.0 : 0.0,
                keyDown(kb, KEY_C) ? 1.0 : 0.0,
                keyDown(kb, KEY_V) ? 1.0 : 0.0,
                0.0
            );
        }
        return vec4(0.0);
    }

    if (px.y == ROW_OBJECTS) {
        if ((r.flags & CHG_OBJECT) != 0u && r.targetId >= 0 && px.x == r.targetId) {
            return vec4(r.objPos, 0.0);
        }
        if ((r.flags & CHG_UNDO_EXEC) != 0u && r.targetId >= 0 && px.x == r.targetId) {
            return vec4(r.objPos, 0.0);
        }
        return ld(buf, px.x, ROW_OBJECTS);
    }

    if (px.y == ROW_DRAG) {
        if ((r.flags & CHG_DRAG) != 0u) {
            if (px.x == 0) return vec4(r.dragVal, 0.0);
            if (px.x == 1) return vec4(r.dragPlane, 0.0);
            if (px.x == 2) return vec4(r.dragAngle, 0.0, 0.0, 0.0);
        }
        return ld(buf, px.x, ROW_DRAG);
    }

    if (px.y == ROW_ROTATIONS) {
        if ((r.flags & CHG_OBJECT) != 0u && r.targetId >= 0 && px.x == r.targetId) {
            return r.objQuat;
        }
        if ((r.flags & CHG_UNDO_EXEC) != 0u && r.targetId >= 0 && px.x == r.targetId) {
            return r.objQuat;
        }
        return ld(buf, px.x, ROW_ROTATIONS);
    }

    if (px.y == ROW_SIZES) {
        if ((r.flags & CHG_OBJECT) != 0u && r.targetId >= 0 && px.x == r.targetId) {
            return vec4(r.objSiz, 0.0);
        }
        if ((r.flags & CHG_UNDO_EXEC) != 0u && r.targetId >= 0 && px.x == r.targetId) {
            return vec4(r.objSiz, 0.0);
        }
        return ld(buf, px.x, ROW_SIZES);
    }

    if (px.y == ROW_DRAG_ROT) {
        if ((r.flags & CHG_DRAG) != 0u) {
            return r.dragQuat;
        }
        return ld(buf, px.x, ROW_DRAG_ROT);
    }

    // Undo state storage
    if (px.y == ROW_UNDO) {
        if ((r.flags & CHG_UNDO) != 0u) {
            if (px.x == 0) return vec4(float(r.targetId), 0.0, 0.0, 0.0);
            if (px.x == 1) return vec4(r.undoPos, 0.0);
            if (px.x == 2) return vec4(r.undoSiz, 0.0);
        }
        // After undo execution, clear undo slot
        if ((r.flags & CHG_UNDO_EXEC) != 0u && px.x == 0) return vec4(-1.0, 0.0, 0.0, 0.0);
        return ld(buf, px.x, ROW_UNDO);
    }

    if (px.y == ROW_UNDO_ROT) {
        if ((r.flags & CHG_UNDO) != 0u) return r.undoQuat;
        return ld(buf, px.x, ROW_UNDO_ROT);
    }

    // Clipboard storage
    if (px.y == ROW_CLIPBOARD) {
        if ((r.flags & CHG_CLIPBOARD) != 0u) {
            if (px.x == 0) return vec4(r.clipPos, 1.0);  // .w = 1.0 means clipboard has data
            if (px.x == 1) return vec4(r.clipSiz, 0.0);
        }
        return ld(buf, px.x, ROW_CLIPBOARD);
    }

    if (px.y == ROW_CLIP_ROT) {
        if ((r.flags & CHG_CLIPBOARD) != 0u) return r.clipQuat;
        return ld(buf, px.x, ROW_CLIP_ROT);
    }

    // Material state storage
    if (px.y == ROW_MATERIAL) {
        if ((r.flags & CHG_MATERIAL) != 0u) {
            if (px.x == 0) return vec4(r.exposure, r.sunAngle, r.roughness, r.metalness);
            if (px.x == 1) return vec4(r.hsv, 0.0);
        }
        return ld(buf, px.x, ROW_MATERIAL);
    }

    // Panel state storage
    if (px.y == ROW_PANEL) {
        if ((r.flags & CHG_PANEL) != 0u) {
            if (px.x == 0) return vec4(float(r.sliderId), r.sliderDragStart, 0.0, 0.0);
        }
        return ld(buf, px.x, ROW_PANEL);
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
    if (px.y > ROW_PANEL) {
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

    fragColor = applyIxResult(px, r, inp, iChannel0, iKeyboard);
}

