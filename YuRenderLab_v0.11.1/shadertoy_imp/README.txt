========================================
  Shadertoy.com 部署指南
========================================

在 https://www.shadertoy.com/new 上创建新 Shader，按以下步骤配置：

1. 创建 Buffer 和 Tab
   ------------------------------------------------------------
   点击底部 "+" 按钮，添加以下标签页：
   - Buffer A
   - Buffer B
   - Buffer D
   (Buffer C 不使用，无需创建)

2. 粘贴代码
   ------------------------------------------------------------
   将 ref/ 文件夹中的代码分别粘贴到对应标签页：

   Common.glsl   -> "Common" 标签页
   Buffer A.glsl -> "Buffer A" 标签页
   Buffer B.glsl -> "Buffer B" 标签页
   Buffer D.glsl -> "Buffer D" 标签页
   Image.glsl    -> "Image" 标签页

3. 配置 iChannel 输入
   ------------------------------------------------------------

   【Buffer A】
   iChannel0 = Buffer A   (filter: Nearest, wrap: Clamp)
   iChannel1 = Cubemap     (选择 "Uffizi Gallery" 或任意 Cubemap)
                            (filter: MipMap, wrap: Clamp)

   【Buffer B】
   iChannel0 = Buffer A   (filter: Linear, wrap: Repeat)
   iChannel1 = Buffer B   (filter: Linear, wrap: Clamp)

   【Buffer D】
   iChannel0 = Buffer D   (filter: Nearest, wrap: Clamp)
   iChannel1 = Keyboard   (点击输入源列表中的 "Keyboard")
                            (filter: Nearest, wrap: Clamp)

   【Image】
   iChannel0 = Buffer D   (filter: Nearest, wrap: Clamp)
   iChannel1 = Buffer A   (filter: Linear, wrap: Repeat)
   iChannel2 = Buffer B   (filter: Linear, wrap: Clamp)

4. 运行
   ------------------------------------------------------------
   点击播放按钮。首帧会较慢（计算 SH 和预滤波环境贴图）。
   
   操作方式：
   - 鼠标拖拽: 旋转视角
   - W/A/S/D:  前后左右移动
   - Space:    上升
   - Ctrl:     下降
   - Shift:    加速移动

5. 注意事项
   ------------------------------------------------------------
   - Buffer A 的 iChannel1 必须是 CubeMap 类型
   - 设置 Filter/Wrap 模式很重要，否则可能出现采样错误
   - Keyboard 输入必须在 Buffer D 的 iChannel1 中配置
   - 如果画面全黑，等待几帧让预计算完成
