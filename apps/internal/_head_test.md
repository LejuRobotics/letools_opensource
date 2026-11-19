# 头部接口分层测试

面向人工真机验证。调用链保持 Node → Skill → IHardware → HeadPoseMixin → LowLevelSDKManager → SDK。

## 脚本结构

六个脚本各自展示完整流程，不接受 `--execute`、`--mock`、`--yaw`、`--pitch` 等命令行参数：

- Adapter：创建硬件 → initialize → 直接调用接口 → 检查结果 → shutdown。
- Skill：创建硬件 → initialize → 技能 initialize/execute/is_finished → shutdown。
- Node：配置共享硬件 → tick_once → 检查 SUCCESS 和黑板输出 → 释放共享硬件。

`_head_test.py` 只保留共用硬件配置、目标/结果校验和 dry-run 检查，不调度测试、不调用硬件、不管理生命周期。

## 人工设置与运行

setter 脚本顶部的 `TARGET_YAW`、`TARGET_PITCH` 默认为 `None`。操作人员确认现场安全、实际关节范围后填写（单位：度），未填写时脚本失败退出，不初始化硬件。getter 不需要角度输入；Node getter 可修改顶部的 `OUTPUT_KEY`（默认 `head_pose`）。

路径基准：以下路径均相对 LeTools 仓库根目录。在已配置的 ROS/SDK 环境中，由操作人员从仓库根目录按需单独运行：

```bash
python3 apps/internal/adapters/set_head_pose.py
python3 apps/internal/adapters/get_head_pose.py
python3 apps/internal/skills/set_head_pose.py
python3 apps/internal/skills/get_head_pose.py
python3 apps/internal/nodes/set_head_pose.py
python3 apps/internal/nodes/get_head_pose.py
```

不要批量运行六个脚本；按需单独验证。Node 运行前取消 `STUDIO_DRY_RUN`。脚本只在 main 中连接硬件，导入文件不会执行测试。

## 配置和结果

只启用 low SDKManager，跳过相机、末端执行器、StateManager 和力控发布器。不做手臂模式切换、躯干复位、头部扫描或自动回零。

setter 成功只表示目标发送成功，不表示物理到位。getter 返回 SDK 缓存的 yaw/pitch 字典（度），不保证首帧或新鲜度。成功退出码 0，校验/初始化/调用失败退出码 1。

原 SDK 初始化仍有 `/republish_tfs` 等依赖；原生命周期误报 Manager 初始化成功的问题未在本次修复。
