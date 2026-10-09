# LeTools 行为树场景运行环境说明

## 一、三档运行环境

| 档位 | 启动方式 | 前置依赖 | 能验证什么 | 不能验证什么 |
|---|---|---|---|---|
| 干跑（dry-run） | `--dry-run [--tick-once]` | 无硬件、无 ROS | 树能否加载、节点类名是否匹配、参数与黑板连线是否通 | 任何运动结果；运动节点内部被 `STUDIO_DRY_RUN=1` 短路 |
| MuJoCo 仿真 | 启动 ROS + MuJoCo 后端后正常运行 | `kuavo_humanoid_sdk`、`kuavo-ros-opensource` | 关节 / 手臂 / 头 / 躯干 / 底盘的完整运动链路、IK 求解与轨迹下发 | 真实传感器、真实末端、JiBot 底盘服务 |
| 真机 | 同上 + 下位机 | 上述全部 + 真实相机 / 末端 / 爪 / 力控 | 全量 | - |

两点澄清：

1. **干跑不是仿真。** 它把运动节点整体短路，只证明 import 链路、JSON 加载与节点类名映射无误，不触发 IK 求解与轨迹下发。所以干跑跑通不代表能仿真，更不代表能上真机。
2. **仿真不是缩小的真机。** JiBot 底盘的三个节点走的是 `leju_mobile_base_msgs` 的 `BaseMove` / `MoveToTarget` / `CheckArrived` ROS 服务（见 `adapters/hardware/leju_wheeled/mixins/jibot/chassis_mixin.py`），仿真环境没有这套服务。因此含 JiBot 节点的场景在仿真下无法直接运行。

## 二、如何判定一个场景属于哪档

按节点类名扫描 `py_tree.json` 与 `py_tree_child.json` 的 `"name"` 字段：

```bash
grep -rho '"name"[[:space:]]*:[[:space:]]*"[^"]*"' \
  orchestration/scenarios/<场景>/*.json | sort -u
```

| 扫到的节点 | 结论 |
|---|---|
| `*JibotMove` | 真机 + JiBot，仿真下无法运行 |
| `*Percep` / `*Tag*` / `Apriltag*` / `Inference*` | 需真实相机或外部视觉服务 |
| 力控 / 真空 / 爪类 | 需真实末端 |
| 只有 `*SdkMove`（关节、手臂、头、躯干） | 仿真 / 真机均可 |
| 任意节点组合 | 均可干跑 |

> ⚠️ **按节点类名判，不要按文件名判。** `dismantle_box_demo_action` 的子树叫 `demo_jibot_nav.json`，但里面全是纯 SDK 节点，实际不依赖 JiBot。

## 三、场景清单

规模：15 个场景目录 / 26 棵可运行主树。**全部场景均支持干跑**，下表不再重复标注。

### A. 真机 + JiBot（仿真下无法运行）- 7 个场景 / 18 棵树

| 场景 | 主树数 | 说明 |
|---|---:|---|
| `jibot_nav_v1` | 1 | 导航专用示例，JiBot 相对移动 / 绝对移动 / 到达检查三件套 |
| `dismantle_box` | 1 | 同上三件套 |
| `dismantle_box_demo` | 1 | 同上三件套 |
| `dismantle_box_internal` | 7 | 7 个子场景环境要求一致（`dismantle_box_dynamic` 少一个相对移动节点）；JiBot 三件套 + 视觉位姿注入 + 真空吸盘 + 压力检测 |
| `palletize_box_internal` | 6 | 6 个子场景环境要求一致；JiBot 三件套 + 真空吸盘 + 位姿校验 |
| `depalletize_bin_v1_internal` | 1 | JiBot 绝对移动 + 到达检查；另依赖外部已运行的 LingBot 视觉服务 |
| `grasp_mtbf_v1` | 1 | JiBot 三件套 + `NodePercep` 视觉 + 底盘点动 / 停止 |

### B. 仿真 / 真机均可（无 JiBot 依赖）- 6 个场景 / 6 棵树

| 场景 | 主树数 | 说明 |
|---|---:|---|
| `refactored_sdk_atomic_v1` | 1 | 最简：关节 / 头 / 躯干 / 底盘相对位移，纯 SDK |
| `refactored_sdk_arm_v1` | 1 | 纯手臂 SDK：世界系 / 局部系末端轨迹 |
| `studio_smoke_v1` | 1 | 短动作冒烟：底盘短移 + 腿 + 手臂基座轨迹 |
| `dismantle_box_demo_action` | 1 | 名字含 jibot，实为纯 SDK 动作演示，无 JiBot 依赖 |
| `refactored_sdk_single_tag_pick_v0` | 1 | README 记录支持干跑 / MuJoCo / 真机三链；真机下 30s 未识别到 tag 则降级注入假 tag 继续 |
| `wheel_arm_single_tag_pick_v1` | 1 | README 记录 MuJoCo 端到端跑通（397 帧双臂轨迹） |

### C. 需真机传感器 / 末端（仿真下需自备对应外设，未验证）- 2 个场景 / 2 棵树

| 场景 | 主树数 | 说明 |
|---|---:|---|
| `grasp_ring_pick_place` | 1 | `InferenceControl` 视觉推理 + 夹爪 + 导航底盘 |
| `zhaofeng_feeding_internal` | 1 | AprilTag 视觉 + 力控 + 爪 |
