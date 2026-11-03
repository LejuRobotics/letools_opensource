# 原子技能库：调用说明入口

[返回文档首页](../../README.md) · [技能接口总表](../04-skills-原子技能.md) · [案例库](../../04-简单案例运行/案例库/README.md)

本索引按源码组织**单项能力**，不是一份可直接在任何机器人上执行的动作清单。当前主力是 `skills/atomic/refactored_sdk/` 顶层的 40 个技能文件（不计 `__init__.py`）。旧版 `manipulation/`、`motion/` 与感知目录分别标明，不与主力技能数相加后宣称同一套接口。

| 控制领域 | 调用说明 |
|---|---|
| 手臂、末端与 TimedCmd | [01-手臂与末端](01-手臂与末端.md) |
| 底盘、导航与闭环伺服 | [02-底盘与导航](02-底盘与导航.md) |
| 头部、下肢、躯干 | [03-头部下肢与躯干](03-头部下肢与躯干.md) |
| 抓取、放置、真空与压力 | [04-抓取与末端工具](04-抓取与末端工具.md) |
| 推理与感知目录 | [05-感知与伺服](05-感知与伺服.md) |
| 等待与人工确认 | [06-等待与编排](06-等待与编排.md) |
| 旧版技能 | [07-旧版技能](07-旧版技能.md) |

## 调用与验证口径

各页的代码片段给出真实的模块名、技能类、参数类和 `initialize → execute → is_finished` 调用顺序。**源码核对**只证明这些名称和参数存在，不等于动作已在当前仿真或真机完成。需要硬件的片段必须先按[环境安装](../../02-环境安装/03-安装步骤.md)初始化 ROS、SDK 和适配器，并核对坐标系、角度单位、目标位姿及现场安全范围；默认参数只是源码默认值，不保证适合你的机器人。

通用调用循环如下。每个技能小节的片段先导入类并构造 `skill`、`params`，再调用这个函数；连续监测类技能需要外部事件才能结束，必要时由上层调用 `cancel()`。

```python
import time

def run_skill(skill, params):
    result = skill.initialize(params)
    if not result.success:
        raise RuntimeError(f"initialize failed: {result.message}")
    while not skill.is_finished():
        result = skill.execute()
        if not result.success:
            raise RuntimeError(f"execute failed: {result.message}")
        time.sleep(0.05)
    return result
```

硬件技能片段使用 `hw`，其创建和 `initialize()` 检查见[已核对的头部控制示例](../04-skills-原子技能.md#直接构造运行一个技能)；使用完毕需调用 `hw.shutdown()`。无硬件技能直接构造。**不要把 `--dry-run --tick-once` 的树加载成功当作运动、视觉或硬件服务已验证**。示例的运行验证状态在各页注明；未取得对应设备或服务的实测结果时，不写“已跑通”。

单项技能的 `Result.success`、`message` 和 `is_finished()` 应一并检查。返回成功但仍未完成的技能需继续 tick；失败时先看错误消息，再按[FAQ](../../06-常见问题FAQ/)排查。场景内还要核对技能对应的[行为树节点](../11-节点清单.md)与[案例](../../04-简单案例运行/案例库/README.md)。
