# JiBot 底盘服务测试 (Tier 2)

## 定位

**T2 是适配器层标准接口验证层**，使用 `LejuWheeledArmHardware` 的 `_jibot` 后缀方法验证 JiBot 底盘服务调用。

在 LeTools 的分层测试架构中：

| 层级 | 目录 | 接口方式 | 目的 |
|------|------|---------|------|
| T1 | `jibot_upper_machine_python_tests/` | rospy 直调 ROS 服务 | 底层基准：ROS 服务连通性 |
| **T2 (本目录)** | `apps/jibot/` | `LejuWheeledArmHardware._jibot` 方法 | 适配器层：Mixin 封装验证 |

## 目录结构

```
apps/jibot/
├── README.md
├── __init__.py
├── _scaffold.py                                # JiBot 专用脚手架
├── test_base_move.py                           # T2: base_move_relative_jibot()
├── test_move_to_target.py                      # T2: base_move_to_target_jibot()
├── test_check_arrived.py                       # T2: check_arrived_jibot()
├── test_enable_vel_control.py                  # T2: enable_vel_control_jibot()
└── jibot_upper_machine_python_tests/            # T1 原生脚本（参考）
    ├── base_move.py
    ├── check_arrived.py
    ├── move_to_target.py
    ├── enable_vel_control.py
    └── 上位机迁移测试说明.md
```

## 接口列表

| 测试脚本 | 适配器方法 | ROS 服务 | 功能 |
|---------|-----------|---------|------|
| `test_base_move.py` | `base_move_relative_jibot()` | `/move_base/base_move` | 底盘相对移动 |
| `test_move_to_target.py` | `base_move_to_target_jibot()` | `/move_base/move_to_target` | 底盘 map 绝对目标点移动 |
| `test_check_arrived.py` | `check_arrived_jibot()` | `/move_base/check_arrived` | 任务到达检查 |
| `test_enable_vel_control.py` | `enable_vel_control_jibot()` | `/enable_vel_control` | 速度控制权限切换 |

## 运行方式

```bash
# 单个脚本
python3 apps/jibot/test_base_move.py
python3 apps/jibot/test_move_to_target.py
python3 apps/jibot/test_check_arrived.py
python3 apps/jibot/test_enable_vel_control.py

# pytest 运行
pytest apps/jibot/test_base_move.py -v
```

## 测试脚本模板

```python
import unittest
from adapters.hardware.leju_wheeled.hardware import LejuWheeledArmHardware
from apps.jibot._scaffold import jibot_setup
from core.domain.chassis_options import MoveToTargetOptions

class TestBaseMove(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.hardware = LejuWheeledArmHardware(config={'skip_sdk_managers': True})
        cls.hardware.initialize()
        jibot_setup()

    @classmethod
    def tearDownClass(cls):
        cls.hardware.shutdown()

    def test_01_forward_small(self):
        """小位移前进 0.2m"""
        result = cls.hardware.base_move_relative_jibot(x=0.2, y=0.0, theta=0.0)
        self.assertTrue(result.success, f"失败: {result.message}")
```

## 环境要求

- ROS Noetic + Python 3
- `leju_mobile_base_msgs` 已编译（`catkin build leju_mobile_base_msgs`）
- 底盘 Jarvis 服务已启动（`/move_base/*` 可用）
- `/enable_vel_control` 需要 AAEON 下位机 kuavo-ros-control 启动（可选）
- `config={'skip_sdk_managers': True}` 跳过 SDK 初始化，仅需 ROS

---

**最后更新**: 2026-06-03
**状态**: ✅ 4/4 实现
