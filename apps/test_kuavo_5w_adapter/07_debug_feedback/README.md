# 07_debug_feedback - 状态反馈测试模块
## 分层结构

```text
07_debug_feedback 状态反馈测试模块
├── 测试入口层
│   ├── test_complete_state_feedback.py  # 端到端状态反馈测试，自动发指令并检查反馈
│   ├── verify_state_data.py             # 实时状态监控工具，周期性读取反馈数据
│   └── diagnose_ros_topics.py           # ROS 话题诊断工具，用于确认反馈话题是否存在、有无数据
│
├── 适配器调用层
│   └── LejuWheeledArmHardware           # 通过状态反馈接口读取底盘、手臂、腿部、MPC、力矩、末端位姿等数据
│
├── Mixin 能力层
│   └── state_feedback_mixin.py          # 状态反馈能力集中封装，向测试脚本提供 get_xxx 类方法
│
└── ROS 反馈层
    ├── 到达时间话题                      # 底盘、躯干、手臂、腿部、末端等 reach_time
    ├── 控制模式话题                      # MPC 控制模式
    └── 机器人状态话题                    # 加速度、力矩、末端位姿等实时状态
```

## 📖 概述

本模块用于测试和验证 Kuavo 5-W 机器人的状态反馈功能，包括：
- 到达时间反馈（5种）
- MPC控制模式
- 本体加速度
- 关节力矩
- 末端位姿

## 🚀 快速开始

### 1. 完整状态反馈测试（推荐）

```bash
cd ~/LeTools
python3 apps/test_kuavo_5w_app/07_debug_feedback/test_complete_state_feedback.py
```

**功能**：
- 依次发送底盘、躯干、手臂、腿部控制指令
- 每次发送后立即检查对应的到达时间反馈
- 验证所有8种状态反馈是否正常工作
- 生成详细的测试报告

**预期输出**：
```
📊 测试结果统计:
   ✅ 通过: 8/8
   ❌ 失败: 0/8

🎉 所有测试通过！状态反馈功能正常工作。
```

---

### 2. 实时状态监控

```bash
python3 apps/test_kuavo_5w_app/07_debug_feedback/verify_state_data.py
```

**功能**：
- 每3秒刷新一次状态显示
- 实时查看所有9种状态数据
- 按 Ctrl+C 退出

**注意**：到达时间反馈需要先发送控制指令才会显示数据。

---

## 📋 测试脚本说明

### test_complete_state_feedback.py

**用途**：完整的端到端测试

**测试项目**：
1. 底盘位置到达时间
2. 躯干位姿到达时间
3. 手臂关节到达时间
4. 腿部关节到达时间
5. MPC控制模式
6. 本体加速度
7. 关节力矩
8. 末端位姿

**特点**：
- 自动发送控制指令触发反馈
- 立即验证反馈数据
- 生成详细测试报告
- 适合手动测试和演示

---

### verify_state_data.py

**用途**：实时状态监控工具

**显示内容**：
- 9种状态的实时数据
- 数据有效性检查
- 统计分析
- 异常警告

**特点**：
- 持续运行，定期刷新
- 直观的数据展示
- 适合调试和观察
- 需要配合控制指令使用

---

## 🔧 常见问题

### Q1: 为什么到达时间显示"无数据"？

**A**: 到达时间反馈是被动触发的，需要先发送控制指令。

**解决方案**：
```python
# 先发送指令
hardware.send_base_pose(x=0.1, y=0.0, yaw=0.0)

# 再获取反馈
reach_time = hardware.get_reach_time('cmd_pose')
```

或者运行 `test_complete_state_feedback.py`，它会自动发送指令并验证反馈。

---

### Q2: 如何只测试某一种状态？

**A**: 可以修改 `test_complete_state_feedback.py`，注释掉不需要的测试：

```python
# 只测试底盘位置
results['底盘位置到达时间'] = test_cmd_pose_reach_time(hardware)
# results['躯干位姿到达时间'] = test_torso_pose_reach_time(hardware)  # 注释掉
# ...
```

---

### Q3: 本体加速度的Z轴为什么接近0？

**A**: `/humanoid_wheel/bodyAcc` 不包含重力加速度，它只是底盘的平动和旋转加速度。

这是正常现象，不是错误。如果需要IMU数据（包含重力），需要使用其他话题。

---

### Q4: 关节力矩中某个关节力矩很大（如-161 Nm）是否正常？

**A**: 是的，这是正常的。joint_1（髋关节）在站立状态下需要承受较大负载。

只要力矩在电机额定范围内，就是安全的。

---

## 📊 状态反馈类型说明

### 1. 到达时间反馈（5种）

| 类型 | 控制指令 | 反馈话题 | 单位 |
|------|---------|---------|------|
| 底盘位置 | `send_base_pose()` | `/lb_cmd_pose_reach_time` | 秒 |
| 躯干位姿 | `send_torso_pose()` | `/lb_torso_pose_reach_time` | 秒 |
| 手臂关节 | `send_arm_joint_trajectory()` | `/lb_arm_joint_reach_time/left` | 秒 |
| 腿部关节 | `send_leg_joint_command()` | `/lb_leg_joint_reach_time` | 秒 |
| 手臂末端 | `send_two_arm_hand_pose()` | `/lb_arm_ee_reach_time/left` | 秒 |

**特点**：
- 被动触发，需要先发送指令
- 表示预计到达目标位置的时间
- 用于运动规划和同步

---

### 2. MPC控制模式

**话题**: `/mobile_manipulator/lb_mpc_control_mode`  
**类型**: Int8  
**频率**: 50Hz

**模式值**：
- 0: NO_CONTROL - 无控制
- 1: ARM_ONLY - 仅手臂
- 2: BASE_ONLY - 仅基座
- 3: BASE_ARM - 基座+手臂
- 4: ARM_EE_ONLY - 仅手臂末端

---

### 3. 本体加速度

**话题**: `/humanoid_wheel/bodyAcc`  
**类型**: Float64MultiArray  
**格式**: `[acc_x, acc_y, acc_yaw]`

**注意**：
- 不包含重力加速度
- 是底盘的平动和旋转加速度
- 不是IMU数据

---

### 4. 关节力矩

**话题**: `/humanoid_wheel/torque`  
**类型**: Float64MultiArray  
**格式**: `[下肢扭矩(4), 上肢扭矩(14)]`

**关节顺序**：
- 0-3: 下肢4个关节
- 4-17: 上肢14个关节（左右臂各7个）

---

### 5. 末端位姿

**话题**: `/humanoid_wheel/eePoses`  
**类型**: Float64MultiArray  
**格式**: `[左臂位姿(6), 右臂位姿(6)]`

**每个位姿格式**: `[x, y, z, yaw, pitch, roll]`
- 位置: x, y, z (米)
- 姿态: yaw, pitch, roll (弧度，欧拉角)

---

## 📚 相关文档

- [状态反馈修复报告](STATE_FEEDBACK_FIX_REPORT.md) - 详细的修复过程和原理
- [话题修正报告](TOPIC_CORRECTION_REPORT.md) - ROS话题名称和类型的修正
- [官方接口文档](../../../../kuavo-ros-opensource/docs/4开发接口/Kuavo%205-W%20接口使用文档.md)

---

## 🛠️ 开发指南

### 添加新的状态反馈测试

1. 在 `test_complete_state_feedback.py` 中添加测试函数：

```python
def test_new_feedback(hardware):
    """测试新的状态反馈"""
    print_separator("测试X: 新反馈")
    
    # 发送控制指令（如果需要）
    # result = hardware.send_xxx(...)
    
    # 获取反馈数据
    data = hardware.get_xxx()
    
    if data is not None:
        logger.info(f"✅ 收到反馈: {data}")
        return True
    else:
        logger.warning("⚠️  未收到反馈")
        return False
```

2. 在 `main()` 函数中调用：

```python
results['新反馈'] = test_new_feedback(hardware)
```

3. 运行测试验证

---

### 修改验证逻辑

编辑 `verify_state_data.py` 中的对应验证函数：

```python
def verify_xxx(hardware):
    """验证xxx状态"""
    data = hardware.get_xxx()
    
    if data is not None:
        # 添加验证逻辑
        if is_valid(data):
            print("✅ 正常")
            return True
        else:
            print("❌ 异常")
            return False
    else:
        print("⚠️  无数据")
        return None
```

---

## ✨ 总结

本模块提供了精简高效的状态反馈测试工具链：

- ✅ **test_complete_state_feedback.py** - 完整的端到端测试（推荐）
- ✅ **verify_state_data.py** - 实时状态监控

所有测试都已通过验证，状态反馈功能正常工作！

---

**最后更新**: 2026-05-16  
**维护者**: Kuavo Development Team

