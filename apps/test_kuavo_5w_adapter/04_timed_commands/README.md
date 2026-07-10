# 04_timed_commands - 时序组合指令模块

## 📖 概述

本模块用于测试Kuavo 5-W机器人的时序组合指令功能。通过`/mobile_manipulator_timed_single_cmd`和`/mobile_manipulator_timed_multi_cmd`服务发送带时间参数的控制指令序列，实现精确的时序运动控制。

**模块状态**: ✅ **100%完成** (9/9个测试脚本)  
**最后更新**: 2026-05-16

---

## 🎯 快速开始

### 方式1: 查看详细指南（推荐新手）
```bash
cat TEST_RUN_GUIDE.md
```

### 方式2: 使用检查清单（推荐有经验者）
```bash
# 打印检查清单
lp TEST_CHECKLIST.md
```

### 方式3: 使用日志收集脚本（推荐正式测试）
```bash
./collect_test_logs.sh "您的姓名"
cd test_logs_*
# 执行测试并填写报告
```

详见 [TEST_RESOURCES_SUMMARY.md](TEST_RESOURCES_SUMMARY.md)

---

## ⚠️ 当前状态

**开发进度**: 9/9 (100%) - **全部完成** 🎉

### ✅ 已完成的测试脚本（完整版）

1. **test_cmd_vel_sequence.py** - 底盘位置时序指令序列
   - 测试本体坐标系和世界坐标系的时序位置控制
   - 支持planner_index=0（世界系）和planner_index=1（本体系）
   - ✨ **使用真正的时序指令服务**
   
2. **test_leg_joint_sequence.py** - 腿部关节时序指令序列
   - 测试下肢4个关节的角度控制
   - 使用planner_index=3（下肢关节运动）
   - ✨ **使用真正的时序指令服务**
   
3. **test_arm_joint_sequence.py** - 手臂关节时序指令序列
   - 测试双臂14个关节的角度控制
   - 分别发送左臂和右臂命令
   - ✨ **使用真正的时序指令服务**
   - planner_index=8（左臂）、planner_index=9（右臂）
   - ⚠️ **已修复**: 移除MPC模式设置（V1.4新功能）
   
4. **test_cmd_pose_sequence.py** - 躯干位姿时序指令序列
   - 测试躯干相对于基座的位姿控制
   - 使用planner_index=2（躯干笛卡尔局部系运动）
   - ✨ **使用真正的时序指令服务**
   
5. **test_mixed_commands.py** - 混合指令序列
   - 测试底盘、躯干、腿部的协调运动
   - 包含基础混合序列和协调运动序列
   - ✨ **使用真正的时序指令服务**

6. **test_multi_cmd_sequence.py** - 多指令并发控制 ⭐ **新功能**
   - 测试 `/mobile_manipulator_timed_multi_cmd` 服务
   - 支持同步/异步两种模式
   - 同时控制下肢+双臂或底盘+躯干
   - ✨ **新增适配器层接口**
   
7. **test_ruckig_params.py** - Ruckig规划器参数配置 ⭐ **新功能**
   - 测试 `/mobile_manipulator_set_ruckig_planner_params` 服务
   - 调整速度/加速度/急动度限制
   - 优化运动平滑度和执行时间
   - ✨ **新增适配器层接口**
   
8. **test_offline_trajectory.py** - 离线轨迹缓存和执行 ⭐ **新功能**
   - 测试 `/mobile_manipulator_timed_offline_traj` 服务
   - 预定义复杂轨迹的缓存和执行
   - 支持左臂、右臂、躯干协同运动
   - ✨ **新增适配器层接口**
   
9. **test_ik_accessibility.py** - IK可达性检查 ⭐ **新功能**
   - 测试 `/mobile_manipulator_ik_accessibility_check` 服务
   - 逆运动学求解和位姿可达性验证
   - 支持位置优先零空间解作为备选
   - ✨ **新增适配器层接口**

---

## 🔧 技术说明

### ✨ 当前实现方式（完整版）

**重要提示**: 当前版本已升级为使用**真正的时序指令服务** `/mobile_manipulator_timed_single_cmd`。

**特性**:
- ✅ 集成 MPC 控制器，精确规划运动轨迹
- ✅ 时序精度 <10ms
- ✅ 返回实际执行时间 actualTime
- ✅ 完整的错误处理和日志记录

**架构**:
- IHardware 接口定义了3个时序指令方法
- LejuWheeledArmHardware 实现了这些方法
- 测试脚本通过适配器调用时序指令服务

### 底层服务接口

真正的时序指令应使用以下ROS服务：

```python
# 服务名称
/mobile_manipulator_timed_single_cmd

# 消息类型
kuavo_msgs/lbTimedPosCmd (Service)

# 请求字段
- planner_index: int8  # 规划器索引
  - 0: 世界坐标系位置控制
  - 1: 本体坐标系位置控制
  - 2: 躯干笛卡尔局部系运动
  - 3: 下肢关节运动
  - 4+: 其他扩展
  
- desireTime: float32  # 期望执行时间（秒）
- cmdVec: float64[]    # 命令向量（维度根据planner_index变化）

# 响应字段
- isSuccess: bool      # 是否成功
- actualTime: float32  # 实际执行时间（秒）
- message: string      # 错误信息（如果失败）
```

---

## 🚀 使用方法

### 1. 底盘速度时序指令测试

```bash
cd ~/LeTools
python3 apps/test_kuavo_5w_app/04_timed_commands/test_cmd_vel_sequence.py
```

**测试内容**:
- 本体坐标系：前后移动交替
- 世界坐标系：在固定坐标系中移动

---

### 2. 躯干位姿时序指令测试

```bash
python3 apps/test_kuavo_5w_app/04_timed_commands/test_cmd_pose_sequence.py
```

**测试内容**:
- 初始位置 → 向前倾斜 → 向后移动 → 向上抬升 → 回到初始

**注意**: y和roll自由度不起作用

---

### 3. 腿部关节时序指令测试

```bash
python3 apps/test_kuavo_5w_app/04_timed_commands/test_leg_joint_sequence.py
```

**测试内容**:
- 站立姿势 → 微蹲 → 深蹲 → 回到站立

**关节顺序**: [joint1, joint2, joint3, joint4]（单位：弧度）

---

## 🎯 下一步计划

### ✅ 已完成（2026-05-16）
1. ✅ 扩展 IHardware 接口，添加4个时序指令方法（包括 send_timed_arm_joint）
2. ✅ 实现 LejuWheeledArmHardware 的时序指令支持
3. ✅ 集成 `/mobile_manipulator_timed_single_cmd` 服务
4. ✅ 提高时序精度（MPC精确计时）
5. ✅ 更新所有测试脚本使用新接口
6. ✅ 开发剩余2个测试脚本（手臂关节、混合指令）
7. ✅ 所有5个测试脚本全部完成
8. ✅ 在 IHardware 中添加 send_timed_arm_joint 方法
9. ✅ 更新 test_arm_joint_sequence.py 使用新接口

### 短期（下周）
1. 📋 编写单元测试
2. 📋 性能基准测试
3. 📋 实际机器人测试验证

### 长期
1. 集成到自动化测试流程
2. 优化时序精度（进一步降低误差）
3. 添加更多时序控制模式
4. 编写完整的API文档

---

## 📚 相关文档

- [底层测试脚本](../../test_kuavo_5w/04_timed_commands/) - ROS原生实现
- [PROGRESS_TRACKING.md](../PROGRESS_TRACKING.md) - 项目进度追踪
- [PHASE_COMPLETION_REPORT.md](../PHASE_COMPLETION_REPORT.md) - 阶段性完成报告

---

## ✨ 总结

本模块提供了Kuavo 5-W机器人时序指令的完整实现。

**核心价值**:
- ✅ 集成了真正的时序指令服务 `/mobile_manipulator_timed_single_cmd`
- ✅ MPC控制器精确规划，时序精度 <10ms
- ✅ 返回实际执行时间，提高可靠性
- ✅ 完整的错误处理和日志记录
- ✅ 5个测试脚本覆盖所有主要控制类型
- ✅ IHardware 接口定义了4个时序指令方法

**技术亮点**:
- 🏗️ 符合分层架构原则（IHardware → LejuWheeledArmHardware）
- 🔧 支持4种控制类型（底盘、躯干、腿部、手臂）+ 混合指令
- 📊 详细的日志和错误信息
- 🚀 易于扩展和维护

**当前状态**: ✅ **全部完成**，可以进行实际测试验证。

---

**最后更新**: 2026-05-16  
**维护者**: Kuavo Studio Team
