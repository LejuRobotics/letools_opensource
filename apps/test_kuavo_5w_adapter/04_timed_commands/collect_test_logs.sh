#!/bin/bash
# 04_timed_commands 测试日志收集脚本
# 用法: ./collect_test_logs.sh [测试人员姓名]

TESTER_NAME=${1:-"unknown"}
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
LOG_DIR="test_logs_${TIMESTAMP}_${TESTER_NAME}"

echo "=========================================="
echo "  04_timed_commands 测试日志收集"
echo "=========================================="
echo ""
echo "测试人员: ${TESTER_NAME}"
echo "时间戳: ${TIMESTAMP}"
echo "日志目录: ${LOG_DIR}"
echo ""

# 创建日志目录
mkdir -p "${LOG_DIR}"

# 复制当前项目的日志文件
echo "📋 正在收集日志文件..."

if [ -d "../../log" ]; then
    cp -r ../../log "${LOG_DIR}/" 2>/dev/null
    echo "✅ 已复制项目日志"
else
    echo "⚠️  项目日志目录不存在"
fi

# 复制测试指南和检查清单
echo "📋 正在复制测试文档..."
cp TEST_RUN_GUIDE.md "${LOG_DIR}/" 2>/dev/null
cp TEST_CHECKLIST.md "${LOG_DIR}/" 2>/dev/null
cp README.md "${LOG_DIR}/" 2>/dev/null
echo "✅ 已复制测试文档"

# 创建测试报告模板
echo "📋 正在创建测试报告模板..."
cat > "${LOG_DIR}/test_report.md" << EOF
# 04_timed_commands 测试报告

**测试人员**: ${TESTER_NAME}  
**测试时间**: $(date +"%Y-%m-%d %H:%M:%S")  
**时间戳**: ${TIMESTAMP}

---

## 📊 测试结果汇总

| 测试脚本 | 状态 | 执行时间 | 备注 |
|---------|------|---------|------|
| test_cmd_vel_sequence.py | ⬜ | | |
| test_leg_joint_sequence.py | ⬜ | | |
| test_arm_joint_sequence.py | ⬜ | | |
| test_cmd_pose_sequence.py | ⬜ | | |
| test_mixed_commands.py | ⬜ | | |
| test_multi_cmd_sequence.py | ⬜ | | |
| test_ruckig_params.py | ⬜ | | |
| test_offline_trajectory.py | ⬜ | | |
| test_ik_accessibility.py | ⬜ | | |

**通过数量**: ___ / 9

---

## 🔍 详细测试结果

### 1. test_cmd_vel_sequence.py
- **状态**: ⬜ 通过 / ⬜ 警告 / ⬜ 失败
- **执行时间**: ______
- **问题描述**: 
- **截图/日志**: 

### 2. test_leg_joint_sequence.py
- **状态**: ⬜ 通过 / ⬜ 警告 / ⬜ 失败
- **执行时间**: ______
- **问题描述**: 
- **截图/日志**: 

### 3. test_arm_joint_sequence.py
- **状态**: ⬜ 通过 / ⬜ 警告 / ⬜ 失败
- **执行时间**: ______
- **问题描述**: 
- **截图/日志**: 

### 4. test_cmd_pose_sequence.py
- **状态**: ⬜ 通过 / ⬜ 警告 / ⬜ 失败
- **执行时间**: ______
- **问题描述**: 
- **截图/日志**: 

### 5. test_mixed_commands.py
- **状态**: ⬜ 通过 / ⬜ 警告 / ⬜ 失败
- **执行时间**: ______
- **问题描述**: 
- **截图/日志**: 

### 6. test_multi_cmd_sequence.py
- **状态**: ⬜ 通过 / ⬜ 警告 / ⬜ 失败
- **执行时间**: ______
- **问题描述**: 
- **截图/日志**: 

### 7. test_ruckig_params.py
- **状态**: ⬜ 通过 / ⬜ 警告 / ⬜ 失败
- **执行时间**: ______
- **问题描述**: 
- **截图/日志**: 

### 8. test_offline_trajectory.py
- **状态**: ⬜ 通过 / ⬜ 警告 / ⬜ 失败
- **执行时间**: ______
- **问题描述**: 
- **截图/日志**: 

### 9. test_ik_accessibility.py
- **状态**: ⬜ 通过 / ⬜ 警告 / ⬜ 失败
- **执行时间**: ______
- **问题描述**: 
- **截图/日志**: 

---

## 💡 总体评价

**整体状态**: ⬜ 优秀 / ⬜ 良好 / ⬜ 一般 / ⬜ 需改进

**主要优点**:


**发现的问题**:


**改进建议**:


---

## 📸 附件

- [ ] 终端输出截图
- [ ] RViz可视化截图
- [ ] 机器人运动视频
- [ ] 其他相关材料

---

**报告完成时间**: _______________  
**审核人员**: _______________
EOF

echo "✅ 已创建测试报告模板"

# 创建README
cat > "${LOG_DIR}/README.txt" << EOF
==========================================
  04_timed_commands 测试日志包
==========================================

测试人员: ${TESTER_NAME}
测试时间: $(date +"%Y-%m-%d %H:%M:%S")

包含文件:
- log/              : 项目日志文件
- test_report.md    : 测试报告模板（请填写）
- TEST_RUN_GUIDE.md : 测试运行指南
- TEST_CHECKLIST.md : 快速测试检查清单
- README.md         : 模块说明文档

使用说明:
1. 按照 TEST_RUN_GUIDE.md 执行测试
2. 使用 TEST_CHECKLIST.md 记录结果
3. 填写 test_report.md 测试报告
4. 如有截图或视频，放入此目录
5. 完成后压缩整个目录归档

祝测试顺利！
==========================================
EOF

echo "✅ 已创建README"

# 显示完成信息
echo ""
echo "=========================================="
echo "  ✅ 日志收集完成！"
echo "=========================================="
echo ""
echo "日志目录: ${LOG_DIR}"
echo ""
echo "下一步:"
echo "1. cd ${LOG_DIR}"
echo "2. 查看 TEST_RUN_GUIDE.md 了解测试步骤"
echo "3. 打印 TEST_CHECKLIST.md 用于记录"
echo "4. 执行测试并填写 test_report.md"
echo "5. 完成后压缩归档: tar -czf ${LOG_DIR}.tar.gz ${LOG_DIR}/"
echo ""
echo "祝测试顺利！🚀"
echo ""
