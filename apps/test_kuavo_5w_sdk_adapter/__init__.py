"""
Kuavo 5W 重构版测试套件

使用 Core 层 SDK 管理器进行控制，与 test_kuavo_5w_app（使用 ROS 话题）完全隔离。

目录结构:
    01_base_control/      - 底盘控制测试
    02_lower_body/        - 下肢控制测试
    03_arm_control/       - 手臂控制测试
    04_timed_commands/    - 时序指令测试
    05_force_control/     - 力控测试
    06_services/          - 服务测试
    07_debug_feedback/    - 调试反馈测试
    config/               - 配置文件
"""
