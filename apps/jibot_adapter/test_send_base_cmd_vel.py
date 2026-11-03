#!/usr/bin/env python3
"""JiBot 底盘原生速度通道验证脚本（适配器层）。

⚠️ 本脚本会真的让底盘动 —— 只由人在确认现场安全后手动运行，
   任何 agent 都不许执行。

脚本自动切换 /enable_vel_control：先把权限切到 false（jibot 底盘导航接管），
发完速度并保持零速后再恢复到切换前的值。任何退出路径都不会跳过恢复。

对应适配器接口:
    LejuWheeledArmHardware.send_base_cmd_vel_tick()
    LejuWheeledArmHardware.start_cmd_vel_stream()
    LejuWheeledArmHardware.update_cmd_vel_setpoint()
    LejuWheeledArmHardware.stop_cmd_vel_stream()
    LejuWheeledArmHardware.get_motion_gate()
    LejuWheeledArmHardware.get_vel_control_state_jibot()
    LejuWheeledArmHardware.enable_vel_control_jibot()
ROS 话题:
    /move_base/base_cmd_vel          geometry_msgs/Twist
    /move_base/base_cmd_vel_status   leju_mobile_base_msgs/BaseCmdVelStatus
    /enable_vel_control_state        std_msgs/Bool

用法:
    python3 apps/jibot_adapter/test_send_base_cmd_vel.py \
        --vx 0.05 --vy 0 --wz 0 --duration 2.0
"""

import argparse
import os
import sys
import time

project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.insert(0, project_root)

import rospy
from adapters.hardware.leju_wheeled.hardware import LejuWheeledArmHardware

VEL_CONTROL_TOPIC = "/enable_vel_control_state"
VEL_CONTROL_SERVICE = "/enable_vel_control"
VEL_CONTROL_TIMEOUT = 3.0

# 读不到当前权限且没有 --assume-state：无法保证恢复，中止
EXIT_STATE_UNKNOWN = 4
# 切换失败或回读仍为 true：一帧速度都不发
EXIT_SWITCH_FAILED = 5
# 恢复原权限失败：需要人工恢复
EXIT_RESTORE_FAILED = 6


def parse_args():
    parser = argparse.ArgumentParser(
        description="JiBot 底盘原生速度通道验证（会驱动底盘运动）"
    )
    parser.add_argument("--vx", type=float, default=0.05, help="前向速度 m/s（默认 0.05）")
    parser.add_argument("--vy", type=float, default=0.0, help="横向速度 m/s（默认 0）")
    parser.add_argument("--wz", type=float, default=0.0, help="角速度 rad/s（默认 0）")
    parser.add_argument("--duration", type=float, default=2.0, help="速度保持时长 s（默认 2.0）")
    parser.add_argument(
        "--assume-state",
        choices=("true", "false"),
        default=None,
        help=(
            "读不到当前权限时，由操作者声明权限当前值，脚本按该值切换并在结束时恢复"
            "（默认：读不到就中止）"
        ),
    )
    return parser.parse_args()


def format_state(state):
    if state is None:
        return "未知"
    return "true" if state else "false"


def print_manual_restore(original):
    """恢复失败时给出可直接执行的人工命令。"""
    print("请人工恢复权限：")
    print(
        "  rosservice call %s \"data: %s\""
        % (VEL_CONTROL_SERVICE, "true" if original else "false")
    )
    print(f"并确认 {VEL_CONTROL_TOPIC} 回到 {format_state(original)}")


def print_summary(original, switched, after_switch, restored):
    """打印一行前后状态；未切换时「切换后」就是原状态。"""
    print(
        "vel_control 原状态=%s → 切换后=%s → 恢复到=%s"
        % (
            format_state(original),
            format_state(after_switch) if switched else format_state(original),
            format_state(restored),
        )
    )


def read_vel_control_state(hardware):
    """读当前权限；读不到返回 None。"""
    try:
        result = hardware.get_vel_control_state_jibot(timeout=VEL_CONTROL_TIMEOUT)
    except Exception as exc:  # noqa: BLE001 - 读不到按「未知」处理
        print(f"WARN: 读取 {VEL_CONTROL_TOPIC} 异常: {exc}")
        return None
    if result.success and result.data:
        return result.data.get("state")
    print(f"WARN: 读取 {VEL_CONTROL_TOPIC} 失败: {result.message}")
    return None


def switch_vel_control(hardware, enable):
    """切换权限并回读；返回 (服务是否成功, 回读值, 描述)。"""
    result = hardware.enable_vel_control_jibot(enable)
    after = result.data.get("state_after") if result.data else None
    return result.success, after, result.message


def acquire_vel_control(hardware, args):
    """读原状态并切到 false。

    :return: (原状态, 是否切换过, 切换后状态, 退出码)
             退出码非 0 表示未取得权限，调用方不得发送任何速度。
    """
    original = read_vel_control_state(hardware)
    if original is None:
        if args.assume_state is None:
            print(f"读不到 {VEL_CONTROL_TOPIC}，无法保证恢复，中止（未切换、未发速度）")
            print("  - 确认下位机在发布该话题后重试；")
            print("  - 或明确传入 --assume-state true|false，由操作者担保当前状态。")
            return None, False, None, EXIT_STATE_UNKNOWN
        original = args.assume_state == "true"
        print("=" * 60)
        print(f"WARN: 读不到 {VEL_CONTROL_TOPIC}，按 --assume-state 声明的状态执行")
        print(f"WARN: 当前权限按 {format_state(original)} 处理，恢复正确性由操作者担保")
        print("=" * 60)

    print(f"vel_control 原状态={format_state(original)}")
    if original is False:
        print("权限已是 false（jibot 底盘导航接管），不重复切换")
        return original, False, False, 0

    print("切换 /enable_vel_control=False ...")
    try:
        ok, after, message = switch_vel_control(hardware, False)
    except Exception as exc:  # noqa: BLE001 - 服务不可用等一律按切换失败
        print(f"切换失败，不发送任何速度: {exc}")
        # 服务可能已收到请求后才异常，仍旧按「切换过」处理，退出时恢复
        return original, True, None, EXIT_SWITCH_FAILED

    if not ok:
        print(f"切换被服务拒绝，不发送任何速度: {message}")
        return original, True, after, EXIT_SWITCH_FAILED
    if after is True:
        print("切换后回读仍为 true，不发送任何速度")
        return original, True, after, EXIT_SWITCH_FAILED
    if after is None:
        print("WARN: 切换后回读不到状态，按服务返回继续（无法二次确认）")
    else:
        print(f"切换后 vel_control={format_state(after)}")
    return original, True, after, 0


def restore_vel_control(hardware, original, switched, after_switch):
    """把权限恢复到切换前的值；返回是否确认恢复成功。"""
    if not switched:
        if original is not None:
            print(f"vel_control 未切换，保持 {format_state(original)}")
            print_summary(original, False, None, original)
        return True

    try:
        ok, after, message = switch_vel_control(hardware, bool(original))
    except Exception as exc:  # noqa: BLE001 - 恢复失败必须显式提示
        ok, after, message = False, None, str(exc)

    restored = True
    if not ok:
        print(f"❌ 恢复 vel_control={format_state(original)} 失败: {message}")
        print_manual_restore(original)
        restored = False
    elif after is not None and after != original:
        print(f"❌ 恢复后回读仍为 {format_state(after)}")
        print_manual_restore(original)
        restored = False
    elif after is None:
        print("WARN: 恢复服务返回成功，但回读不到状态，无法二次确认")
    else:
        print(f"vel_control 恢复到={format_state(original)}")

    # 汇总只贴回读到的值；回读不可用就写「未知」，不拿推断冒充观测
    print_summary(original, True, after_switch, after)
    return restored


def run_motion(hardware, args):
    """倒计时 + 起流 + 限时保持 + 停流；返回退出码。"""
    print(f"\n前：motion_gate={hardware.get_motion_gate()}")

    for remaining in (3, 2, 1):
        print(f"  {remaining} 秒后开始运动 ...")
        time.sleep(1.0)

    start_result = hardware.start_cmd_vel_stream()
    if not start_result.success:
        print(f"❌ 启动速度流失败: {start_result.message}")
        return 1

    hardware.update_cmd_vel_setpoint(args.vx, args.vy, args.wz)
    time.sleep(args.duration)

    stop_result = hardware.stop_cmd_vel_stream()
    if not stop_result.success:
        print(f"⚠️ 停止速度流返回失败: {stop_result.message}")

    print(f"\n后：motion_gate={hardware.get_motion_gate()}")
    return 0


def main():
    args = parse_args()

    if not rospy.core.is_initialized():
        rospy.init_node('jibot_test_send_base_cmd_vel', anonymous=True)

    print(f"\n{'='*70}")
    print("JiBot 底盘原生速度通道验证 (/move_base/base_cmd_vel)")
    print(f"{'='*70}")
    print(
        f"  vx={args.vx} m/s  vy={args.vy} m/s  wz={args.wz} rad/s  "
        f"duration={args.duration}s"
    )

    hardware = LejuWheeledArmHardware(config={'skip_sdk_managers': True})
    result = hardware.initialize()
    if not result.success:
        print(f"❌ 硬件初始化失败: {result.message}")
        return 1

    exit_code = 0
    original_state = None
    switched = False
    after_switch = None
    try:
        # 切权限必须早于第一帧非零速度
        original_state, switched, after_switch, acquire_code = acquire_vel_control(
            hardware, args
        )
        if acquire_code != 0:
            exit_code = acquire_code
        else:
            exit_code = run_motion(hardware, args)

    except KeyboardInterrupt:
        print("\n⚠️  测试被用户中断")
        exit_code = 130
    except Exception as exc:
        print(f"\n❌ 测试执行出错: {exc}")
        import traceback
        traceback.print_exc()
        exit_code = 1

    finally:
        # 顺序：停流（含零速保持） → 恢复权限 → 关闭硬件；异常路径也不跳过
        try:
            hardware.stop_cmd_vel_stream()
        except Exception as exc:  # noqa: BLE001 - 停流失败不能吞掉后面的恢复
            print(f"⚠️ 停流异常: {exc}")
        if not restore_vel_control(hardware, original_state, switched, after_switch):
            exit_code = EXIT_RESTORE_FAILED
        try:
            hardware.shutdown()
            print("✅ 硬件已关闭")
        except Exception as exc:  # noqa: BLE001
            print(f"⚠️ 硬件关闭异常: {exc}")

    return exit_code


if __name__ == '__main__':
    print(f"\n{'='*70}")
    print("JiBot 底盘原生速度通道验证（会驱动底盘运动）")
    print(f"{'='*70}")

    sys.exit(main())
