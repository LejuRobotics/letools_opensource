#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""独立推理脚本（由 InferenceControl 节点以子进程方式调用）。

在 letools conda 环境中运行，负责：
- 初始化 ROS 节点
- 创建 Kuavo 推理环境（Kuavo-Sim / Kuavo-Real）
- 加载策略（ACT / Diffusion / client）
- 运行推理循环，以夹爪驱动反馈确认夹住物体
- 通过 stdout 输出状态标记，供父进程（行为树节点）监控

状态标记协议（stdout，每行一个）：
  [INFER] started              — 推理已启动
  [INFER] step <n>             — 第 n 步完成
  [INFER] done                 — 驱动持续报告夹住物体，推理完成
  [INFER] error <message>      — 推理出错
  [INFER] timeout              — 达到最大步数未完成

退出码：0=成功，1=失败
"""

import argparse
import atexit
import os
import sys
import time
import json
import packaging.version


# 确保项目根目录在 PYTHONPATH 中
_STUDIO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
_LETOOLS_ROOT = os.path.join(_STUDIO_ROOT, "LeTools-Learning")
_NODES_DIR = os.path.join(_STUDIO_ROOT, "orchestration", "nodes")
_DEFAULT_GRASP_FEEDBACK_CONFIG = os.path.join(
    _STUDIO_ROOT,
    "orchestration", "scenarios", "grasp_ring_pick_place", "board.json",
)
for p in [_STUDIO_ROOT, _LETOOLS_ROOT, _NODES_DIR]:
    if p not in sys.path:
        sys.path.insert(0, p)


def _emit(msg):
    """输出状态标记到 stdout（立即 flush）。"""
    print(f"[INFER] {msg}", flush=True)


def _load_feedback_config(path):
    """Load feedback config from a standalone file or a scenario board."""
    with open(path, encoding="utf-8") as stream:
        data = json.load(stream)
    if isinstance(data, dict) and isinstance(data.get("grasp_feedback"), dict):
        return data["grasp_feedback"]
    return data


def main():
    parser = argparse.ArgumentParser(description="Kuavo 推理子进程")
    parser.add_argument("--config", required=True, help="推理配置文件路径")
    parser.add_argument("--pretrained-path", default="", help="策略权重路径")
    parser.add_argument("--policy-type", default="", help="策略类型 (act/diffusion/client)")
    parser.add_argument("--task-prompt", default="", help="任务提示词")
    parser.add_argument("--max-episode-steps", type=int, default=0, help="最大推理步数")
    parser.add_argument("--device", default="", help="推理设备 (cuda/cpu)")
    parser.add_argument("--inference-env", default="", help="推理环境 (sim/real)")
    parser.add_argument("--gripper-pre-position", type=float, default=50, help="夹爪半开位置")
    parser.add_argument(
        "--grasp-feedback-config",
        default=_DEFAULT_GRASP_FEEDBACK_CONFIG,
        help="夹爪反馈成功判据配置；可直接指向场景 board.json",
    )
    args = parser.parse_args()

    try:
        _run_inference(args)
    except Exception as e:
        import traceback
        traceback.print_exc()
        _emit(f"error {e}")
        sys.exit(1)


def _run_inference(args):
    setup_started = time.monotonic()
    import rospy
    import gymnasium as gym
    import torch

    import kuavo_deploy.kuavo_env  # noqa: F401  注册 Kuavo-Sim/Kuavo-Real
    from kuavo_deploy.config import load_kuavo_config
    from kuavo_deploy.utils.policy_loader import (
        load_native_policy_bundle,
        inject_task_prompt,
    )

    # ---------- 加载配置 ----------
    cfg = load_kuavo_config(args.config)
    inf = cfg.inference
    env_cfg = cfg.env

    # 用命令行参数覆盖配置
    if args.policy_type:
        inf.policy_type = args.policy_type
    if args.pretrained_path:
        inf.pretrained_path = args.pretrained_path
    if args.task_prompt:
        inf.task_prompt = args.task_prompt
    if args.max_episode_steps:
        inf.max_episode_steps = args.max_episode_steps
    if args.device:
        inf.device = args.device
    if args.inference_env:
        env_cfg.inference_env = args.inference_env

    # 创建机器人环境前，先校验夹爪反馈判断配置。
    feedback_config = _load_feedback_config(args.grasp_feedback_config)
    # 直接导入同目录模块，使推理子进程独立加载反馈判断功能。
    from grasp_feedback import GraspFeedbackState, RosGraspFeedbackObserver
    GraspFeedbackState(feedback_config)

    max_steps = int(inf.max_episode_steps)
    gripper_pre = args.gripper_pre_position

    which_arm = env_cfg.which_arm
    if which_arm not in ("both", "right"):
        raise ValueError("当前圆环判据监控右夹爪，仅支持 which_arm=both/right")

    # 逐步动作明细写文件，避免父进程逐行读取 stdout 时堵塞推理。
    import logging

    log_dir = os.path.join(_STUDIO_ROOT, "log")
    os.makedirs(log_dir, exist_ok=True)
    log_path = os.path.join(log_dir, f"inference_{time.strftime('%Y%m%d_%H%M%S')}_{os.getpid()}.log")
    robot_logger = logging.getLogger("robot")
    detail_handler = logging.FileHandler(log_path, encoding="utf-8")
    detail_handler.setFormatter(logging.Formatter("%(asctime)s %(levelname)s %(message)s"))
    robot_logger.addHandler(detail_handler)
    for handler in robot_logger.handlers:
        if isinstance(handler, logging.StreamHandler) and not isinstance(handler, logging.FileHandler):
            handler.setLevel(logging.WARNING)
    robot_logger.propagate = False
    _emit(f"detail_log {log_path}")

    # ---------- 初始化 ROS ----------
    rospy.init_node("inference_control_subprocess", anonymous=True)

    # ---------- 创建环境 ----------
    env_name = "Kuavo-Sim" if env_cfg.inference_env == "sim" else "Kuavo-Real"
    _emit(f"正在创建推理环境：{env_name} (which_arm={which_arm}, ros_rate={env_cfg.ros_rate})")
    env = gym.make(env_name, max_episode_steps=max_steps, config=cfg)
    while hasattr(env, "env"):
        env = env.env
    ros_rate = float(env_cfg.ros_rate)

    # ---------- 加载策略 ----------
    device = torch.device(inf.device)
    is_client = inf.policy_type == "client"
    if is_client:
        from kuavo_deploy.kuavo_service.client import PolicyClient
        policy = PolicyClient(task_prompt=inf.task_prompt)
        preprocessor = lambda obs: obs
        postprocessor = lambda action: action
    else:
        policy, preprocessor, postprocessor, _ = load_native_policy_bundle(
            pretrained_path=inf.pretrained_path, device=device, strict=True
        )
    if inf.policy_type == "act" and inf.act_n_action_steps is not None:
        if policy.config.temporal_ensemble_coeff is not None:
            raise ValueError("覆盖 act_n_action_steps 前需关闭时序集成（temporal_ensemble_coeff）")
        if inf.act_n_action_steps > policy.config.chunk_size:
            raise ValueError("act_n_action_steps 不能大于模型的 chunk_size")
        policy.config.n_action_steps = inf.act_n_action_steps
    _emit(f"策略已加载：{inf.policy_type} ({inf.pretrained_path})")
    if inf.policy_type == "act":
        _emit(f"ACT chunk_size={policy.config.chunk_size} n_action_steps={policy.config.n_action_steps} ros_rate={ros_rate:g} max_steps={max_steps}")

    # ---------- 环境复位与夹爪半开准备 ----------
    policy.reset()
    _, _ = env.reset()
    _send_claw(gripper_pre, gripper_pre)
    time.sleep(1.0)
    observation = env.get_obs()

    feedback_observer = RosGraspFeedbackObserver(feedback_config)
    atexit.register(feedback_observer.close)
    _emit(f"grasp_feedback topic={feedback_config['claw_topic']}; 右夹爪运动或到位后，state=3连续保持{feedback_config['hold_seconds']:g}秒确认抓取成功")
    _emit(f"setup_seconds {time.monotonic() - setup_started:.3f}")
    _emit("started")

    # ---------- 推理循环 ----------
    step_count = 0
    last_step_time = time.time()
    run_started = time.monotonic()

    try:
        while not rospy.is_shutdown():
            feedback = feedback_observer.evaluate()
            if feedback["success_allowed"]:
                _emit(f"claw_grabbed state=3 held={feedback['hold_seconds']:.3f}s")
                _emit("done")
                return

            if step_count >= max_steps:
                _emit("timeout")
                sys.exit(1)

            # 节流
            now = time.time()
            if now - last_step_time < 1.0 / ros_rate:
                time.sleep(0.001)
                continue

            # 一步推理
            predict_started = time.monotonic()
            obs = observation
            if not is_client:
                obs = inject_task_prompt(obs, inf.task_prompt)
            obs = preprocessor(obs)

            with torch.inference_mode():
                action = policy.select_action(obs)
            action = postprocessor(action)
            numpy_action = action.squeeze(0).cpu().numpy()

            predict_seconds = time.monotonic() - predict_started
            step_started = time.monotonic()
            observation, _, _, _, _ = env.step(numpy_action)
            step_count += 1
            last_step_time = time.time()

            if step_count % 10 == 0:
                _emit(f"step {step_count}")
                elapsed = time.monotonic() - run_started
                _emit(f"timing elapsed={elapsed:.3f}s effective_hz={step_count / elapsed:.2f} "
                      f"predict_last={predict_seconds:.3f}s env_step_last={time.monotonic() - step_started:.3f}s")
            if inf.policy_type == "act" and (step_count - 1) % policy.config.n_action_steps == 0:
                _emit(f"chunk_predict step={step_count} seconds={predict_seconds:.3f}")
        raise RuntimeError("ROS 已关闭，抓取确认尚未完成")
    finally:
        feedback_observer.close()
        env.close()


def _send_claw(left_pos, right_pos):
    import rospy
    from kuavo_msgs.msg import lejuClawCommand, endEffectorData

    pub = rospy.Publisher("/leju_claw_command", lejuClawCommand, queue_size=1)
    msg = lejuClawCommand()
    msg.data = endEffectorData()
    msg.data.name = ["left_claw", "right_claw"]
    msg.data.position = [float(left_pos), float(right_pos)]
    msg.data.velocity = [50.0, 50.0]
    msg.data.effort = [1.0, 1.0]
    pub.publish(msg)


if __name__ == "__main__":
    main()
