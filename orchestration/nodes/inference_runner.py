#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""独立推理脚本（由 InferenceControl 节点以子进程方式调用）。

在 letools conda 环境中运行，负责：
- 初始化 ROS 节点
- 创建 Kuavo 推理环境（Kuavo-Sim / Kuavo-Real）
- 加载策略（ACT / Diffusion / client）
- 运行推理循环，检测夹爪闭合
- 通过 stdout 输出状态标记，供父进程（行为树节点）监控

状态标记协议（stdout，每行一个）：
  [INFER] started              — 推理已启动
  [INFER] step <n>             — 第 n 步完成
  [INFER] grip <val>           — 当前夹爪值
  [INFER] gripper_closed        — 检测到夹爪闭合
  [INFER] done                 — 推理完成（抓取成功）
  [INFER] error <message>      — 推理出错
  [INFER] timeout              — 达到最大步数未完成

退出码：0=成功，1=失败
"""

import argparse
import os
import sys
import time
import json

import numpy as np

# 确保项目根目录在 PYTHONPATH 中
_STUDIO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
_LETOOLS_ROOT = os.path.join(_STUDIO_ROOT, "LeTools-Learning")
for p in [_STUDIO_ROOT, _LETOOLS_ROOT]:
    if p not in sys.path:
        sys.path.insert(0, p)


def _emit(msg):
    """输出状态标记到 stdout（立即 flush）。"""
    print(f"[INFER] {msg}", flush=True)


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
    parser.add_argument("--gripper-close-threshold", type=float, default=0.3, help="夹爪闭合阈值")
    parser.add_argument("--gripper-hold-frames", type=int, default=5, help="连续多少帧判定闭合")
    parser.add_argument("--post-close-steps", type=int, default=10, help="闭合后再保持步数")
    args = parser.parse_args()

    try:
        _run_inference(args)
    except Exception as e:
        import traceback
        traceback.print_exc()
        _emit(f"error {e}")
        sys.exit(1)


def _run_inference(args):
    import rospy
    import gymnasium as gym
    import torch
    import numpy as np

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

    max_steps = int(inf.max_episode_steps)
    close_threshold = args.gripper_close_threshold
    hold_frames = args.gripper_hold_frames
    post_close_steps = args.post_close_steps
    gripper_pre = args.gripper_pre_position

    # ---------- 夹爪索引 ----------
    which_arm = env_cfg.which_arm
    if which_arm == "both":
        grip_idx = 15
    elif which_arm in ("left", "right"):
        grip_idx = 7
    else:
        raise ValueError(f"Unsupported which_arm: {which_arm}")

    # ---------- 初始化 ROS ----------
    rospy.init_node("inference_control_subprocess", anonymous=True)

    # ---------- 创建环境 ----------
    env_name = "Kuavo-Sim" if env_cfg.inference_env == "sim" else "Kuavo-Real"
    _emit(f"creating env: {env_name} (which_arm={which_arm}, ros_rate={env_cfg.ros_rate})")
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
    _emit(f"policy loaded: {inf.policy_type}")

    # ---------- reset + 半开夹爪 ----------
    policy.reset()
    _, _ = env.reset()
    _send_claw(gripper_pre, gripper_pre)
    time.sleep(1.0)
    observation = env.get_obs()
    _emit("started")

    # ---------- 推理循环 ----------
    step_count = 0
    hold_count = 0
    gripped = False
    post_close_count = 0
    last_step_time = time.time()

    while True:
        # 完成
        if gripped and post_close_count >= post_close_steps:
            _emit("done")
            env.close()
            return

        # 超时
        if step_count >= max_steps:
            _emit("timeout")
            env.close()
            sys.exit(1)

        # 节流
        now = time.time()
        if now - last_step_time < 1.0 / ros_rate:
            time.sleep(0.001)
            continue

        # 一步推理
        obs = observation
        if not is_client:
            obs = inject_task_prompt(obs, inf.task_prompt)
        obs = preprocessor(obs)

        with torch.inference_mode():
            action = policy.select_action(obs)
        action = postprocessor(action)
        numpy_action = action.squeeze(0).cpu().numpy()

        observation, _, _, _, _ = env.step(numpy_action)
        step_count += 1
        last_step_time = time.time()

        # 夹爪判定
        grip_val = _get_gripper_value(observation, grip_idx)
        if grip_val is not None:
            _emit(f"grip {grip_val:.3f}")
            if grip_val < close_threshold:
                hold_count += 1
            else:
                hold_count = 0
            if not gripped and hold_count >= hold_frames:
                gripped = True
                hold_count = 0
                _emit("gripper_closed")
                _emit(f"post_close for {post_close_steps} steps")

        if gripped:
            post_close_count += 1

        if step_count % 10 == 0:
            _emit(f"step {step_count}")


def _get_gripper_value(obs, grip_idx):
    state = obs.get("observation.state")
    if state is None:
        return None
    if hasattr(state, "detach"):
        state = state.detach().cpu().numpy()
    state = np.asarray(state).reshape(-1)
    if len(state) <= grip_idx:
        return None
    return float(state[grip_idx])


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
