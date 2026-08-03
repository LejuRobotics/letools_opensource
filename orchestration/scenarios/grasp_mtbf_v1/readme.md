# grasp_mtbf_v1 — 搬箱场景（LeTools 版）

从 `embodied` 迁移的单轮搬箱流程。

```
look_for_tag 子树（Parallel: NodeHeadSequence ∥ NodePercep ∥ NodeWaitForBlackboard）
  → 找到箱子 tag，写 latest_tag_<pick_tag_id>
walk_to_tag 子树（NodeComputePickGoal → NodeWheelWalk）
  → 站位 = tag.odom + stand_in_tag_pos，cmd_pos_world 走过去
PrepareArmForGraspMove（手臂到预抓取位，fixed_base→ARM_ONLY）
grasp_box_lb 子树
  → CalcLegMove(offset_z=0.15) 蹲下
  → CalcArmPoseMove(box_grasp_step1) 算双臂关键点（tag 系）
  → MoveArmBaseTargetPoseMove(close='1,2') 逐点下发、夹爪在关键点 1/2 闭合
  → BaseMoveRelativeJibotMove(-0.3m) 抱箱后退
CalcLegMove(offset_z=1.2) 站立抬箱
look_for_tag 子树（找放置 tag，写 latest_tag_<place_tag_id>）
walk_to_tag 子树（走到放置站位）
CalcLegMove(offset_z=1.25) 蹲到放置高度
place_box_lb 子树
  → CalcArmPoseMove(box_place)
  → MoveArmBaseTargetPoseMove(open='0,1') 张开夹爪放箱
BaseMoveRelativeJibotMove(-0.5m) 后退
CalcLegMove(offset_z=0.8) 恢复站姿
PrepareArmForGraspMove 手臂复位
```

## 黑板

| key | 写入 | 读取 | 内容 |
|---|---|---|---|
| `latest_tag_<id>` | NodePercep / NodeWaitForBlackboard | NodeComputePickGoal、MoveArmBaseTargetPoseMove | TagDetection（odom 系） |
| `latest_tag_<id>_version` | NodePercep | — | tag 更新计数 |
| `walk_goal` | NodeComputePickGoal | NodeWheelWalk | 底盘目标 Pose（odom 系） |
| `is_walk_goal_new` | NodeComputePickGoal | NodeWheelWalk | 新目标标志 |
| `ArmPoseAndWrench` | CalcArmPoseMove | MoveArmBaseTargetPoseMove | 双臂关键点（tag 系）+ wrench（全零保留字段） |
| `ArmMoveResult` | MoveArmBaseTargetPoseMove | — | 手臂移动是否成功 |

## 配置

需优先修改对应 config 中的 apriltag_tags.yaml，apriltag_settings.yaml，camera_config.yaml 等配置文件以适配场景下的二维码识别

## 运行

```bash
# dry-run 结构验证
python3 apps/test_upper_init/run_behavior_tree_json.py \
  --scenario orchestration/scenarios/grasp_mtbf_v1 --dry-run --tick-once

# 真机
python3 apps/test_upper_init/run_behavior_tree_json.py \
  --scenario orchestration/scenarios/grasp_mtbf_v1
```

## 调参入口（board.json）

| 参数 | 默认 | 说明 |
|---|---|---|
| `pick_tag_id` / `place_tag_id` | 4 / 0 | 取/放两侧的 tag ID（按现场改） |
| `tag_ids` | [4, 9] | NodePercep 监听列表 |
| `box_length/width/height` | 0.4/0.3/0.25 | 箱子尺寸（米） |
| `pick_stand_in_tag_pos` | [0,0,0.6] | 取箱站位在 tag 系的偏移 |
| `place_stand_in_tag_pos` | [0,0,0.8] | 放箱站位在 tag 系的偏移 |
| `use_virtual_tag` | false | true 时不等真 tag，直接用 `virtual_tag_pose_in_odom`（调试） |
其余相关节点参数等参考对应 node 文件夹下 define_manifest

## dry-run 状态

`--dry-run --tick-once` 通过：整树加载 + 单次 tick 根状态 SUCCESS。
