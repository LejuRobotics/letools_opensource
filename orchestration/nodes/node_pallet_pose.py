# -*- coding: utf-8 -*-
"""NodePalletPose：读黑板上的 tag 位姿 → 反推木托盘位姿 → 写回黑板。

上游是 `NodePercep`（它把 `PerceptionAdapter` 的检测写进 `latest_tag_<id>`）。
本节点在它下游补上「tag → 托盘」这一环，输出托盘位姿供抓取类节点消费。
`NodePercep` 之前的那段（检测、相机、TF）一律复用，本文件不碰。

**与 NodePercep 的一个区别：本节点不访问硬件。** tag 位姿已经在黑板上，
托盘位姿是纯计算，所以这里连 `get_shared_hardware()` 都不调 —— 也正因为
如此，它可以脱离 ROS 单测（见下方）。

黑板契约（照抄 NodePercep 的写法，便于和其他节点共存）
    读  latest_tag_<id>            TagDetection
    读  latest_tag_<id>_version    int，用来判断 tag 是否更新过
    写  <key>                      Pose6D，托盘位姿（默认 key 是 latest_pallet）
    写  <key>_version              int，每算一次自增

节点持续返回 RUNNING（与 NodePercep 一样，由 Parallel 父节点决定何时收），
并且在 tag 版本号没变时不重复计算 —— 与 `NodeSourceTagToArmGoalSingleTag`
用的是同一套版本门禁。

标定结果从 `config/pallet_tag.yaml` 读（由 `pallet_calibrate.py` 生成），
**不把矩阵写进场景 JSON** —— 矩阵太长、太容易改错，而且标定结果属于现场
数据，不该跟场景定义混在一起。

运行与测试
----------
    # 单测：不需要硬件、不需要 ROS，CI 会跑这条
    # （.gitlab-ci.yml 的 verify:opensource → pytest orchestration/nodes/tests/ -m unit）
    pytest orchestration/nodes/tests/test_node_pallet_pose.py -m unit -v

    # 在场景里 dry-run：验证能被工厂按类名解析、黑板键注册成功
    python3 apps/test_upper_init/run_behavior_tree_json.py \
        --scenario orchestration/scenarios/<场景目录> --dry-run --tick-once

    # 真机跑：先确认两件事，否则这个节点会直接 FAILURE
    #   1. config/camera_config.yaml 里 launch_apriltag: true
    #      （现在是 false，此时 hardware.perception 是 None，NodePercep 会先崩）
    #   2. config/apriltag_tags.yaml 里有你要用的两个 tag id 和尺寸
    python3 apps/test_upper_init/run_behavior_tree_json.py \
        --scenario orchestration/scenarios/<场景目录>
"""
from pathlib import Path

import py_trees
import yaml
from py_trees.common import Status

from core.common.logger import get_logger
from orchestration.nodes.base_node import BaseAction
from orchestration.nodes.utils.blackboard import (
    is_dry_run,
    read_blackboard,
    read_version,
)
from skills.atomic.perception.pallet_pose.skill import (
    PalletPoseParams,
    PalletPoseSkill,
)

logger = get_logger(__name__)

# 基于源码位置解析，不依赖进程工作目录（框架里 lifecycle_mixin 也是这么做的）
_DEFAULT_CONFIG = Path(__file__).resolve().parents[2] / "config" / "pallet_tag.yaml"


class NodePalletPose(BaseAction):
    """由若干 tag 的位姿反推木托盘位姿，写到黑板。

    params:
        tag_ids      参与反识别的 tag id 列表，如 [0, 1]
        key          写黑板的键名，默认 latest_pallet
        window       时间窗宽度，0（默认）表示不平滑
        config_path  标定结果路径，默认 config/pallet_tag.yaml
    """

    def __init__(self, name, label, namespace, params):
        super(NodePalletPose, self).__init__(name, label, namespace, params)
        self._tag_ids = [int(t) for t in self.params.get("tag_ids", [])]
        self._key = str(self.params.get("key", "latest_pallet"))
        self._window = int(self.params.get("window", 0))
        self._config_path = Path(str(self.params.get("config_path", _DEFAULT_CONFIG)))

        self._pallet_tag = {}
        self._skill = None
        self._version_seen = None
        # "这个 tag 键从未被写过"只说一次（按 id 去重），见 `update()`。
        self._missing_warned = set()

        for tag_id in self._tag_ids:
            self.global_blackboard.register_key(
                key=f"latest_tag_{tag_id}", access=py_trees.common.Access.READ
            )
            self.global_blackboard.register_key(
                key=f"latest_tag_{tag_id}_version", access=py_trees.common.Access.READ
            )
        for key in (self._key, f"{self._key}_version"):
            self.global_blackboard.register_key(
                key=key, access=py_trees.common.Access.WRITE
            )
        # 先置初值，免得其他节点 getattr 时 KeyError（register_key 在 py_trees 2.x
        # 里只注册权限，不创建值）—— 这是 NodePercep 踩过并写在注释里的坑
        self.global_blackboard.set(self._key, None)
        setattr(self.global_blackboard, f"{self._key}_version", 0)

    # "这个键从未被写过"的哨兵。不能拿 `None` 或 0 代替：`None` 是"写过、值是
    # None"，0 是"写过、值是 0" —— 而这条判断要的正是"**从来没写过**"。
    # 与 `node_pallet_servo.NodePalletServo._MISSING` 同一个理由、同一个写法。
    _MISSING = object()

    def initialise(self):
        self._skill = None
        self._version_seen = None
        self._missing_warned = set()     # 一次性警告按运行重整
        self._pallet_tag = self._load_calibration()
        if self._pallet_tag:
            self.feedback_message = f"已加载标定: tag {sorted(self._pallet_tag)}"
            missing = [t for t in self._tag_ids if t not in self._pallet_tag]
            if missing:
                self.feedback_message += f"；未标定、将被忽略: {missing}"
        else:
            self.feedback_message = (
                f"没有读到标定结果 {self._config_path} —— "
                f"先跑 pallet_calibrate.py 生成它"
            )

    def _load_calibration(self):
        """读 config/pallet_tag.yaml 里的 per-tag 变换。读不到就返回空。"""
        try:
            with open(self._config_path, "r", encoding="utf-8") as stream:
                data = yaml.safe_load(stream) or {}
        except OSError:
            return {}
        by_tag = data.get("T_pallet_tag_by_marker") or {}
        table = {}
        for key, value in by_tag.items():
            try:
                table[int(key)] = value
            except (TypeError, ValueError):
                continue
        return table

    def update(self):
        if is_dry_run():
            return Status.SUCCESS
        if not self._pallet_tag:
            return Status.FAILURE

        tag_poses = {}
        newest_version = None
        for tag_id in self._tag_ids:
            if tag_id not in self._pallet_tag:
                # 没标定过的 id 直接跳过。这不算错误 —— 可能只是那个 tag 还没
                # 来得及标，或者场景里配了一个暂时用不上的 id。让它走到底再被
                # 技能判失败的话，节点会在"等一个永远不会来的标定"时反复 FAILURE。
                continue
            detection = read_blackboard(self.global_blackboard,
                                         f"latest_tag_{tag_id}", self._MISSING)
            if detection is self._MISSING:
                # 键**从未被写过** —— 上游的 `tag_ids` 里没有这个 id（或者树上
                # 压根没有那个生产者）。以前这里抛 `KeyError` 掀翻整棵树；现在
                # 只是"没有数据"。但**必须喊一次**：否则故障从"当场炸"变成
                # "永远等"，而后者更难查（feedback 只有一句"等 tag"）。按 id
                # 去重，不是每 tick 一条。
                if tag_id not in self._missing_warned:
                    self._missing_warned.add(tag_id)
                    logger.warning(
                        "latest_tag_%d **从未被写过** —— 本节点的 tag_ids=%s，"
                        "而上游生产者的 tag_ids 必须把它全覆盖（两个列表要一致）。"
                        "现在这个 id 永远不会参与反算。", tag_id, self._tag_ids)
                continue
            if detection is None:
                continue
            pose = getattr(detection, "pose_in_world", None)
            if pose is None:
                continue
            tag_poses[tag_id] = pose
            newest_version = max(newest_version or 0,
                                 read_version(self.global_blackboard,
                                               f"latest_tag_{tag_id}_version"))

        if not tag_poses:
            if self._missing_warned:
                # 把"等不到"与"永远不会来"在 feedback 里分开 —— 两者都是一句
                # "等 tag" 的话，界面上看不出是还没检测到还是配上错了。
                missing = ",".join(str(i) for i in sorted(self._missing_warned))
                self.feedback_message = (
                    f"等 tag —— 但 id {missing} 的 latest_tag_<id> "
                    f"**从未被写过**（上游的 tag_ids 没覆盖它们），"
                    f"这些 id 永远不会来；本节点 tag_ids={self._tag_ids}")
            else:
                self.feedback_message = "等 tag（黑板上还没有可用位姿）"
            return Status.RUNNING

        # 版本没动就不再重算：否则每个 tick（50 Hz）都会白算一遍
        if newest_version is not None and newest_version == self._version_seen:
            return Status.RUNNING

        if self._skill is None:
            self._skill = PalletPoseSkill(window=self._window)

        init = self._skill.initialize(PalletPoseParams(
            tag_poses=tag_poses, pallet_tag=self._pallet_tag))
        if not init.success:
            self.feedback_message = init.message
            return Status.FAILURE

        result = self._skill.execute()
        if not result.success or not result.data:
            self.feedback_message = result.message
            return Status.FAILURE

        self.global_blackboard.set(self._key, result.data["pose"])
        current = getattr(self.global_blackboard, f"{self._key}_version", 0)
        setattr(self.global_blackboard, f"{self._key}_version", current + 1)
        self._version_seen = newest_version
        self.feedback_message = result.message
        return Status.RUNNING
