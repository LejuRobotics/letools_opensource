# -*- coding: utf-8 -*-
"""把离线点点产出的模拟输入写进黑板，冒充 ``NodePalletPose`` 与 YOLO 适配器。

**只为离线联调存在。** 真机上 ``latest_pallet`` 由 ``NodePalletPose``（二维码
反算）写、``latest_box_obs`` 由 YOLO 适配器写，这个节点不参与——用 ``enabled:
false`` 把它关掉即可。

``write_pallet: false`` 是**真机过渡期**用的第三种状态：托盘已经是真数据
（``NodePalletPose`` 从二维码反算），只有箱子还得靠手点 —— 此时这个节点只写
``latest_box_obs``，把 ``latest_pallet`` 整个让给真生产者。不加这个开关的话，
两边会同时往 ``latest_pallet`` 上写，伺服读到哪个取决于 tick 顺序，**没有任何
报错**。详见 ``params`` 表里的 ``write_pallet``。

它写的是**真实链路里的那两个键**，所以伺服节点一行都不用改；这就是设计文档
要的"接缝留好，真机接入时下游代码一行不改"。

输入文件由 ``apps/test_camera_internal/pallet_servo_sim/pick_servo_inputs.py``
生成。``box`` 是**一条**时只写一次；是**列表**时逐 tick 播放一条、版本号递增
——这样离线就能看到伺服误差随箱子移动而"实时"变化。

本节点是个**源节点**，所以有东西可写时持续返回 **RUNNING**（`NodePercep` /
`NodePalletPose` 同口径：自己决定写什么，何时收由父节点定），播完也保持
RUNNING 但一个字都不写。**不能返回 SUCCESS**：py_trees 的
``Behaviour.tick()`` 在 ``status != RUNNING`` 时**每 tick 重进 ``initialise()``**，
而 ``initialise()`` 会把播放游标归零 —— 于是"列表"只剩第一条被反复重写（版本号
还一直涨，下游拿同一个框每帧重算），"播完停手"根本没发生过。同理，坏输入是
FAILURE（终点，父节点能反应），``enabled: false`` 则是 SUCCESS（不占着树，
真机上关掉它才不会被它卡住）。

与 ``node_inject_single_tag.py`` 的一个区别：那边把版本号**硬编码成 1**，于是
第二次写不会让下游的版本门禁打开。这里每写一次都自增，下游才会重算；自增的基数
取**黑板上的当前值**（``NodePalletPose.update`` 的原样），所以版本号单调，重进
``initialise()`` 也不会掉回去。

坏输入（读不到 / 不是合法 JSON / 字段不对）一律变成 **FAILURE + feedback**，
绝不把异常抛出 ``initialise()``：py_trees 的 ``Behaviour.tick()`` **不接**这两个
钩子抛出的异常，抛出去就是整棵树连伺服节点每帧的日志一起没 —— 比 FAILURE 糟得多。
"""
from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import py_trees
from py_trees.common import Status

from core.common.logger import get_logger
from core.common.transform import matrix_to_pose6d
from core.domain.pose import Pose6D
from orchestration.nodes.base_node import BaseAction
from orchestration.nodes.utils.blackboard import (
    bump_version,
    is_dry_run,
    read_version,
)
from skills.atomic.perception.pallet_servo.algorithm import (
    BoxObservation,
    parse_bool_param,
)

logger = get_logger(__name__)


_BOX_CORNER_FIELDS = ("u1", "v1", "u2", "v2")


def _parse_box(entry, index: int) -> Dict[str, Any]:
    """一条箱子观测 → 归一化后的 dict：数字先转好，`update()` 里不再有会抛的转换。"""
    where = f"box[{index}]"
    if not isinstance(entry, dict):
        raise ValueError(f"{where} 不是对象，是 {type(entry).__name__} —— "
                         f"每条箱子观测都得是 "
                         f"{{{', '.join(_BOX_CORNER_FIELDS)}}} 这样的对象")
    for field in _BOX_CORNER_FIELDS:
        if field not in entry:
            raise ValueError(f"{where} 缺 `{field}` —— 每条箱子观测都必须有 "
                             f"{'/'.join(_BOX_CORNER_FIELDS)}（框的两个角）")
    parsed: Dict[str, Any] = {}
    for field in _BOX_CORNER_FIELDS:
        try:
            parsed[field] = float(entry[field])
        except (TypeError, ValueError) as exc:
            raise ValueError(f"{where} 的 `{field}` 不是数字"
                             f"（{entry[field]!r}）：{exc}")
    for field, fallback in (("confidence", 1.0), ("stamp", 0.0)):
        try:
            parsed[field] = float(entry.get(field, fallback))
        except (TypeError, ValueError) as exc:
            raise ValueError(f"{where} 的 `{field}` 不是数字：{exc}")
    # quad 原样带过去：本节点不解释它，"四个角不合规"由伺服算法那边 Reject
    parsed["quad"] = entry.get("quad")
    parsed["label"] = str(entry.get("label", ""))
    return parsed


def _parse_sim_payload(payload,
                       want_pallet: bool = True
                       ) -> Tuple[Optional[Pose6D], List[Dict[str, Any]]]:
    """把工具写出的 payload 拧成 (托盘位姿, 箱子观测列表)。

    任何字段不对都抛 **`ValueError` 并点名是哪个字段** —— 抽成独立函数是为了
    `initialise()` 的一个 `try` 就能盖住所有字段，而不是每读一个键包一层。

    `want_pallet=False` 时**不读也不校验** `pallet_pose`（那份数据这个节点不写，
    校验它只会凭空多一个失败模式），位姿返回 `None`。`box` 任何时候都要。
    """
    if not isinstance(payload, dict):
        raise ValueError(f"顶层不是 JSON 对象，是 {type(payload).__name__}")
    pose: Optional[Pose6D] = None
    if want_pallet:
        if "pallet_pose" not in payload:
            raise ValueError("没有 `pallet_pose` 字段（托盘位姿）")
        try:
            values = [float(v) for v in payload["pallet_pose"]]
        except (TypeError, ValueError) as exc:
            raise ValueError(f"`pallet_pose` 不是一串数字"
                             f"（{payload['pallet_pose']!r}）：{exc}")
        if len(values) != 6:
            raise ValueError(f"`pallet_pose` 要 6 个数（x, y, z, yaw, pitch, roll），"
                             f"实际 {len(values)} 个：{payload['pallet_pose']!r}")
        pose = Pose6D(*values)
    if "box" not in payload:
        raise ValueError("没有 `box` 字段（箱子观测；一条对象或一个列表）")
    raw = payload["box"]
    if raw is None:
        raise ValueError("`box` 是 null —— 要一条箱子观测（对象）或一个列表")
    entries = list(raw) if isinstance(raw, list) else [raw]
    return pose, [_parse_box(entry, i) for i, entry in enumerate(entries)]


class NodeInjectServoInput(BaseAction):
    """把一份模拟输入写到黑板上的 ``latest_pallet`` 与 ``latest_box_obs``。

    状态：有东西可写 / 播完了都返回 **RUNNING**（源节点口径，见模块 docstring），
    坏输入返回 FAILURE，``enabled: false`` 与 dry-run 返回 SUCCESS。

    params:
        sim_path      由 pick_servo_inputs.py 生成的 JSON（必填）
        pallet_key    写托盘位姿的键，默认 latest_pallet
        box_key       写箱子观测的键，默认 latest_box_obs
        write_pallet  **本节点写不写托盘位姿**，默认 true。
                      false = 只写箱子，`latest_pallet` 整个让给真生产者
                      （真机过渡期：托盘走 apriltag、箱子还在手点）。
                      此时 JSON 里的 `pallet_pose` 不读也不校验 —— 它不归本
                      节点管，为它失败只会凭空多一个失败模式。
                      ⚠️ 不设这个开关而让 `NodePalletPose` 与本节点同时在树上，
                      两者会**同时往 `latest_pallet` 上写**，伺服读到哪一份取决于
                      tick 顺序，**没有任何报错**。
        enabled       false 时什么都不写，默认 true（真机上用它关掉本节点）。
                      认 JSON 布尔值和 true/false、1/0、yes/no 字符串；别的值
                      **按关掉处理**并报错
    """

    def __init__(self, name, label, namespace, params):
        super().__init__(name, label, namespace, params)
        self._sim_path = Path(str(self.params.get("sim_path", "")))
        self._pallet_key = str(self.params.get("pallet_key", "latest_pallet"))
        self._box_key = str(self.params.get("box_key", "latest_box_obs"))
        try:
            # 解析器在算法层（`parse_bool_param`），与 `node_pallet_servo` 的
            # `use_distortion` **共用同一份实现**：字符串布尔这个坑两边都有，
            # 各写一遍就是分叉的入口。这里解释不了的值 = 抛 ValueError →
            # 关掉 + 报错（见下），严的方向是安全的。
            self._enabled = parse_bool_param(self.params.get("enabled", True),
                                             "enabled")
            self._enabled_error = ""
        except ValueError as exc:
            # 解释不了 = **关掉**，但必须喊出来：静默按 true 跑才是要命的那半边
            # （真机上跑起来就再也停不下来）。feedback 里再说一遍，见 initialise。
            self._enabled = False
            self._enabled_error = str(exc)
            logger.error("NodeInjectServoInput 参数错误：%s —— 按 enabled=false 处理",
                         exc)
        # 解释不了同样按**默认 true** 之外的安全方向走：写托盘是默认行为，报错
        # 就退回默认，跟 `enabled` 那半边"从严"的方向相反 —— 因为这里的"严"
        # 是**什么都不写**，那会让下游永远等输入，比写多了更难查。
        try:
            self._write_pallet = parse_bool_param(
                self.params.get("write_pallet", True), "write_pallet")
        except ValueError as exc:
            self._write_pallet = True
            logger.error("NodeInjectServoInput 参数错误：%s —— 按 write_pallet=true 处理",
                         exc)

        # 只注册**自己会写的**键。不写托盘时连键都不碰：`NodePalletPose` 负责
        # 注册并置初值，本节点再 set 一次 None 会把它的初值覆盖掉（构造顺序决定
        # 谁后写，而"谁后写"不该影响结果）。
        keys = [self._box_key, f"{self._box_key}_version"]
        if self._write_pallet:
            keys += [self._pallet_key, f"{self._pallet_key}_version"]
        for key in keys:
            self.global_blackboard.register_key(
                key=key, access=py_trees.common.Access.WRITE)
        # 先置初值，免得别的节点 `getattr` 时 KeyError：`register_key` 在
        # py_trees 2.x 里只注册权限、不创建值（`NodePalletPose.__init__` 为同一个
        # 坑写的同一段）。本节点的用途就是"和真生产者长得一模一样"，照抄。
        self.global_blackboard.set(self._box_key, None)
        setattr(self.global_blackboard, f"{self._box_key}_version", 0)
        if self._write_pallet:
            self.global_blackboard.set(self._pallet_key, None)
            setattr(self.global_blackboard, f"{self._pallet_key}_version", 0)

        self._boxes = []
        self._cursor = 0
        self._pose: Optional[Pose6D] = None
        self._pallet_version = 0
        self._box_version = 0
        # 上一次喊过的坏输入原文。**只在这里初始化**：坏输入会让 `update()` 返回
        # FAILURE，py_trees 于是每 tick 重进 `initialise()` —— 在 initialise 里清
        # 它，去重就永远不会生效。
        self._last_reject: Optional[str] = None

    def _reject(self, message):
        """坏输入的统一出口：feedback_message 每 tick 更新，ERROR **只在原文变了时喊**。

        坏输入 → `update()` 返回 FAILURE → py_trees 每 tick 重进 `initialise()`，同一
        份坏文件于是按 tick 频率重读、重读一次喊一次（跑到 50 Hz 时现场只剩这一条）。
        伺服节点用 `_log_throttled` 解决同一个问题；这里按**原文比对**：消息里带着
        路径、点名到具体字段，还带异常原文，所以"输入真变了"自然会再喊一次。

        读文件本身**不省**：文件被修好之后必须能自己恢复，那正是重进 `initialise()`
        的唯一好处 —— 只把日志去了重。
        """
        self.feedback_message = message
        if message != self._last_reject:
            self._last_reject = message
            logger.error("%s", message)

    def initialise(self):
        self._boxes = []
        self._cursor = 0
        self._pose = None
        # 版本号接着**黑板上的当前值**涨（真生产者 `NodePalletPose.update` 就是
        # 这么写的）。这里归零会让重播时的版本从 3 掉回 1 —— 今天消费侧是等式
        # 门禁看不出，换成单调门禁就会把回退当成"陈旧数据"永远跳过。
        if self._write_pallet:
            self._pallet_version = read_version(
                self.global_blackboard, f"{self._pallet_key}_version")
        self._box_version = read_version(
            self.global_blackboard, f"{self._box_key}_version")
        if self._enabled_error:
            self._reject(f"enabled 参数不对（{self._enabled_error}）—— 已按"
                         f" enabled=false 关掉，一个键都不写：改场景 JSON 里的"
                         f" enabled 才能让它干活")
            return
        if not self._enabled:
            self.feedback_message = "enabled=false，不写任何东西"
            return
        try:
            payload = json.loads(self._sim_path.read_text(encoding="utf-8"))
        except OSError as exc:
            self._reject(
                f"读不到模拟输入 {self._sim_path}: {exc} —— 先跑 "
                f"apps/test_camera_internal/pallet_servo_sim/pick_servo_inputs.py")
            return
        except ValueError as exc:
            # `json.JSONDecodeError` 是 `ValueError`（`UnicodeDecodeError` 也是）：
            # 工具用的是**非原子**的 `write_text`，tick 撞上写盘就会读到半截文件。
            self._reject(
                f"模拟输入 {self._sim_path} 不是合法 JSON：{exc} —— 多半是 "
                f"pick_servo_inputs.py 正在写盘（非原子 write_text），等它写完；"
                f"手改过就重新生成一份")
            return
        try:
            self._pose, self._boxes = _parse_sim_payload(
                payload, want_pallet=self._write_pallet)
        except (OSError, ValueError, KeyError, TypeError) as exc:
            # 解析不了的输入**只许是 FAILURE**（update 里返回）：py_trees 的
            # `Behaviour.tick()` **不接** `initialise()`/`update()` 抛出的异常，
            # 抛出去就是整棵树（连伺服节点每帧的日志）一起没，比 FAILURE 糟得多。
            # 四个类都列上是因为它们各自真会发生：缺顶层键 → `KeyError`、位姿个数
            # 不对 → `TypeError`、值不是数字 → `ValueError`（`box: null` 变成
            # `[None]` 再取 `u1` 也是 `TypeError`）—— 而列表形状的 `box`
            # **只能手写**（工具永远写单条对象），手写就会漏字段。
            self._reject(f"模拟输入 {self._sim_path} 字段不对：{exc} —— "
                         f"用 pick_servo_inputs.py 重新生成一份，别手改")
            return
        # 装载成功：把"上次喊过什么"清掉，于是同一个坏输入**再次出现**时会再喊一次
        # （去重的是"同一份坏输入反复重读"，不是"永远只喊一次"）。
        self._last_reject = None

        # 设计文档 §6.1：离线点点这一层要记的东西。K/D/image_size/use_distortion
        # 也打出来：这份 JSON 的**唯一读者就是本节点**，而工具写 use_distortion
        # 就是为了让下游拿它对参数（"别两边默默分叉"）—— 与 NodePalletServo 的
        # params 不一致必须看得见，不能悄悄算错。
        logger.info(
            "模拟输入就绪：%s，托盘位姿 %s，箱子 %d 条，ref_edges=%s，"
            "pallet_size_mm=%s，use_distortion=%s，K=%s，D=%s，image_size=%s "
            "—— 拿去和 NodePalletServo 的 params 对一遍",
            self._sim_path,
            ("**不写**（write_pallet=false，托盘交给 NodePalletPose）"
             if not self._write_pallet
             else f"{[round(v, 4) for v in self._pose.to_list()]}（**冒充 base_link**）"),
            len(self._boxes), payload.get("ref_edges"),
            payload.get("pallet_size_mm"), payload.get("use_distortion"),
            payload.get("K"), payload.get("D"), payload.get("image_size"))
        self.feedback_message = (
            f"已装载 {self._sim_path.name}：{len(self._boxes)} 条箱子观测"
            + ("" if self._write_pallet else "（只写箱子，托盘归 NodePalletPose）"))

    def update(self):
        if self._enabled_error:
            # 参数错是**配置错误**，不是"这一帧没数据"：FAILURE 才在树上看得见
            # （dry-run 也照报 —— 配置错跟硬件的干系是两回事）。
            return Status.FAILURE
        if is_dry_run():
            # 与 `NodePercep` / `NodePalletPose` 同口径：dry-run 一律 SUCCESS
            # （不碰硬件，也不占着树 —— `--tick-once` 那种跑法才收得了场）。
            return Status.SUCCESS
        if not self._enabled:
            # 关掉 = 本节点不是这条链路上的参与者，**不占着树**：SUCCESS 让父节点
            # 照常往下走。若在这里返回 RUNNING，"把模拟源关掉"本身就会把整条分支
            # 永远卡住 —— 与"真机上必须能关掉"的硬约束正好相反。
            return Status.SUCCESS
        # 位姿为 None 只在 write_pallet=true 时算坏输入；不写托盘的那一路上
        # `_parse_sim_payload` 本来就不解析它，None 是**正常**的。
        if (self._write_pallet and self._pose is None) or not self._boxes:
            # 坏输入（`initialise()` 已经喊过）：FAILURE 是**终点**，父节点看得见、
            # 能反应；这不是"这一帧还没数据"。
            return Status.FAILURE

        # 播完就**停手**，不要拿最后一条反复写：每写一次版本号就涨一次，
        # 下游的版本门禁会跟着重算同一个数，10 Hz 白算。
        #
        # 但**不能返回 SUCCESS**：`Behaviour.tick()` 在 `status != RUNNING` 时每 tick
        # 都重进 `initialise()`，而 `initialise()` 会把游标归零 —— 于是列表从头再播
        # 一遍（盒子的位置在最后一条与第一条之间来回跳），"停手"根本没发生过。
        # 源节点的口径是"自己决定写什么、父节点决定何时收"：`NodePercep` 与
        # `NodePalletPose` 在没有新东西可算时同样返回 RUNNING（见
        # `node_pallet_pose.update` 的"版本没动就不再重算"那一句）。播完继续 RUNNING、
        # 一个字都不写，最后一帧因此留在黑板上给伺服节点一直用。
        if self._cursor >= len(self._boxes):
            self.feedback_message = f"播放完毕（{len(self._boxes)} 条）"
            return Status.RUNNING

        raw = self._boxes[self._cursor]
        # 这里的数**已经是 float / str**：`_parse_box` 转好的（它的 docstring 就是
        # 这个承诺），所以这里没有第二次转换，也就没有会抛的转换。
        obs = BoxObservation(
            u1=raw["u1"], v1=raw["v1"],
            u2=raw["u2"], v2=raw["v2"],
            quad=raw.get("quad"),
            label=raw.get("label", ""),
            confidence=raw.get("confidence", 1.0),
            stamp=raw.get("stamp", 0.0))
        if obs.stamp <= 0.0:
            obs.stamp = time.time()

        if self._write_pallet:
            setattr(self.global_blackboard, self._pallet_key, self._pose)
            self._pallet_version = bump_version(self.global_blackboard, self._pallet_key)

        setattr(self.global_blackboard, self._box_key, obs)
        self._box_version = bump_version(self.global_blackboard, self._box_key)

        self._cursor += 1
        self.feedback_message = (f"播放第 {self._cursor}/{len(self._boxes)} 条"
                                 f"箱子观测")
        # 日志只说**真写了的**键。`write_pallet=false` 时 `_pallet_version` 停在
        # 构造时的 0，照旧打出来就是在报"写了 latest_pallet v0"——一句假话，
        # 而这条 DEBUG 正是排查"到底写没写"时唯一会看的东西。
        logger.debug("模拟源写入：%s / %s v%d（%s）",
                     (f"{self._pallet_key} v{self._pallet_version}"
                      if self._write_pallet else "托盘不写（write_pallet=false）"),
                     self._box_key, self._box_version, self.feedback_message)
        # RUNNING：源节点继续持有这一支，`initialise()` 因此**不会**被重进
        # （游标保住，列表才会一条条往下播）。写还是只在有**新的一条**时写。
        return Status.RUNNING
