# -*- coding: utf-8 -*-
"""IHardware 契约完整性守卫。

守的是「接口加了方法、实现没跟上」这一类静默失效：抽象方法漏实现时类根本
实例化不了，可一旦有人在别处加了默认实现，问题就会推迟到运行期才炸。

三条规矩（DOCS-144 §5）：

1. 受检适配器的 ``__abstractmethods__`` 必须为空；
2. 3.6 节新增的四个契约方法在实现类上与接口**签名一致**（参数名与默认值）——
   调用方是按接口写的，实现类改个默认值就可能悄悄改变行为；
3. ``LejuBipedalHardware`` **显式豁免**：它当前不实现 IHardware 的任何方法、
   连导入都失败，本批不动它。豁免表不能变成逃兵通道，所以另有一条测试要求
   「被豁免的类确实仍不可用」——真补齐了，那条测试会红，提醒把豁免删掉。

本文件要 import 适配器，因此需要 ROS 环境（``source /opt/ros/noetic/setup.bash``）；
没有 rospy 时整个文件 skip，纯文档类测试不受影响。
"""

import inspect
import sys
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

pytest.importorskip("rospy", reason="契约测试要 import 适配器，需要 ROS 环境")

from adapters.hardware.leju_wheeled.hardware import LejuWheeledArmHardware  # noqa: E402
from core.interfaces.i_hardware import IHardware  # noqa: E402

# 3.6 节新增的四个契约方法
NEW_CONTRACTS = (
    "get_arm_joint_positions",
    "solve_ik",
    "get_transform",
    "check_joint_path",
)

CHECKED_CLASSES = (LejuWheeledArmHardware,)

# 显式豁免：类名 → 豁免理由
EXEMPT_CLASSES = {
    "LejuBipedalHardware": (
        "未实现 IHardware 的任何方法，且模块导入即失败"
        "（hardware.py 引用不存在的 core.domain.Pose）；"
        "补齐后从豁免表移除并加入 CHECKED_CLASSES"),
}


@pytest.mark.unit
@pytest.mark.parametrize("hardware_class", CHECKED_CLASSES,
                         ids=lambda cls: cls.__name__)
def test_no_abstract_methods_left(hardware_class):
    """IHardware 的抽象方法必须全部有实现。"""
    missing = sorted(hardware_class.__abstractmethods__)
    assert not missing, (
        f"{hardware_class.__name__} 仍未实现 {missing}；"
        "IHardware 新增抽象方法后必须同步实现，否则该类无法实例化")


@pytest.mark.unit
@pytest.mark.parametrize("name", NEW_CONTRACTS)
def test_new_contract_is_callable(name):
    """四个新契约方法在实现类上可调用（不是抽象占位）。"""
    method = getattr(LejuWheeledArmHardware, name, None)
    assert callable(method), f"LejuWheeledArmHardware 未实现 {name}()"


@pytest.mark.unit
@pytest.mark.parametrize("name", NEW_CONTRACTS)
def test_new_contract_signature_matches_interface(name):
    """参数名与默认值必须与 IHardware 一致，调用方才不会按两套默认值跑。"""
    interface = inspect.signature(getattr(IHardware, name))
    implementation = inspect.signature(getattr(LejuWheeledArmHardware, name))
    assert list(implementation.parameters) == list(interface.parameters), (
        f"{name} 参数名与接口不一致："
        f"接口 {list(interface.parameters)}，实现 {list(implementation.parameters)}")
    for parameter_name, interface_parameter in interface.parameters.items():
        implementation_parameter = implementation.parameters[parameter_name]
        assert implementation_parameter.default == interface_parameter.default, (
            f"{name} 的 {parameter_name} 默认值与接口不一致："
            f"接口 {interface_parameter.default!r}，实现 {implementation_parameter.default!r}")


@pytest.mark.unit
def test_exempt_class_is_still_unusable():
    """豁免只对「确实用不了」的类成立。

    谁把自己已经补齐的实现塞进豁免表，这条会红——豁免用来标记已知欠账，
    不是用来跳过检查的。``LejuBipedalHardware`` 今天连导入都失败
    （``adapters/hardware/leju_bipedal/hardware.py`` 引用了不存在的
    ``core.domain.Pose``），比「有抽象方法」更彻底地不可用。
    """
    try:
        from adapters.hardware.leju_bipedal import LejuBipedalHardware
    except ImportError as exc:
        assert "LejuBipedalHardware" in EXEMPT_CLASSES, (
            f"leju_bipedal 适配器导入失败（{exc}），豁免表里却没有它")
        return
    assert LejuBipedalHardware.__abstractmethods__, (
        f"{LejuBipedalHardware.__name__} 已经能导入且实现了全部抽象方法，"
        f"不应再豁免（理由：{EXEMPT_CLASSES.get(LejuBipedalHardware.__name__)}）")
