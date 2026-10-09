# -*- coding: utf-8 -*-
"""`wait_for_next_synchronized_camera_frame()` 的**解包契约**。

这个接口返回的是 `Result`、不是 `CameraFrame`，而 `Result` 是普通 dataclass、
**没有 `__getattr__` 代理**。于是下面这种写法是静默必炸的：

    frame = hw.camera.wait_for_next_synchronized_camera_frame(cam)
    if frame is None:        # ← 永不成立：Result 恒为真值（实测 bool(Result.ok()) is True）
        ...
    frame.color_image        # ← AttributeError

两个工具都这么写过，而且**它们的真机抓帧路径因此从来没有跑通过**：
`apps/test_camera_internal/pallet_servo_sim/pick_servo_inputs.py`（`--capture`）与
`apps/test_camera_internal/pallet_calibration/pallet_calibrate.py`（标定）。

**为什么这个测试放在这里而不是工具旁边**：`apps/test_camera_internal/**` 被 CI
的 rsync 排除（`.gitlab-ci.yml` 的 `--exclude='*_internal/'`），放那儿的测试
**永远进不了 CI**；`orchestration/nodes/tests/` 是 `verify:opensource` 实际跑的
目录。解包逻辑提到 `core/interfaces/i_camera.py` 之后，它第一次被测得到。

运行：
    pytest orchestration/nodes/tests/test_camera_frame_contract.py -m unit -v
"""
from types import SimpleNamespace

import numpy as np
import pytest

from core.domain.camera import CameraFrame
from core.domain.result import Result
from core.interfaces.i_camera import unwrap_synchronized_frame

pytestmark = pytest.mark.unit


def _frame():
    return CameraFrame(color_image=np.zeros((4, 4, 3), np.uint8),
                       depth_image=np.full((4, 4), 1000, np.uint16),
                       color_frame_id="cam_optical")


# --------------------------------------------------------------------------- #
# 成功路径
# --------------------------------------------------------------------------- #
def test_unwraps_a_successful_result():
    frame = _frame()
    got, why = unwrap_synchronized_frame(Result.ok("synced", data=frame))
    assert got is frame
    assert why == ""


def test_accepts_a_bare_frame_for_adapters_that_return_one():
    """有的适配器可能直接给帧。**这不是契约**，但别把能用的东西判死。"""
    frame = _frame()
    got, why = unwrap_synchronized_frame(frame)
    assert got is frame and why == ""


def test_accepts_a_duck_typed_frame():
    """不要求 `isinstance(CameraFrame)`：有 `color_image` 就当帧。

    这条是刻意留的余地 —— 契约里没有禁止适配器返回自己的帧类型，用 isinstance
    卡死会让「换一个适配器」变成先改这里。
    """
    duck = SimpleNamespace(color_image=np.zeros((2, 2, 3), np.uint8))
    got, why = unwrap_synchronized_frame(Result.ok(data=duck))
    assert got is duck and why == ""


# --------------------------------------------------------------------------- #
# 失败路径：**每一条都必须给出非空原因**，绝不能返回裸 None 让调用方去猜
# --------------------------------------------------------------------------- #
def test_failed_result_reports_message_and_error_code():
    """失败时必须把 `error_code` 带出来。

    适配器会给出 `RGBD_SYNC_DISABLED` / `CAMERA_NOT_INITIALIZED` /
    `RGBD_SYNC_TIMEOUT` 这些真正有用的码 —— 操作员靠它们区分「相机没起来」
    和「等超时了」，把码吞掉就只能看到一句泛泛的提示。
    """
    got, why = unwrap_synchronized_frame(
        Result.fail("等超时", error_code="RGBD_SYNC_TIMEOUT"))
    assert got is None
    assert "等超时" in why and "RGBD_SYNC_TIMEOUT" in why


def test_failed_result_without_error_code_still_reports():
    got, why = unwrap_synchronized_frame(Result.fail("相机不支持同步帧"))
    assert got is None
    assert "相机不支持同步帧" in why


def test_success_but_no_data_is_an_error_not_a_silent_pass():
    """`success=True` 但 `data=None` —— 接口成功却没给帧。

    这条最容易漏：只判 `success` 的实现会放一个 `None` 过去，调用方随后
    在 `frame.color_image` 上炸，而且炸在**离真正的原因很远**的地方。
    """
    got, why = unwrap_synchronized_frame(Result.ok("synced", data=None))
    assert got is None
    assert why, "必须给出原因，不能静默返回 None"


def test_data_of_the_wrong_type_is_rejected():
    got, why = unwrap_synchronized_frame(Result.ok(data="这不是帧"))
    assert got is None
    assert "str" in why or "不是" in why


def test_none_and_unknown_objects_never_raise():
    """坏输入只许返回 `(None, 原因)`，**绝不许抛**。

    调用方（两个工具）在 `initialise()`/`main()` 里，而 py_trees 不接这些钩子
    抛出的异常 —— 抛出去就是整棵树连每帧日志一起没。所以这一层必须兜住。
    """
    for bad in (None, 42, "字符串", object(), SimpleNamespace(success=None)):
        got, why = unwrap_synchronized_frame(bad)
        assert got is None, f"{bad!r} 不该解出帧"
        assert why, f"{bad!r} 必须给出原因"


# --------------------------------------------------------------------------- #
# 那条**墓碑**：把修复回退掉，这个文件必须红
# --------------------------------------------------------------------------- #
def test_the_original_bug_would_have_been_caught_here():
    """把「Result 当帧用」那个写法原样复现一遍，确认它确实是坏的。

    这条不是测我们的实现，是**留个墓碑**记着这个坑长什么样 —— 新人（或者
    未来的我）看到上面那段的写法会以为是多余的一步，这条用可执行的代码说明
    它不是。断言的是**旧写法会炸**，所以它永远不会因为我们改了实现而失效。
    """
    result = Result.ok("synced", data=_frame())
    assert bool(result) is True, "Result 恒为真值 —— 所以 `if frame is None` 永不触发"
    with pytest.raises(AttributeError):
        _ = result.color_image
    # 而正确解包拿得到帧
    got, _ = unwrap_synchronized_frame(result)
    assert got is result.data
