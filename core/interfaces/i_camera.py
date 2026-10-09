# core/interfaces/i_camera.py
from abc import ABC, abstractmethod
from typing import Optional, Dict, Any, Tuple
from ..domain.result import Result
from ..domain.camera import CameraFrame, CameraInfo, PointCloudData, DepthData, CameraStatus

class ICamera(ABC):
    """相机接口定义

    职责：相机生命周期管理 + 原始图像/深度/点云数据获取。
    """

    @abstractmethod
    def initialize(self, config: Dict[str, Any]) -> Result:
        """初始化相机"""
        pass

    @abstractmethod
    def shutdown(self) -> Result:
        """关闭相机"""
        pass

    @abstractmethod
    def is_connected(self) -> bool:
        """检查相机是否连接"""
        pass

    # 原始数据获取
    @abstractmethod
    def get_camera_frame(self, camera_name: str = "camera") -> Optional[CameraFrame]:
        """获取指定相机的最新帧数据（RGB uint8 HWC + 深度）"""
        pass

    def get_synchronized_camera_frame(
        self,
        camera_name: str = "camera",
    ) -> Optional[CameraFrame]:
        """获取最近一组已按原始时间戳配对的 RGB uint8 HWC + Depth 帧。

        这是可选能力；默认实现用于兼容尚未支持 RGBD 同步的相机适配器。
        """
        return None

    def wait_for_next_synchronized_camera_frame(
        self,
        camera_name: str = "camera",
        timeout_sec: float = 2.0,
    ) -> Result:
        """等待调用之后产生的下一组同步 RGBD 帧。"""
        return Result.fail(
            f"Camera {camera_name} does not support synchronized RGBD capture",
            error_code="RGBD_SYNC_UNSUPPORTED",
        )

    @abstractmethod
    def get_depth_data(self, camera_name: str = "camera") -> Optional[DepthData]:
        """获取指定相机的深度图数据"""
        pass

    @abstractmethod
    def get_point_cloud(self, camera_name: str = "camera") -> Optional[PointCloudData]:
        """获取指定相机的点云数据"""
        pass

    @abstractmethod
    def get_camera_info(self, camera_name: str = "camera") -> Optional[CameraInfo]:
        """获取指定相机的参数信息"""
        pass

    @abstractmethod
    def get_camera_status(self, camera_name: str = "camera") -> Optional[CameraStatus]:
        """获取指定相机的运行状态"""
        pass

    # 生命周期
    @abstractmethod
    def start_camera(self, camera_name: str = "camera") -> bool:
        """启动指定相机"""
        pass

    @abstractmethod
    def stop_camera(self, camera_name: str = "camera") -> bool:
        """停止指定相机"""
        pass


def unwrap_synchronized_frame(result) -> Tuple[Optional[CameraFrame], str]:
    """`wait_for_next_synchronized_camera_frame()` 的返回值 → `(帧, 失败原因)`。

    ⚠️ **那个接口返回的是 `Result`，不是 `CameraFrame`**（见上面它的签名）。
    这一步不能省：`Result` 是普通 dataclass，**没有 `__getattr__` 代理**到 `.data`，
    所以下面这种写法是**静默必炸**的：

        frame = hw.camera.wait_for_next_synchronized_camera_frame(cam)
        if frame is None:        # ← 永不成立：Result 恒为真值（实测）
            ...
        frame.color_image        # ← AttributeError: 'Result' object has no attribute

    **两个工具都栽在这里过**，而且因此它们的「真机抓帧」路径**从来没有跑通过**：

    - `apps/test_camera_internal/pallet_servo_sim/pick_servo_inputs.py`（`--capture`）
    - `apps/test_camera_internal/pallet_calibration/pallet_calibrate.py`（标定）

    两边 docstring 都写着"抓一帧"，但 `main()` 都不接异常 → 真机上一按就是 traceback。

    放在**接口旁边**（而不是各工具里各抄一份）还有个实际理由：这段判据只有放在
    这里才**测得到**。`apps/test_camera_internal/**` 被 CI 的 rsync 排除
    （`.gitlab-ci.yml` 的 `--exclude='*_internal/'`），放工具旁边的测试永远进不了
    CI；放这儿，`orchestration/nodes/tests/` 就能用 `-m unit` 钉住它。

    返回 `(None, 原因)` 时**原因一定非空**，调用方直接打日志即可 —— 别再把
    `error_code` 吞掉（适配器会给出 `RGBD_SYNC_DISABLED` / `CAMERA_NOT_INITIALIZED`
    / `RGBD_SYNC_TIMEOUT` 这些真正有用的码）。
    """
    if result is None:
        return None, "接口返回了 None（预期是 Result）"
    if hasattr(result, "color_image"):
        # 兼容直接返回帧的适配器：这不是契约，但别把能用的东西判死。
        return result, ""
    if not hasattr(result, "success"):
        return None, (f"接口返回了 {type(result).__name__}，既不是 Result "
                      f"也不是帧 —— 解包不了")
    if not result.success:
        msg = str(getattr(result, "message", "") or "（没有 message）")
        code = getattr(result, "error_code", None)
        return None, f"{msg}（error_code={code}）" if code else msg
    frame = getattr(result, "data", None)
    if frame is None:
        return None, "Result.success 为真但 data 是 None（接口成功却没给帧）"
    if not hasattr(frame, "color_image"):
        return None, f"Result.data 是 {type(frame).__name__}，看着不是相机帧"
    return frame, ""
