# core/interfaces/i_camera.py
from abc import ABC, abstractmethod
from typing import Optional, Dict, Any
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
