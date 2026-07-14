# core/interfaces/i_perception.py
from abc import ABC, abstractmethod
from typing import List, Optional, Dict, Any
from ..domain.perception import TagDetection, ObjectDetection, PerceptionResult
from ..domain.camera import CameraFrame, PointCloudData, DepthData, CameraInfo, CameraStatus
from ..interfaces.i_camera import ICamera

class IPerception(ABC):
    """感知层基础接口

    职责：算法层输出（Tag 检测、物体检测）。
    不再暴露相机生命周期方法，通过依赖注入的 ICamera 获取数据。
    """

    @abstractmethod
    def initialize(self, camera: ICamera, config: Dict[str, Any] = None) -> bool:
        """初始化感知模块

        Args:
            camera: 注入的 ICamera 实例（依赖注入）
            config: 配置字典
        """
        pass

    @abstractmethod
    def shutdown(self) -> None:
        """关闭感知模块并释放资源"""
        pass

    # 算法输出
    @abstractmethod
    def get_tag_detections(self) -> List[TagDetection]:
        """获取当前帧的标签检测结果"""
        pass

    @abstractmethod
    def get_object_detections(self) -> List[ObjectDetection]:
        """获取当前帧的通用物体检测结果（预留，当前返回空）"""
        pass

    @abstractmethod
    def get_latest_result(self) -> Optional[PerceptionResult]:
        """获取最近一次的完整感知结果"""
        pass

    # 委托查询（便捷方法，内部委托注入的 ICamera）
    @abstractmethod
    def get_camera_frame(self, camera_name: str = "camera") -> Optional[CameraFrame]:
        """获取指定相机的最新帧数据"""
        pass

    @abstractmethod
    def get_point_cloud(self, camera_name: str = "camera") -> Optional[PointCloudData]:
        """获取指定相机的点云数据"""
        pass

    @abstractmethod
    def get_depth_data(self, camera_name: str = "camera") -> Optional[DepthData]:
        """获取指定相机的深度图数据"""
        pass

    @abstractmethod
    def get_camera_info(self, camera_name: str = "camera") -> Optional[CameraInfo]:
        """获取指定相机的参数信息"""
        pass

    @abstractmethod
    def get_camera_status(self, camera_name: str = "camera") -> Optional[CameraStatus]:
        """获取指定相机的运行状态"""
        pass
