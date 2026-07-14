# skills/atomic/perception/camera_capture/skill.py
from core.interfaces.i_skill import ISkill
from core.interfaces.i_camera import ICamera
from core.domain.result import Result
from core.domain.skill_params import SkillParams
from core.common.logger import get_logger
from core.common.registry import register_skill

logger = get_logger(__name__)

class CameraCaptureParams(SkillParams):
    """相机捕获参数"""
    camera_type: str = "head"  # head, left_wrist, right_wrist
    save_path: str = ""
    timeout: float = 10.0

@register_skill('camera_capture')
class CameraCaptureSkill(ISkill):
    """相机捕获技能"""
    
    def __init__(self, camera: ICamera):
        self.camera = camera
        self.params: CameraCaptureParams = None
        self._is_finished = False
    
    @property
    def name(self) -> str:
        return "camera_capture"
    
    def initialize(self, params: CameraCaptureParams) -> Result:
        """初始化技能"""
        if not isinstance(params, CameraCaptureParams):
            return Result.fail("Invalid parameter type for CameraCaptureSkill")
        
        self.params = params
        self._is_finished = False
        return Result.ok()
    
    def execute(self) -> Result:
        """执行技能"""
        if self._is_finished:
            return Result.ok("Task already finished")
        
        # 获取相机帧
        frame = self.camera.get_frame(self.params.camera_type)
        if frame is None:
            return Result.fail("Failed to capture frame")
        
        # 保存图像（如果指定了保存路径）
        if self.params.save_path:
            try:
                import cv2
                cv2.imwrite(self.params.save_path, frame.color_image)
                logger.info(f"Image saved to {self.params.save_path}")
            except Exception as e:
                logger.error(f"Failed to save image: {str(e)}")
        
        self._is_finished = True
        return Result.ok("Camera capture successful")
    
    def cancel(self) -> Result:
        """取消技能"""
        self._is_finished = True
        return Result.ok("Camera capture cancelled")
    
    def is_finished(self) -> bool:
        """检查技能是否完成"""
        return self._is_finished