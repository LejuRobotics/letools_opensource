# LeTools/core/domain/result.py
from dataclasses import dataclass
from typing import Optional, Any

@dataclass
class Result:
    """
    统一的操作结果封装。
    用于在 Core、Adapter 和 Skill 层之间传递执行状态和错误信息。
    """
    success: bool
    message: str = ""
    data: Optional[Any] = None  # 可选的返回数据，如查询到的传感器数值
    error_code: Optional[str] = None  # 可选的错误码，方便上层做针对性处理
    
    @staticmethod
    def ok(msg="Success", data=None):
        return Result(success=True, message=msg, data=data)

    @staticmethod
    def fail(msg="Failed", error_code=None, data=None):
        return Result(success=False, message=msg, error_code=error_code, data=data)