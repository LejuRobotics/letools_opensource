"""Load the real-ring stage runtime from the Learning checkout on demand."""
import importlib.util
from pathlib import Path
import sys

_path = Path(__file__).resolve().parents[2] / "LeTools-Learning/tools/ring_real/stage_skill.py"
if not _path.is_file():
    raise ImportError("真机圆环执行文件缺失；请先安装配套源码: " + str(_path))
_spec = importlib.util.spec_from_file_location(__name__, _path)
_module = importlib.util.module_from_spec(_spec)
sys.modules[__name__] = _module
_spec.loader.exec_module(_module)
