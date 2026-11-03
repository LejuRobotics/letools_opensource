#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""托盘检测的**后端选择**自检（纯函数，不需要 TensorRT / 模型 / ROS）。

跑法：
    cd <仓库根>
    python3 skills/atomic/perception/pallet_detect/tests/test_engine_backend.py

只钉"选哪个后端、拿到哪个路径"，不加载 engine —— engine 的真实推理要在目标机
（Orin + TRT 8.5）上验：`trtexec --loadEngine=... --warmUp=5 --duration=2`，
或在节点启动日志里看 `后端=engine` 与输入/输出形状。
"""
from __future__ import annotations

import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[5]))

from skills.atomic.perception.pallet_detect.detect import (  # noqa: E402
    DEFAULT_ENGINE_PATH, DEFAULT_MODEL_PATH, _select_backend)
from skills.atomic.perception.pallet_detect.detect_pose import (  # noqa: E402
    DEFAULT_POSE_ENGINE_PATH, DEFAULT_POSE_MODEL_PATH, _select_pose_backend)

_FAILED = []


def check(name: str, cond: bool, detail: str = '') -> None:
    if cond:
        print(f'  ✓ {name}')
    else:
        print(f'  ✗ {name}  {detail}')
        _FAILED.append(name)


def _raises(fn, exc_type):
    try:
        fn()
    except exc_type as exc:
        return exc
    except Exception as exc:                        # noqa: BLE001
        return exc
    return None


def main() -> int:
    print('默认路径：engine 与 onnx 同目录、不同后缀')
    check('OBB onnx 默认路径', DEFAULT_MODEL_PATH.name == 'pallet_obb.onnx',
          str(DEFAULT_MODEL_PATH))
    check('OBB engine 默认路径',
          DEFAULT_ENGINE_PATH.name == 'pallet_obb_fp32.engine',
          str(DEFAULT_ENGINE_PATH))
    check('POSE onnx 默认路径',
          DEFAULT_POSE_MODEL_PATH.name == 'pallet_pose.onnx',
          str(DEFAULT_POSE_MODEL_PATH))
    check('POSE engine 默认路径',
          DEFAULT_POSE_ENGINE_PATH.name == 'pallet_pose_fp32.engine',
          str(DEFAULT_POSE_ENGINE_PATH))

    with tempfile.TemporaryDirectory() as td:
        engine = Path(td) / 'x.engine'
        engine.write_bytes(b'not-a-real-engine')
        model = Path(td) / 'x.onnx'

        print('OBB：三档 backend')
        check("auto + engine 在 -> engine",
              _select_backend({'backend': 'auto', 'engine_path': str(engine),
                               'model_path': str(model)}) == ('engine', engine))
        check("auto + engine 不在 -> onnx",
              _select_backend({'backend': 'auto',
                               'engine_path': str(Path(td) / 'missing.engine'),
                               'model_path': str(model)}) == ('onnx', model))
        check("onnx 强制 -> onnx（有 engine 也不用）",
              _select_backend({'backend': 'onnx', 'engine_path': str(engine),
                               'model_path': str(model)}) == ('onnx', model))
        err = _raises(lambda: _select_backend(
            {'backend': 'engine',
             'engine_path': str(Path(td) / 'missing.engine')}), FileNotFoundError)
        check("engine 强制 + 缺失 -> FileNotFoundError", isinstance(err, FileNotFoundError),
              repr(err))
        check('错误信息里有 trtexec 导出命令',
              err is not None and 'trtexec' in str(err), str(err))
        err = _raises(lambda: _select_backend({'backend': 'bogus'}), ValueError)
        check("非法 backend -> ValueError", isinstance(err, ValueError), repr(err))

        print('POSE：同一套规则')
        check("auto + engine 在 -> engine",
              _select_pose_backend({'backend': 'auto', 'engine_path': str(engine),
                                    'model_path': str(model)}) == ('engine', engine))
        check("auto + engine 不在 -> onnx",
              _select_pose_backend({'backend': 'auto',
                                    'engine_path': str(Path(td) / 'missing.engine'),
                                    'model_path': str(model)}) == ('onnx', model))
        err = _raises(lambda: _select_pose_backend(
            {'backend': 'engine',
             'engine_path': str(Path(td) / 'missing.engine')}), FileNotFoundError)
        check("engine 强制 + 缺失 -> FileNotFoundError",
              isinstance(err, FileNotFoundError), repr(err))

    print()
    if _FAILED:
        print(f'✗ 失败 {len(_FAILED)} 条：{_FAILED}')
        return 1
    print('✓ 全部通过')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
