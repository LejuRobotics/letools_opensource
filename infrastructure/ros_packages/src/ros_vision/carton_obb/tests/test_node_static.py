#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""`carton_obb_detect` 节点的**静态自检**（读源码，不需要 ROS / 相机 / 模型）。

    cd <仓库根>
    python3 infrastructure/ros_packages/src/ros_vision/detection_industrial_yolo/
        carton_obb/tests/test_node_static.py

节点类只在真机上构造（`main()` 里），单元测试碰不到它 —— 所以**它里面的错只有
上机器才会现形**。这个文件用最笨但最有效的办法兜住：把源码当文本读，
钉住几条"错了也静默"的东西。托盘那条链有同款检查
（`pallet_detection/tests/test_pallet_detection_node.py`）。

本文件验五件事：

1. **内参只打一次**：`_on_camera_info` 必须有 `if self.k is not None: return` 守卫，
   且 CameraInfo 的 `Subscriber(...)` 返回值**被接住**（`self._info_sub =`）。
   ⚠️ 这两条缺一不可 —— 从前就是缺了它们，内参那行 loginfo **以相机频率反复打印**，
   把日志里真正要看的拒绝原因与耗时全埋了，而注释还写着"只记一次"。
2. **有"成功也打"的心跳**：`_run` 的**每条路径**都要调 `_heartbeat`。
   没有它，「一切正常」与「线程死了 / 相机停了」在日志上长得一模一样
   （托盘链 2026-09-29 踩过这个坑）。
3. **拒绝原因进日志**：`no_detection` 那条要带 NMS 后的候选数。
4. **统一日志接上了、且在 `rospy.init_node()` 之后**（顺序反了静默失效），
   并且**带回退**（单独 roslaunch 时仓库根不一定在 sys.path 上）。
5. **仓库根按特征找**，不数 `parents[N]` —— 这个包 2026-10-15 挪过位置
   （`detection_industrial_yolo/carton_obb/` → `ros_vision/carton_obb/`），
   数层数的写法挪一次断一次。
"""
from __future__ import annotations

import ast
import re
import sys
from pathlib import Path

_HERE = Path(__file__).resolve()
_NODE = _HERE.parent.parent / 'scripts' / 'carton_obb_detect.py'

_fails: list[str] = []


def check(name: str, cond: bool, detail: str = '') -> None:
    if cond:
        print(f'  PASS  {name}')
    else:
        print(f'  FAIL  {name}  {detail}')
        _fails.append(name)


def _src() -> str:
    return _NODE.read_text(encoding='utf-8')


def _code_lines() -> list[str]:
    """源码里**去掉注释与文档字符串**之后的行。

    ⚠️ 必须这么做：本文件要钉的几件事（比如"别用 `self.__dict__['_info_sub']`"）
    正好也会出现在**解释它们的注释与 docstring 里** —— 直接对全文做子串检查，
    会把"我在警告你不要这么写"当成"你正在这么写"。
    """
    lines = []
    for raw in _src().splitlines():
        stripped = raw.strip()
        if stripped.startswith('#'):
            continue
        lines.append(raw.split('#', 1)[0] if '#' in raw else raw)
    return lines


def test_camera_info_is_logged_once():
    """★ 内参**只打一次**：守卫 + 接住订阅返回值，两条缺一不可。"""
    src = _src()
    check('找到 _on_camera_info', 'def _on_camera_info' in src)
    body = src.split('def _on_camera_info', 1)[-1].split('\n    def ', 1)[0]
    check('有 `if self.k is not None: return` 守卫（没有就会 30Hz 刷屏）',
          re.search(r'if self\.k is not None:[^\n]*\n\s*return', body) is not None,
          '守卫缺失 —— 内参会以相机频率反复打印')
    check('守卫在打印**之前**',
          body.find('is not None') < body.find('loginfo'),
          '守卫在 loginfo 之后等于没写')
    check('CameraInfo 订阅的返回值被接住（`self._info_sub =`）',
          'self._info_sub = rospy.Subscriber(' in src,
          '裸调用 rospy.Subscriber(...) 的话 unregister() 永远拿不到句柄')
    check('`self.k = None` 在 __init__ 里先声明（否则守卫读未定义属性）',
          'self.k = None' in src,
          '没找到 self.k = None')
    unreg = [ln for ln in _code_lines() if '.unregister()' in ln]
    check('注销的是真的那个句柄（不是 self.__dict__[...] 取一个没赋过的键）',
          bool(unreg) and all('self._info_sub.unregister()' in ln for ln in unreg),
          f'注销行：{unreg}')


def test_heartbeat_exists_and_covers_every_path():
    """★ **成功也打**的心跳，且覆盖 `_run` 的每条路径。"""
    src = _src()
    check('有 _heartbeat 方法',
          'def _heartbeat(self)' in src, '没找到心跳')
    check('心跳周期参数 ~heartbeat_s',
          "get_param('~heartbeat_s'" in src, '没找到 ~heartbeat_s')
    check('心跳能关掉（<=0 时直接返回）',
          'if self.heartbeat_s <= 0.0:' in src, '没找到关闭开关')

    run_body = src.split('def _run(self)', 1)[-1].split('\n    def ', 1)[0]
    n_continue = run_body.count('continue')
    n_hb = run_body.count('self._heartbeat()')
    check(f'_run 里每条 continue 前都有心跳（continue {n_continue} 处 / 心跳 {n_hb} 处）',
          n_hb >= n_continue,
          f'continue={n_continue} 心跳={n_hb} —— 漏掉的路径上"什么都没发生"与"挂了"分不开')
    check('收到空队列（相机停了）那条路径也打心跳',
          re.search(r'except queue\.Empty:\s*\n(\s*#[^\n]*\n)*\s*self\._heartbeat\(\)',
                    run_body) is not None,
          'queue.Empty 分支没打心跳 —— 相机停了这个最重要的场景反而静默')


def test_reject_reason_reaches_the_log():
    """★ "没检到箱子"必须带**候选数** —— 否则排查时分不清是网络没出框还是被过滤掉了。"""
    src = _src()
    check('no_detection 的拒绝串带 n_after_nms',
          "no_detection,n_after_nms=" in src, '没找到带候选数的拒绝串')
    check('拒绝串写进 self._last_reject（心跳第二行会重打）',
          'self._last_reject = f\'no_detection,n_after_nms={len(dets)}\'' in src,
          '没找到 _last_reject 的赋值')
    check('推理失败也记原因',
          "self._last_reject = f'infer_failed:" in src, '推理失败没记拒绝原因')
    check('成功后清掉拒绝原因',
          'self._last_reject = None' in src, '成功后没清 _last_reject')


def test_unified_logging_wired_after_init_node():
    """★ 统一日志接上了、**在 `rospy.init_node()` 之后**、且**带回退**。"""
    src = _src()
    check('logger 的 import 带 try/except 回退',
          'from core.common.logger import init_logging as _init_logging' in src
          and '_init_logging = None' in src,
          '没找到带回退的 import')
    # ⚠️ 断言用**短片段**，不要用一整句：这些提示语在源码里是**跨行拼接**的
    # （相邻字符串字面量自动连接），源码原文里找不到连成一行的整句。
    # 写成一整句的话，下次只是换了断行位置用例就会红 —— 那是断言脆，不是代码错。
    check('回退时说清楚日志去哪了',
          '日志只走 rospy' in src and '不落盘' in src, '没找到回退告警')
    check('回退告警指出真实原因（不是路径问题，是缺依赖/旧部署）',
          'colorlog' in src,
          "告警还在说'仓库根不在 sys.path 上' —— 现在节点自己会插根，那会把人带偏")
    main_src = src.split('def main()', 1)[-1]
    i_node = main_src.find("init_node('carton_obb_detect')")
    i_log = main_src.find('_init_unified_logging(')
    check("init_logging 在 rospy.init_node **之后**（反了就静默失效）",
          0 <= i_node < i_log, f'init_node@{i_node} _init_unified_logging@{i_log}')
    check('启动时打出日志文件路径',
          'get_active_log_file' in src, '没找到 get_active_log_file')


def test_repo_root_is_found_by_feature_not_by_depth():
    """★ 仓库根按**特征**找（含 `skills/` 与 `infrastructure/` 的祖先），不数层数。"""
    src = _src()
    check('有 _repo_root() 助手', 'def _repo_root()' in src, '没找到 _repo_root')
    check("不数 parents[N]（本包挪过一次位置）",
          not re.search(r"parents\[\d+\]", src),
          '还在数 parents[N] —— 挪目录必断，而且断在 import 上、只有真机才现形')
    check('按特征找：同时含 skills/ 与 infrastructure/',
          "(parent / 'skills').is_dir()" in src
          and "(parent / 'infrastructure').is_dir()" in src,
          '没找到特征判据')
    check('找不到时**不抛**（回退路径本来就允许失败）',
          'return here.parents[-1]' in src, '没找到兜底返回')


def test_only_one_output_topic():
    """★ **只发一条话题**（`/box/detection`）。

    从前还发一路 `/box/yolo_box`（`PolygonStamped`，给叠加图画灰框），2026-10-15 删了。
    它发的是四角的**轴对齐包围盒**，而黑板上的 `BoxObservation.u1/v1/v2/v2` 本来就是
    同一个数（`node_box_obs` 用 `min/max` 填的）—— 纯冗余，删掉零信息损失。
    这条网防止它被"顺手加回来"：多一路话题就多一处要同步的契约。
    """
    src = _src()
    code = '\n'.join(_code_lines())
    check('不再有 box_out 参数', 'box_out' not in code, '又出现了 box_out')
    check('不再建第二个 Publisher（只发一条话题）',
          code.count('rospy.Publisher(') == 1,
          f'Publisher 有 {code.count("rospy.Publisher(")} 个')
    # ⚠️ 用 **AST** 判 import，不用文本匹配：本文件的 docstring 与节点里都写着
    # "从前发 /box/yolo_box（PolygonStamped）"这类说明，文本匹配会把
    # "我在说明它被删了"当成"它还 import 着"。
    imported = set()
    for node in ast.walk(ast.parse(src)):
        if isinstance(node, ast.ImportFrom):
            imported |= {a.name for a in node.names}
    check('不再 import PolygonStamped',
          'PolygonStamped' not in imported, f'import 里有 {sorted(imported)}')
    check('发布的是 BoxDetection（四角那条）',
          'rospy.Publisher(self.out_topic, BoxDetection' in src,
          '没找到 BoxDetection 的 Publisher')
    check('默认输出话题是 /box/detection',
          "get_param('~detection_out', '/box/detection')" in src,
          '默认话题不对')


def test_shared_backend_contract():
    """★ 用公共服务时，输出必须与本地推理**同口径**（这几条错了是静默的差异）。

    1. 请求服务时 `nms_iou=1.0` —— 让服务端**不要** NMS。服务端只按 IoU 去重，
       本地 `nms_rotated` 还有 IoMin（管小框套大框），且两边栅格化口径不同。
       让服务端先砍一刀，结果就可能与本地推理差一两个候选。
    2. 转换走 `dets_from_rotated_rects`（有与 `decode_obb` 的逐字段等价性测试），
       不在节点里手搓 `cv2.boxPoints`。
    3. 发布用**响应里的 `source_header`**：服务端推理的是它自己缓存的最新帧，
       不是本节点队列里那一帧；用错 header 会让下游配对错帧。
    4. 同一 stamp 不重复进时间窗。
    5. 服务连续失败要能切回本地（只告警不切换 = 服务挂了整条链静默没数）。
    """
    src = _src()
    code = '\n'.join(_code_lines())
    check('请求服务时传 nms_iou=1.0（服务端不要 NMS）',
          'nms_iou=1.0' in code, '没找到 nms_iou=1.0 —— 服务端会先按 IoU 砍一刀')
    check('服务输出走 dets_from_rotated_rects',
          'dets_from_rotated_rects(' in code, '没找到转换器调用')
    check('发布用 source_header（不是本地队列那一帧的 header）',
          'self._publish(out, best, dets, ms, src_header)' in code,
          '发布用的还是 color_msg.header —— 服务端推理的可能是另一帧')
    check('同一 stamp 不重复进时间窗',
          '_last_src_stamp' in code and '_n_dup_frames' in code,
          '没找到重复帧守卫')
    check('共享服务连续失败切回本地',
          'def _shared_failed' in code and '_load_local()' in code
          and 'shared_fail_limit' in code,
          '没找到切回本地的逻辑')
    check('InferCartonObb 缺失时只告警、不崩（旧部署机还能起）',
          'InferCartonObb = None' in src,
          'srv 没编译进工作空间时节点会直接 ImportError 起不来')


def main() -> int:
    if not _NODE.exists():
        print(f'找不到节点脚本：{_NODE}')
        return 1
    print('== 内参只打一次（守卫 + 接住订阅句柄） ==')
    test_camera_info_is_logged_once()
    print('== 心跳覆盖 _run 的每条路径 ==')
    test_heartbeat_exists_and_covers_every_path()
    print('== 拒绝原因进日志 ==')
    test_reject_reason_reaches_the_log()
    print('== 只发一条话题（/box/yolo_box 已删） ==')
    test_only_one_output_topic()
    print('== 统一日志接线（顺序 + 回退） ==')
    test_unified_logging_wired_after_init_node()
    print('== 仓库根按特征找，不数层数 ==')
    test_repo_root_is_found_by_feature_not_by_depth()
    print('== 公共服务后端与本地推理同口径 ==')
    test_shared_backend_contract()
    print()
    if _fails:
        print(f'{len(_fails)} 项失败：{", ".join(_fails)}')
        return 1
    print('全部通过')
    return 0


if __name__ == '__main__':
    sys.exit(main())
