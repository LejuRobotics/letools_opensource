# -*- coding: utf-8 -*-
"""文档路径基准声明的不变量断言。

纯文件系统扫描：不依赖 ROS，不 import 项目代码，秒级完成。
只断言外部可观察行为（文档里有没有写路径基准声明），不核对声明的具体措辞，
这样声明怎么改写测试都不会误报。

背景：上一批把文档里的 `/media/data/LeTools/...` 绝对路径改写成裸相对路径时，
有一部分只改了路径、没写基准，读者无从知道命令该在哪执行。本守卫把
「裸顶层路径出现 >= 3 处就必须写明基准」这条规则固化下来。
"""

import re
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]

# 外部/引入树：这些目录下的文档讲的是第三方代码自己的相对路径，
# 不以 LeTools 仓库根为基准，不适用本规则。
EXCLUDED_TREES = (
    "third_party",
    "infrastructure/ros_packages/src",
    "detectron2",
    "lib/pysixd",
    "lib/egl_renderer",
    "docs/modules",
    "docs/superpowers",
    # SDK 子模块：与 third_party 同类——子模块内的文档讲的是 SDK 自己的相对路径，
    # 不以 LeTools 仓库根为基准。只排除子模块这一棵，不整棵排除 drivers/：后者口径
    # 过宽，将来会掩盖 drivers/ 下真出现的问题。加非顶层项不改变 TOP_LEVEL_DIRS
    # （它比对的是顶层目录名）。
    "drivers/leju/kuavo_humanoid_sdk",
)

# 命中任一关键词即视为该文件已写明路径基准。
DECLARATION_KEYWORDS = (
    "路径基准",
    "仓库根目录",
    "仓库根",
    "现场上位机",
    "不是通用路径",
    "事实记录",
    "现场核查",
)

# 阈值取 3：实测 >= 1 会把三十多个文件卷进来（太吵），>= 5 会漏掉真缺口。
MIN_OCCURRENCES = 3

# 命令式出现的首 token（`python3 apps/...`、`bash scripts/...`）。
# 另接受任何以 .sh / .py 结尾的脚本名。
COMMAND_HEADS = ("python3", "python", "bash", "sh", "cd", "ros2", "colcon", "source")

# 顶层目录名：从文件系统动态取，将来加目录不必改测试。
TOP_LEVEL_DIRS = tuple(
    path.name
    for path in sorted(REPO_ROOT.iterdir())
    if path.is_dir()
    and not path.name.startswith(".")
    and path.name not in EXCLUDED_TREES
)


def _starts_with_top_dir(text):
    """text 是否以「某个顶层目录 + /」开头，即一段裸顶层路径。"""
    return any(text.startswith(top + "/") for top in TOP_LEVEL_DIRS)


def _is_command_line(text):
    """命令式出现：首 token 是命令或 .sh/.py 脚本，次 token 是裸顶层路径。"""
    stripped = text.strip().lstrip("$>#").strip()
    if not stripped or stripped.startswith("#"):
        return False
    tokens = stripped.split()
    if len(tokens) < 2:
        return False
    head = tokens[0]
    if head not in COMMAND_HEADS and not head.endswith((".sh", ".py")):
        return False
    return _starts_with_top_dir(tokens[1].lstrip("'\""))


def _is_bare_path_line(text):
    """正文行去掉列表符号、引用符号后，直接以裸顶层路径起头。"""
    stripped = text.strip().lstrip("-*>0123456789. ").strip()
    return _starts_with_top_dir(stripped)


def _split_regions(text):
    """把正文切成（围栏代码块行, 代码块外的正文行, 行内代码 span）。"""
    fenced, prose, spans = [], [], []
    buffer, in_block = [], False
    for line in text.splitlines():
        if line.lstrip().startswith("```"):
            if in_block:
                fenced.extend(buffer)
            buffer, in_block = [], not in_block
            continue
        if in_block:
            buffer.append(line)
        else:
            prose.append(line)
            spans.extend(re.findall(r"`([^`]+)`", line))
    return fenced, prose, spans


def count_occurrences(text):
    """统计单个文件内「以裸顶层路径开头的命令式出现」的处数。

    三个区域分别计：
    - 围栏代码块内的命令行（`python3 apps/...`）
    - 正文行直接以 `<顶层目录>/` 起头者
    - 正文行内代码 span 里的命令式出现
    """
    fenced, prose, spans = _split_regions(text)
    total = sum(1 for line in fenced if _is_command_line(line))
    total += sum(1 for line in prose if _is_bare_path_line(line))
    total += sum(1 for span in spans if _is_command_line(span))
    return total


def _iter_markdown_files():
    """按扫描范围逐个产出（相对仓库根路径, 绝对路径）。"""
    for path in sorted(REPO_ROOT.rglob("*.md")):
        relative = path.relative_to(REPO_ROOT)
        if any(part.startswith(".") for part in relative.parts):
            continue
        posix = relative.as_posix()
        if any(posix == tree or posix.startswith(tree + "/") for tree in EXCLUDED_TREES):
            continue
        yield relative, path


@pytest.mark.unit
def test_scan_scope_is_usable():
    """守卫自身可用性：顶层目录与扫描结果不能为空，否则计数恒为 0、永远绿。"""
    assert TOP_LEVEL_DIRS, "没能从 %s 取到顶层目录，路径规则会退化为恒真" % REPO_ROOT
    assert list(_iter_markdown_files()), "没扫到任何 Markdown，路径规则会退化为恒真"


@pytest.mark.unit
def test_bare_paths_require_baseline_declaration():
    """裸顶层路径出现 >= 3 处的文档，必须写明路径基准。"""
    offenders = []
    for relative, path in _iter_markdown_files():
        text = path.read_text(encoding="utf-8")
        if any(keyword in text for keyword in DECLARATION_KEYWORDS):
            continue
        occurrences = count_occurrences(text)
        if occurrences >= MIN_OCCURRENCES:
            offenders.append("%s（%d 处）" % (relative.as_posix(), occurrences))
    assert not offenders, (
        "以下文档出现了 >= %d 处以裸顶层路径开头的命令，却没写明路径基准"
        "（相对谁、命令在哪执行），读者无从判断命令该在哪跑：\n  %s"
        % (MIN_OCCURRENCES, "\n  ".join(offenders))
    )
