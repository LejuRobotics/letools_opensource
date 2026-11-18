"""场景路径存在性的机械断言。

纯文件系统扫描：不依赖 ROS，不 import 项目代码，秒级完成。
只断言外部可观察行为（文档与代码里写的 orchestration/scenarios/<seg>
路径指向的文件/目录真实存在），不校验正文内容。

背景：场景目录批量迁入 ``scenarios_internal/`` 时，markdown 链接有
``test_docs_links.py`` 把关，但反引号里的裸路径、命令示例和 py/sh 里的
字符串路径没有任何测试覆盖，同一批改动漏掉 44 处失效引用全靠人工评审
兜出来。本守卫把「写了 scenarios 路径就必须真实存在」固化下来。
"""

import re
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]

# 只扫描文本形态的源；二进制与生成物不扫
_SCAN_SUFFIXES = {".md", ".py", ".sh", ".json", ".yml", ".yaml", ".txt"}
_SCAN_ROOTS = [
    REPO_ROOT / "docs",
    REPO_ROOT / "docs_internal",
    REPO_ROOT / "apps",
    REPO_ROOT / "skills",
    REPO_ROOT / "orchestration",
    REPO_ROOT / "README.md",
]
# module_internal 是第三方/内部镜像树，有自己的 scenarios 副本，路径不共用
_SKIP_PARTS = {"__pycache__", ".git", "node_modules", "module_internal", "开发者原稿"}
# `.gitignore` 里的本地目录：`docs/superpowers/` 是个人计划/规格草稿，
# 不属于仓库内容，CI 检出时不存在。它可能引用迁移前的旧场景路径，
# 本地扫描会误报，所以按相对路径前缀整体跳过。
_SKIP_REL_PREFIXES = ("docs/superpowers",)
_ARCHIVED_FILES = {"docs_internal/文档审查与站点同步诊断-2026-10-31.md"}

# 已知"文档引用但文件未发布"的路径：正文已显式声明未随仓发布，不算失效
_KNOWN_UNPUBLISHED = {
    # haicheng tools docstring 里的相对引用，基线 8b2d59d9 即如此（评审已核实）
    "nav/teach_navigation.py",
    "orchestration/scenarios/zhaofeng_feeding_internal/py_tree_place_perception_test.json",
    "orchestration/scenarios/zhaofeng_feeding_internal/py_tree_nav_to_place_point.json",
}

# orchestration/scenarios/<seg>...：seg 取到路径段结束，允许出现路径中段或行尾
_SCENARIO_PATH_RE = re.compile(
    r"orchestration/scenarios/[A-Za-z0-9_\-][A-Za-z0-9_\-/.\\\\]*"
)

# 带占位符的模板路径无法做存在性断言
_TEMPLATE_SEGMENTS = {"<场景名>", "<scene>", "<your_scenario>", "${", "<", "move_head_demo", "my_custom_demo", "my_scene"}


def _iter_files():
    for root in _SCAN_ROOTS:
        if root.is_file():
            yield root
            continue
        for path in root.rglob("*"):
            if not path.is_file():
                continue
            if _SKIP_PARTS & set(path.parts):
                continue
            rel = str(path.relative_to(REPO_ROOT))
            if any(rel == prefix or rel.startswith(prefix + "/")
                   for prefix in _SKIP_REL_PREFIXES):
                continue
            if path.suffix not in _SCAN_SUFFIXES:
                continue
            yield path


def _is_existing_repo_path(text):
    """文本形态的仓库相对路径是否存在；剥掉行尾标点与 markdown 尾巴。"""
    cleaned = text.replace("\\_", "_")  # markdown 转义
    cleaned = cleaned.rstrip(".,;:`\"')。，；：")
    cleaned = cleaned.split("#", 1)[0].rstrip("/")
    if not cleaned:
        return False
    if any(seg in cleaned for seg in _TEMPLATE_SEGMENTS):
        return True  # 模板路径跳过存在性判断，不算失效
    if cleaned in _KNOWN_UNPUBLISHED:
        return True  # 正文已标注"未随本仓发布"，豁免
    return (REPO_ROOT / cleaned).exists()


@pytest.mark.unit
def test_scenario_path_references_exist():
    """文档与代码里出现的 orchestration/scenarios/<seg> 路径必须真实存在。"""
    stale = []
    for path in _iter_files():
        try:
            text = path.read_text(encoding="utf-8")
        except (UnicodeDecodeError, OSError):
            continue
        rel = path.relative_to(REPO_ROOT)
        if str(rel) in _ARCHIVED_FILES:
            continue
        for line_no, line in enumerate(text.splitlines(), 1):
            # 同一行里合法的 scenarios_internal 前缀路径会整体匹配到，
            # 存在性判断自然通过，无需特判
            for match in _SCENARIO_PATH_RE.finditer(line):
                if not _is_existing_repo_path(match.group(0)):
                    stale.append((str(rel), line_no, match.group(0)))
    assert not stale, (
        "出现指向不存在场景路径的引用：\n"
        + "\n".join(f"  {src}:{no} -> {dst}" for src, no, dst in sorted(stale))
    )


@pytest.mark.unit
def test_scan_scope_covers_key_roots():
    """扫描根必须真实存在，防止目录改名后守卫静默缩水。"""
    for root in _SCAN_ROOTS:
        assert root.exists(), f"扫描根不存在：{root}"


# —— 第二层检测：拼接形态与相对形态（第一层正则匹配不到的） ——

# os.path.join(..., "orchestration", "scenarios", "<seg>"...) 与
# Path(...) / "scenarios" / "<seg>" 形态：同处一行含 "scenarios" 字面量 +
# 已迁/顶层场景目录名，即校验 orchestration/scenarios/<名> 或
# orchestration/scenarios/scenarios_internal/<名> 是否存在
_KNOWN_SEGMENT_RE = re.compile(r'"([A-Za-z0-9_\-]+)"')

# --scenario scenarios/<seg> 等相对写法：按 orchestration/ 解析
_REL_SCENARIO_RE = re.compile(r"(?<![\w/])scenarios/([A-Za-z0-9_\-][A-Za-z0-9_\-/.]*)")


def _known_scenario_names():
    base = REPO_ROOT / "orchestration" / "scenarios"
    names = set()
    if base.is_dir():
        names |= {p.name for p in base.iterdir() if p.is_dir()}
        inner = base / "scenarios_internal"
        if inner.is_dir():
            names |= {p.name for p in inner.iterdir() if p.is_dir()}
    return names


def _check_joined_and_relative():
    """返回 [(文件, 行号, 描述)]：拼接/相对形态的失效场景路径。"""
    problems = []
    known = _known_scenario_names()
    for path in _iter_files():
        rel = path.relative_to(REPO_ROOT)
        if str(rel) in _ARCHIVED_FILES:
            continue
        try:
            text = path.read_text(encoding="utf-8")
        except (UnicodeDecodeError, OSError):
            continue
        for line_no, line in enumerate(text.splitlines(), 1):
            if "scenarios" not in line:
                continue
            # 1) 拼接形态："scenarios" 后的字面量按顺序拼成路径，整体必须存在
            #    （评审三指出逐名判断恒 False 的死代码缺陷，按其补丁替换）
            if '"scenarios"' in line:
                toks = re.findall(r'"([A-Za-z0-9_\-./]+)"', line)
                if "scenarios" in toks:
                    tail = toks[toks.index("scenarios") + 1:]
                    if tail:
                        target = REPO_ROOT / "orchestration" / "scenarios" / "/".join(tail)
                        if not target.exists():
                            problems.append((str(rel), line_no,
                                             f"joined path -> {target.relative_to(REPO_ROOT)}"))
            # 2) 相对形态：--scenario scenarios/<seg>（前面不是 orchestration/）
            for m in _REL_SCENARIO_RE.finditer(line):
                if "orchestration/scenarios" in line:
                    continue  # 第一层已覆盖全路径形态
                seg = m.group(1).rstrip("/.,;:`")
                if any(seg in _TEMPLATE_SEGMENTS for _ in [0]):
                    continue
                if seg in _KNOWN_UNPUBLISHED:
                    continue
                target = REPO_ROOT / "orchestration" / "scenarios" / seg
                if not target.exists():
                    problems.append((str(rel), line_no, f"relative path -> scenarios/{seg}"))
    return problems


@pytest.mark.unit
def test_joined_and_relative_scenario_paths_exist():
    """拼接形态与相对形态的场景路径引用也必须真实存在。"""
    problems = _check_joined_and_relative()
    assert not problems, (
        "拼接/相对形态的失效场景路径：\n"
        + "\n".join(f"  {src}:{no} -> {desc}" for src, no, desc in sorted(problems))
    )


def test_joined_detection_catches_historical_bugs(tmp_path, monkeypatch):
    """死代码回归自测：两条曾让 runner 崩溃的拼接形态必须被命中（评审三要求）。"""
    import sys

    g = sys.modules[__name__]

    probes = {
        "a.py": 'scenarios_root = os.path.join(studio_root, "orchestration", "scenarios", "palletize_box_internal")\n',
        "b.py": 'root = Path(__file__).resolve().parents[2] / "scenarios" / "dismantle_box_internal"\n',
    }
    # 探针文件放进被扫描的 apps/ 根下临时子目录，避免污染仓库正文
    probe_dir = REPO_ROOT / "apps" / ".guard_probe_tmp"
    probe_dir.mkdir(exist_ok=True)
    try:
        for name, content in probes.items():
            (probe_dir / name).write_text(content)
        monkeypatch.setattr(g, "_iter_files", lambda: [probe_dir / n for n in probes])
        problems = g._check_joined_and_relative()
        hit_desc = " ".join(desc for _, _, desc in problems)
        assert "palletize_box_internal" in hit_desc, f"探针 a 未命中: {problems}"
        assert "dismantle_box_internal" in hit_desc, f"探针 b 未命中: {problems}"
    finally:
        for name in probes:
            (probe_dir / name).unlink(missing_ok=True)
        probe_dir.rmdir()


@pytest.mark.unit
def test_gitignored_superpowers_docs_are_not_scanned():
    """`docs/superpowers/` 是 .gitignore 里的个人计划/规格目录，不属于仓库内容。

    它可能引用迁移前的旧场景路径；CI 检出时根本不存在，本地却会让守卫误报。
    扫描范围必须只覆盖**会进仓库**的文档。
    """
    files = _iter_files()
    offenders = [p for p in files
                 if str(p.relative_to(REPO_ROOT)).startswith("docs/superpowers/")]
    assert not offenders, offenders
