# -*- coding: utf-8 -*-
"""文档站链接可达性与篇目数的机械断言。

纯文件系统扫描：不依赖 ROS，不 import 项目代码，秒级完成。
只断言外部可观察行为（目标文件存在、篇目数），不断言正文内容。

与 ``test_docs_structure.py`` 分开成两个模块：那边管「目录长对了没有」，
这边管「点得开吗、数对得上吗」，扫描逻辑完全不同，合并只会让两边都难读。
凡是「路径在不在、文件有几个」的存在性断言都归那边，本模块只管链接可达性与篇目数。
"""

import re
import urllib.parse
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]

# 不扫描：git 子模块（内容不属于本仓库）、第三方 vendored 包
EXCLUDED_PREFIXES = (
    "drivers/leju/kuavo_humanoid_sdk/",  # git submodule
    "third_party/",
    # 上游第三方 ROS 包，README 里引用的 AUTHORS.md / LICENSE 未随包引入
    "infrastructure/ros_packages/src/kuavo_tf2_web_republisher/",
)

# 用户 2026-10-10 定案：扩大到全仓源码目录（原先只扫 docs/**/*.md 与根 README.md）
# docs_internal/ 已移出 docs/ 到仓库根（同日第二次定案），必须显式列出，
# 否则那 13 份内部文档会悄悄掉出链接扫描范围。
SCAN_ROOTS = (
    "docs",
    "docs_internal",
    "README.md",
    "apps",
    "orchestration",
    "adapters",
    "core",
    "skills",
    "drivers",
    "infrastructure",
)

# 各章目标篇目数。2026-10-12 用户定案在 03 章增补节点清单（现为 11-节点清单.md）
# （03 章 10 → 11 篇），已回写飞书《16_LeTools项目文档目录方案》。
# 2026-10-31：内部人员专用篇目（发布与贡献、代码规范与评审、版本记录、目录结构全貌等）
# 移入 docs_internal/完整文档/，公开文档收敛为 7 章。
# 模仿学习案例改为 04 章第 11 篇示教（05 章减为 7 篇、04 章增为 12 篇）。
CHAPTER_ARTICLE_COUNTS = {
    "01-介绍": 7,
    "02-环境安装": 5,
    "03-接口详细文档": 11,
    "04-简单案例运行": 12,
    "05-进阶开发指南": 7,
    "06-常见问题FAQ": 8,
    "07-补充内容": 2,
    "08-贡献指南": 6,
}
TOTAL_ARTICLES = sum(CHAPTER_ARTICLE_COUNTS.values())  # 58，外加首页共 59 条 .md

# 历史失效链接已修复或取消链接；后续新增失效链接不允许入库。
KNOWN_DEAD_LINKS = ()


def _iter_markdown_links(text):
    """逐个产出 markdown 链接的目标串。

    不用 ``\\[([^\\]]*)\\]\\(([^)]*)\\)`` 这种朴素正则：本仓库的篇名含**半角括号**
    （``12-搬箱案例精讲(一).md``）与 **URL 编码**（``%20``），朴素正则会提前在
    括号或 ``)`` 处截断，把好好的链接读成断链。同一棵文档树实测假阳性 22 处。

    改为：从左到右找 ``](``，按**括号深度配对**取完整目标串，再交给调用方解码。
    """
    cursor = 0
    while True:
        open_at = text.find("](", cursor)
        if open_at < 0:
            return
        i = open_at + 2
        depth = 1
        while i < len(text) and depth:
            if text[i] == "(":
                depth += 1
            elif text[i] == ")":
                depth -= 1
            i += 1
        yield text[open_at + 2 : i - 1]
        cursor = i


# 跨文件锚点校验只关心的范围：文档站自身（不含各模块 README 与源码注释）
DOC_SITE_ROOTS = ("docs", "README.md")


def _is_skipped_target(target):
    """外链、纯锚点、绝对路径都不归本测试管。"""
    return target.startswith(("http://", "https://", "mailto:", "#", "/"))


def _split_target(raw):
    """把链接目标原文拆成 ``(路径, 锚点)``：按写法取出目标串，再切开 ``#`` 与 ``?query``。

    CommonMark 的目标串有两种写法：``<路径>``（**允许含空格**，可后跟 ``"标题"``）与
    裸路径（**不得含空格**，同样可后跟标题）。两种都要能解析：本仓的篇名含空格
    （``10-Skill→Node→场景 层级关系与选型建议.md``），写法是 ``<...>``，
    不剥尖括号就会把好好的链接读成断链。

    含空格却**没**用 ``<>`` 包起来的写法不是合法链接（渲染器不会生成 ``<a>``），
    由 ``test_no_unescaped_space_in_link_target`` 单独报出；这里仍按原样取出，
    以便该用例能指出具体是哪个目标。
    """
    text = raw.strip()
    if text.startswith("<"):
        close_at = text.find(">")
        target = text[1:close_at] if close_at >= 0 else text[1:]
    else:
        # 剥掉可选标题：目标串与标题之间以空白 + 引号分隔
        target = re.split(r"""\s+["']""", text, maxsplit=1)[0]
    path_part, _, anchor = target.partition("#")
    return path_part.split("?")[0], anchor


# 目标串里未被 ``<>`` 包住的空白字符：这种写法渲染器不认，链文本会原样显示成字面量。
_UNESCAPED_SPACE_RE = re.compile(r"[ \t]")


def _iter_markdown_sources(roots):
    """按根枚举 .md：目录取 ``**/*.md``，文件直收；跳过 ``__pycache__`` 与排除前缀。"""
    for root in roots:
        path = REPO_ROOT / root
        candidates = path.glob("**/*.md") if path.is_dir() else [path]
        for source in candidates:
            if not source.is_file() or "__pycache__" in source.parts:
                continue
            if source.relative_to(REPO_ROOT).as_posix().startswith(EXCLUDED_PREFIXES):
                continue
            yield source


def _collect_dead_links():
    """扫描 SCAN_ROOTS 下的全部 .md，返回 (源文件, 目标) 集合。"""
    dead = set()
    for source in _iter_markdown_sources(SCAN_ROOTS):
        rel = source.relative_to(REPO_ROOT).as_posix()
        text = source.read_text(encoding="utf-8", errors="replace")
        for raw in _iter_markdown_links(text):
            raw = raw.strip()
            if not raw or _is_skipped_target(raw):
                continue
            target = urllib.parse.unquote(_split_target(raw)[0])
            if not target:
                continue
            if not (source.parent / target).exists():
                dead.add((rel, target))
    return dead


def _heading_slug(heading):
    """近似 GitHub 的标题锚点算法：小写 → 去掉标点与 emoji（保留字母/数字/下划线/空格/连字符）→ 空格换连字符。"""
    slug = heading.strip().lower()
    slug = re.sub(r"[^\w\s-]", "", slug)  # \w 含下划线，故下划线保留
    return slug.replace(" ", "-")


def _iter_heading_slugs(path):
    """产出目标 .md 里全部 ATX 标题的 slug（跳过 fenced code block 内的行）。"""
    slugs = set()
    in_fence = False
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        if line.lstrip().startswith("```"):
            in_fence = not in_fence
            continue
        if in_fence:
            continue
        match = re.match(r"^#{1,6}\s+(.*)$", line)
        if match:
            slugs.add(_heading_slug(match.group(1)))
    return slugs


@pytest.mark.unit
def test_no_dead_links_beyond_baseline():
    """除已登记基线外，不得出现新的死链。

    基线说明见 ``KNOWN_DEAD_LINKS``；文档站首页与根 README 的交叉引用均已可达，
    全库死链数恰好等于基线条数。
    """
    assert len(KNOWN_DEAD_LINKS) == len(set(KNOWN_DEAD_LINKS)), (
        "基线条目重复：set() 差集会静默折叠，长度数字会撒谎"
    )
    unexpected = _collect_dead_links() - set(KNOWN_DEAD_LINKS)
    assert not unexpected, "出现新的死链：\n" + "\n".join(
        f"  {src} -> {dst}" for src, dst in sorted(unexpected)
    )


@pytest.mark.unit
def test_baseline_entries_still_present():
    """基线条目必须仍然真的是死链。

    若某条被上游修好了，本用例会红，提醒把它从基线里删掉 —— 基线不该成为
    「永远不再检查」的垃圾桶。
    """
    dead = _collect_dead_links()
    fixed = [entry for entry in KNOWN_DEAD_LINKS if entry not in dead]
    assert not fixed, "基线条目已被修好，请从 KNOWN_DEAD_LINKS 移除：\n" + "\n".join(
        f"  {src} -> {dst}" for src, dst in sorted(fixed)
    )


@pytest.mark.unit
def test_cross_file_anchor_targets_exist():
    """跨文件链接的 #锚点必须命中目标 .md 的标题 slug 集合（零容忍，不设基线）。

    只查 ``DOC_SITE_ROOTS``（``docs/**/*.md`` 与根 ``README.md``）里「带路径 + 带 #锚点」的相对链接：
    外链、纯页内锚点、目标非 .md 或目标不存在的都跳过（后两者归死链用例管）。
    范围不跟着 ``SCAN_ROOTS`` 走：各模块 README 与源码注释不是文档站，锚点失配不归本用例。
    """
    missing = []
    for source in _iter_markdown_sources(DOC_SITE_ROOTS):
        text = source.read_text(encoding="utf-8", errors="replace")
        for raw in _iter_markdown_links(text):
            raw = raw.strip()
            if not raw or _is_skipped_target(raw) or "#" not in raw:
                continue
            path_part, anchor = _split_target(raw)
            if not path_part:
                continue  # 纯页内锚点
            target = urllib.parse.unquote(path_part)
            if not target.lower().endswith(".md"):
                continue
            target_path = (source.parent / target).resolve()
            if not target_path.is_file():
                continue  # 目标缺失归死链断言管
            if urllib.parse.unquote(anchor) not in _iter_heading_slugs(target_path):
                missing.append((str(source.relative_to(REPO_ROOT)), raw))
    assert not missing, "跨文件锚点失配：\n" + "\n".join(
        f"  {src} -> {raw}" for src, raw in sorted(missing)
    )


@pytest.mark.unit
def test_no_unescaped_space_in_link_target():
    """带空格的链接目标必须写成 ``<路径>`` 或把空格写成 ``%20``，不能裸着写。

    渲染器对裸空格的写法不生成 ``<a>``，链文本会原样显示成字面量——**而文件本身存在**，
    所以死链断言看不见它（目标路径确实可解析），只能靠这条形态校验兜住。

    判定范围与 ``KNOWN_DEAD_LINKS`` 无关：这是**形态**约束（写法对不对），
    不是**可达性**约束（点不点得开）。已登记的基线条目是历史欠账，不是让新错误
    合法化的依据，故基线不参与本用例。
    """
    offenders = []
    for source in _iter_markdown_sources(SCAN_ROOTS):
        rel = source.relative_to(REPO_ROOT).as_posix()
        text = source.read_text(encoding="utf-8", errors="replace")
        for lineno, line in enumerate(text.splitlines(), 1):
            for raw in _iter_markdown_links(line):
                stripped = raw.strip()
                if not stripped or _is_skipped_target(stripped) or stripped.startswith("<"):
                    continue
                target = re.split(r"""\s+["']""", stripped, maxsplit=1)[0]
                if _UNESCAPED_SPACE_RE.search(target):
                    offenders.append(
                        f"  {rel}:{lineno} 目标原文 {stripped!r}；"
                        f"期望：写成 <{target}> 或把空格写成 %20，实际：裸空格"
                    )
    assert not offenders, "链接目标含未转义空格：\n" + "\n".join(offenders)


@pytest.mark.unit
@pytest.mark.parametrize("chapter,expected", sorted(CHAPTER_ARTICLE_COUNTS.items()))
def test_chapter_article_count(chapter, expected):
    """每章顶层 .md 篇数等于目标数（不含章内子目录）。"""
    directory = REPO_ROOT / "docs" / chapter
    assert directory.is_dir(), f"缺少章节目录 docs/{chapter}/"
    actual = len(list(directory.glob("*.md")))
    assert actual == expected, f"docs/{chapter}/ 篇数为 {actual}，期望 {expected}"


@pytest.mark.unit
def test_total_article_count():
    """全部章篇目合计 58 篇，加首页 docs/README.md 共 59 条 .md。"""
    total = sum(
        len(list((REPO_ROOT / "docs" / chapter).glob("*.md")))
        for chapter in CHAPTER_ARTICLE_COUNTS
    )
    assert total == TOTAL_ARTICLES, f"篇目合计 {total}，期望 {TOTAL_ARTICLES}"
    assert (REPO_ROOT / "docs" / "README.md").is_file(), "缺少首页 docs/README.md"
