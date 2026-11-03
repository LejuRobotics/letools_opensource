# -*- coding: utf-8 -*-
"""文档目录结构的不变量断言。

纯文件系统扫描：不依赖 ROS，不 import 项目代码，秒级完成。
只断言外部可观察行为（目录名、文件存在），不断言正文内容，
这样正文怎么改写测试都不会误报。
"""

from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
DOCS = REPO_ROOT / "docs"

# 内部文档面：在**仓库根**，与 module_internal/ 等同级（不在 docs/ 下）。
# 2026-10-10 第二次定案把它从 docs/ 移出；CI 的 rsync --exclude='*_internal/'
# 按 basename 匹配，两种位置实测均正确排除。
DOCS_INTERNAL = REPO_ROOT / "docs_internal"

# 章节目录名，原与飞书《16_LeTools项目文档目录方案》逐字一致；
# 2026-10-31 收敛开源文档：「发布与贡献」移入 docs_internal/完整文档/，补充内容改为 07 章。
CHAPTERS = (
    "01-介绍",
    "02-环境安装",
    "03-接口详细文档",
    "04-简单案例运行",
    "05-进阶开发指南",
    "06-常见问题FAQ",
    "07-补充内容",
    "08-贡献指南",
)

# docs_internal/ 应包含的 13 份文件（相对 docs_internal/ 的 POSIX 路径）。
# 用白名单而不是「恰 13 份」计数：计数断言会误伤在该目录下新增文档
# （2026-10-13 新增一份报告即掉到 1 failed, 28 passed）。
# 白名单只防「既有文件被删/改名/挪走」，新增文件不再触发失败。
DOCS_INTERNAL_FILES = (
    "托盘伺服部署与跑通.md",
    "app_debug_reference/README.md",
    "app_debug_reference/TEST_SCRIPT_MAPPING.md",
    "architecture/PYTREES_NODE_CLASSIFICATION.md",
    "architecture/STUDIO_EMBODIED_MIGRATION_BLUEPRINT.md",
    "architecture/TEST_REFACTORED_SKILL_MAPPING.md",
    "guides/CAMERA_HARDWARE_TEST_GUIDE.md",
    "guides/CI_GUIDE.md",
    "guides/CODE_REVIEW_CHECKLIST.md",
    "guides/DEPALLETIZE_BIN_MIGRATION_GUIDE.md",
    "guides/LOGGING_COMPLETE_GUIDE.md",
    "guides/ROS_ADAPTER_BEST_PRACTICES.md",
    "guides/SDK_Integration_Guide.md",
)


@pytest.mark.unit
@pytest.mark.parametrize("name", CHAPTERS)
def test_chapter_directory_exists(name):
    """8 个章节目录逐个存在且是目录。"""
    assert (DOCS / name).is_dir(), f"缺少章节目录 docs/{name}/"


@pytest.mark.unit
def test_all_chapters_present():
    """章节目录集合恰好等于目标集合，没有漏建。"""
    actual = {p.name for p in DOCS.iterdir() if p.is_dir() and p.name in CHAPTERS}
    assert actual == set(CHAPTERS), f"章节目录集合不符，缺少：{set(CHAPTERS) - actual}"


@pytest.mark.unit
def test_homepage_exists():
    """首页 docs/README.md 存在且非空。"""
    homepage = DOCS / "README.md"
    assert homepage.is_file(), "缺少首页 docs/README.md"
    assert homepage.stat().st_size > 0, "docs/README.md 是空文件"


@pytest.mark.unit
def test_adr_not_in_repo():
    """ADR 已移出仓库（用户 2026-10-10 定案：设计决策不含公开面）。

    迁到仓库外 ``.scratch/letools-docs-restructure/adr/``；此处只断言它不再
    出现于仓库内，防止以后不小心把它加回来。
    """
    assert not (DOCS / "adr").exists(), "docs/adr/ 应已移出仓库"
    assert not (REPO_ROOT / "docs" / "adr").exists(), "docs/adr/ 应已移出仓库"


@pytest.mark.unit
def test_docs0929_directory_gone():
    """旧素材目录 docs0929/ 整个消失（内容已按章归口）。"""
    assert not (DOCS / "docs0929").exists(), "docs/docs0929/ 仍然存在"


@pytest.mark.unit
def test_no_executables_under_docs():
    """docs/ 下不再躺着可执行文件（3 个脚本已移出）。"""
    stray = [
        str(p.relative_to(REPO_ROOT))
        for pattern in ("*.sh", "*.py")
        for p in DOCS.rglob(pattern)
        if "__pycache__" not in p.parts
    ]
    assert not stray, "docs/ 下不应存在脚本：" + ", ".join(sorted(stray))


@pytest.mark.unit
def test_docs_internal_moved_to_root():
    """docs_internal/ 在仓库根（与 module_internal/ 同级），既有 13 份文件都在。"""
    assert DOCS_INTERNAL.is_dir(), "缺少仓库根 docs_internal/"
    assert not (DOCS / "docs_internal").exists(), "docs/docs_internal/ 应已移走"
    actual = {
        p.relative_to(DOCS_INTERNAL).as_posix()
        for p in DOCS_INTERNAL.rglob("*")
        if p.is_file()
    }
    missing = set(DOCS_INTERNAL_FILES) - actual
    assert not missing, f"docs_internal/ 缺少文件：{sorted(missing)}"
