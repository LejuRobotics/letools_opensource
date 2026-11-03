# -*- coding: utf-8 -*-
"""文档 ↔ 代码一致性守卫：把「手工核对得出的数字」变成「持续保证」。

与另两个测试文件的分工：

- ``test_docs_structure.py``：目录长对了没有；
- ``test_docs_links.py``：链接点得开吗、篇目数对不对；
- 本文件：**文档里写的数字与集合，和代码里的实际事实是否一致**。

三条硬规矩（DOCS-29）：

1. **读事实源，不复制事实**：节点数从 ``@define_manifest`` 装饰器数出来，技能数从
   ``skills/atomic/refactored_sdk/*.py`` 数出来，不把 57 / 33 / 11 / 107 写进断言。
2. **不读本机绝对路径**：一律从 ``Path(__file__).resolve().parents[1]`` 起算。
3. **期望值当前就不成立时用 ``xfail`` 标注并写明阻塞项**，不放宽断言去迁就。

纯 AST + 文件系统扫描：不 import 项目代码（避免拉起 ROS / SDK），秒级完成。
"""

import ast
import re
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]
DOCS = REPO_ROOT / "docs"
SKILLS = REPO_ROOT / "skills"
NODES_DIR = REPO_ROOT / "orchestration" / "nodes"
REFACTORED_SDK_DIR = SKILLS / "atomic" / "refactored_sdk"
MIXINS_DIR = REPO_ROOT / "adapters" / "hardware" / "leju_wheeled" / "mixins"

# ---------------------------------------------------------------------------
# 守卫 1｜E1 例外计数
# ---------------------------------------------------------------------------

# 声明 E1 规模的两篇（同源，口径必须一致）：01-04 面向读者、05-01 面向贡献者。
E1_DOC_PATHS = (
    "docs/01-介绍/04-系统架构与分层.md",
    "docs/05-进阶开发指南/01-扩展总则与分层规范.md",
)

# 计数口径（DOCS-24 定案）：**只用文件数**。扫描 ``skills/**/*.py`` 里
# ``import adapters`` / ``from adapters``（AST 判定，注释与文档字符串里的
# 提及不算），再按文档自己声明的「是否排除 ``*_internal/``」决定算哪个数：
# 不排除 = 11 文件，排除 = 9 文件。
_E1_HEADING_RE = re.compile(r"^###\s+E1[：:]")
_HEADING_RE = re.compile(r"^#{2,3}\s")
_E1_SUMMARY_ROW_RE = re.compile(r"^\|\s*E1\s*\|")
# 声明形态按仓库现有写法认：``**实测规模：11 处 / 6 文件。**`` 与
# 小结表里的 ``**11 处 / 6 文件**``——数字落在加粗段里，且是「N 文件」而不是「N 处」。
# ``N`` 与「文件」之间**有「个」也算**：正文主声明写「9 个文件」、括注写「11 文件」，
# 少认了前者就等于放着主数字不守（DOCS-30 MEDIUM 1）。
_BOLD_RE = re.compile(r"\*\*([^*]+)\*\*")
_FILE_COUNT_RE = re.compile(r"(\d+)\s*个?\s*文件")
# 口径声明：文档自己写清「排除 ``*_internal/``」时按 9 文件算，没写就按 11 文件算。
# 要求「排除」与 ``_internal`` 出现在同一句里，避免 E1 里提到别的 ``_internal`` 路径
# （例如表格里列了 ``haichen_internal/`` 的技能文件）被误读成口径声明。
# 2026-10-31：公开文档不再出现 ``*_internal`` 字样，改写为「按开源发布范围统计」——
# 开源产物不含 ``*_internal/``，两种写法口径相同，均接受。
_E1_EXCLUDE_CALIBER_RE = re.compile(r"排除[^。；\n]{0,30}_internal|开源发布范围")


def _imports_adapters(path):
    """该文件是否 import 了 ``adapters``（含 ``adapters.xxx``）。"""
    tree = ast.parse(path.read_text(encoding="utf-8"))
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            if any(alias.name == "adapters" or alias.name.startswith("adapters.") for alias in node.names):
                return True
        elif isinstance(node, ast.ImportFrom):
            module = node.module or ""
            if module == "adapters" or module.startswith("adapters."):
                return True
    return False


def _adapters_importer_files(exclude_internal):
    """``skills/`` 下 import 了 ``adapters`` 的文件清单（仓库相对路径，已排序）。

    ``exclude_internal=True`` 时跳过路径中含 ``*_internal/`` 的文件——这是文档
    声明的另一种口径，两种都得算得出来，才能和文档声明对上号。
    """
    found = []
    for path in sorted(SKILLS.rglob("*.py")):
        rel = path.relative_to(REPO_ROOT)
        if exclude_internal and any(part.endswith("_internal") for part in rel.parts):
            continue
        if _imports_adapters(path):
            found.append(rel.as_posix())
    return found


def _e1_declarations(text):
    """产出 ``(行号, 声明片段)``：E1 小节正文 + 小结表里的 E1 行。

    只取这两处：文档别处的「N 个文件」（例如 E2 的「45 个技能文件」）说的是别的
    口径，混进来只会让守卫误报。
    """
    lines = text.splitlines()
    out = []
    for index, line in enumerate(lines):
        if not _E1_HEADING_RE.match(line):
            continue
        for offset in range(index + 1, len(lines)):
            if _HEADING_RE.match(lines[offset]):
                break
            out.append((offset + 1, lines[offset]))
        break
    for index, line in enumerate(lines):
        if _E1_SUMMARY_ROW_RE.match(line):
            out.append((index + 1, line))
    return out


def _declared_file_counts(snippet):
    """从一条声明里抽出「N 文件」的 N；有加粗段就只看加粗段，没有就看整行。"""
    bold = _BOLD_RE.findall(snippet)
    scope = "\n".join(bold) if bold else snippet
    return [int(number) for number in _FILE_COUNT_RE.findall(scope)]


@pytest.mark.unit
def test_e1_exception_count():
    """E1 规模声明必须等于实测文件数，且用了排除口径就得声明出来。

    计数口径（DOCS-24 定案，与文档声明一致）：**只用文件数**，不断言「N 处」。
    扫描 ``skills/**/*.py`` 的 ``import adapters`` / ``from adapters``（AST 判定，
    注释与文档字符串里的提及不算）。两种口径都算得出来：

    - **不排除 ``*_internal/``：11 文件**（默认口径）
    - **排除 ``*_internal/``：9 文件**（文档须在 E1 小节里写明「排除 ``*_internal/``」）

    规则：写的数字必须真等于某个口径的实测值（否则是编的）；写的是排除口径的值时，
    文档必须把这条口径声明出来（否则读者无从复现）。两篇（``01-04`` / ``05-01``）
    都要有声明。
    """
    all_count = len(_adapters_importer_files(exclude_internal=False))
    excl_count = len(_adapters_importer_files(exclude_internal=True))
    problems = []
    for doc_path in E1_DOC_PATHS:
        text = (REPO_ROOT / doc_path).read_text(encoding="utf-8")
        declarations = _e1_declarations(text)
        if not declarations:
            problems.append(f"{doc_path}：E1 小节与小结表里都没有规模声明")
            continue
        section_text = "\n".join(snippet for _, snippet in declarations)
        declares_excl_caliber = bool(_E1_EXCLUDE_CALIBER_RE.search(section_text))
        declared_any = False
        for lineno, snippet in declarations:
            for declared in _declared_file_counts(snippet):
                declared_any = True
                if declared not in (all_count, excl_count):
                    problems.append(
                        f"{doc_path}:{lineno} 声明 {declared} 文件，"
                        f"期望：{all_count} 文件（不排除 *_internal/）或 {excl_count} 文件（排除），"
                        f"实际：{declared}；原文：{snippet.strip()}"
                    )
                elif declared == excl_count and not declares_excl_caliber:
                    problems.append(
                        f"{doc_path}:{lineno} 声明 {declared} 文件（排除 *_internal/ 口径），"
                        f"期望：E1 小节里写明「排除 *_internal/」，实际：未声明该口径；"
                        f"原文：{snippet.strip()}"
                    )
        if not declared_any:
            problems.append(
                f"{doc_path}：E1 小节声明存在但未给出「N 文件」数字"
                f"（实测 {all_count} 文件，口径：不排除 *_internal/）"
            )
    assert not problems, "E1 计数与文档声明不一致：\n" + "\n".join(f"  {p}" for p in problems)


@pytest.mark.unit
def test_e1_file_count_guard_catches_primary_declaration_drift():
    """注入漂移自检：把 E1 主声明的「N 个文件」改数，守卫必须能取到它。

    防的是 DOCS-30 MEDIUM 1 那类缺口——正则只认「N 文件」不认「N 个文件」，
    于是带「个」的主声明整个逃过校验，守卫常年绿灯（``test_e1_exception_count``
    是状态断言，文档漂移时它不会红，所以缺口能活下来）。

    断言的是「取得到这个数字」，不是「整条守卫会红」：后者依赖文档当前值，
    前者只依赖取数逻辑，注入任意数字都成立。
    """
    healthy = "**实测规模：9 个文件（不排除 `*_internal/` 时为 11 文件）。**"
    drifted = "**实测规模：8 个文件（不排除 `*_internal/` 时为 11 文件）。**"
    assert 9 in _declared_file_counts(healthy), "守卫取不到主声明 9（被「个」挡住了）"
    assert 8 in _declared_file_counts(drifted), "守卫取不到漂移后的 8"
    # 括注里不带「个」的次要数字同样要在，两种写法都得认。
    assert 11 in _declared_file_counts(healthy), "守卫取不到括注里的 11"
    # 小结表这一行在加粗段里写「N 个文件」，改前正则会取到空列表、整条路径空转。
    assert 9 in _declared_file_counts("| E1 | `skills` → `adapters` | **9 个文件** |"), (
        "守卫取不到小结表形态的 9（加粗段里只有「9 个文件」可认）"
    )


# ---------------------------------------------------------------------------
# 守卫 2｜代码围栏外的裸 <JSON>
# ---------------------------------------------------------------------------

# 行内代码（`` `<JSON>` ``）是 DOCS-28 允许的占位写法：渲染成字面量，不会被当成
# HTML 标签吞掉，故要先把它整段抹掉再找。
_INLINE_CODE_RE = re.compile(r"`[^`]*`")


def _bare_json_hits():
    """``docs/**/*.md`` 里「围栏外且不在行内代码里」的 ``<JSON>`` 命中清单。"""
    hits = []
    for path in sorted(DOCS.rglob("*.md")):
        in_fence = False
        text = path.read_text(encoding="utf-8", errors="replace")
        for lineno, line in enumerate(text.splitlines(), 1):
            if line.lstrip().startswith("```"):
                in_fence = not in_fence
                continue
            if in_fence:
                continue
            if "<JSON>" in _INLINE_CODE_RE.sub("``", line):
                hits.append((path.relative_to(REPO_ROOT).as_posix(), lineno, line.strip()))
    return hits


@pytest.mark.unit
def test_no_bare_json_placeholder():
    """正文里不得出现裸 ``<JSON>``：Markdown 渲染器会把尖括号当 HTML 标签整段吞掉。

    判定按「围栏外」定义，不按当前计数写死：```json 围栏内的 ``<JSON>`` 是示例内容，
    不报错；行内代码 `` `<JSON>` `` 是允许的占位写法，也不算裸。
    """
    hits = _bare_json_hits()
    assert not hits, "docs/ 正文里出现裸 <JSON>（应换成真实片段或行内代码）：\n" + "\n".join(
        f"  {path}:{lineno}  {line}" for path, lineno, line in hits
    )


# ---------------------------------------------------------------------------
# 守卫 4｜节点清单 ↔ @define_manifest
# ---------------------------------------------------------------------------

NODE_LIST_DOC = "docs/03-接口详细文档/11-节点清单.md"

# 篇内条目形态：``### 2.1 `ArmEeBurstTimedMove` ``
_NODE_ENTRY_RE = re.compile(r"^###\s+(\d+\.\d+)\s+`([^`]+)`")


def _manifest_registered_classes():
    """顶层 ``orchestration/nodes/*.py`` 里挂了 ``@define_manifest`` 的类 → 文件名。

    只看类级装饰器（``@define_manifest(...)`` 或裸 ``@define_manifest``），
    不递归子目录——清单篇自己声明口径是「``orchestration/nodes/*.py``（顶层）」。
    """
    classes = {}
    for path in sorted(NODES_DIR.glob("*.py")):
        tree = ast.parse(path.read_text(encoding="utf-8"))
        for node in tree.body:
            if not isinstance(node, ast.ClassDef):
                continue
            for decorator in node.decorator_list:
                target = decorator.func if isinstance(decorator, ast.Call) else decorator
                name = getattr(target, "id", None) or getattr(target, "attr", None)
                if name == "define_manifest":
                    classes[node.name] = path.name
    return classes


def _node_list_entries():
    """``11-节点清单.md`` 里的 ``### N.M `ClassName` `` 条目 → 小节号。"""
    entries = {}
    text = (REPO_ROOT / NODE_LIST_DOC).read_text(encoding="utf-8")
    for line in text.splitlines():
        match = _NODE_ENTRY_RE.match(line)
        if match:
            entries[match.group(2)] = match.group(1)
    return entries


@pytest.mark.unit
def test_nodes_manifest_sync():
    """manifest 注册类集合与节点清单条目集合必须双向相等。

    数字（57 = 57）只是结论，断言的是**集合相等**：多注册一个类却没写进清单篇、
    或清单篇留着一个已被删掉的类，两个方向都要报出来。
    """
    registered = _manifest_registered_classes()
    documented = _node_list_entries()
    only_doc = sorted(set(documented) - set(registered))
    only_src = sorted(set(registered) - set(documented))
    assert not (only_doc or only_src), (
        f"{NODE_LIST_DOC} 与 @define_manifest 注册类不一致"
        f"（文档 {len(documented)} 条 / 源码 {len(registered)} 个，期望：双向差集为空）：\n"
        + "\n".join(f"  仅文档有：{name}（小节 {documented[name]}）" for name in only_doc)
        + "\n".join(f"  仅源码有：{name}（orchestration/nodes/{registered[name]}）" for name in only_src)
    )


# ---------------------------------------------------------------------------
# 守卫 5｜refactored_sdk 技能文件 ↔ 技能总表
# ---------------------------------------------------------------------------

SKILLS_DOC = "docs/03-接口详细文档/04-skills-原子技能.md"
SKILLS_TABLE_HEADING = "### refactored_sdk 技能总表"
# 总表行形态：``| `arm_ee_traj_local_sdk.py` | `ArmEETrajLocalSdkSkill` | ... |``
_SKILL_ROW_RE = re.compile(r"^\|\s*`([^`]+\.py)`\s*\|")


def _refactored_sdk_files():
    """``skills/atomic/refactored_sdk/*.py``（不含 ``__init__.py``）文件名集合。"""
    return {path.name for path in REFACTORED_SDK_DIR.glob("*.py") if path.name != "__init__.py"}


def _skills_table_rows():
    """技能总表里被列出的 **.py 名 → 行号**；只取该小节，不误收别处的表格行。"""
    lines = (REPO_ROOT / SKILLS_DOC).read_text(encoding="utf-8").splitlines()
    rows = {}
    in_section = False
    for lineno, line in enumerate(lines, 1):
        if line.startswith("### "):
            in_section = line.startswith(SKILLS_TABLE_HEADING)
            continue
        if not in_section:
            continue
        match = _SKILL_ROW_RE.match(line)
        if match:
            rows[match.group(1)] = lineno
    return rows


@pytest.mark.unit
def test_skills_refactored_sdk_table():
    """技能总表列出的文件集合必须与 ``refactored_sdk/`` 实际文件集合双向相等。

    没有技能类的 ``__init__.py`` 不入表，也不算源；只比文件集合，不比方法签名——
    表格里的「Adapter 调用」「功能」两列是描述，不是可机械核对的契约。
    """
    files = _refactored_sdk_files()
    rows = _skills_table_rows()
    only_doc = sorted(set(rows) - files)
    only_src = sorted(files - set(rows))
    assert not (only_doc or only_src), (
        f"{SKILLS_DOC} 技能总表与 {REFACTORED_SDK_DIR.relative_to(REPO_ROOT).as_posix()}/ 不一致"
        f"（文档 {len(rows)} 行 / 源码 {len(files)} 个文件，期望：双向差集为空）：\n"
        + "\n".join(f"  仅文档有：{name}（第 {rows[name]} 行），源码里没有这个文件" for name in only_doc)
        + "\n".join(f"  仅源码有：{name}，总表未列出" for name in only_src)
    )


# ---------------------------------------------------------------------------
# 守卫 6｜行内编号交叉引用
# ---------------------------------------------------------------------------

# 引用形态：`` `NN-MM-篇名` `` / `` `NN-MM` `` / `` `NN-MM-篇名.md` ``。
# 只校验 ``NN-MM`` 编号段能在该章下解析到一个真实文件，**不**要求篇名逐字等于
# 文件名——仓库里存在「用正文标题称呼」的写法（`` `08-01-术语表` `` 指向
# ``01-参考资料.md``、`` `08-04-许可证` `` 指向 ``03-许可证.md``），断言篇名会产出
# 一批假阳性，把一个干净的守卫变成噪音源。
_XREF_RE = re.compile(r"`(\d{2})-(\d{2})(?:-([^`]*?))?`")


def _chapter_article_numbers():
    """``docs/`` 下的章节目录 → 该章顶层篇目的两位编号集合。"""
    chapters = {}
    for directory in DOCS.iterdir():
        if not directory.is_dir() or not re.match(r"^\d{2}-", directory.name):
            continue
        chapters[directory.name[:2]] = {
            path.name[:2] for path in directory.glob("*.md") if re.match(r"^\d{2}-", path.name)
        }
    return chapters


@pytest.mark.unit
def test_doc_cross_reference_numbering():
    """文档行内代码里裸写的 ``NN-MM`` 引用，必须指向该章真实存在的篇目。

    链接语法（``[文本](路径)``）由 ``test_docs_links`` / ``test_cross_file_anchor_targets_exist``
    管；这类**行内代码里的编号引用**不在其覆盖面内，故单列一条。
    """
    chapters = _chapter_article_numbers()
    problems = []
    for path in sorted(DOCS.rglob("*.md")):
        rel = path.relative_to(REPO_ROOT).as_posix()
        in_fence = False
        for lineno, line in enumerate(path.read_text(encoding="utf-8", errors="replace").splitlines(), 1):
            if line.lstrip().startswith("```"):
                in_fence = not in_fence
                continue
            if in_fence:
                continue  # 代码示例里的编号不是交叉引用
            for match in _XREF_RE.finditer(line):
                chapter, article, title = match.group(1), match.group(2), match.group(3)
                known = chapters.get(chapter)
                if known is None:
                    problems.append(
                        f"{rel}:{lineno} 引用 {match.group(0)}，期望：docs/ 下存在 {chapter}-* 章节目录，实际：不存在"
                    )
                elif article not in known:
                    problems.append(
                        f"{rel}:{lineno} 引用 {match.group(0)}，期望：{chapter} 章存在 {article}-* 篇，"
                        f"实际：该章顶层篇目编号为 {sorted(known)}"
                    )
    assert not problems, "行内编号引用指向不存在的篇目：\n" + "\n".join(f"  {p}" for p in problems)


# ---------------------------------------------------------------------------
# 守卫 7｜契约覆盖率分母的排除范围
# ---------------------------------------------------------------------------

# 契约覆盖率分母的文件集口径：``mixins/`` 下**平铺**的 Mixin 文件（含包 ``__init__.py``），
# 不递归子目录。``mixins/jibot/`` 与 ``mixins/jiateng/`` 的品牌底盘方法经 DOCS-34
# 判为「内部实现 / 待收口」，不属 IHardware 契约，故不计入分母。
# 决策依据：DOCS-24（D1 边界）+ DOCS-34（Q5 = b）。**改动本口径需重新决策。**
COVERAGE_EXCLUDED_SUBDIRS = ("jibot", "jiateng")


def _public_method_names(paths):
    """收集文件里类体的公有方法名（不统计方法内嵌套函数、不统计私有名）。"""
    names = set()
    for path in paths:
        tree = ast.parse(path.read_text(encoding="utf-8"))
        for node in ast.walk(tree):
            if not isinstance(node, ast.ClassDef):
                continue
            for member in node.body:
                if isinstance(member, (ast.FunctionDef, ast.AsyncFunctionDef)) and not member.name.startswith("_"):
                    names.add(member.name)
    return names


def _coverage_denominator_methods():
    """分母集合：``mixins/`` 平铺扫描（``glob``，不递归）。"""
    return _public_method_names(sorted(MIXINS_DIR.glob("*.py")))


def _coverage_excluded_methods():
    """排除项集合：``mixins/<子目录>/`` 递归收集（目录被删或改名时结果是空集）。"""
    paths = []
    for subdir in COVERAGE_EXCLUDED_SUBDIRS:
        paths.extend(sorted((MIXINS_DIR / subdir).glob("**/*.py")))
    return _public_method_names(paths)


@pytest.mark.unit
def test_contract_coverage_denominator():
    """锁定契约覆盖率分母的**范围规则**，不锁数字。

    要防的漂移：有人把 ``mixins/`` 扫描从「平铺」改成「递归」，分母无声地涨一截，
    DOCS-23 记录的覆盖率跟着变，却没人知道为什么。

    分母随代码浮动是应该的，所以这里**不硬编码 107**；锁死的是「哪些文件算进分母、
    哪些不算」这条规则。排除目录被删除或改名 = 口径失效，同样报错。
    """
    denominator = _coverage_denominator_methods()
    excluded = _coverage_excluded_methods()

    missing_dirs = [name for name in COVERAGE_EXCLUDED_SUBDIRS if not (MIXINS_DIR / name).is_dir()]
    assert not missing_dirs, (
        f"排除目录缺失：{missing_dirs}（期望 {MIXINS_DIR.relative_to(REPO_ROOT).as_posix()}/ 下存在），"
        "说明本节口径已失效，需按 DOCS-24 / DOCS-34 重新决策"
    )
    empty_dirs = [
        name
        for name in COVERAGE_EXCLUDED_SUBDIRS
        if not _public_method_names(sorted((MIXINS_DIR / name).glob("**/*.py")))
    ]
    assert not empty_dirs, (
        f"排除目录里收集不到任何公有方法：{empty_dirs}，期望：各至少 1 个，实际：0；"
        "说明本节口径已失效，需按 DOCS-24 / DOCS-34 重新决策"
    )

    overlap = sorted(denominator & excluded)
    assert not overlap, (
        f"分母集合与排除项集合有交集（分母 {len(denominator)} 个方法 / 排除项 {len(excluded)} 个方法），"
        f"期望：交集为空，实际交集 {len(overlap)} 个：\n"
        + "\n".join(f"  {name}" for name in overlap)
        + "\n  分母由 mixins/ 平铺扫描得出；交集非空说明扫描范围被改成了递归。"
        "\n  分母的排除范围由 DOCS-24（D1 边界）+ DOCS-34（Q5 = b）决定，改动需重新决策。"
    )
