# -*- coding: utf-8 -*-
"""发布一致性守卫：依赖清单与 SDK 版本的单一事实来源。

只读事实源、不复制事实——包名、分支名、tag、commit 都从仓库内的权威文件或
git 元数据读出后再互相比对，本文件不写死这些值（不硬编码 "1.4.5" / "master" /
包名列表）。纯文件系统 + git 扫描：不 import 项目代码，不依赖 ROS，秒级完成。

无法判定的情况（子模块未初始化、浅克隆缺 ref）一律 pytest.skip + 原因，
不静默通过。
"""

from __future__ import annotations

import difflib
import re
import subprocess
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parents[1]

# CI 装 ci_scripts/ 那份（.gitlab-ci.yml 的 pip3 install -r），下游用户装根目录
# 那份（随开源产物同步）。两者必须逐字节一致，否则同一份代码在两处装出不同依赖。
REQUIREMENTS_FILES = ("requirements.txt", "ci_scripts/requirements.txt")

# SDK 版本锁定文件：安装版本的唯一权威源（.gitmodules 与 sdk_version.env 自身注释均指向它）
SDK_VERSION_ENV = Path("scripts/kuavo_humanoid_sdk_tools/sdk_version.env")

# SDK 子模块路径，与 scripts/install_sdk.sh 的 SUBMODULE_PATH 同值
SDK_SUBMODULE_PATH = "drivers/leju/kuavo_humanoid_sdk"

# 03 章 SDK 篇（stage 3 摘掉「(占位)」后缀后仍匹配）
SDK_DOC_DIR = Path("docs/03-接口详细文档")

_SEMVER_RE = re.compile(r"\b\d+\.\d+\.\d+\b")
_COMMIT_RE = re.compile(r"\b[0-9a-f]{40}\b")
_ENV_VALUE_RE = r'^{key}=(?:"([^"]*)"|(\S+))\s*$'


def _git(*args: str, cwd: Path = REPO_ROOT) -> subprocess.CompletedProcess:
    return subprocess.run(
        ["git", *args],
        cwd=str(cwd),
        capture_output=True,
        text=True,
        encoding="utf-8",
        errors="replace",
    )


def test_requirements_ssot():
    """两份 requirements.txt 必须逐字节一致。

    断言「同步」而非「内容正确」：唯一要做的事是防止未来有人只改一份，
    使 CI 与下游用户拿到不同依赖。
    """
    texts = {}
    for rel in REQUIREMENTS_FILES:
        path = REPO_ROOT / rel
        if not path.is_file():
            pytest.fail(
                f"依赖清单缺失：{rel}（期望存在）。两个安装入口都得有文件——"
                f"CI 装 {REQUIREMENTS_FILES[1]}，下游用户装 {REQUIREMENTS_FILES[0]}。"
            )
        texts[rel] = path.read_text(encoding="utf-8")

    left, right = REQUIREMENTS_FILES
    if texts[left] != texts[right]:
        diff = "\n".join(
            difflib.unified_diff(
                texts[left].splitlines(),
                texts[right].splitlines(),
                fromfile=left,
                tofile=right,
                lineterm="",
            )
        )
        pytest.fail(
            f"{left} 与 {right} 内容不一致（期望逐字节相同，实际有差异）。"
            f"请把改动同步到两份文件：\n{diff}"
        )


def _read_env_versions() -> dict:
    """解析 sdk_version.env 的 KEY="VALUE" 行。

    用正则解析而不是 source：不执行文件里的任何 shell 片段。
    """
    path = REPO_ROOT / SDK_VERSION_ENV
    assert path.is_file(), f"缺少 SDK 版本锁定文件 {SDK_VERSION_ENV}（期望存在，它是安装版本的唯一权威源）"
    text = path.read_text(encoding="utf-8")
    values = {}
    for key in ("SDK_REPO_BRANCH", "SDK_REPO_TAG"):
        match = re.search(_ENV_VALUE_RE.format(key=key), text, re.MULTILINE)
        values[key] = (match.group(1) or match.group(2)) if match else None
    return values


def _submodule_unusable_reason() -> str | None:
    """子模块不可用于 git 查询时返回原因，可用时返回 None。

    注意「目录存在但未初始化」：此时 `git -C <sub>` 会一路向上解析到父仓库，
    查询看似成功却查到父仓库的数据——必须显式识别，否则守卫会假通过。
    """
    sub = REPO_ROOT / SDK_SUBMODULE_PATH
    if not sub.is_dir():
        return f"子模块目录 {SDK_SUBMODULE_PATH} 不存在"
    top = _git("-C", str(sub), "rev-parse", "--show-toplevel")
    if top.returncode != 0:
        return f"{SDK_SUBMODULE_PATH} 未初始化：{top.stderr.strip()}"
    if Path(top.stdout.strip()).resolve() != sub.resolve():
        return f"{SDK_SUBMODULE_PATH} 未初始化（git 解析到父仓库 {top.stdout.strip()}）"
    return None


def _repo_git_unusable_reason() -> str | None:
    """本仓库自身不可用于 git 查询时返回原因，可用时返回 None。

    与 `_submodule_unusable_reason()` 同理：开源制成品（`git archive` 导出、
    没有 `.git`）里 `git rev-parse` 会因「不是 git 仓库」退出 128；若只看
    returncode，父目录恰是另一个仓库时还会查到那个仓库的数据而不自知。
    """
    top = _git("rev-parse", "--show-toplevel")
    if top.returncode != 0:
        return f"非 git 工作副本（如开源制成品，没有 .git）：{top.stderr.strip()}"
    if Path(top.stdout.strip()).resolve() != REPO_ROOT:
        return f"git 解析到的仓库根不是本目录（{top.stdout.strip()}）"
    return None


def _rev_parse(sub: Path, rev: str) -> str | None:
    result = _git("-C", str(sub), "rev-parse", "--verify", "--quiet", f"{rev}^{{commit}}")
    out = result.stdout.strip()
    return out if result.returncode == 0 and out else None


# 基线分支：远端有子模块保护钩子，重放脚本推分支时会把 .gitmodules 钉回这些分支上的
# 历史形态。判据允许从这里取「已知历史形态」，避免依赖重放分支自身的提交历史。
_BASELINE_REFS = (
    "refs/remotes/origin/dev",
    "refs/remotes/origin/master",
    "refs/heads/dev",
    "refs/heads/master",
)


def _gitmodules_blob_at(rev: str) -> str | None:
    """某个 rev 上 .gitmodules 的 blob 哈希；读不到（rev 不存在 / 无该路径）返回 None。"""
    result = _git("rev-parse", "--verify", "--quiet", f"{rev}:.gitmodules")
    out = result.stdout.strip()
    return out if result.returncode == 0 and out else None


def _gitmodules_pinned_back_blob() -> str | None:
    """当前 .gitmodules 若与仓库已知的某个历史形态逐字节相同，返回该 blob 哈希。

    判据：`git hash-object .gitmodules` 必须命中「本分支提交历史上 .gitmodules 的某个
    blob」或「基线分支（dev / master）上的 .gitmodules blob」。

    为什么这个判据是充分的：blob 哈希相同 ⇔ 内容逐字节相同。DOCS-25 把 branch 由 dev
    改为 master 并补了注释，得到的是一个**仓库历史里从未出现过的新 blob**；而重放把
    .gitmodules 整文件钉回基线形态，得到的是**历史里已存在的 blob**。所以「命中已知
    历史形态」能排除「被改成别的值」。

    安全性（放宽不变成放空）：改成历史里没出现过的第三个值 ⇒ blob 是新的 ⇒ 不命中 ⇒
    照旧走下面的相等断言失败。实测见任务交付说明。

    代价（已知且被本任务的裁定接受）：把 .gitmodules 回退到 dev 的历史形态本身也是
    一种「命中」。这正是本任务要放宽的那个场景，无法在不依赖远端钩子状态的前提下与
    重放固定区分开；真解由放行 .gitmodules 后的回滚承担。

    判据得不到（不是 git 工作副本、拿不到 git 对象、浅克隆窗口里没有该路径的历史）时
    返回 None，调用方照原断言失败，不静默放行。
    """
    current = _git("hash-object", "--", ".gitmodules")
    current_blob = current.stdout.strip()
    if current.returncode != 0 or not current_blob:
        return None

    revs = _git("rev-list", "HEAD", "--", ".gitmodules").stdout.split()
    known_blobs = set()
    for rev in [*revs, *_BASELINE_REFS]:
        blob = _gitmodules_blob_at(rev)
        if blob is not None:
            known_blobs.add(blob)
    return current_blob if known_blobs and current_blob in known_blobs else None


def _sdk_doc_path() -> Path:
    docs = sorted((REPO_ROOT / SDK_DOC_DIR).glob("06-*.md"))
    assert docs, f"{SDK_DOC_DIR}/06-*.md 不存在（SDK 篇缺失或改名）"
    return docs[0]


def test_sdk_version_doc_single_source():
    """SDK 版本的三条腿 + 文档必须同源。

    三条腿：
      1. sdk_version.env 的 (SDK_REPO_BRANCH, SDK_REPO_TAG)  —— 权威源
      2. .gitmodules 中该子模块的 branch 字段
      3. 超级项目 gitlink pin（git ls-tree HEAD）所在的分支与 tag

    这防的是真实出现过的矛盾：.gitmodules 写 dev，而 sdk_version.env 与 pin
    都指向 master/1.4.5——只校验「文档 = env」是防不住它的，那种断言在修复前
    就已经是绿的。
    """
    env = _read_env_versions()
    branch, tag = env["SDK_REPO_BRANCH"], env["SDK_REPO_TAG"]
    assert branch and tag, (
        f"{SDK_VERSION_ENV} 未同时声明 SDK_REPO_BRANCH / SDK_REPO_TAG"
        f"（实际 branch={branch!r}，tag={tag!r}）；两者共同指向同一 commit，必须同时存在。"
    )

    # —— 腿 2：.gitmodules 的 branch ——
    leg2_skip_reason = None
    gm = _git("config", "-f", ".gitmodules", "--get", f"submodule.{SDK_SUBMODULE_PATH}.branch")
    gm_branch = gm.stdout.strip()
    assert gm_branch, (
        f".gitmodules 缺 submodule.{SDK_SUBMODULE_PATH}.branch 字段："
        f"期望 {branch!r}（与 {SDK_VERSION_ENV} 的 SDK_REPO_BRANCH 一致），实际无该字段。"
    )
    if gm_branch != branch:
        # —— 临时放宽（DOCS-98）——
        # 远端对 .gitmodules 设了子模块保护钩子，改了推不上去；重放脚本为把本分支推上
        # 远端，把 .gitmodules 整文件钉回基线形态（dev），于是这里呈现出「env=master、
        # .gitmodules=dev」的假漂移。放宽只对「可识别为钉回」的形态生效，判据见
        # _gitmodules_pinned_back_blob()；判不出钉回时仍按下面的相等断言失败。
        # 恢复条件：.gitmodules 放行（钩子摘掉）后应删掉本放宽，恢复下面的相等断言。
        pinned_blob = _gitmodules_pinned_back_blob()
        if pinned_blob is not None:
            leg2_skip_reason = (
                f"腿 2 已临时放宽（DOCS-98）：.gitmodules 被流水线环境钉回基线形态，非真实漂移，"
                f"故跳过其相等断言。.gitmodules.branch 实际 {gm_branch!r}，"
                f"期望 {branch!r}（来自 {SDK_VERSION_ENV} 的 SDK_REPO_BRANCH）；"
                f"当前 .gitmodules 的 blob {pinned_blob} 与仓库已知历史形态逐字节相同，"
                f"即整文件被钉回而非被改成别的值。待 .gitmodules 放行后应回滚本放宽、恢复该断言。"
            )
    if leg2_skip_reason is None:
        assert gm_branch == branch, (
            f".gitmodules 的子模块跟踪分支与版本权威源不一致："
            f"期望 {branch!r}（来自 {SDK_VERSION_ENV} 的 SDK_REPO_BRANCH），"
            f"实际 {gm_branch!r}（来自 .gitmodules）。"
            f"两者不同时，`git submodule update --remote` 会把子模块拉到与 pin 不同的分支上。"
        )

    # —— 腿 3a：超级项目记录的 pin ——
    # pin 只能从未导出成品的 git 元数据里读；无 .git 时无法判定，按文件顶部
    # 声明的策略 skip，不当成失败（开源制成品里跑测试不应红）。
    repo_reason = _repo_git_unusable_reason()
    if repo_reason is not None:
        pytest.skip(
            f"{repo_reason}；无法读 `git ls-tree HEAD` 的 gitlink pin，"
            f"故未校验 pin、06 篇文档与 pin 的一致性。"
            f"需比对时在带 .git 的工作副本里运行本测试。"
        )
    tree = _git("ls-tree", "HEAD", "--", SDK_SUBMODULE_PATH)
    fields = tree.stdout.split()
    assert tree.returncode == 0 and len(fields) >= 3 and fields[0] == "160000", (
        f"HEAD 中取不到 {SDK_SUBMODULE_PATH} 的 gitlink："
        f"`git ls-tree HEAD -- {SDK_SUBMODULE_PATH}` 输出 {tree.stdout.strip()!r}"
    )
    pin = fields[2]

    # —— 文档侧：06 篇里写出来的值必须能在权威源里对上 ——
    doc = _sdk_doc_path()
    rel_doc = doc.relative_to(REPO_ROOT).as_posix()
    for lineno, line in enumerate(doc.read_text(encoding="utf-8").splitlines(), start=1):
        for token in _COMMIT_RE.findall(line):
            assert token == pin, (
                f"{rel_doc}:{lineno} 写的 commit 与 pin 不一致："
                f"期望 {pin}（来自 `git ls-tree HEAD -- {SDK_SUBMODULE_PATH}`），实际 {token}。"
            )
        # 只查「引用版本权威源的那一行」上的 N.N.N——避免误伤同篇里其他版本号
        # （如 Ubuntu 20.04.6 之类），那样的误报会挡住无关的文档改动。
        if "sdk_version.env" in line:
            for token in _SEMVER_RE.findall(line):
                assert token == tag, (
                    f"{rel_doc}:{lineno} 在引用 {SDK_VERSION_ENV} 的同一句里写了版本 {token}："
                    f"期望 {tag!r}（来自 {SDK_VERSION_ENV} 的 SDK_REPO_TAG），实际 {token!r}。"
                )

    # —— 腿 3b/3c：需要子模块仓库内的 refs，判不了就明确 skip ——
    reason = _submodule_unusable_reason()
    if reason is not None:
        pytest.skip(
            f"{reason}；已校验 .gitmodules.branch、gitlink pin 与 06 篇文档，"
            f"未校验子模块内的 tag↔pin 与 pin↔分支。"
            f"需比对时执行：git submodule update --init {SDK_SUBMODULE_PATH}"
        )

    sub = REPO_ROOT / SDK_SUBMODULE_PATH
    tag_commit = _rev_parse(sub, tag)
    if tag_commit is None:
        pytest.skip(
            f"子模块本地解析不到 tag {tag!r}（浅克隆或未取到该 ref）："
            f"{SDK_VERSION_ENV} 声明了 SDK_REPO_TAG={tag!r}，此处无法判定 tag↔pin；"
            f"其余三条腿已校验。"
        )
    assert tag_commit == pin, (
        f"子模块 tag 与 pin 不一致：期望二者同指一个 commit，"
        f"实际 tag {tag!r} → {tag_commit}（子模块内），pin → {pin}（{SDK_VERSION_ENV} 所在仓库的 HEAD）。"
    )

    if _git("-C", str(sub), "rev-parse", "--is-shallow-repository").stdout.strip() == "true":
        pytest.skip(
            f"子模块是浅克隆，祖先关系不可判定；已校验 tag↔pin 与其余三条腿，"
            f"未校验 pin 是否在分支 {branch!r} 上。"
        )

    branch_tip = _rev_parse(sub, f"refs/heads/{branch}") or _rev_parse(
        sub, f"refs/remotes/origin/{branch}"
    )
    if branch_tip is None:
        pytest.skip(
            f"子模块本地解析不到分支 {branch!r}（浅克隆或未取到该 ref）："
            f"无法判定 pin 是否在该分支上；其余各腿已校验。"
        )

    anc = _git("-C", str(sub), "merge-base", "--is-ancestor", pin, branch_tip)
    assert anc.returncode == 0, (
        f"pin 不在 {SDK_VERSION_ENV} 声明的分支上："
        f"期望 pin {pin} 是 {branch!r}（tip {branch_tip}）的祖先，"
        f"实际 `merge-base --is-ancestor` 退出码 {anc.returncode}。"
    )

    # 腿 2 的放宽放在最后：上面各腿都已实际执行并保留断言，skip 只摘掉腿 2 一项，
    # 不会连带跳过 pin / 文档 / 子模块 refs 的校验。
    if leg2_skip_reason is not None:
        pytest.skip(leg2_skip_reason)
