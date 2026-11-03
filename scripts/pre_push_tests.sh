#!/usr/bin/env bash
# LeTools 推送前门禁：跑本地能跑绿的那部分用例。
#
# 为什么只跑一部分：`pytest.ini` 的 `testpaths` 里有 4 个 sdk_manager 测试文件
# 在模块顶层 `import kuavo_humanoid_sdk`，该包由 submodule 提供。未装 SDK 时
# 裸跑 `pytest` 会在 **collection 阶段直接 Interrupted**，所以这里只显式拾起
# 不依赖 SDK 的三个根。装了 SDK 后想全跑，用 `pytest` 而非本脚本。
#
# 与 CI 的区别：CI 的 `test:unit` job 跑裸 `pytest` 并 deselect 其中 19 条
# （那些在本机是能跑的）。本脚本是**推送前的快速自检**，不替代 CI。
#
# **前提：必须先 source ROS**，否则 `adapters/` 那三个根会全部 collection error
# （`ModuleNotFoundError: rospy` / `tf` / `message_filters`），门禁必红，
# 且报错看起来像代码问题，实际只是环境没进：
#
#     source /opt/ros/noetic/setup.bash   # noetic 是本机实际发行版
#     bash scripts/pre_push_tests.sh
set -euo pipefail

REPO_ROOT="$(git rev-parse --show-toplevel)"
cd "$REPO_ROOT"

if ! command -v python3 >/dev/null 2>&1; then
  echo "pre-push: 找不到 python3，跳过单测门禁" >&2
  exit 0
fi

# 仓库没有 pytest.ini 时静默放行（本脚本按 LeTools 布局写的，不适用于别的仓库）
[ -f pytest.ini ] || exit 0

echo "pre-push: 跑本地可跑的用例（tests / core / adapters 全绿子集）..."
if python3 -m pytest tests core/common/tests adapters/hardware/leju_wheeled/tests -q; then
  exit 0
fi

echo "" >&2
echo "pre-push: 单测不过，推送已中止（--no-verify 可跳过，CI 仍会拦）。" >&2
exit 1
