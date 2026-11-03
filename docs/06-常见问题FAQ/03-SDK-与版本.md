# 03 SDK 与版本

[返回文档首页](../README.md) · [02-环境安装](../02-环境安装/) · [03-接口详细文档](../03-接口详细文档/)

> 本篇收录 LeTools SDK 安装、版本锁定、子模块与开源版本对齐的典型问题。
> SDK 统一用 `scripts/install_sdk.sh` 安装（版本由 `scripts/kuavo_humanoid_sdk_tools/sdk_version.env` 锁定），用 `scripts/remove_local_sdk.sh` 卸载。

---

## 1. 问题：确认锁定版本

**原因**：master 分支会继续前进，若直接用它的 HEAD 安装，会与文档描述的版本不一致，无法复现问题。

**解决方案**：版本以锁定文件为准：

```bash
cd ~/letools_opensource
cat scripts/kuavo_humanoid_sdk_tools/sdk_version.env
```

版本以该文件为准，例如 `SDK_REPO_BRANCH="master"`、`SDK_REPO_TAG="1.4.5"`。Tag 是正式复现基线；即使 master 后续前进，也不能用新的 master HEAD 冒充 1.4.5。

**成功标准**：`sdk_version.env` 内容完整且与你要复现的目标版本一致。

---

## 2. 问题：子模块下载超时

**原因**：网络或代理问题；GitCode 仓库不可达。

**现象**：`fatal: unable to access ... timed out`。

**判断命令**

```bash
git ls-remote https://gitcode.com/OpenLET/kuavo-ros-opensource.git
git submodule status drivers/leju/kuavo_humanoid_sdk
```

若第一条就超时，这是网络或代理问题。网络恢复后执行：

```bash
cd ~/letools_opensource
git submodule sync --recursive
git submodule update --init --recursive drivers/leju/kuavo_humanoid_sdk
chmod +x scripts/install_sdk.sh
./scripts/install_sdk.sh
```

**成功标准**：子模块目录非空，安装脚本退出码为 0。

---

## 3. 问题：SDK 安装后仍不能 import

**原因**：Python 解释器与 pip 环境不一致（多为 Conda 抢占），或 ROS 环境未加载。

**判断命令**

```bash
which python3
python3 -m pip show kuavo-humanoid-sdk
python3 -c 'import kuavo_humanoid_sdk; print(kuavo_humanoid_sdk.__file__)'
python3 -c 'import kuavo_msgs, ocs2_msgs; print("ROS messages ready")'
```

**解决命令**

```bash
conda deactivate
source /opt/ros/noetic/setup.bash
source ~/letools_opensource/infrastructure/ros_packages/devel/setup.bash
cd ~/letools_opensource
./scripts/install_sdk.sh
python3 -c 'from kuavo_humanoid_sdk import KuavoRobot; print("SDK Ready")'
```

**成功标准**：输出 `SDK Ready`，模块和 pip 来自同一 Python 环境。

**日志定位**：保存安装脚本完整输出，同时提供 `which python3` 和 `python3 -m pip --version`。

> SDK 安装阶段的依赖安装（empy、消息包编译等）见 [01 安装与环境](01-安装与环境.md)。

---

## 4. 问题：`.gitmodules` 中 SDK 子模块的 branch 是 `dev`，与 `sdk_version.env` 的 `master` 不一致

**原因**：两者用途不同，不需要一致。`sdk_version.env` 的 `SDK_REPO_BRANCH` + `SDK_REPO_TAG` 是安装版本的唯一依据，`install_sdk.sh` 按其中的 tag 安装；`.gitmodules` 中的 `branch` 只在执行 `git submodule update --remote` 时决定更新目标，不影响 `install_sdk.sh` 安装的版本。

**判断命令**

```bash
cat scripts/kuavo_humanoid_sdk_tools/sdk_version.env | grep '^SDK_REPO'
git -C drivers/leju/kuavo_humanoid_sdk describe --tags --exact-match   # 子模块当前所在 tag
pip3 show kuavo-humanoid-sdk | grep -i '^version'                      # 已安装的 SDK 版本
```

**解决方案**：以 `sdk_version.env` 为准，用 `./scripts/install_sdk.sh` 安装。**不要**执行 `git submodule update --remote`：它会把子模块移到 `.gitmodules` 所写分支的最新提交，而不是锁定的 tag。如果已经执行过，重新运行 `./scripts/install_sdk.sh` 即可回到锁定版本。

**成功标准**：后两条命令的输出与 `sdk_version.env` 中的 `SDK_REPO_TAG` 一致（当前为 `1.4.5`）。

---

## 5. 问题：V63 或特殊底盘版本对齐后仍失败

**原因**：特殊机型可能需要专用分支，但分支名与硬件适用范围必须由维护者确认，文档不会预先声明。

**解决方案**：特殊机型可能需要专用分支，但分支名和硬件范围必须由维护者确认。反馈时提供机器人型号、硬件版本、下位机 commit、LeTools commit、SDK tag 和缺失 Service。不要把未确认的 beta 分支直接用于真机。

**成功标准**：维护者明确给出适配分支，且在同一环境复现通过。

---

## 相关文档

- 需要对齐的接口与版本兼容约定见 [03-接口详细文档](../03-接口详细文档/)
- 反馈版本问题的信息模板见 [07 开闭源与文档](07-开闭源与文档.md)
- 环境变量 `ROBOT_VERSION` 与编译的配合见 [01 安装与环境](01-安装与环境.md)
