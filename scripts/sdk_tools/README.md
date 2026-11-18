# sdk_tools — Kuavo Humanoid SDK 安装与版本管理

SDK 的安装、卸载、本地安装和版本管理都在这个目录里。它取代了原来的 `install_sdk.sh`、
`remove_local_sdk.sh`、`install_local_sdk.sh` 和 `kuavo_humanoid_sdk_tools/`，这些旧脚本已删除。

## 常用命令

```bash
scripts/sdk_tools/install.sh              # 在线安装配套版本（首次安装、升级、回退后重装）
scripts/sdk_tools/install_local.sh        # 安装本地已有的 SDK 源码，不访问远程仓库
scripts/sdk_tools/remove.sh               # 卸载；--source 连源码一起删，--purge 彻底清除
scripts/sdk_tools/version.sh              # 查看配套版本 / 源码版本 / pip 安装版本是否一致
scripts/sdk_tools/version.sh list         # 列出远程仓库可用的 tag
scripts/sdk_tools/version.sh use 1.5.0    # 切换到其它版本（记为本地覆盖）
scripts/sdk_tools/version.sh reset        # 回到配套版本
```

每个脚本都支持 `--help`。前置条件：先编译 ROS 消息包
（`catkin build kuavo_msgs ocs2_msgs`）。

## 版本从哪来

| 来源 | 文件 | 说明 |
|---|---|---|
| 配套版本 | `scripts/sdk_tools/sdk_version.env` | 随 LeTools 发版的权威源，入库。维护者用 `version.sh pin` 修改 |
| 本地覆盖 | `scripts/sdk_tools/sdk_config.sh` | 不入库，由 `version.sh use / reset` 维护 |
| 单次指定 | `install.sh --tag <tag>` | 只影响本次安装 |

优先级：单次指定 > 本地覆盖 > 配套版本。没有 `sdk_config.sh`、或其中两项版本字段为空时，装的就是配套版本。

`sdk_config.sh` 不是必需的，所有配置项都有默认值，可用项见 `sdk_config.sh.example`。脚本不会自动生成它，
也不会改动 `sdk_config.sh.example`；`version.sh use / reset` 只改写其中 `SDK_REPO_BRANCH`、`SDK_REPO_TAG` 两行。

## 实现要点

- **只下载 SDK 目录**：上游仓库整仓约 3.5GB，SDK 目录约 4MB。首次安装的 git 部分只需几秒，
  仓库数据约 5MB。已有的完整克隆可以直接沿用，升级时同样只下载 SDK 目录的变化。
- **不运行、不修改上游的 `install.sh`**：消息包拷贝和 `pip install -e` 由 `common.sh` 直接完成。
  不再原地 sed 上游文件，不改全局 pip 源，不 `sudo chmod`。
- **重复运行很快**：源码和 pip 都已是目标版本时只刷新消息包，不重装（`--force` 可强制）。
- **版本号干净**：按 tag 安装得到 `1.4.5`，不会出现 `1.4.5a0`。
- **缺消息包时在联网前报错**，而不是下载完才失败。
- **源码目录处于 detached HEAD**，停在目标 tag 上，不创建本地分支。

## 关于 SDK 源码目录里的 git

`drivers/leju/kuavo_humanoid_sdk` 是一个只含目标 tag 单个提交的浅克隆，并且只有 SDK 目录的文件内容。
在这个目录里 `git status`、`git diff`、`git describe` 正常可用；`git show`、`git log -p` 这类要读取
提交全部文件内容的命令会报 `unable to read` / `missing blob`，这是预期行为。

工具在安装结束后会关闭该仓库的按需下载（promisor）。如果不关，上面这类命令或 IDE 的提交视图会
把整仓约 1.3GB 的文件静默下载下来。切换版本请始终用 `version.sh use` 或 `install.sh`，不要在该
目录里手动 `git fetch` / `git checkout`。
