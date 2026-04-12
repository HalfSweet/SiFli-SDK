# 00. 目标、边界与决策

## 1. 本次要解决的问题

当前 SDK 安装/导出体系存在三个根问题：

1. Python 解释器与 Python 包没有在仓库内锁定，安装结果会随外部环境漂移。
2. `tools/tools.json` 只描述“可安装版本”，不描述“当前 SDK checkout 必须绑定哪个版本”。
3. `export` 不判断当前 checkout、当前工具链、当前 Conan 配置是否与安装时一致，因此很容易出现“能导出，但环境其实已经漂移”的情况。

## 2. 本次重构的硬目标

### 2.1 Python 相关

- 使用 `uv` 作为唯一受支持的 Python 引导方式。
- 不再要求用户预先准备可用 `python`。
- Python 版本固定到单一小版本。
- Python 依赖在 repo 内锁定，并能离线重放到同一结果。

### 2.2 SDK 与工具链绑定

- SDK 源码来源标识从 `version.txt` 升级到 `git rev-parse HEAD`。
- 环境兼容性不直接绑定 `git HEAD`，而是绑定对主要环境文件计算出的兼容指纹。
- 工具链版本由 repo 内 lock 文件固定，不再依赖 `tools.json` 中的 `recommended` 推导。
- export 时只导出 lock 声明的工具版本，不接受系统 PATH 抢占。

### 2.3 export 自检与自修复

- export 启动时必须检测环境是否匹配 lock。
- 首次失配时允许用户选择：
  - 本次自动修复
  - 后续自动修复
  - 后续不自动修复
- 选择结果按 `repo_root + profile` 持久化。

### 2.4 多 profile

- 支持多个 profile。
- 每个 profile 都必须锁死 Python、tools、Conan、默认 targets。
- 无参数时始终走 `default` profile。

## 3. 明确不做的事情

- 不支持没有 `uv` 的安装/导出路径。
- 不支持 fish/cmd 本次首发实现。
- 不继续兼容“系统 PATH 上同名工具优先”的旧行为。
- 不继续依赖远程 constraints 文件作为 Python 依赖真相源。
- 不保留仓库里已失效的 `legacy export` 假回退链路。

## 4. 兼容性要求

### 4.1 必须兼容

- 无参数 install/export 的调用方式。
- bash 与 PowerShell 两条主链路。
- `SIFLI_SDK_TOOLS_PATH` 自定义工具目录。

### 4.2 可不兼容

- install/export 的内部参数。
- `tools/sifli_sdk_tools.py` 的旧命令语义。
- `activate.py` / `export_utils` 的职责划分。

## 5. 关键设计决策

### 5.1 真相源分层

- `tools/tools.json`：下载目录，只描述制品。
- `tools/locks/<profile>/lock.json`：该 profile 的绑定清单，描述当前 SDK/profile 应该使用的 Python、tool、Conan 配置。
- `tools/locks/<profile>/pyproject.toml`
- `tools/locks/<profile>/uv.lock`
- `~/.sifli/sifli-sdk-env.json` 或 `${SIFLI_SDK_TOOLS_PATH}/sifli-sdk-env.json`：安装态与用户偏好状态。

### 5.2 Python 版本

- 首版固定 `3.12.0`。
- 后续升级 Python 版本必须同步修改：
  - profile lock
  - `pyproject.toml`
  - `uv.lock`
  - 测试矩阵

### 5.3 checkout 漂移规则

- 记录并展示 `HEAD`，用于源码来源追溯。
- 是否需要重装由环境兼容指纹决定，而不是单独由 `HEAD` 决定。
- worktree dirty 仅告警，不阻断，不触发重装。

### 5.4 export 的自动修复边界

- export 只做“环境重建”，不做业务构建、不做项目级 codegen。
- export 自动修复的动作固定为：
  - 重新同步 Python env
  - 安装/修复锁定工具版本
  - 重新配置 Conan
  - 更新本地状态文件
