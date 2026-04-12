# 05. 兼容性、迁移与 breaking change

## 1. 保留项

以下行为必须保留：

### 1.1 外部入口

- `./install.sh`
- `.\install.ps1`
- `. ./export.sh`
- `.\export.ps1`

### 1.2 自定义工具路径

- 继续支持 `SIFLI_SDK_TOOLS_PATH`

### 1.3 target 概念

- 继续支持：
  - target 选择

其默认值改由 profile lock 决定。

## 2. breaking change 清单

### 2.1 删除系统 Python 前置依赖

旧行为：

- 用户必须先准备可用 Python

新行为：

- 只要求 `uv`
- Python 由 `uv` 负责准备

影响：

- 旧文档里所有“先装 Python 再 install”的描述都需要改写

### 2.2 删除运行时 constraints 下载

旧行为：

- constraints 在安装时从远端下载

新行为：

- 依赖锁在 repo 内

影响：

- `tools/requirements/*.txt` 不再是最终依赖真相源

### 2.3 删除 `recommended` 版本推导

旧行为：

- 实际工具版本默认取 `tools.json` 里的 `recommended`

新行为：

- 实际工具版本来自 profile lock

影响：

- 修改 `tools.json` 不再自动改变当前 SDK 应使用的版本

### 2.4 删除默认 PATH 抢占

旧行为：

- 系统 PATH 上的工具可能影响实际使用版本

新行为：

- export 只导出 lock 指定版本

影响：

- 依赖“系统 PATH 上某个 gcc/cmake 恰好可用”的用户流程会失效

### 2.5 删除 legacy export 死路径

旧行为：

- 根脚本保留 `SIFLI_SDK_LEGACY_EXPORT` 分支，但目标文件并不存在

新行为：

- 直接移除

影响：

- 没有真实用户损失，因为当前分支本来就不可用

### 2.6 删除 Python dependency features

旧行为：

- Python requirements 按 `core`、`pytest`、`docs`、`ci`、`gdbgui`、`test-specific` 分散管理

新行为：

- uv 工程只保留一套完整依赖集合
- 不再通过 feature/extras 选择 Python 依赖子集

影响：

- `--enable-*` / `--disable-*` 不再作为 Python 环境维度存在
- 这是有意的简化，目的是消除多套 Python 依赖图带来的维护和漂移成本

### 2.7 下载路径与安装路径解耦

旧行为：

- 下载缓存、解压路径、最终安装路径基本都围绕 `${SIFLI_SDK_TOOLS_PATH}` 内部隐式组织

新行为：

- install 明确区分：
  - `install_root`
  - `cache_root`
  - `staging_root`

影响：

- 文档与脚本参数需要增加路径与镜像配置说明
- 但这能更好支持受限环境、共享缓存和网络加速

### 2.8 区分 tool 镜像与 Python 包镜像

旧行为：

- 文档和脚本里没有明确区分 tool 归档下载源与 Python 包索引源

新行为：

- tool 归档继续走 artifact source 配置
- `uv sync` 单独使用 Python package index 配置

影响：

- 运维配置会多一个维度
- 但能避免“切换 tool 镜像时顺带错误影响 Python 包解析”

## 3. 新旧能力对齐矩阵

| 能力 | 旧实现 | 新实现 |
| --- | --- | --- |
| 无参 install | 基于系统 Python + requirements + constraints | 基于 `uv` + profile lock + profile 关联的 `uv.lock` |
| 无参 export | 基于系统 Python 跳板导出 | 基于 profile lock 与本地状态直接导出 |
| SDK 来源标识 | `version.txt` | `git HEAD` |
| 环境兼容判断 | 无 | 主要环境文件的 compat hash |
| Python 锁定 | 无 | profile 关联的 `uv.lock` |
| tools 版本锁定 | `recommended` | profile lock |
| Conan 绑定 | 只跟 `version.txt` 相关 | 跟 profile lock 相关 |
| 多 profile | 无 | 有 |
| export 自动修复 | 无 | 有 |

## 4. 迁移策略

### 4.0 profile 目录布局

新布局收敛为：

- `tools/locks/<profile>/lock.json`
- `tools/locks/<profile>/pyproject.toml`
- `tools/locks/<profile>/uv.lock`

这样每个 profile 的绑定清单与 Python 锁文件都在同一目录中，不再额外引入 `tools/python_env/` 作为第二个顶层来源。

### 4.1 首次切到新系统

建议策略：

1. 直接视作“未安装”
2. 不尝试把旧 venv/旧 state 自动转换成新状态
3. 让用户执行一次新的 install

原因：

- 旧状态信息不足以判断 lock 一致性
- 自动迁移容易把错误环境包装成“看起来合法”

### 4.2 旧工具目录

保留现有：

- `${tools_path}/tools/<tool>/<version>`

理由：

- 缓存可继续复用
- download/install 逻辑无需重写制品布局

### 4.4 旧下载缓存

如果历史版本已经在 `${tools_path}/dist` 下缓存归档：

- 可以作为新 `cache_root` 的迁移来源
- 但不建议继续把 `dist/` 当作唯一缓存目录约定

新文档应统一使用：

- `cache_root`
- `staging_root`
- `install_root`

### 4.3 旧 Conan 缓存

新系统启用 profile 隔离后：

- 原 `${tools_path}/conan` 视为旧全局缓存
- 新 profile 从 `${tools_path}/conan/<profile>` 开始

不建议自动迁移旧 Conan home。

### 4.4 兼容指纹迁移

新系统首次安装后，会写入：

- `sdk.env_compat_sha256`

旧状态文件没有这一字段时：

- 直接视为未安装
- 不尝试从旧字段推导 compat hash

## 5. 文档迁移清单

需要更新的内容包括：

- quickstart 安装文档
- quickstart export 文档
- “不能用 pyenv 管理系统 Python”这类旧表述
- 自定义工具路径章节
- Windows Terminal / alias 配置示例

新的文档叙述应改为：

- 需要 `uv`
- 无需预装 Python
- lock 文件由 repo 管理
- export 可能在失配时要求用户确认或自动重建

## 6. 模块替换关系

### 6.1 主链路保留

- 根脚本文件名保留
- `tools.json` 保留

### 6.2 主链路降级

建议从主链路移除：

- `tools/detect_python.sh`
- `tools/activate.py`
- `tools/export_utils/*`

这些模块可以：

- 暂时保留文件
- 但不再作为 install/export 主逻辑的一部分

删除策略：

- 主链路切换完成后，不建议继续“为了保险”长期保留
- 对这批纯跳板脚本，应在兼容窗口结束后直接删除

### 6.3 `tools/sifli_sdk_tools.py`

建议角色调整为：

- 继续提供 tool metadata、download、install 的通用能力
- 不再承担 Python env / export 主控职责

这能最大化复用现有制品下载逻辑，同时避免继续把复杂状态逻辑堆在一个大脚本里。

### 6.4 脚本删除顺序

建议删除顺序固定为：

1. 先完成新 install/export 主链路
2. 再删除纯跳板脚本：
   - `tools/detect_python.sh`
   - `tools/activate.py`
   - `tools/export_utils/*`
3. 再判断是否删除：
   - `tools/install_util.py`
   - `tools/check_python_dependencies.py`
   - `tools/python_version_checker.py`
4. 最后收敛 `tools/sifli_sdk_tools.py` 的职责，只保留底层工具安装能力

当前阶段说明：

- 在新主链路还未实现之前，以上脚本都不能直接删除
- 当前仓库中可安全立刻删除的旧 Python 脚本数量为 `0`
