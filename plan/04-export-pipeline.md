# 04. Export Pipeline 设计

## 1. 总体目标

export 的职责不再是“盲目把 PATH 拼起来”，而是：

1. 读取 profile lock
2. 判断本地安装态是否与 lock 一致
3. 必要时触发交互或自动修复
4. 生成 bash / PowerShell 可消费的环境导出脚本

## 2. 入口脚本设计

### 2.1 `export.sh`

外部行为保持：

```bash
. ./export.sh
. ./export.sh --profile default
```

内部行为改为：

1. 定位 SDK 根目录
2. 解析 `profile`
3. 读取 `${SIFLI_SDK_TOOLS_PATH}/python_env/<profile>/current-env.txt`
4. 用该 venv 的 Python 调用新环境管理器
5. 新环境管理器输出一个临时 shell 脚本路径
6. `export.sh` source 这个临时脚本

### 2.2 `export.ps1`

外部行为保持：

```powershell
.\export.ps1
.\export.ps1 --profile default
```

内部行为改为：

1. 定位 SDK 根目录
2. 解析 `profile`
3. 读取 `${SIFLI_SDK_TOOLS_PATH}/python_env/<profile>/current-env.txt`
4. 用该 venv 的 Python 调用新环境管理器
5. 新环境管理器输出一个临时 ps1 文件路径
6. `export.ps1` dot-source 这个文件

## 3. 失配检测器

## 3.1 必须比较的对象

export 每次都必须比较：

1. `profile lock`
2. 当前计算出的环境兼容指纹
3. 本地安装态 state
4. 当前 Python env 是否存在

### 3.2 失配条件

出现以下任一条件就视为失配：

- state 不存在
- state 的 `sdk.env_compat_algorithm` 不是当前实现支持的版本
- state 的 `sdk.env_compat_sha256` 与当前计算结果不一致
- state 的 `python.version` 与 lock 的 `python.version` 不一致
- state 的 `conan.config_id` 与 lock 的 `conan.config_id` 不一致
- state 的 required tools 与 lock 不一致
- state 的 targets 与 profile 默认值不一致
- Python env 目录或解释器不存在

根脚本层面的附加约束：

- 如果 `current-env.txt` 缺失
- 或 `current-env.txt` 指向的 Python 不存在

则 `export.sh` / `export.ps1` 直接失败，提示先执行 install。

这类错误不再进入 export 自修复流程。

以下情况不单独视为失配：

- `git HEAD` 变化，但 `env_compat_sha256` 未变化

处理方式：

- 允许直接继续 export
- 最多输出轻量提示，说明源码更新了但环境仍兼容

### 3.3 dirty worktree

如果当前 worktree dirty：

- 打 warning
- 只要主要环境文件导致的兼容指纹没有变化，就不计入失配原因

这是因为：

- dirty 只说明“源码有未提交改动”，不等于“环境定义已变”

建议：

- 将当前 dirty 状态仅记录到本地 state 的 `sdk.dirty`
- 不写入 repo-side `lock.json`

## 4. 用户交互与持久化

### 4.1 三种选择

export 检测到失配时，用户可选：

1. `once`
2. `always`
3. `never`

### 4.2 语义

- `once`
  - 本次自动修复
  - 状态文件仍保存为 `ask`
- `always`
  - 本次自动修复
  - 后续同一 `repo_root#profile` 下失配时静默自动修复
- `never`
  - 本次失败
  - 后续同一 `repo_root#profile` 下失配时静默失败

### 4.3 非交互场景

如果 export 运行在非 TTY 场景且当前偏好是 `ask`：

- 直接按 `never` 处理
- 不做隐式修复

原因：

- 非交互场景下擅自重建环境风险更大

## 5. export 自修复流程

当 export 决定自动修复时，固定执行：

1. 用 profile 默认 targets 重跑 install
2. 刷新 state 与 `current-env.txt`
3. 如果 Python env 路径变化，则重启到新的 profile venv Python
4. 重新进入 export 判定
5. 生成环境脚本

重要约束：

- export 不能自己“半修半导出”
- 只允许通过完整 install 流程恢复

## 6. PATH 组装器

### 6.1 管理路径列表

每次 export 都生成一组“当前 profile 管理的路径”：

1. profile Python env 的 `bin` / `Scripts`
2. `path_order` 中声明的 tool export path
3. SDK `tools/` 目录

### 6.2 去重规则

环境变量中新增：

- `SIFLI_SDK_MANAGED_PATHS`

export 时：

1. 读取旧的 `SIFLI_SDK_MANAGED_PATHS`
2. 从当前 PATH 中移除这些旧路径
3. 再把新路径按 `path_order` 头插

这样能稳定处理：

- 同一 SDK 重复 export
- profile 切换
- 不同 checkout 切换

### 6.3 可选工具的导出规则

- required tool：必须安装并进入 PATH
- optional tool：
  - 如果当前 profile 状态里已安装且版本匹配，则进入 PATH
  - 如果没装，不报错，不进入 PATH

这保持与当前 no-arg 行为一致，特别是：

- Linux/macOS 下默认不强制装 `cmake`

## 7. 环境变量集

首版 export 输出的变量集合：

- `SIFLI_SDK_PATH`
- `SIFLI_SDK_VERSION`
- `SIFLI_SDK_GIT_HEAD`
- `SIFLI_SDK_PROFILE`
- `SIFLI_SDK_PYTHON_ENV_PATH`
- `SIFLI_SDK_MANAGED_PATHS`
- `SIFLI_SDK_TOOLS_INSTALL_CMD`
- `SIFLI_SDK_TOOLS_EXPORT_CMD`
- `SIFLI_SDK`
- `CONAN_HOME`
- `ENV_ROOT`
- `PKGS_ROOT`
- `PKGS_DIR`
- `RTT_CC`
- `PYTHONPATH`
- `PATH`
- `RTT_EXEC_PATH`

## 8. shell-specific 输出

### 8.1 bash

输出一个临时 `.sh` 文件，内容包括：

- `export VAR=...`
- 完整 PATH
- 成功提示

### 8.2 PowerShell

输出一个临时 `.ps1` 文件，内容包括：

- `$Env:VAR="..."`
- `sdk.py` 的包装函数
- 成功提示

## 9. 旧导出链路如何处理

当前链路：

- `export.sh/ps1`
- `activate.py`
- `export_utils/activate_venv.py`
- `sifli_sdk_tools.py export`

新设计建议：

- 根脚本直接调用新管理器
- `activate.py` / `export_utils` 退出主链路

处理原则：

- 可以保留文件一段时间，作为迁移过渡
- 但主入口不再依赖它们

## 10. 失败模式

### 10.1 lock 与当前 checkout 不一致

- 如果偏好允许自动修复：触发 install
- 否则失败

### 10.2 Python env 丢失

- 同上

### 10.3 required tool 丢失

- 同上

### 10.4 optional tool 丢失

- 不失败
- 不导出该工具路径

### 10.5 当前 shell 已被旧 SDK/profile 污染

- 通过 `SIFLI_SDK_MANAGED_PATHS` 先清理旧路径再导出新路径
