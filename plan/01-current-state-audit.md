# 01. 现状审计

## 1. 入口脚本现状

### 1.1 `install.sh`

当前行为：

1. 用 `tools/detect_python.sh` 在系统 PATH 中寻找 `python3/python/python3.9...`
2. 直接用系统 Python 调 `tools/python_version_checker.py`
3. 用系统 Python 调 `tools/install_util.py` 解析 features
4. 用系统 Python 调 `tools/sifli_sdk_tools.py install-python-env`
5. 再用刚创建出来的 venv Python 调 `tools/sifli_sdk_tools.py install`

问题：

- 系统里没有 Python 时 install 直接失败。
- venv 的 Python 版本由“当前找到的系统 Python minor version”决定，天然漂移。
- Python 包版本没有在仓库内锁死。

### 1.2 `install.ps1`

与 `install.sh` 本质相同，只是 PowerShell 包装不同。

问题完全相同：

- 先依赖系统 `python`
- 再依赖运行时 constraints
- 再依赖 `recommended` 工具版本

### 1.3 `export.sh`

当前行为：

1. 推断 `sdk_path`
2. 再次依赖 `tools/detect_python.sh`
3. 用系统 Python 调 `tools/activate.py --export`
4. `activate.py` 再跳到 venv Python 执行 `tools/export_utils/activate_venv.py`
5. 最终由 `tools/sifli_sdk_tools.py export --format key-value` 拼出环境变量

问题：

- export 仍然依赖系统 Python。
- 调用链过长，`detect_python.sh -> activate.py -> activate_venv.py -> sifli_sdk_tools.py export`
- 环境不一致时只做局部检查。

### 1.4 `export.ps1`

当前行为：

1. 直接调用 `python tools/activate.py --export`
2. 输出临时 ps1 文件路径并 dot-source

问题：

- 仍然硬依赖系统 `python`
- 仍然走旧导出链路

## 2. Python 环境现状

### 2.1 版本锁定方式

当前只要求：

- 最低 Python 版本 `3.9+`
- 由 `tools/python_version_checker.py` 和 `tools/detect_python.sh` 检查

这意味着：

- 同一个 SDK checkout 在不同机器上可能使用 `3.9`、`3.10`、`3.11`、`3.12`
- venv 目录名也会跟随 minor version 变化

### 2.2 依赖锁定方式

当前依赖来自两层：

- repo 内 `tools/requirements/*.txt`
- 运行时下载的 `sdk.constraints.v<version>.txt`

缺陷：

- repo 内 requirements 不带版本
- constraints 不在 repo 内，无法 code review
- install 结果依赖远端文件是否变化、缓存是否命中

## 3. 工具链现状

### 3.1 `tools/tools.json` 的职责

当前 `tools.json` 负责：

- 工具名
- download URL / sha256 / size
- export path
- supported_targets
- install 类型（always/on_request/never）
- 版本列表与 `recommended` 标记

当前不负责：

- 当前 SDK checkout 必须绑定哪个版本
- 不同 profile 应该绑定哪个版本

### 3.2 当前 no-arg install 行为

基于现有 `tools.json` 与平台 override：

- `sftool`：默认安装
- `arm-none-eabi-gcc`：默认安装
- `sdk-exe`：仅 Windows 默认安装
- `cmake`：仅 Windows 默认安装；Linux/macOS 为可选

这是一条重要的兼容基线，后续无参数 install 需要保留这套默认安装范围。

## 4. 状态文件现状

当前状态文件：`${SIFLI_SDK_TOOLS_PATH}/sifli-sdk-env.json`

当前只记录：

- SDK version
- SDK path
- features
- targets

当前不记录：

- `git HEAD`
- profile
- Python lock 摘要
- profile lock 摘要
- 实际安装的 tool 版本
- Conan 配置版本
- 用户对 export 自修复的偏好

## 5. Conan 现状

当前 `action_install` 在工具安装后：

1. 下载 `sdk.conan-config.v<version>.zip`
2. 用 venv 里的 `conan` 执行 `config install`
3. 添加固定 remote

问题：

- 配置版本只与 `version.txt` 绑定，不与 `git HEAD` 或 profile 绑定
- `CONAN_HOME` 只设置到全局 `${tools_path}/conan`
- 多 profile 场景会互相污染

## 6. 导出链路现状问题

### 6.1 实际检查过少

当前 export 主要检查：

- venv 是否存在
- `VENV_VER_FILE` 中的 SDK 版本号是否与当前 `version.txt` 一致
- 工具是否在 PATH 或本地工具目录可找到

当前不检查：

- 当前 checkout `HEAD` 是否变化
- tools.json / profile lock 是否变化
- Python lock 是否变化
- Conan 配置是否变化
- 当前 PATH 是否已被旧 SDK/profile 污染

### 6.2 当前“旧环境切换”机制不稳定

当前通过 `ENVState` 记录导出的变量列表并尝试在下一次 export 前清掉。

问题：

- 逻辑散落在 `sifli_sdk_tools.py` 和 `export_utils`
- 只知道“之前导出了哪些变量”，不知道“当前 profile 应该导出什么”
- 对多 profile 不够稳定

## 7. 已知死路径

根脚本中仍保留：

- `tools/legacy_exports/export_legacy.sh`
- `tools/legacy_exports/export_legacy.ps1`

但仓库里实际不存在 `tools/legacy_exports/` 目录。

结论：

- 这不是“备用路径”，而是已经失效的死引用
- 后续实现不能再把它视为兼容目标

## 8. 现状对齐矩阵

| 项目 | 当前行为 | 后续处理 |
| --- | --- | --- |
| 无参数 install/export 命令形式 | 保持不变 | 保留 |
| 系统 Python 引导 | 必需 | 删除 |
| 运行时 constraints 下载 | 必需 | 删除 |
| `tools.json` 的下载目录职责 | 已存在 | 保留 |
| `tools.json` 作为绑定真相源 | 不支持 | 删除该用法 |
| `version.txt` 级别环境匹配 | 已存在 | 升级为 `git HEAD + env compat hash` |
| `sifli-sdk-env.json` | 已存在但信息不足 | 扩展 |
| Conan 初始化 | 已存在 | 重构并前置环境 |
| `legacy export` | 已失效 | 删除 |

## 9. 旧 Python 脚本删除审计

基于当前仓库状态，下面这些脚本虽然在新设计里会变成多余，但**现在还不能直接删除**，因为现有主链路仍然直接引用它们：

- `tools/detect_python.sh`
- `tools/activate.py`
- `tools/export_utils/activate_venv.py`
- `tools/export_utils/console_output.py`
- `tools/export_utils/shell_types.py`
- `tools/export_utils/utils.py`
- `tools/install_util.py`
- `tools/check_python_dependencies.py`
- `tools/python_version_checker.py`
- `tools/sifli_sdk_tools.py`

结论：

- 当前阶段可安全立即删除的旧 Python 脚本数量：`0`
- 只有在新 install/export 主链路完全切换之后，才允许删除旧跳板脚本

### 9.1 切换后可删除

当新 install/export 主链路完全落地后，以下脚本应直接删除，而不是长期保留：

- `tools/detect_python.sh`
- `tools/activate.py`
- `tools/export_utils/activate_venv.py`
- `tools/export_utils/console_output.py`
- `tools/export_utils/shell_types.py`
- `tools/export_utils/utils.py`

原因：

- 它们存在的主要目的就是支撑“系统 Python -> activate.py -> export_utils -> sifli_sdk_tools.py export”这条旧链路
- 新方案中这一整条链路都不再存在

### 9.2 切换后再判断是否删除

以下脚本不能先删，需要等新实现落地后再判断是否保留：

- `tools/install_util.py`
- `tools/check_python_dependencies.py`
- `tools/python_version_checker.py`

判断原则：

- 如果新主链路已经完全不再调用它们，就删除
- 如果还有小范围兼容/诊断用途，可以短期保留，但必须降级为非主链路

### 9.3 必须保留并重构职责

- `tools/sifli_sdk_tools.py`

原因：

- 其中仍然包含 tool metadata、下载、校验、安装等可复用能力
- 但它不应继续承担 Python env 和 export 主控职责
