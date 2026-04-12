# 03. Install Pipeline 设计

## 1. 总体目标

install 的职责是从 repo lock 出发，构建一个完全可复现的本地环境。

install 只做四件事：

1. 解析 profile / targets
2. 用 `uv` 准备锁定 Python 环境
3. 安装该 profile 需要的工具版本
4. 初始化该 profile 的 Conan 配置并写入状态文件

补充：

- 下载路径、缓存路径、临时解压路径属于“本机部署配置”，不是 profile lock 的一部分。

## 2. 入口脚本设计

### 2.1 `install.sh`

外部行为保持：

```bash
./install.sh
./install.sh sf32lb52
```

内部行为改为：

1. 定位 SDK 根目录
2. 检查 `uv` 是否存在
3. 不再调用 `detect_python.sh`
4. 直接调用新的环境管理器，例如：
   - `uv run --python 3.12.0 --no-project tools/<new-manager>.py install ...`

### 2.2 `install.ps1`

外部行为保持：

```powershell
.\install.ps1
.\install.ps1 sf32lb52
```

内部行为同样统一到 `uv` 引导的 Python 管理器。

## 3. install 参数策略

### 3.1 无参数

无参数必须等价于：

- `profile = default`
- `targets = profile.defaults.targets`

### 3.2 兼容参数

建议保留的兼容输入：

- `sf32xxx` 位置参数，解释为 target

### 3.3 新参数

新增：

- `--profile <name>`

允许后续扩展：

- `--targets <comma-separated>`
- `--cache-dir <path>`
- `--staging-dir <path>`
- `--mirror <url>`
- `--offline`
- `--from-bundle <dir>`

优先级规则建议：

1. CLI 参数
2. 环境变量
3. 本机 `config.json`
4. 默认值

建议环境变量：

- `SIFLI_SDK_TOOLS_PATH`
- `SIFLI_SDK_CACHE_PATH`
- `SIFLI_SDK_STAGING_PATH`
- `SIFLI_SDK_OFFLINE`
- `SIFLI_SDK_MIRROR`
- `SIFLI_SDK_MIRROR_PREFIX_MAP`
- `SIFLI_SDK_PYPI_DEFAULT_INDEX`
- `SIFLI_SDK_PYPI_INDEX`
- `SIFLI_SDK_PYPI_INDEX_STRATEGY`

补充约束：

- `install_root` 不再允许由 `config.json` 覆盖。
- install 根目录只由：
  - `SIFLI_SDK_TOOLS_PATH`
  - 或默认 `~/.sifli`
  决定。

## 4. Python 环境安装器

### 4.1 解释器准备

固定流程：

1. 读取 profile lock 的 `python.version`
2. 执行 `uv python install <version>`

首版固定：

- `3.12.0`

### 4.2 环境目录

固定落到：

- `${SIFLI_SDK_TOOLS_PATH}/python_env/<profile>/py3.12.0`

并在 install 成功后更新：

- `${SIFLI_SDK_TOOLS_PATH}/python_env/<profile>/current-env.txt`

该指针文件用于让根 `export.sh` / `export.ps1` 在不启动 Python bootstrap 的前提下定位当前 profile venv。

### 4.3 依赖同步

固定命令模型：

```bash
UV_PROJECT_ENVIRONMENT=<env_path> \
uv sync --project tools/locks/<profile> --locked --python 3.12.0
```

规则：

- `--project` 指向当前 profile 对应的 lock 目录
- 必须使用 `--locked`
- 不再存在 extras 选择分支
- 所有 Python 依赖始终一次性完整安装

### 4.4 `uv sync` 镜像与索引设计

`uv sync` 使用的 Python 包源，必须与 tool 归档下载源分开处理。

建议映射关系：

- 本机配置 `python_packages.default_index`
  - 转换为 `UV_DEFAULT_INDEX`
- 本机配置 `python_packages.indexes`
  - 转换为 `UV_INDEX`
- 本机配置 `python_packages.index_strategy`
  - 转换为 `UV_INDEX_STRATEGY`

典型场景：

1. 替换默认 PyPI 镜像
   - 例如使用公司内网 PyPI 代理作为 `default_index`
2. 增加额外 index
   - 例如私有包仓
3. 完全离线
   - `uv sync` 只指向本地 simple index 或本地镜像目录

设计要求：

- install 对 tool 归档和 Python 包索引分别建模
- 不允许把 `SIFLI_SDK_MIRROR` 同时拿去覆盖 tool 归档和 `uv sync` index
- Python 包镜像配置变化，不应导致 profile lock 变化

### 4.5 `uv sync` 认证

如果 `uv sync` 访问私有 index，建议支持：

- 命名 index + `UV_INDEX_<NAME>_USERNAME/PASSWORD`
- 或 `netrc`
- 或 keyring

首版文档不建议把认证信息写入 repo 中的 `pyproject.toml` 或 profile lock。

### 4.6 为什么不再保留 pip/constraints 流程

旧流程的问题不是“命令不好看”，而是“真相源不在 repo 内”。

因此新设计明确删除：

- `python -m venv`
- `python -m pip install ...`
- 运行时下载 `sdk.constraints.*`

## 5. tools 安装器

### 5.1 版本来源

必须来自：

- `tools/locks/<profile>/lock.json`

不能来自：

- `tools.json` 的 `recommended`

### 5.2 默认安装范围

默认安装范围仍由 `tools.json` 的 `install` 与 platform override 决定：

- `always`：无参数 install 必装
- `on_request`：无参数 install 不装
- `never`：无参数 install 不管

这样做的原因：

- 保留当前 no-arg install 的行为边界
- 只替换“版本选择逻辑”，不替换“默认安装范围逻辑”

### 5.3 版本决策算法

对每个当前平台、当前 targets 下可见的 tool：

1. 先看 `tools.json` 是否允许当前平台/target
2. 再看 profile lock 中是否定义了版本
3. 该版本必须在 `tools.json` 中真实存在
4. 最终安装该精确版本

### 5.4 本地版本复用

如果 `${tools_path}/tools/<tool>/<version>` 已存在且通过校验：

- 直接复用
- 不重新下载

如果存在但执行校验失败：

- 删除该版本目录
- 重新下载并安装

### 5.5 下载、缓存与安装路径解耦

install 流程中需要区分三类目录：

- `install_root`
  - 最终安装位置，例如 `${SIFLI_SDK_TOOLS_PATH}`
- `cache_root`
  - 下载缓存目录，例如 `${SIFLI_SDK_TOOLS_PATH}/cache`
- `staging_root`
  - 解压与校验的临时目录，例如 `${SIFLI_SDK_TOOLS_PATH}/staging`

原因：

- 受限环境中，最终安装路径和可写缓存路径可能不是同一个位置
- 网络加速通常依赖可共享缓存或预下载 bundle
- staging 可以保证安装是“先准备完成、再切换生效”

### 5.6 下载源解析顺序

建议 install 在获取任一归档时按以下顺序查找：

1. 当前 `cache_root`
2. 用户显式指定的 bundle 目录
3. 本机配置中的本地镜像目录
4. 本机配置中的网络镜像
5. 官方上游

规则：

- 每次命中归档后都必须校验 sha256/size
- 只有校验通过后才允许进入安装阶段

注意：

- 这里的 source 顺序只适用于 tool 归档和 Conan 配置包
- 不适用于 `uv sync` 的 Python 包索引解析

### 5.7 原子安装流程

对每个 tool/conan 配置包，建议流程固定为：

1. 下载或命中缓存到 `cache_root`
2. 在 `staging_root` 解压
3. 在 `staging_root` 内完成结构校验和可执行检查
4. 通过后原子移动到 `${install_root}/tools/<tool>/<version>`

这样可以避免：

- 半解压状态污染最终目录
- 网络中断后留下看似存在但实际损坏的安装目录

### 5.8 离线与 bundle

建议增加配套的下载模式：

- `download --profile <name> --bundle <dir>`

用途：

- 预拉取某个 profile 需要的全部 Python 包索引与 tool 归档
- 为受限环境生成可搬运的离线包目录

离线安装时建议支持：

- `install --from-bundle <dir> --offline`

行为：

- 只允许查本地 bundle 与本地 cache
- 不访问任何网络源

补充：

- 如果离线环境还需要执行 `uv sync`，则必须同时准备本地 Python 包索引镜像
- 也就是说，离线 bundle 至少要覆盖两类内容：
  - tool/conan 归档
  - Python 包索引或 wheel/simple 源

## 6. Conan 初始化器

### 6.1 初始化前提

Conan 相关动作必须在以下条件都满足之后执行：

- profile Python env 已存在
- `CONAN_HOME` 已按 profile 设置
- 当前 lock 已确定

### 6.2 `CONAN_HOME`

固定为：

- `${SIFLI_SDK_TOOLS_PATH}/conan/<profile>`

### 6.3 Conan config 版本

固定由 profile lock 的 `conan.config_id` 决定。

install 流程：

1. 根据 `config_id` 定位 zip 文件
2. 本地没有则下载
3. 执行 `conan config install`
4. 执行 `conan remote add ... --force`

### 6.4 为什么不能继续复用全局 Conan home

因为用户要求 profile 级版本锁定。如果仍共享一个全局 `CONAN_HOME`：

- profile A 的 config/remote 会污染 profile B
- export 无法判断当前 Conan 环境到底属于哪个 profile

## 7. 状态写入规则

install 成功后必须一次性写入：

- `sdk.version_txt`
- `sdk.git_head`
- `sdk.env_compat_algorithm`
- `sdk.env_compat_sha256`
- `locks.profile_lock_sha256`
- `locks.uv_lock_sha256`
- `python.version`
- `python.env_path`
- `targets`
- `tools`
- `conan.config_id`
- `conan.home`
- `cache_root`
- `install_root`

其中：

- `env_compat_algorithm` 首版固定为 `v1`
- `env_compat_sha256` 按 `02-data-model.md` 中定义的规范算法计算
- `git_head` 只记录来源，不作为唯一重装条件

如果用户是从 export 的交互修复进入 install：

- 还要一并更新 `preferences.auto_reconcile`

## 8. install 失败模式

### 8.1 `uv` 不存在

- 直接失败
- 不提供 fallback

### 8.2 profile lock 缺失

- 直接失败
- 不允许回落到 `recommended`

### 8.3 lock 指向不存在的 tool version

- 直接失败
- 这是仓库配置错误，不是用户环境错误

### 8.4 Python env 同步失败

- 直接失败
- 不写入安装态 state

### 8.5 tool 安装失败

- 直接失败
- 允许已安装成功的部分保留缓存
- 不写入新的完整安装态 state

### 8.6 Conan 配置失败

- 直接失败
- 不写入新的完整安装态 state
