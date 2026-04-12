# 02. 数据模型设计

## 1. 设计原则

- repo 内数据负责声明“应该是什么”
- 本地状态负责记录“已经装成了什么”和“用户选择了什么”
- 任何 export/install 判定都必须来源于显式字段，不允许运行时猜测

## 2. Profile Lock

路径：

- `tools/locks/<profile>/lock.json`

首版必须存在：

- `tools/locks/default/lock.json`

## 3. Profile Lock JSON 结构

```json
{
  "schema_version": 1,
  "profile": "default",
  "python": {
    "version": "3.12.0",
    "project_dir": "tools/locks/default",
    "lock_file": "tools/locks/default/uv.lock"
  },
  "defaults": {
    "targets": ["all"]
  },
  "tools": {
    "sftool": "0.1.16",
    "sdk-exe": "0.1.1",
    "cmake": "3.30.2",
    "arm-none-eabi-gcc": "14.2.1"
  },
  "path_order": [
    "sdk-exe",
    "sftool",
    "cmake",
    "arm-none-eabi-gcc"
  ],
  "conan": {
    "config_id": "sdk.conan-config.v2.4",
    "remote_name": "artifactory",
    "remote_url": "https://jfrog.sifli.com/artifactory/api/conan/conan-local",
    "home_subdir": "default"
  }
}
```

## 4. 字段职责

### 4.1 `profile` 与 `schema_version`

- `schema_version`
  - profile lock 的结构版本
- `profile`
  - 仅用于声明 profile 名，不参与环境兼容判断

## 4.1.1 环境兼容指纹

为了避免“SDK 有普通代码更新但环境定义未变时强制重装”，需要引入自动计算的兼容指纹：

- `env_compat_sha256`
- `env_compat_algorithm`

职责：

- 作为 export/install 判定环境是否仍兼容的主依据
- 替代“只要 `git HEAD` 变化就重装”的粗粒度策略

首版算法名固定为：

- `env_compat_algorithm = "v1"`

`v1` 的输入固定为三类主要文件：

1. `tools/locks/<profile>/lock.json`
   - 直接使用其中“会影响安装结果”的环境字段
2. `tools/locks/<profile>/pyproject.toml`
3. `tools/locks/<profile>/uv.lock`

`v1` 必须按下面的精确定义计算，不允许“主要文件大概 hash 一下”：

### 4.1.1.1 `lock.json` 的抽取规则

从 `tools/locks/<profile>/lock.json` 中只抽取以下字段进入 compat payload：

```json
{
  "python": {
    "version": "...",
    "project_dir": "...",
    "lock_file": "..."
  },
  "defaults": {
    "targets": [...]
  },
  "tools": {
    "...": "..."
  },
  "conan": {
    "config_id": "...",
    "remote_name": "...",
    "remote_url": "..."
  }
}
```

明确排除这些字段：

- `profile`
- `schema_version`
- `conan.home_subdir`
- 任何未来新增但未列入 compat payload 的字段

原因：

- 这些字段只影响声明结构、本地路径或其他非兼容性语义，不影响“是否必须重装”

### 4.1.1.2 `pyproject.toml` 的归一化规则

1. 以 TOML 解析 `tools/locks/<profile>/pyproject.toml`
2. 忽略注释、空行、原始键顺序
3. 将解析结果转成 canonical JSON：
   - UTF-8 编码
   - key 全部按字典序递归排序
   - 不保留多余空白
   - 列表顺序保持 TOML 原意，不做额外排序

### 4.1.1.3 `uv.lock` 的归一化规则

1. 以 TOML 解析 `tools/locks/<profile>/uv.lock`
2. 忽略注释、空行、原始键顺序
3. 将解析结果转成 canonical JSON：
   - UTF-8 编码
   - key 全部按字典序递归排序
   - 不保留多余空白
   - 列表顺序保持 lock 文件原意，不做额外排序

### 4.1.1.4 拼接规则

得到三段 canonical bytes 后，按下面的固定顺序拼接：

1. `b"lock-json\0" + <normalized lock payload bytes>`
2. `b"\0pyproject-toml\0" + <normalized pyproject bytes>`
3. `b"\0uv-lock\0" + <normalized uv.lock bytes>`

然后对最终字节串计算：

- `sha256`

输出十六进制小写字符串，写入：

- `sdk.env_compat_sha256`

### 4.1.1.5 变更规则

以下变化不会改变 `env_compat_sha256`：

- `git HEAD` 变化
- `lock.json` 中只影响结构版本、提示或本地路径的字段变化
- `pyproject.toml` / `uv.lock` 的注释、空白、键顺序变化

以下变化必须改变 `env_compat_sha256`：

- Python 版本变化
- targets 默认值变化
- tools 版本绑定变化
- Conan config / remote 变化
- `pyproject.toml` 语义变化
- `uv.lock` 语义变化

这样：

- SDK 代码、文档、示例更新不会自动触发重装
- 只有真正影响环境定义的文件变化才会改变兼容指纹

### 4.2 `python`

- `version`
  - 锁定 Python 解释器小版本
- `project_dir`
  - 当前 profile 的 uv project 根目录
- `lock_file`
  - 当前 profile 的 uv 锁文件路径

### 4.3 `defaults`

- `targets`
  - 无参数 install/export 默认 target 集合

### 4.4 `tools`

- 键是 `tools.json` 中的 tool name
- 值是该 tool 对应的固定版本
- 只表达版本绑定，不表达“是否默认安装”

默认安装范围仍然由现有 `tools.json` 的 `install` 与 platform override 决定。

### 4.5 `path_order`

- export 时 PATH 的注入顺序
- 只对当前 profile 选中的工具生效
- `path_order` 中未列出的工具，即使已安装也不进入 PATH

### 4.6 `conan`

- `config_id`
  - 映射到具体 Conan 配置制品，例如 `sdk.conan-config.v2.4.zip`
- `remote_name`
  - Conan remote 名称
- `remote_url`
  - Conan remote 地址
- `home_subdir`
  - 当前 profile 的 Conan home 相对子目录

## 5. Python uv 工程

路径：

- `tools/locks/<profile>/pyproject.toml`
- `tools/locks/<profile>/uv.lock`

规则：

- 每个 profile 都有一套与自身目录直接对应的 uv project。
- uv 工程只保留一套完整依赖集合
- 不再使用 `core`、`pytest`、`docs`、`ci`、`gdbgui`、`test-specific` 这类 extras
- install 不再根据 feature 选择不同 Python 依赖图
- export 不修改依赖，只校验锁定环境是否匹配

说明：

- profile 之间如果 Python 依赖完全一致，可以让各自的 `pyproject.toml` 与 `uv.lock` 内容一致。
- 但路径仍按 profile 隔离，这样更符合 profile 的直觉模型，也便于未来独立演进。

## 6. 本地状态文件

路径：

- `${SIFLI_SDK_TOOLS_PATH}/sifli-sdk-env.json`
- 默认即 `~/.sifli/sifli-sdk-env.json`

结构：

```json
{
  "schema_version": 1,
  "repos": {
    "/abs/sdk/root": {
      "profiles": {
        "default": {
          "installed": {
            "sdk": {
              "version_txt": "v2.4.0",
              "git_head": "f356243a33a022199ef8fe1f80f036f8383898fb",
              "dirty": false,
              "env_compat_algorithm": "v1",
              "env_compat_sha256": "...."
            },
            "locks": {
              "profile_lock_sha256": "....",
              "uv_lock_sha256": "...."
            },
            "python": {
              "version": "3.12.0",
              "env_path": "/home/user/.sifli/python_env/default/py3.12.0"
            },
            "targets": ["sf32lb52"],
            "tools": {
              "sftool": "0.1.16",
              "arm-none-eabi-gcc": "14.2.1"
            },
            "conan": {
              "config_id": "sdk.conan-config.v2.4",
              "home": "/home/user/.sifli/conan/default"
            }
          },
          "preferences": {
            "auto_reconcile": "ask"
          }
        }
      }
    }
  }
}
```

## 7. 本机环境配置

除了 repo 内的 profile 数据，还需要一份“本机 transport 配置”。

路径建议：

- `${SIFLI_SDK_TOOLS_PATH}/config.json`
- 或默认 `~/.sifli/config.json`

职责：

- 决定“下载到哪里、解压到哪里、优先从哪里取包”
- 不决定“应该装什么版本”

建议结构：

```json
{
  "cache_root": "~/.sifli/cache",
  "staging_root": "~/.sifli/staging",
  "offline": false,
  "python_packages": {
    "default_index": "https://pypi.org/simple",
    "indexes": [],
    "index_strategy": "first-index"
  },
  "sources": [
    {
      "type": "local-cache",
      "path": "~/.sifli/cache/dist"
    },
    {
      "type": "mirror",
      "url": "https://mirror.example.com/sifli-sdk"
    },
    {
      "type": "upstream",
      "url": "https://downloads.sifli.com/dl/sifli-sdk"
    }
  ]
}
```

字段语义：

- `cache_root`
  - 下载缓存目录，用于保存归档包、校验结果、bundle 缓存
- `staging_root`
  - 安装临时目录，下载和解压先在这里完成，校验通过后再原子移动到安装目录
- `offline`
  - 为 `true` 时，禁止访问网络源，只允许本地 cache/bundle
- `python_packages`
  - `uv sync` 使用的 Python 包索引配置
  - 与 tool artifact 的下载源分开管理
  - `default_index` 对应 `uv` 的默认 index
  - `indexes` 对应额外 index 列表
  - `index_strategy` 对应 `uv` 的 index 策略，首版建议保持 `first-index`
- `sources`
  - 按顺序定义下载源优先级

注意：

- install 根目录不再由 `config.json` 决定。
- install/export 共同使用：
  - `SIFLI_SDK_TOOLS_PATH`
  - 或默认 `~/.sifli`
- 本机环境配置属于“部署环境”，不应写入 profile lock。
- profile lock 只描述版本绑定，不描述镜像、代理、缓存目录。
- Python 包镜像和 tool 归档镜像必须分离建模，不能共用同一组 source 字段。

## 7.1 Python 包索引配置

`uv sync` 的镜像配置建议通过本机环境配置或环境变量注入，而不是写进 profile lock。

原因：

- profile lock 要表达“依赖版本”，不是“从哪个 PyPI 镜像拉”
- 不同机器、不同网络区域、不同 CI 环境通常会使用不同 index
- 这和 tool 归档镜像一样，都属于 transport 层配置

建议映射到 `uv` 官方能力：

- `UV_DEFAULT_INDEX`
- `UV_INDEX`
- `UV_INDEX_STRATEGY`

如需私有源认证，优先使用：

- `UV_INDEX_<NAME>_USERNAME`
- `UV_INDEX_<NAME>_PASSWORD`
- 或 `netrc` / keyring

说明：

- 如果只是替换默认 PyPI 镜像，优先设置 `default_index`
- 如果需要增加额外索引，使用 `indexes`
- 如果要避免依赖混源，首版保持 `first-index`

## 8. `installed` 字段语义

- `sdk.version_txt`
  - 安装时对应的 SDK 版本文本
- `sdk.git_head`
  - 安装时对应的源码头
- `sdk.dirty`
  - 安装时工作树是否 dirty，仅用于诊断
- `sdk.env_compat_algorithm`
  - 兼容指纹的算法版本
- `sdk.env_compat_sha256`
  - 安装时计算出的环境兼容指纹
- `locks.profile_lock_sha256`
  - 安装时 profile lock 原始内容摘要
- `locks.uv_lock_sha256`
  - 安装时 uv lock 原始内容摘要
- `python.version`
  - 实际安装环境使用的 Python 版本
- `python.env_path`
  - 实际 Python 环境路径
- `targets`
  - 安装时解析后的 target 列表
- `tools`
  - 实际安装并由当前 profile 管理的 tool 版本
- `conan.config_id`
  - 安装时使用的 Conan 配置版本
- `conan.home`
  - Conan home 路径

判定层级：

- `env_compat_algorithm`
  - 必须先匹配算法版本
- `env_compat_sha256`
  - 主判定字段
- `git_head`
  - 来源诊断字段
- `profile_lock_sha256` / `uv_lock_sha256`
  - 辅助诊断字段

## 9. `preferences` 字段语义

目前只定义一个字段：

- `auto_reconcile`
  - `ask`
  - `always`
  - `never`

解释：

- `ask`
  - export 发现失配时询问用户
- `always`
  - export 发现失配时静默重建
- `never`
  - export 发现失配时静默失败

## 10. 目录布局规则

### 9.1 Python env

建议目录：

- `${SIFLI_SDK_TOOLS_PATH}/python_env/<profile>/py3.12.0`
- `${SIFLI_SDK_TOOLS_PATH}/python_env/<profile>/current-env.txt`

理由：

- 与 profile 强绑定
- 与 Python 版本强绑定
- 不再跟随“当前启动脚本的 Python minor”漂移

### 9.2 Python lock project

建议目录：

- `tools/locks/<profile>/pyproject.toml`
- `tools/locks/<profile>/uv.lock`

理由：

- 让 profile 与 Python lock 在路径上直接一一对应
- 让同一个 profile 的 `lock.json`、`pyproject.toml`、`uv.lock` 放在同一目录
- 便于 code review 时判断哪个 profile 改了依赖
- 避免一个共享 `uv.lock` 被多个 profile 交叉引用带来的理解成本
- 即使首版所有 profile 共享同一套 Python 依赖内容，也不需要引入 extras 机制

### 9.3 Conan home

建议目录：

- `${SIFLI_SDK_TOOLS_PATH}/conan/<profile>`

理由：

- 多 profile 隔离
- 不同 SDK lock 互不污染 Conan 缓存与 remote 配置

### 9.4 工具目录

继续保留：

- `${SIFLI_SDK_TOOLS_PATH}/tools/<tool>/<version>`

理由：

- 与现有 `tools.json`、download/install 逻辑兼容
- 允许历史版本共存，便于 profile 共享缓存

### 9.5 下载缓存目录

建议目录：

- `${SIFLI_SDK_TOOLS_PATH}/cache/dist`

理由：

- 下载与安装解耦
- 同一归档可以被多个 profile 复用
- 更适合受限环境、内网镜像和离线部署

### 9.6 临时解压目录

建议目录：

- `${SIFLI_SDK_TOOLS_PATH}/staging`

理由：

- 避免在最终安装目录中边下载边解压
- 解压、校验、可执行检查都可在 staging 完成
- 通过后再原子移动到最终安装目录

### 9.7 Python 包缓存

建议：

- 不自行设计另一套 Python wheel 缓存协议
- 直接复用 uv 自带缓存机制
- 本机配置只负责指定 index，不负责重写 uv 内部缓存格式

理由：

- 避免和 uv 本身的缓存逻辑冲突
- 保持与 `uv sync` 官方行为一致
## 11. 旧数据的处理方式

### 10.1 `tools.json`

保留职责：

- 下载地址
- 平台制品信息
- export path 模板
- 安装类型

删除职责：

- 当前绑定版本真相源

### 10.2 旧 `sifli-sdk-env.json`

处理原则：

- 允许读取旧文件
- 但不尝试兼容旧字段做“智能推导”
- 发现旧结构时直接当作未安装状态处理，并由新 install 重新写入
