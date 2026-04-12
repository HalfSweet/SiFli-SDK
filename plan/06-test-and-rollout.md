# 06. 测试、分阶段实施与回归要求

## 1. 实施阶段

建议按 6 个阶段落地，不要一次性大改。

### Phase 1: 数据模型落地

交付物：

- `tools/locks/default/lock.json`
- `tools/locks/<profile>/pyproject.toml`
- `tools/locks/<profile>/uv.lock`
- 新状态文件 schema

完成标准：

- 还未切换 install/export 主入口
- 但 lock 和 state 模型已经固定

### Phase 2: install 主链路切换

交付物：

- `install.sh`
- `install.ps1`
- 新 Python env 管理器

完成标准：

- 无参数 install 在 bash / PowerShell 下可运行
- 不再依赖系统 Python

### Phase 3: tools 与 Conan 绑定

交付物：

- tools 版本安装逻辑改为由 profile lock 驱动
- Conan home 与 config 改成 profile 绑定

完成标准：

- install 后 state 记录完整

### Phase 4: export 主链路切换

交付物：

- `export.sh`
- `export.ps1`
- 失配检测器
- 自动修复逻辑

完成标准：

- export 不再依赖旧 `activate.py` 主链路
- 可进行交互修复或静默失败

### Phase 5: 兼容层收口

交付物：

- 清理 legacy export 死路径
- 文档迁移
- 旧 helper 的降级或下线

### Phase 6: 回归与发布

交付物：

- 完整测试记录
- 更新后的用户文档

## 2. 测试矩阵

## 2.1 install 基础场景

- bash 无参数 install
- PowerShell 无参数 install
- 自定义 `SIFLI_SDK_TOOLS_PATH`
- 自定义 `cache_root`
- 自定义 `staging_root`
- `--profile default`
- 显式指定 target
- `--mirror`
- `--offline`
- `--from-bundle`
- 自定义 Python package default index
- 自定义 Python package extra indexes

## 2.2 export 基础场景

- bash 无参数 export
- PowerShell 无参数 export
- 同一 shell 重复 export
- 不同 profile 之间切换 export

## 2.3 漂移场景

- 修改 `tools/locks/default/lock.json`
- 修改 `tools/locks/<profile>/uv.lock`
- 删除 Python env
- 删除 required tool 目录
- 删除缓存中的归档后重试下载
- staging 中断后再次 install
- 修改 checkout 到另一个 commit
- 修改普通源码文件但不改主要环境文件
- 仅产生 dirty worktree
- 删除 Conan profile home
- Python package mirror 不可用时的 `uv sync` 失败与恢复
- Python package mirror 与 tool mirror 同时配置时的优先级验证

## 2.4 用户偏好场景

- `ask -> once`
- `ask -> always`
- `ask -> never`
- `always` 后再次失配
- `never` 后再次失配
- 非 TTY 下 `ask`

## 2.5 兼容场景

- no-arg install 保持当前默认安装范围
- no-arg export 保持当前调用方式
- Linux/macOS 下默认不强制装 `cmake`
- Windows 下默认仍安装 `cmake` 与 `sdk-exe`

## 3. 关键断言

每条测试都应验证以下断言中的若干项：

- 当前 PATH 只包含 lock 指定版本
- `SIFLI_SDK_GIT_HEAD` 等于 lock 中的 git head
- state 中记录的 `sdk.env_compat_sha256` 等于当前计算结果
- `SIFLI_SDK_PROFILE` 等于当前 profile
- `SIFLI_SDK_PYTHON_ENV_PATH` 指向 profile 专属 venv
- `CONAN_HOME` 指向 profile 专属目录
- state 文件中的摘要与当前 lock 一致

## 4. 回归风险

### 4.1 风险最高的点

- PowerShell 导出方式变化
- PATH 去重与多次 export
- Conan home/profile 隔离
- `sdk.py` 对新环境变量与新检查命令的适配
- 下载镜像、bundle、cache_root 与 install_root 的优先级组合
- Python package index 与 tool artifact source 两套 transport 配置并存

### 4.2 风险控制策略

- install 与 export 分阶段切换，不一次性同时改主链路
- 优先先做默认 profile
- 先做 bash/PowerShell，不扩展其他 shell

## 5. 验收标准

本次重构完成的最低标准：

1. 无参数 install/export 在 bash、PowerShell 下都可用
2. 安装不依赖系统 Python
3. Python 依赖在 repo 内有锁文件
4. export 能识别 checkout/tool/conan/python 漂移
5. export 支持询问并持久化自修复策略
6. 多 profile 的数据结构与状态隔离已经成立
7. 所有 breaking change 都在文档中明确说明
