# 09. 测试、回归与验收

本文件只记录可执行任务，不重复背景设计。

## 目标

补齐验证矩阵和最终验收口径，确保重构不是“能跑一次”，而是可重复、可回归。

## 前置依赖

- `01-data-model-and-state.md`
- `02-manager-cli-and-entrypoints.md`
- `03-python-env-and-uv.md`
- `04-tools-install-pipeline.md`
- `05-conan-profile-binding.md`
- `06-export-drift-detection-and-reconcile.md`
- `07-shell-export-and-path-management.md`
- `08-compat-cleanup-and-docs.md`

## Checklist

- [x] 增加 compat hash 单元测试：注释/空白/键顺序不影响结果，语义变化必须改变结果。
- [x] 增加 `sdk.py check_environment()` 单元测试：`SIFLI_SDK_PATH`、`SIFLI_SDK_PYTHON_ENV_PATH`、解释器归属校验和迁移报错。
- [x] 增加 `sifli_sdk_tools.py` 旧 env 子命令禁用测试：`export/install-python-env/get-install-python-env/check-python-dependencies`。
- [ ] 增加 state/config 读写测试：schema、默认值、优先级、错误输入。
- [ ] 增加 target/tool 解析测试：默认范围、platform override、required/optional 分流。
- [ ] 增加 PATH 管理测试：重复 export、切 profile、切 checkout、managed paths 清理。
- [ ] 增加 bash/zsh 冒烟测试：无参数 install/export、自定义 `SIFLI_SDK_TOOLS_PATH`、自定义 cache/staging、`--profile default`、显式 target。
- [ ] 增加漂移测试：改 lock、改 `uv.lock`、删 Python env、删 required tool、删 Conan home、切 commit、只改普通源码、dirty worktree。
- [ ] 增加偏好测试：`ask->once`、`ask->always`、`ask->never`、非 TTY `ask`。
- [x] 明确 Windows/PowerShell 验证项需要在 CI 或目标机执行。
- [x] 记录当前本机没有 `pwsh`，本地只能完成 bash/zsh 自动化验证。
- [x] 固定最终验收标准：bash/PowerShell 无参数 install/export 可用；不依赖系统 Python；repo 内有 Python lock；export 能识别并处理环境漂移；多 profile 数据结构已成立。

## 完成判定

- 测试矩阵、回归项和最终验收标准都已落到仓库并可逐项打勾。
- 实现完成后可直接按本文件关闭验收项。
