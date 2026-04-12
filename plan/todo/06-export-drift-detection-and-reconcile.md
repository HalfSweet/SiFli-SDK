# 06. Export 漂移检测与自修复

本文件只记录可执行任务，不重复背景设计。

## 目标

让 export 从“盲目拼 PATH”改为“先校验安装态，再按偏好修复或失败”。

## 前置依赖

- `01-data-model-and-state.md`
- `02-manager-cli-and-entrypoints.md`
- `03-python-env-and-uv.md`
- `04-tools-install-pipeline.md`
- `05-conan-profile-binding.md`

## Checklist

- [x] export 每次都重新读取 profile lock 和本地 state。
- [x] export 每次都重新计算 `env_compat_sha256`。
- [x] 根 `export.sh` / `export.ps1` 改为直接使用 install venv，而不是 `uv run` bootstrap。
- [x] 缺少 `current-env.txt` 或 install venv 解释器时直接失败，不进入 export 自修复。
- [x] 实现 compat hash 的精确算法：lock 子集抽取、`pyproject.toml` canonical TOML->JSON、`uv.lock` canonical TOML->JSON、固定顺序拼接、`sha256`。
- [x] 明确 `git HEAD` 变化但 compat hash 不变时只告警不重装。
- [x] dirty worktree 只告警，不作为失配条件。
- [x] 实现失配条件：state 缺失、算法版本不支持、compat hash 不一致、Python 版本不一致、Conan config 不一致、required tools 不一致、targets 不一致、Python env 缺失。
- [x] 实现 `once|always|never` 的交互语义和持久化。
- [x] 实现非 TTY 且偏好为 `ask` 时按 `never` 处理。
- [x] 固定 export 自修复只能通过“按 profile 默认 targets 重跑完整 install”完成。
- [x] 自修复成功后如果 Python env 路径变化，必须重启到新的 venv Python 再继续 export。
- [x] 自修复成功后必须重新进入 export 判定，而不是半修半导出。

## 完成判定

- export 能稳定区分源码变化和环境变化。
- 环境失配时能按偏好自动修复或确定性失败。
