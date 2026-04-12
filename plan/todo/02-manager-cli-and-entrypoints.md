# 02. 管理器 CLI 与入口脚本

本文件只记录可执行任务，不重复背景设计。

## 目标

引入统一环境管理器，替换 install/export 当前依赖的旧 Python 跳板链路。

## 前置依赖

- `01-data-model-and-state.md`

## Checklist

- [x] 新增统一入口 `tools/sdk_env.py`，承接 `install`、`export`、`check` 主控职责。
- [x] 固定 `sdk_env.py` 的稳定 CLI：`install`、`export`、`check`。
- [x] 保留外部入口：`install.sh`、`install.ps1`、`export.sh`、`export.ps1`。
- [x] 将 `install.sh` 切到 `uv run --python 3.12.0 --no-project tools/sdk_env.py install ...`。
- [x] 将 `install.ps1` 切到同等 `uv run` 调用。
- [x] 将 `export.sh` 切到 `uv run --python 3.12.0 --no-project tools/sdk_env.py export --shell ...`。
- [x] 将 `export.ps1` 切到同等 `uv run` 调用。
- [x] 保留兼容 target 位置参数：以 `sf32` 开头的参数继续解释为 target。
- [x] 新增 `--profile <name>`。
- [x] 固定无参数行为：`profile=default`，`targets=lock.defaults.targets`。
- [x] 明确 `--targets` 与兼容 target 位置参数冲突时直接报错。
- [x] 明确 `uv` 缺失时直接失败，不提供 fallback。

## 完成判定

- 根脚本只负责 SDK 根目录定位、`uv` 检查和参数透传。
- install/export 主链路不再调用 `detect_python.sh`、`install_util.py`、`activate.py`。
