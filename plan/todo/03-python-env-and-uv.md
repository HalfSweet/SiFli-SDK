# 03. Python 环境与 uv

本文件只记录可执行任务，不重复背景设计。

## 目标

将 Python 解释器准备和依赖安装统一切到 `uv`，消除系统 Python 和运行时 constraints 漂移。

## 前置依赖

- `01-data-model-and-state.md`
- `02-manager-cli-and-entrypoints.md`

## Checklist

- [x] 固定 Python 版本为 `3.12.0`。
- [x] 实现 `uv python install 3.12.0` 作为唯一解释器准备方式。
- [x] 固定 Python env 路径为 `${install_root}/python_env/<profile>/py3.12.0`。
- [x] 实现 `UV_PROJECT_ENVIRONMENT=<env_path> uv sync --project tools/locks/<profile> --locked --python 3.12.0`。
- [x] 删除运行时 constraints 下载路径。
- [x] 删除 feature/extras 驱动的多套 Python 依赖图。
- [x] 明确 `pyproject.toml` 和 `uv.lock` 的语义变化必须触发环境漂移。
- [x] 实现 Python package index 配置到 `UV_DEFAULT_INDEX`、`UV_INDEX`、`UV_INDEX_STRATEGY` 的映射。
- [x] 明确 Python package index 配置与 tool/conan 下载源完全分离。
- [x] 明确私有 index 认证不写入 repo lock。

## 完成判定

- install 不依赖系统 Python。
- repo 内的 `pyproject.toml` 和 `uv.lock` 成为 Python 依赖真相源。
