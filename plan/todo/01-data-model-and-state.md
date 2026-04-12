# 01. 数据模型与状态文件

本文件只记录可执行任务，不重复背景设计。

## 目标

固定 profile lock、本地 state 和本机 transport config 的结构，后续 install/export 不再临时猜字段。

## 前置依赖

- 无

## Checklist

- [x] 固定 `tools/locks/default/lock.json` 的字段集合和语义。
- [x] 固定 `tools/locks/default/pyproject.toml` 与 `tools/locks/default/uv.lock` 为 profile 绑定的一部分。
- [x] 固定本地 state 路径为 `${SIFLI_SDK_TOOLS_PATH}/sifli-sdk-env.json`。
- [x] 固定 state 结构为 `repos -> repo_root -> profiles -> profile -> installed/preferences`。
- [x] 固定 `preferences.auto_reconcile` 只允许 `ask|always|never`。
- [x] 固定本机 config 路径为 `${SIFLI_SDK_TOOLS_PATH}/config.json`。
- [x] 固定 config 字段为 `install_root`、`cache_root`、`staging_root`、`offline`、`python_packages`、`sources`。
- [x] 固定目录规则：`python_env/<profile>/py3.12.0`、`conan/<profile>`、`tools/<tool>/<version>`、`cache/dist`。
- [x] 明确 state 必须一次性成功写入，失败不得留下新的半状态。
- [x] 明确旧 state 不做自动迁移，缺少新 schema 时直接按未安装处理。

## 完成判定

- lock schema、state schema 和本机 config schema 已经落到代码与文档。
- install/export 的判定不再依赖旧 `features/targets` 风格状态结构。
