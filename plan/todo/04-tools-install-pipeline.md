# 04. 工具安装链路

本文件只记录可执行任务，不重复背景设计。

## 目标

将工具安装的版本决策改为完全由 profile lock 驱动，同时保留当前 no-arg install 的默认安装范围。

## 前置依赖

- `01-data-model-and-state.md`
- `02-manager-cli-and-entrypoints.md`
- `03-python-env-and-uv.md`

## Checklist

- [x] 工具版本选择改为只读 `tools/locks/<profile>/lock.json`。
- [x] 保留默认安装范围仍由 `tools.json install` 与 platform override 决定。
- [x] 明确 `tools.json recommended` 不再参与版本决策。
- [x] 实现固定决策流程：平台/target 可见性校验 -> lock 版本校验 -> `tools.json` 版本存在性校验。
- [x] 实现 `install_root`、`cache_root`、`staging_root` 三类目录分离。
- [x] 实现下载源查找顺序：`cache -> from-bundle -> local source -> mirror -> upstream`。
- [x] 所有归档命中后都必须校验 `sha256/size`。
- [x] 所有安装动作必须先在 `staging_root` 完成校验，再原子移动到最终目录。
- [x] required tool 缺失时 install 失败。
- [x] optional tool 未安装时不强制补装。
- [x] install 成功后 state 必须写入当前 profile 管理的实际 tool 版本集合。

## 完成判定

- no-arg install 的默认工具范围保持兼容。
- 实际安装版本只受 profile lock 影响，不再受 `recommended` 漂移影响。
