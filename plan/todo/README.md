# SDK 工具链重构 Todo 总览

本目录只记录可执行任务和完成判定，不重复背景设计。

状态约定：

- `[ ]` 未开始
- `[x]` 已完成

执行顺序：

1. `01-data-model-and-state.md`
2. `02-manager-cli-and-entrypoints.md`
3. `03-python-env-and-uv.md`
4. `04-tools-install-pipeline.md`
5. `05-conan-profile-binding.md`
6. `06-export-drift-detection-and-reconcile.md`
7. `07-shell-export-and-path-management.md`
8. `08-compat-cleanup-and-docs.md`
9. `09-tests-rollout-and-acceptance.md`

阶段映射：

- `Phase 1` 对应 `01`、`03`
- `Phase 2` 对应 `02`、`03`
- `Phase 3` 对应 `04`、`05`
- `Phase 4` 对应 `06`、`07`
- `Phase 5` 对应 `08`
- `Phase 6` 对应 `09`

当前跟踪：

- [x] `01-data-model-and-state.md`
- [x] `02-manager-cli-and-entrypoints.md`
- [x] `03-python-env-and-uv.md`
- [x] `04-tools-install-pipeline.md`
- [x] `05-conan-profile-binding.md`
- [x] `06-export-drift-detection-and-reconcile.md`
- [ ] `07-shell-export-and-path-management.md`
- [ ] `08-compat-cleanup-and-docs.md`
- [ ] `09-tests-rollout-and-acceptance.md`

本轮明确不做：

- fish/cmd 主链路
- 旧 ENV GUI 工具
- 非 `uv` 安装/导出路径
- 旧 venv / 旧 state / 旧 Conan home 的自动迁移

完成判定：

- 以上 9 个 todo 文件都存在且内容足够支持逐项勾选。
- 后续实现时，文件内 checklist 和本页状态需要同步更新。
