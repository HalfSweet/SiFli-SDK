# SDK 安装/导出重构文档索引

本目录只包含设计与实施文档，不包含代码变更。

当前基线：

- SDK 版本：`v2.4.0`
- 当前仓库 `HEAD`：`f356243a33a022199ef8fe1f80f036f8383898fb`
- 本次重构范围：`install.sh`、`install.ps1`、`export.sh`、`export.ps1`、Python 环境管理、工具链绑定、Conan 初始化、profile 化状态管理
- 本次不处理：fish/cmd、旧 ENV GUI 工具、非 `uv` 场景

文档顺序：

1. [00-goals-and-scope.md](./00-goals-and-scope.md)
2. [01-current-state-audit.md](./01-current-state-audit.md)
3. [02-data-model.md](./02-data-model.md)
4. [03-install-pipeline.md](./03-install-pipeline.md)
5. [04-export-pipeline.md](./04-export-pipeline.md)
6. [05-compatibility-and-migration.md](./05-compatibility-and-migration.md)
7. [06-test-and-rollout.md](./06-test-and-rollout.md)
8. [todo/README.md](./todo/README.md)

执行进度请直接看 `todo/README.md` 和各子任务文件中的 checkbox。

阅读建议：

- 先读 `00` 和 `01`，确认问题定义和现状。
- 再读 `02`、`03`、`04`，这是实际实现的主设计。
- 最后读 `05`、`06`，用于判断 breaking change、迁移方式和验证范围。

实施原则：

- 无参数的外部入口必须保留：
  - `./install.sh`
  - `.\install.ps1`
  - `. ./export.sh`
  - `.\export.ps1`
- 参数兼容不是强要求，但需要给出迁移说明。
- 允许 breaking change，但必须在文档里显式列出并解释原因。
