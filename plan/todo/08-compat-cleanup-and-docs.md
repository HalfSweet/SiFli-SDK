# 08. 兼容层收口与文档迁移

本文件只记录可执行任务，不重复背景设计。

## 目标

将旧链路降级为历史遗留，确保用户文档、错误提示和实际主链路一致。

## 前置依赖

- `02-manager-cli-and-entrypoints.md`
- `03-python-env-and-uv.md`
- `04-tools-install-pipeline.md`
- `05-conan-profile-binding.md`
- `06-export-drift-detection-and-reconcile.md`
- `07-shell-export-and-path-management.md`

## Checklist

- [x] 直接删除 `tools/detect_python.sh`。
- [x] 直接删除 `tools/activate.py`。
- [x] 直接删除 `tools/export_utils/*`。
- [x] 直接删除 `tools/install_util.py`。
- [x] 直接删除 `tools/check_python_dependencies.py`。
- [x] 直接删除 `tools/python_version_checker.py`。
- [x] 删除根脚本中的 legacy export 死分支。
- [x] 将 `sifli_sdk_tools.py` 降级为工具元数据、下载、校验、安装的底层能力。
- [x] 对旧 env 子命令给出清晰迁移错误（`export/install-python-env/get-install-python-env/check-python-dependencies`）。
- [x] 更新 quickstart 安装文档：需要 `uv`，不需要预装 Python。
- [x] 更新 quickstart export 文档：export 可能触发失配检查与环境重建。
- [ ] 更新镜像/缓存/离线文档：明确 tool 源和 Python package index 是两套配置。
- [ ] 更新 breaking change 文档：删除 constraints、删除 recommended 推导、删除系统 PATH 抢占、删除 feature/extras。

## 完成判定

- 文档和入口行为一致。
- 旧链路不再是用户可走的主流程。
