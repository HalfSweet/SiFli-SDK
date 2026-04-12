# 07. Shell 导出与 PATH 管理

本文件只记录可执行任务，不重复背景设计。

## 目标

为 bash/zsh/PowerShell 生成稳定、可重复导出的环境脚本，并解决旧环境残留污染问题。

## 前置依赖

- `02-manager-cli-and-entrypoints.md`
- `03-python-env-and-uv.md`
- `04-tools-install-pipeline.md`
- `05-conan-profile-binding.md`
- `06-export-drift-detection-and-reconcile.md`

## Checklist

- [x] export 为 bash/zsh 输出临时 `.sh` 文件路径。
- [x] export 为 PowerShell 输出临时 `.ps1` 文件路径。
- [x] 根 export 脚本实现 `current-env.txt` 指针发现逻辑。
- [x] 固定 export 输出变量集合：`SIFLI_SDK_PATH`、`SIFLI_SDK_VERSION`、`SIFLI_SDK_GIT_HEAD`、`SIFLI_SDK_PROFILE`、`SIFLI_SDK_PYTHON_ENV_PATH`、`SIFLI_SDK_MANAGED_PATHS`、`SIFLI_SDK_TOOLS_INSTALL_CMD`、`SIFLI_SDK_TOOLS_EXPORT_CMD`、`SIFLI_SDK`、`CONAN_HOME`、`ENV_ROOT`、`PKGS_ROOT`、`PKGS_DIR`、`RTT_CC`、`PYTHONPATH`、`PATH`、`RTT_EXEC_PATH`。
- [x] 用 `SIFLI_SDK_MANAGED_PATHS` 替换旧 `ENVState` 临时文件清理机制。
- [x] 实现 PATH 清理逻辑：先移除旧 managed paths，再注入新 managed paths。
- [x] 固定 PATH 注入顺序：Python env、`path_order` 中已安装且匹配版本的 tools、repo `tools/`。
- [x] optional tool 已安装且匹配版本时才进入 PATH。
- [x] required tool 缺失时 export 进入失配处理，不允许带病导出。
- [x] PowerShell 中 `sdk.py` 包装函数必须调用 profile venv 的 Python，而不是系统 `python`。
- [ ] 验证重复 export、profile 切换 export、不同 checkout 切换 export 时 PATH 不污染。

## 完成判定

- 同一 shell 中多次 export、切 profile、切 checkout 都能稳定去重和切换。
- 环境切换不再依赖旧 deactivate 文件机制。
