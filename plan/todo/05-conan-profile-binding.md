# 05. Conan Profile 绑定

本文件只记录可执行任务，不重复背景设计。

## 目标

将 Conan 配置与缓存从全局状态改为 profile 绑定，避免不同 profile 相互污染。

## 前置依赖

- `01-data-model-and-state.md`
- `03-python-env-and-uv.md`
- `04-tools-install-pipeline.md`

## Checklist

- [x] 固定 `CONAN_HOME=${install_root}/conan/<profile>`。
- [x] 固定 Conan config 版本来源为 `lock.json.conan.config_id`。
- [x] 实现 Conan 配置包按与 tools 相同的 cache/staging/source 规则获取。
- [x] 固定 install 顺序：Python env 就绪后再执行 Conan 初始化。
- [x] 固定 Conan 初始化动作为 `config install` 后再 `remote add ... --force`。
- [x] 明确不再复用全局 `${tools_path}/conan`。
- [x] install 成功后 state 必须写入 `conan.config_id` 和 `conan.home`。
- [x] export 漂移检测必须验证 Conan 配置与当前 profile 绑定一致。

## 完成判定

- 多 profile 下 Conan home 和 remote/config 不再互相污染。
- state 能明确描述当前 profile 的 Conan 安装态。
