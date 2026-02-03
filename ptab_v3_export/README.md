# ptab v3（SF32LB52x / GCC）复现包

本目录用于把当前工作区的 **ptab v3（SF32LB52x / GCC）重构结果**以“补丁 + 指引”的形式固化，便于在另一台机器上**按固定基线**快速复现与验证。

## 1) 基线版本（必须一致）

- Superproject（SiFli-SDK）：`6b0c7b90c51a1cb309858752254a0b6e3608873b`
- Submodule `tools/SiliconSchema`：`fb1139b640fa196e755cd3401d49565a56519a77`

> 注意：superproject 的补丁 **不包含** `tools/SiliconSchema` 的 “-dirty” 状态与内容变更；SiliconSchema 的修改通过单独补丁提供。

## 2) 确认后的设计要点（凝练版）

- **v3 规范执行地址字段：`acc: { region, offset }`**
  - `acc` 表示“执行地址/搬运落点”。
  - `exec_region/exec_offset` 仅作为兼容输入被读取，并在解析阶段归一化为 `acc`。
- **兼容宏：`aliases`**
  - 为分区生成额外的旧宏基名：`<ALIAS>_START_ADDR/_SIZE/_OFFSET`。
- **region 视图规则（关键修复点：PSRAM）**
  - NOR：优选 **CBUS/XIP** 视图
  - NAND/RAM/**PSRAM**：优选 **SBUS/base** 视图（PSRAM 不再按 NOR 处理）
- **链接脚本：v3 必须 Jinja2-only**
  - v3 不再使用 `gcc -E` 预处理生成 `link_copy.lds`。
  - v3 若缺少同名 `link.jinja2`，构建直接报错（不 fallback）。
- **bootloader 兼容：`FLASH_BOOT_LOADER_SIZE`**
  - `FLASH_BOOT_LOADER_SIZE` 与 ftab 中 bootloader entry 的 size 优先使用 `mem_map.h` 默认值对齐历史产物；若默认值小于 storage partition size，则取更大值。
- **迁移工具策略（`tools/build/migrate_ptab_to_v3.py`）**
  - legacy “exec-only RAM region（有 `exec` 无 `img`）” → 转成对应 bootloader/app partition 的 `acc`
  - legacy `tags` → 生成 `aliases`（去重，避免与 canonical name 重复）
  - 自动补齐 `calibration`（紧跟 ftab 后）以匹配历史 ftab 基线
  - 过滤 legacy 内部 RAM 的重叠“宏 region”，保证 v3 overlap 校验能通过

详细字段与用法说明：`docs/source/zh_CN/middleware/partition_table_v3.md`

## 3) 应用补丁（跨机器复现步骤）

在另一台机器上：

```bash
## 先把本目录（ptab_v3_export/）拷贝到目标仓库根目录
git checkout 6b0c7b90c51a1cb309858752254a0b6e3608873b
git submodule update --init --recursive
bash ptab_v3_export/apply_patches.sh
```

建议在应用补丁前确保工作区没有已跟踪文件的本地修改（避免 patch 冲突）。

应用完成后，你会得到：

- superproject 中的 ptab v3 实现与模板（GCC only）
- submodule `tools/SiliconSchema` 的 RAM 数据修复补丁

## 4) 依赖安装

最小依赖（只为运行 v3 工具/渲染链接脚本）：

```bash
python3 -m pip install pyyaml Jinja2
```

或完整依赖：

```bash
python3 -m pip install -r tools/requirements/requirements.core.txt
```

## 5) 验证（可复制命令）

### 5.1 v3 文件校验

```bash
python3 tools/build/validate_ptab_v3.py customer/boards/sf32lb52-nano_a128r16/ptab.yaml
python3 tools/build/validate_ptab_v3.py customer/boards/sf32lb52-nano_n16r16/ptab.yaml
```

### 5.2 严格对比回归（预期输出 `OK`）

```bash
python3 tools/build/ptab_v3_regression.py --board sf32lb52-nano_52j --chip SF32LB52JUD6
```

该脚本默认读取基准产物目录：
`example/misc/button/project/build_sf32lb52-nano_52j_hcpu`

如果目标机器没有该目录（通常 build 输出不会被提交），请先生成一次基准产物：

```bash
cd example/misc/button/project
scons --board=sf32lb52-nano_52j
```

## 6) 关键改动文件索引

### 构建系统 / 生成器
- `tools/build/building.py`：v3 走 Jinja2 渲染；v1/v2 保持 `gcc -E`
- `tools/build/gen_link_lds.py`：v3 链接脚本渲染与常量计算
- `tools/build/gen_ftab.py`：v3 直接生成 `ftab.bin`
- `tools/build/resource.py`：v3 `ptab.h` 宏生成（含 aliases/FS_REGION/CODE/bootloader 兼容）
- `tools/build/ptab.py`：v3 解析/归一化/地址解析（含 PSRAM 视图修复）
- `tools/build/validate_ptab_v3.py`：v3 schema + overlap + 唯一性校验
- `tools/build/migrate_ptab_to_v3.py`：legacy ptab.json → v3 ptab.yaml 迁移
- `tools/build/ptab_v3_regression.py`：离线严格对比脚本（宏/lds/ftab）

### Jinja2 链接模板（GCC）
- `drivers/cmsis/sf32lb52x/Templates/gcc/hcpu/link.jinja2`
- `drivers/cmsis/sf32lb52x/Templates/gcc/hcpu/zbt_rom.jinja2`
- `drivers/cmsis/sf32lb52x/Templates/gcc/hcpu/zbt_data.jinja2`
- `drivers/cmsis/sf32lb52x/Templates/gcc/lcpu/link.jinja2`
- `example/boot_loader/project/butterflmicro/ram_v2/link.jinja2`

### SiliconSchema（submodule）
- `tools/SiliconSchema/common/ram/sf32lb52/ram.yaml`：补齐 LPSYS RAM 的 LCPU 视图映射
