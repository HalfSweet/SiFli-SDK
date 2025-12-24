# ptab_v3 讨论与计划

## 0. 目标
- 把 ptab 的职责收敛为“**分区描述 + 生成所需派生物描述**”，并适配不同芯片存储拓扑。
- 能同时支持：链接脚本宏、镜像切分、下载脚本、ftab/ptab 二进制生成等现有链路。
- 为未来扩展（新存储介质/新镜像类型/多核/多镜像）提供清晰语义。

## 1. 范围与非目标
**范围**
- 定义 v3 描述格式（字段语义、校验规则）。
- 定义与“芯片存储拓扑表”的关系与接口。
- 产物映射规则（ptab.h / link.lds / ftab.c / download script / ptab.bin）。
- 迁移策略（v1/v2 -> v3）。
- 去掉 ftab 子工程，改为脚本生成 ftab.bin（参考 `gen_ftab.py`）。
- 重构 `resource.py` 与 `building.py` 中相关生成链路。

**非目标**
- 立即重写所有项目的 ptab 文件（先提供迁移/兼容）。
- 变更业务代码对宏的依赖（尽量保持宏兼容）。

## 2. 设计原则
- **单一事实来源**：v3 ptab 是唯一权威输入。
- **平台差异外置**：芯片内存地图单独维护，ptab 不直接写物理基址。
- **可验证**：强约束字段、可预测的输出。
- **最小破坏**：宏/工具链输出尽量保持 v2 兼容。

## 3. 输入模型（草案）
> **仅支持 YAML**（v3 不再支持 JSON）。

字段（草案）：
- `name` (string, required)\n  - 规则：仅允许小写字母 + `_` + 数字；**数字不能作为开头**。
- `type` (enum: ptab/bootloader/app/data/code/...)  
- `subtype` (string, optional; 如 factory/int_res/nvds/acc)
- `region` (string, required; 逻辑存储区，如 mpi1/mpi2/psram/flash2)
- `offset` (hex/int, required)
- `size` (hex/int, required)
- `core` (enum: HCPU/LCPU/ACPU, optional)
- `exec_region` (string, optional; 非 XIP 执行地址所在 region)
- `exec_offset` (hex/int, optional)
- `attrs` (map, optional; 扩展字段，比如压缩、加密、签名、对齐约束)

### type
- ptab
- bootloader
- app
- data

#### ptab
- primary，默认
ptab段先忽略，不进行任何处理

#### bootloader
- primary，默认

#### app
- factory，默认
- int_res，内置资源，在后期会被切分为多个.bin
- accelerate，app加速，每个ptab中只能存在一个。作用是加速整个app的执行。最终生成的链接脚本中的`__ROM_BASE`将是这个镜像的地址

#### data
- nvds等
data段先忽略，不进行任何处理


## 4. 芯片存储拓扑表（外置）
- 引入 `chip_mem_map`（独立文件或内置数据库）。
- 记录：region 名称、物理基址、可用大小、类型（nor/nand/psram）、对齐要求、可执行属性等。
- v3 解析时通过 region 名称映射到物理地址。

## 5. 产物映射规则（草案）
### 5.1 ptab.h
- **不再使用 `tags`**。`name` 将被转为大写宏名，生成：  
  - `<NAME>_START_ADDR`  
  - `<NAME>_SIZE`  
  - `<NAME>_OFFSET`
- 对 `type=app` 且需要执行的分区：
  - 若 `exec_region` 以及 `type=acc&subtype=app` 存在：生成 `APP_<NAME>_CODE_*` 以 exec 地址为准。
  - 否则使用 `region/offset` 的物理地址（XIP）。
- `CODE_START_ADDR/CODE_SIZE` 规则：优先与 `name==main` 对齐。

### 5.2 link.lds
- `__ROM_BASE/__ROM_SIZE` 应来自 **主程序执行区**（main 的 exec 或 region）。

### 5.3 ftab.c
- **去掉 ftab 子工程**；`ftab.bin` 由脚本生成（参考 `gen_ftab.py`）。  
- `type=app` + `subtype=factory` / `int_res` 等生成镜像表项。
- `exec_region` 决定 xip / run 地址。

### 5.4 download script
- 映射 `type=app`/`data` 等的下载地址（region+offset）。
- 对 `int_res`（镜像切分）输出多段地址。

### 5.5 ptab.bin
参考`gen_ftab.py`进行生成

### 5.6 链接脚本分区命名
- `.romx` 等分区**不再使用数字序号**，统一按分区 `name` 生成（例如 `.rom_main` / `.rom_font`）。
- 原`.romx`的设计需要更改

## 6. 关键场景设计
- **PSRAM 加速**
  - 直接执行：使用 `exec_region/exec_offset`（XIP 或映射执行），暂时不需要关心如何搬运
- **镜像切分**
  - `type=app` + `subtype=int_res` 表示拆分 main.bin；每段生成独立 img 入口。
- **多核/多镜像**
  - 通过 `core: HCPU/LCPU/ACPU` 字段进行区分。

## 7. 校验规则
- 分区不重叠；对齐满足 region 规则。
- 所有对应region中的`size` 不得超过 region 容量。
- `type=bootloader` 必须存在且唯一（按芯片规则）。
- `name` 必须满足：小写字母 + `_` + 数字；数字不能作为开头。
- 有exec_region，整个patb就必须

## 8. 工具链改造计划
1) 新增 v3 解析器（仅支持 YAML）。
2) 增加 chip_mem_map 数据源。
3) `ptab` tool 中增加 v3 适配层（保持现有产物接口）。
4) 去掉 ftab 子工程，新增脚本生成 `ftab.bin`（参考 `gen_ftab.py`）。
5) 重构 `resource.py`（按 v3 结构与新产物流程重写）。
6) 重构 `building.py` 中 `ProgramBinaryBuild/ProgramHexBuild/...` 等生成流程。
7) 链接脚本 `.romx` 规则调整为基于分区 `name`。
8) 新增迁移脚本（v1/v2 -> v3）。
9) 增加验证/单测（覆盖多芯片、多 region、XIP/非 XIP）。

## 9. 迁移策略
- 第一步：v3 与 v2 并行，增加 `version: 3` 标识。
- 第二步：提供自动转换工具，逐步替换 board 的 ptab。
- 第三步：移除旧版本（或只保留兼容读取）。

