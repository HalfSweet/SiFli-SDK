# Partition Table V3 (ptab v3)

本文档详细介绍 SiFli SDK 中 ptab v3 分区表格式的设计、使用方法和迁移指南。

## 概述

ptab v3 是 SiFli SDK 中新一代分区表格式，采用 YAML 格式，相比 v1/v2 的 JSON 格式具有以下优点：

- **更清晰的语义**：使用 `type`/`subtype` 替代复杂的 `tags` 机制
- **抽象的存储层**：通过 `region` 字段引用逻辑存储区域，与芯片拓扑解耦
- **统一的执行地址**：通过 `exec_region`/`exec_offset` 明确指定执行地址
- **自动宏生成**：从 `name` 字段自动生成 C 宏定义
- **更好的验证**：内置验证工具检查分区配置正确性

## 文件格式

### 基本结构

```yaml
# ptab v3 - Partition Table
version: 3
chip: SF32LB52

partitions:
  - name: ftab
    type: ftab
    region: mpi2
    offset: 0
    size: 128KB

  - name: bootloader
    type: bootloader
    region: mpi2
    offset: 0x80000
    size: 64KB
    exec_region: hpsys_ram
    exec_offset: 0x20000
    core: HCPU

  - name: main
    type: app
    subtype: factory
    region: mpi2
    offset: 0xA0000
    size: 4MB
    exec_region: psram1
    exec_offset: 0
    core: HCPU
```

### 顶层字段

| 字段 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `version` | int | 是 | 固定为 `3` |
| `chip` | string | 是 | 芯片型号，如 `SF32LB52`、`SF32LB58` |
| `partitions` | list | 是 | 分区列表 |

### 分区字段

| 字段 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `name` | string | 是 | 分区名称，用于生成 C 宏 |
| `type` | string | 是 | 分区类型 |
| `subtype` | string | 否 | 分区子类型 |
| `region` | string | 是 | 存储区域（逻辑名称） |
| `offset` | int/string | 是 | 区域内偏移，支持 `0x` 前缀和 `KB`/`MB` 后缀 |
| `size` | int/string | 是 | 分区大小 |
| `exec_region` | string | 否 | 执行区域（代码加载到此区域执行） |
| `exec_offset` | int/string | 否 | 执行区域内偏移 |
| `core` | string | 否 | 运行核心：`HCPU` 或 `LCPU` |
| `attrs` | dict | 否 | 自定义属性 |

## 分区类型定义

### type 字段

| 值 | 说明 |
|------|------|
| `ftab` | Flash Table（分区表自身） |
| `bootloader` | 引导加载程序 |
| `app` | 应用程序 |
| `data` | 数据分区 |

### subtype 字段

**当 type=app 时：**

| 值 | 说明 |
|------|------|
| `factory` | 出厂主程序 |
| `dfu` | DFU 升级程序 |
| `accelerate` | 加速区域（psram XIP） |

**当 type=data 时：**

| 值 | 说明 |
|------|------|
| `nvds` | 通用数据存储 |
| `filesystem` | 文件系统（生成 `FS_REGION_*` 宏） |
| `littlefs` | LittleFS 文件系统 |
| `fat` / `fatfs` | FAT 文件系统 |
| `flashdb` | FlashDB 数据库 |
| `ram` | RAM 数据区 |
| `calibration` | 校准数据 |

## 存储区域 (region)

### 支持的区域名称

| 区域 | 说明 |
|------|------|
| `mpi1` | MPI1 Flash（通常为内部 Flash） |
| `mpi2` | MPI2 Flash（通常为外部 Flash） |
| `mpi3`~`mpi5` | 其他 MPI 接口 |
| `psram1` | PSRAM 区域 |
| `hpsys_ram` | HPSYS RAM |
| `lpsys_ram` | LPSYS RAM |

### 地址解析

每个 region 对应两个地址：

- **SBUS 地址（存储地址）**：用于 DMA 访问、下载烧录
- **CBUS 地址（XIP 地址）**：用于 CPU 直接执行

地址映射从 `tools/SiliconSchema/common/mpi/<chip>/mpi.yaml` 加载。

## 宏生成规则

### 基本宏

每个分区自动生成以下宏（`<NAME>` 为大写的 `name` 字段）：

```c
#define <NAME>_START_ADDR    (0x...)  // SBUS 地址
#define <NAME>_SIZE          (0x...)  // 分区大小
#define <NAME>_OFFSET        (0x...)  // 区域内偏移
```

### 执行地址宏

当分区包含 `exec_region` 时，额外生成：

```c
#define APP_<NAME>_CODE_START_ADDR  (0x...)  // CBUS 执行地址
#define APP_<NAME>_CODE_SIZE        (0x...)  // 代码大小
#define APP_<NAME>_CODE_OFFSET      (0x...)  // 执行区域偏移
```

### 特殊宏

**accelerate 分区：**

当 `type=app` 且 `subtype=accelerate` 时生成：

```c
#define CODE_START_ADDR  (0x...)  // XIP 起始地址（用于链接脚本）
#define CODE_SIZE        (0x...)
```

**文件系统分区：**

当 `type=data` 且 `subtype` 为 `filesystem`/`littlefs`/`fat`/`fatfs`/`flashdb` 时生成：

```c
#define FS_REGION_START_ADDR  (0x...)
#define FS_REGION_SIZE        (0x...)
#define FS_REGION_OFFSET      (0x...)
```

## 完整示例

### sf32lb52-nano_a128r16/ptab.yaml

```yaml
# ptab v3 - Partition Table
# Board: sf32lb52-nano_a128r16
# Chip: SF32LB52

version: 3
chip: SF32LB52

partitions:
  # Flash Table
  - name: flash_table
    type: ftab
    region: mpi2
    offset: 0
    size: 128KB

  # 校准数据
  - name: calibration
    type: data
    subtype: calibration
    region: mpi2
    offset: 0x8000
    size: 8KB

  # Bootloader（从 Flash 加载到 RAM 执行）
  - name: bootloader
    type: bootloader
    region: mpi2
    offset: 0x80000
    size: 64KB
    exec_region: hpsys_ram
    exec_offset: 0x20000
    core: HCPU

  # 主程序（从 Flash 加载到 PSRAM 执行）
  - name: main
    type: app
    subtype: factory
    region: mpi2
    offset: 0xA0000
    size: 4MB
    exec_region: psram1
    exec_offset: 0
    core: HCPU

  # DFU 程序
  - name: dfu
    type: app
    subtype: dfu
    region: mpi2
    offset: 0x4A0000
    size: 512KB
    exec_region: psram1
    exec_offset: 0
    core: HCPU

  # 文件系统
  - name: fs
    type: data
    subtype: filesystem
    region: mpi2
    offset: 0x8A0000
    size: 4MB

  # KVDB
  - name: kvdb_dfu
    type: data
    subtype: nvds
    region: mpi2
    offset: 0xCA0000
    size: 16KB

  - name: kvdb_ble
    type: data
    subtype: nvds
    region: mpi2
    offset: 0xCA4000
    size: 16KB

  # PSRAM 数据区
  - name: psram_data
    type: data
    subtype: ram
    region: psram1
    offset: 0x400000
    size: 4MB

  # RAM 区域
  - name: hcpu_ram
    type: data
    subtype: ram
    region: hpsys_ram
    offset: 0
    size: 128KB

  - name: bootloader_ram
    type: data
    subtype: ram
    region: hpsys_ram
    offset: 0x40000
    size: 64KB

  - name: lpsys
    type: data
    subtype: ram
    region: lpsys_ram
    offset: 0
    size: 24KB

  # Accelerate 分区（当使用 exec_region 时必须存在）
  - name: accelerate
    type: app
    subtype: accelerate
    region: psram1
    offset: 0
    size: 4MB
```

## 构建系统集成

### 自动检测

构建系统会自动检测 ptab 格式：

1. 优先查找 `ptab.yaml`（v3 格式）
2. 若不存在则使用 `ptab.json`（v1/v2 格式）

搜索路径优先级：
1. `$BSP_ROOT/<board>/`
2. `$BSP_ROOT/<chip>/`
3. `$BSP_ROOT/`
4. `customer/boards/<board>/`

### 生成产物

| 产物 | 说明 |
|------|------|
| `ptab.h` | C 头文件，包含所有分区宏定义 |
| `ftab.bin` | 二进制 Flash Table，用于 bootloader |
| `link_copy.lds` | 链接脚本（通过 ptab.h 宏配置） |

### ftab.bin 生成

v3 格式下，`ftab.bin` 直接通过 Python 脚本生成，不再编译 ftab 子工程：

```
Generating build_xxx/ftab.bin ...
Generated ftab.bin: build_xxx/ftab.bin (11280 bytes)
```

## 迁移工具

### migrate_ptab_to_v3.py

将 v1/v2 的 `ptab.json` 转换为 v3 的 `ptab.yaml`：

```bash
python tools/build/migrate_ptab_to_v3.py \
    --input customer/boards/<board>/ptab.json \
    --output customer/boards/<board>/ptab.yaml \
    --chip SF32LB52
```

### validate_ptab_v3.py

验证 ptab v3 文件的正确性：

```bash
python tools/build/validate_ptab_v3.py customer/boards/<board>/ptab.yaml
```

验证项目包括：
- 分区名称格式
- 类型/子类型有效性
- 区域名称有效性
- 分区重叠检测
- `exec_region` 使用时 `accelerate` 分区存在性
- `bootloader` 分区唯一性

## 常见问题

### Q: 为什么需要 accelerate 分区？

当使用 `exec_region` 指定代码执行在 PSRAM 时，链接脚本需要知道 XIP 起始地址。`accelerate` 分区的 XIP 地址用作 `CODE_START_ADDR`，bootloader 会将代码加载到此地址执行。

### Q: region 和 exec_region 的区别？

- `region`：代码/数据的**存储位置**（Flash 中的位置）
- `exec_region`：代码的**执行位置**（RAM/PSRAM 中执行的地址）

例如：代码存储在 Flash (`region: mpi2`)，但加载到 PSRAM 执行 (`exec_region: psram1`)。

### Q: 如何添加自定义宏？

使用 `attrs` 字段：

```yaml
- name: my_partition
  type: data
  subtype: nvds
  region: mpi2
  offset: 0x100000
  size: 64KB
  attrs:
    MY_CUSTOM_MACRO: 0x12345678
```

将生成：
```c
#define MY_CUSTOM_MACRO  (0x12345678)
```

### Q: v1/v2 的 tags 在 v3 中如何映射？

| v1/v2 tags | v3 对应 |
|------------|---------|
| `FLASH_TABLE` | `type: ftab` |
| `FLASH_BOOT_LOADER` | `type: bootloader` |
| `FS_REGION` | `type: data, subtype: filesystem` |
| `KVDB_*` | `type: data, subtype: nvds` |
| `app_img` | `type: app` |
| `app_exec` | 使用 `exec_region` 字段 |

## 变更历史

| 版本 | 日期 | 说明 |
|------|------|------|
| v3.0 | 2024-12 | 初始版本，YAML 格式，移除 ftab 子工程 |
