# Partition Table V3 (ptab v3)

本文档详细介绍 SiFli SDK 中 ptab v3 分区表格式的设计、使用方法和迁移指南。

## 概述

ptab v3 是 SiFli SDK 中新一代分区表格式，采用 YAML 格式，相比 v1/v2 的 JSON 格式具有以下优点：

- **更清晰的语义**：使用 `type`/`subtype` 替代复杂的 `tags` 机制
- **抽象的存储层**：通过 `region` 字段引用逻辑存储区域，与芯片拓扑解耦
- **统一的执行地址**：通过 `acc: {region, offset}` 明确指定执行地址/搬运落点
- **自动宏生成**：从 `name` 字段自动生成 C 宏定义
- **更好的验证**：内置验证工具检查分区配置正确性

和 patb v1/v2 不同的是，ptab v3 与构建系统紧密结合，直接生成`ftab.bin`、`ptab.h`，以及使用基于 jinja2 的链接脚本片段等。

## 文件格式

### 基本结构

```yaml
version: 3
chip: SF32LB525UC6

memory:
  - mpi: mpi2
    type: nand
    size: 16777216 # 16MB

partitions:
  # Flash Table (ftab)
  - name: flash_table
    type: ftab
    region: mpi2
    offset: 0
    size: 32KB

  # Calibration data
  - name: calibration
    type: data
    subtype: calibration
    region: mpi2
    offset: 0x8000
    size: 8KB

  # Bootloader （存储在 Flash，执行在 RAM)
  - name: bootloader
    type: bootloader
    region: mpi2
    offset: 0x80000
    size: 64KB
    acc:
      region: hpsys_ram
      offset: 0x20000
    core: HCPU

  # Main application
  - name: hcpu_flash_code
    type: app
    subtype: factory
    region: mpi2
    offset: 0xA0000
    size: 4MB
    acc:
      region: mpi1
      offset: 0
    core: HCPU

  # DFU application
  - name: dfu_flash_code
    type: app
    subtype: dfu
    region: mpi2
    offset: 0x4A0000
    size: 512KB
    acc:
      region: mpi1
      offset: 0
    core: HCPU

  # DFU info region
  - name: dfu_info
    type: data
    subtype: nvds
    region: mpi2
    offset: 0x520000
    size: 128KB

  # DFU download region
  - name: dfu_download_region
    type: data
    subtype: nvds
    region: mpi2
    offset: 0x780000
    size: 1152KB

  # Filesystem region
  - name: fs_region
    type: data
    subtype: filesystem
    region: mpi2
    offset: 0x8A0000
    size: 4MB

  # KVDB for DFU
  - name: kvdb_dfu_region
    type: data
    subtype: nvds
    region: mpi2
    offset: 0xCA0000
    size: 16KB

  # KVDB for BLE
  - name: kvdb_ble_region
    type: data
    subtype: nvds
    region: mpi2
    offset: 0xCA4000
    size: 16KB

  # PSRAM data region
  - name: psram_data
    type: data
    subtype: ram
    region: mpi1
    offset: 0x400000
    size: 4MB

  # HCPU RAM data (shared with bootloader exec)
  - name: hcpu_ram_data
    type: data
    subtype: ram
    region: hpsys_ram
    offset: 0
    size: 128KB

  # Bootloader RAM data
  - name: bootloader_ram_data
    type: data
    subtype: ram
    region: hpsys_ram
    offset: 0x40000
    size: 64KB

  # LPSYS RAM
  - name: lpsys_ram
    type: data
    subtype: ram
    region: lpsys_ram
    offset: 0
    size: 24KB
```

### 顶层字段

| 字段 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `version` | int | 是 | 固定为 `3` |
| `chip` | string | 是 | 芯片型号，如 `SF32LB525UC6`、`SF32LB52JUD6`等 |
| `memory` | list | 否 | 存储区域定义列表 |
| `partitions` | list | 是 | 分区列表 |

## memory 定义

memory 用于定义外挂的存储器的熟悉，如 NAND Flash、NOR Flash 等。

| 字段 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `mpi` | string | 是 | MPI 接口名称，如 `mpi2` |
| `type` | string | 是 | 存储器类型，可选为 `nand`、`nor`、`sd` |
| `size` | string | 是 | 存储器大小，支持 `0x` 前缀和 `KB`/`MB` 后缀 |

### 分区字段

| 字段 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `name` | string | 是 | 分区名称，用于生成 C 宏 |
| `type` | string | 是 | 分区类型 |
| `subtype` | string | 否 | 分区子类型 |
| `region` | string | 是 | 存储区域（逻辑名称） |
| `offset` | int/string | 是 | 区域内偏移，支持 `0x` 前缀和 `KB`/`MB` 后缀 |
| `size` | int/string | 是 | 分区大小 |
| `acc` | dict | 否 | 执行地址定义 |
| `core` | string | 否 | 运行核心：`HCPU` 或 `LCPU` |
| `attrs` | dict | 否 | 自定义属性 |

#### acc 字段

acc 字段用于指定代码不能在 XiP 的时候实际的执行地址（搬运到 psram 或者 sram 中执行）。
| 字段 | 类型 | 必填 | 说明 |
|------|------|------|------|
| `region` | string | 是 | 执行区域（逻辑名称） |
| `offset` | int/string | 是 | 执行区域内偏移，支持 `0x` 前缀和 `KB`/`MB` 后缀 |

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
| `dfu` | DFU 升级程序，需要注意的是 dfu 也是一个独立运行的程序 |

**当 type=data 时：**

| 值 | 说明 |
|------|------|
| `nvds` | 通用数据存储 |
| `flashdb_kv` | FlashDB KV 数据库（自动生成 `KVDB_*` 宏和 `FAL_PART_TABLE`） |
| `filesystem` | 文件系统（生成 `FS_REGION_*` 宏） |
| `littlefs` | LittleFS 文件系统，暂时无实际作用 |
| `fat` / `fatfs` | FAT 文件系统，暂时无实际作用 |
| `flashdb` | FlashDB 数据库，暂时无实际作用 |
| `ram` | RAM 数据区，对于 hpsys_ram 和 lpsys_ram 来说，没有额外改动的情况下可以省略 |
| `calibration` | 校准数据 |
| `raw` | 原始数据区，用于存放由应用程序自行分配的区域 |
| `int_res` | 内置资源，与 raw 不同的是，int_res 是会被链接到最终的 elf 中的，然后我们再通过段名进行切割 |

## 存储区域 (region)

### 支持的区域名称

| 区域 | 说明 |
|------|------|
| `mpix` | 第 x 个 MPI 接口对应的存储器，如 `mpi1`、`mpi2` |
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
### 特殊宏

**CODE_START 分区：**

当 `type=app` 且 `subtype=factory` 时生成：

```c
#define CODE_START_ADDR  (0x...)
#define CODE_SIZE        (0x...)
```

需要注意的是，`CODE_START_ADDR` 对应 `acc` 字段指定的执行地址。如果没有指定 `acc`，则使用存储的 CBUS 地址。

**文件系统分区：**

当 `type=data` 且 `subtype` 为 `filesystem`/`littlefs`/`fat`/`fatfs`/`flashdb` 时生成：

```c
#define FS_REGION_START_ADDR  (0x...)
#define FS_REGION_SIZE        (0x...)
#define FS_REGION_OFFSET      (0x...)
```

**FlashDB KV 分区：**

当 `type=data` 且 `subtype=flashdb_kv` 时生成 KVDB 相关宏：

```c
#define KVDB_<NAME>_REGION_OFFSET  (0x...)
#define KVDB_<NAME>_REGION_SIZE    (0x...)
```

同时自动生成 `FAL_PART_TABLE` 宏，收集所有 `flashdb_kv` 分区：

```c
#define FAL_PART_TABLE \
{ \
    {FAL_PART_MAGIC_WORD, "<name>", NOR_FLASHx_DEV_NAME, <offset>, <size>, 0}, \
    ...
}
```

其中 `NOR_FLASHx_DEV_NAME` 根据 `region` 确定：`mpi1` → `NOR_FLASH1_DEV_NAME`，`mpi2` → `NOR_FLASH2_DEV_NAME`。
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
| `link_copy.lds` | 链接脚本 |

### ftab.bin 生成

v3 格式下，`ftab.bin` 直接通过 Python 脚本生成，不再编译 ftab 子工程：

```
Generating build_xxx/ftab.bin ...
Generated ftab.bin: build_xxx/ftab.bin (11280 bytes)
```

### jinja2 链接脚本模板

在 patb v1/v2 中，链接脚本使用编译器的c风格预处理器进行生成`link_copy.lds`。而在 ptab v3 中，链接脚本片段使用 jinja2 模板生成，模板文件位于 `drivers/cmsis/<chip>` 目录下。和patb v1/v2 类似，如果需要在工程中修改链接模板，可以添加对应的`link.jinja2`文件到工程目录中。另外需要注意的是，patb v3必须使用jinja2模板，不可以再使用c风格预处理器；同样的，patb v1/v2 也不支持jinja2模板。

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
- `bootloader` 分区唯一性

## 常见问题

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
| `app_exec` | 使用 `acc: {region, offset}`（当不能 XIP 或需要固定执行地址时） |

## 变更历史

| 版本 | 日期 | 说明 |
|------|------|------|
| v3.0 | 2024-12 | 初始版本，YAML 格式，移除 ftab 子工程 |
