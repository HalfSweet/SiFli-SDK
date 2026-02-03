from __future__ import annotations

import copy
import json
import logging
import os
import re
from collections import OrderedDict
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple, Union

import yaml

_PTAB_CACHE = {}

# SiliconSchema paths
_SILICON_SCHEMA_PATH = Path(__file__).parent.parent / 'SiliconSchema' / 'common'
_MPI_CONFIG_PATH = _SILICON_SCHEMA_PATH / 'mpi'
_RAM_CONFIG_PATH = _SILICON_SCHEMA_PATH / 'ram'

# Chip config cache
_CHIP_CONFIG_CACHE = {}


class Ptab:
    def __init__(self, path, mems):
        self.path = os.path.abspath(path)
        self.mems = mems
        header = mems[0] if mems else {}
        self._has_header = isinstance(header, dict) and ('version' in header)
        self._version = str(header.get('version')) if self._has_header else None

    @property
    def has_header(self):
        return self._has_header

    @property
    def header(self):
        return self.mems[0] if self._has_header else {}

    @property
    def version(self):
        return self._version if self._has_header else "1"

    def is_v2(self):
        return self.version == "2"

    def is_v3(self):
        return False

    def content_mems(self, clone=True):
        mems = self.mems[1:] if self._has_header else self.mems
        return copy.deepcopy(mems) if clone else mems

    def ensure_default_regions(self):
        mems = self.content_mems()
        if self.is_v2():
            add_default_regions(mems)
        return mems


class PtabV3:
    """ptab v3 格式解析器，使用 YAML 格式

    Notes:
    - v3 uses YAML dict with top-level keys: version/chip/memory/partitions
    - Execution address is described by `acc: {region, offset}` (preferred).
      For backward compatibility, `exec_region/exec_offset` are accepted as
      deprecated aliases and will be normalized into `acc`.
    - `aliases` is used to generate legacy macro base names and to provide
      compatibility with v1/v2 projects.
    """

    def __init__(self, path, data):
        self.path = os.path.abspath(path)
        self._data = data
        self._version = str(data.get('version', '3'))
        self._chip = data.get('chip', '')
        self._partitions = self._normalize_partitions(data.get('partitions', []))
        self._memory = data.get('memory', [])  # 外部存储配置
        self._chip_config = None

    @property
    def version(self):
        return self._version

    @property
    def chip(self):
        return self._chip

    @property
    def memory(self):
        return copy.deepcopy(self._memory)

    @property
    def chip_series(self):
        """获取芯片系列名（小写），如 'sf32lb52'"""
        if self._chip:
            # SF32LB52 -> sf32lb52, SF32LB525UC6 -> sf32lb52
            match = re.match(r'(SF32LB\d{2})', self._chip, re.IGNORECASE)
            if match:
                return match.group(1).lower()
        return None

    @property
    def partitions(self):
        return copy.deepcopy(self._partitions)

    def _normalize_partitions(self, partitions):
        """Normalize partitions to the canonical v3 shape.

        Canonical fields:
        - name/type/subtype/region/offset/size/core/attrs/aliases/acc
        """
        if not isinstance(partitions, list):
            return []

        normalized = []
        for p in partitions:
            if not isinstance(p, dict):
                continue

            name = (p.get('name') or '').strip()
            ptype = (p.get('type') or '').strip()
            subtype = (p.get('subtype') or '').strip() or None

            region = (p.get('region') or '').strip()
            if region == 'psram':
                region = 'psram1'

            offset = p.get('offset', 0)
            size = p.get('size', 0)

            core = (p.get('core') or '').strip() or None

            # attrs: keep as-is but only allow dict
            attrs = p.get('attrs')
            if not isinstance(attrs, dict):
                attrs = {}

            # aliases: list[str]
            aliases = p.get('aliases')
            if aliases is None:
                aliases_list = []
            elif isinstance(aliases, list):
                aliases_list = [str(x).strip() for x in aliases if str(x).strip()]
            else:
                aliases_list = [str(aliases).strip()] if str(aliases).strip() else []

            # acc: canonical execution address
            acc = None
            if isinstance(p.get('acc'), dict):
                acc_region = (p['acc'].get('region') or '').strip()
                if acc_region == 'psram':
                    acc_region = 'psram1'
                acc_offset = p['acc'].get('offset', 0)
                if acc_region:
                    acc = {'region': acc_region, 'offset': acc_offset}
            else:
                # Backward compatible deprecated fields
                exec_region = (p.get('exec_region') or '').strip()
                if exec_region == 'psram':
                    exec_region = 'psram1'
                if exec_region:
                    acc = {'region': exec_region, 'offset': p.get('exec_offset', 0)}

            out = {
                'name': name,
                'type': ptype,
                'region': region,
                'offset': offset,
                'size': size,
            }
            if subtype:
                out['subtype'] = subtype
            if core:
                out['core'] = core
            if attrs:
                out['attrs'] = attrs
            if aliases_list:
                out['aliases'] = aliases_list
            if acc:
                out['acc'] = acc

            normalized.append(out)

        return normalized

    def is_v2(self):
        return False

    def is_v3(self):
        return True

    def get_chip_config(self):
        """获取芯片配置（MPI + RAM + Memory）
        
        合并以下来源：
        1. 基础 MPI/RAM 配置（从 SiliconSchema 加载）
        2. 芯片型号的 SiP memory（从 chip.yaml 获取）
        3. ptab.yaml 中的 memory 字段（外部存储）
        """
        if self._chip_config is None:
            series = self.chip_series
            if series:
                self._chip_config = load_chip_config(series)
            else:
                self._chip_config = {'mpi': {}, 'ram': {}}
            
            # 合并芯片型号的 SiP memory 和 ptab.yaml 的 memory 配置
            self._chip_config = self._merge_memory_config(self._chip_config)
        return self._chip_config

    def _merge_memory_config(self, config):
        """合并 memory 配置到 chip_config
        
        将 ptab.yaml 的 memory 字段和 chip.yaml 的 variant memory 合并到 mpi 配置中。
        memory 字段用于描述每个 mpi 上挂载的存储器类型和大小。
        """
        import copy
        config = copy.deepcopy(config)
        
        # 初始化 memory_info 存储每个 mpi 的存储器信息
        if 'memory_info' not in config:
            config['memory_info'] = {}
        
        # 加载芯片型号的 SiP memory 配置
        chip_memory = self._load_chip_variant_memory()
        for mem in chip_memory:
            mpi = mem.get('mpi')
            if mpi:
                config['memory_info'][mpi] = {
                    'type': mem.get('type', 'unknown'),
                    'size': mem.get('size', 0),
                    'sip': True,
                }
        
        # 合并 ptab.yaml 中的 memory 字段（外部存储，覆盖 SiP）
        for mem in self._memory:
            mpi = mem.get('mpi')
            if mpi:
                config['memory_info'][mpi] = {
                    'type': mem.get('type', 'unknown'),
                    'size': parse_size(mem.get('size', 0)),
                    'sip': False,
                }
        
        return config

    def _load_chip_variant_memory(self):
        """从 chip.yaml 加载精确芯片型号的 memory 配置"""
        if not self._chip:
            return []
        
        # 查找芯片定义文件
        series = self.chip_series
        if not series:
            return []

        chips_root = Path(__file__).parent.parent / 'SiliconSchema' / 'chips'

        # Known naming patterns in this repo:
        # - SF32LB52x/chip.yaml
        # - SF32LB52_X/chip.yaml
        preferred_dirs = [
            chips_root / f'{series.upper()}x',
            chips_root / f'{series.upper()}_X',
        ]

        # Also scan any matching directories for robustness
        scan_dirs = []
        try:
            for d in chips_root.iterdir():
                if not d.is_dir():
                    continue
                if d.name.upper().startswith(series.upper()):
                    scan_dirs.append(d)
        except Exception:
            scan_dirs = []

        checked = []
        for d in preferred_dirs + scan_dirs:
            chip_file = d / 'chip.yaml'
            if not chip_file.exists():
                continue
            if chip_file in checked:
                continue
            checked.append(chip_file)

            try:
                with open(chip_file) as f:
                    chip_data = yaml.safe_load(f)
            except Exception:
                continue

            variants = chip_data.get('variants', [])
            for variant in variants:
                if variant.get('part_number', '').upper() == self._chip.upper():
                    return variant.get('memory', []) or []

        return []

    def content_mems(self, clone=True):
        """转换为 v1/v2 兼容的 mems 格式，用于向后兼容"""
        return self._convert_to_mems(clone)

    def _convert_to_mems(self, clone=True):
        """将 v3 partitions 转换为 v2 mems 格式"""
        chip_config = self.get_chip_config()
        mems_dict = OrderedDict()

        for partition in self._partitions:
            region = partition.get('region', '')
            if not region:
                continue

            # 获取 region 基地址
            core = partition.get('core')
            sbus_base, cbus_base = resolve_region_address(region, 0, chip_config, core=core)

            # Select a "download/storage" base address to mimic v1/v2 behaviour
            mem_type = _get_region_memory_type(region, chip_config).lower()
            if mem_type == 'nor':
                mem_base = cbus_base
            else:
                mem_base = sbus_base

            mem_name = region

            if mem_name not in mems_dict:
                mems_dict[mem_name] = {
                    'mem': mem_name,
                    'base': '0x{:08X}'.format(mem_base),
                    'regions': []
                }

            # 转换分区
            offset = parse_size(partition.get('offset', 0))
            size = parse_size(partition.get('size', 0))
            name = partition.get('name', '')
            ptype = partition.get('type', '')
            subtype = partition.get('subtype', '')

            # Collect tags for v1/v2 compatibility.
            tags = []
            for alias in partition.get('aliases', []) or []:
                alias = str(alias).strip()
                if alias:
                    tags.append(alias)

            # Always include NAME as a tag for convenience.
            if name:
                tags.append(name.upper())

            # Auto-add FS_REGION tag for filesystem partitions (used by some builders)
            if ptype == 'data' and subtype in ('littlefs', 'fat', 'fatfs', 'flashdb', 'filesystem'):
                tags.append('FS_REGION')

            region_entry = {
                'offset': '0x{:08X}'.format(offset),
                'max_size': '0x{:08X}'.format(size),
                'tags': tags,
                'name': name,
            }

            # 构建 type 列表
            type_list = []
            if ptype in ('bootloader', 'app'):
                type_list.append('app_img')

            acc = partition.get('acc')
            if ptype == 'app' and not acc:
                # XIP case: execution region is the storage region itself
                type_list.append('app_exec')
            if type_list:
                region_entry['type'] = type_list

            if partition.get('core'):
                region_entry['core'] = partition['core']

            mems_dict[mem_name]['regions'].append(region_entry)

            # If `acc` is present and differs from storage, add an exec-only region.
            if acc and isinstance(acc, dict):
                exec_region = (acc.get('region') or '').strip()
                if exec_region == 'psram':
                    exec_region = 'psram1'
                if exec_region and exec_region != region:
                    exec_offset = parse_size(acc.get('offset', 0))
                    exec_sbus_base, exec_cbus_base = resolve_region_address(exec_region, 0, chip_config, core=core)

                    exec_mem_type = _get_region_memory_type(exec_region, chip_config).lower()
                    if exec_mem_type == 'nor':
                        exec_mem_base = exec_cbus_base
                    else:
                        exec_mem_base = exec_sbus_base

                    if exec_region not in mems_dict:
                        mems_dict[exec_region] = {
                            'mem': exec_region,
                            'base': '0x{:08X}'.format(exec_mem_base),
                            'regions': []
                        }

                    exec_tags = []
                    if ptype == 'bootloader':
                        # For v1/v2 compatibility, bootloader exec region is tagged.
                        exec_tags.append('FLASH_BOOT_LOADER')

                    exec_entry = {
                        'offset': '0x{:08X}'.format(exec_offset),
                        'max_size': '0x{:08X}'.format(size),
                        'tags': exec_tags,
                        'name': name,
                        'type': ['app_exec']
                    }
                    if partition.get('core'):
                        exec_entry['core'] = partition['core']
                    mems_dict[exec_region]['regions'].append(exec_entry)

        mems = list(mems_dict.values())
        return copy.deepcopy(mems) if clone else mems

    def ensure_default_regions(self):
        return self.content_mems()


# ============================================================================
# Size parsing utilities
# ============================================================================

_SIZE_UNITS = {
    'kb': 1024,
    'k': 1024,
    'mb': 1024 * 1024,
    'm': 1024 * 1024,
    'gb': 1024 * 1024 * 1024,
    'g': 1024 * 1024 * 1024,
}


def parse_size(value: Union[int, str]) -> int:
    """解析大小值，支持多种格式

    支持格式：
    - 整数: 4096
    - 十六进制字符串: "0x1000"
    - 带单位字符串: "4KB", "4kb", "4K", "2MB"

    Returns:
        int: 字节数
    """
    if isinstance(value, int):
        return value
    if not isinstance(value, str):
        return int(value)

    value = value.strip()
    if not value:
        return 0

    # 十六进制
    if value.lower().startswith('0x'):
        return int(value, 16)

    # 带单位
    match = re.match(r'^(\d+)\s*([a-zA-Z]+)$', value)
    if match:
        num = int(match.group(1))
        unit = match.group(2).lower()
        if unit in _SIZE_UNITS:
            return num * _SIZE_UNITS[unit]
        raise ValueError(f"Unknown size unit: {match.group(2)}")

    # 纯数字
    return int(value)


# ============================================================================
# Chip configuration loading
# ============================================================================

def load_chip_config(chip_series: str) -> Dict[str, Any]:
    """加载芯片 MPI + RAM 配置

    Args:
        chip_series: 芯片系列名，如 'sf32lb52', 'sf32lb56', 'sf32lb58'

    Returns:
        dict: {'mpi': {...}, 'ram': {...}}
    """
    if chip_series in _CHIP_CONFIG_CACHE:
        return _CHIP_CONFIG_CACHE[chip_series]

    config = {'mpi': {}, 'ram': {}}

    # 加载 MPI 配置
    mpi_file = _MPI_CONFIG_PATH / chip_series / 'mpi.yaml'
    if mpi_file.exists():
        with open(mpi_file) as f:
            mpi_data = yaml.safe_load(f)
            config['mpi'] = mpi_data.get('mpis', {})

    # 加载 RAM 配置
    ram_file = _RAM_CONFIG_PATH / chip_series / 'ram.yaml'
    if ram_file.exists():
        with open(ram_file) as f:
            config['ram'] = yaml.safe_load(f)

    _CHIP_CONFIG_CACHE[chip_series] = config
    return config


def resolve_region_address(
    region: str,
    offset: int,
    chip_config: Dict[str, Any],
    core: Optional[str] = None,
) -> Tuple[int, int]:
    """解析 region 到物理地址

    Args:
        region: region 名称，如 'mpi1', 'mpi2', 'hpsys_ram'
        offset: 偏移量
        chip_config: load_chip_config() 返回的配置

    Returns:
        tuple: (sbus_addr, cbus_addr)
    """
    mpi_config = chip_config.get('mpi', {})
    ram_config = chip_config.get('ram', {})

    core_key = None
    if core:
        core_lower = str(core).lower()
        if core_lower in ('hcpu', 'lcpu', 'acpu'):
            core_key = core_lower

    # MPI region
    if region in mpi_config:
        mpi = mpi_config[region]
        base_offset = mpi.get('base', {}).get('offset', 0)
        sbus_addr = base_offset + offset
        if 'xip' in mpi:
            xip_offset = mpi['xip'].get('offset', 0)
            cbus_addr = xip_offset + offset
        else:
            cbus_addr = sbus_addr
        return sbus_addr, cbus_addr

    # RAM region (hpsys_ram -> hpsys.ram)
    if region == 'hpsys_ram' or region.startswith('hpsys'):
        hpsys = ram_config.get('hpsys', {})
        ram = hpsys.get('ram', {})
        # 使用第一个 ram bank 的 offset 作为基地址（按 core 选择）
        for _, bank in ram.items():
            if core_key and core_key in bank:
                base_offset = bank.get(core_key, {}).get('offset')
                if base_offset is not None:
                    return base_offset + offset, base_offset + offset
            hcpu = bank.get('hcpu', {})
            base_offset = hcpu.get('offset', 0x20000000)
            return base_offset + offset, base_offset + offset
        # 默认值
        return 0x20000000 + offset, 0x20000000 + offset

    if region == 'lpsys_ram' or region.startswith('lpsys'):
        lpsys = ram_config.get('lpsys', {})
        ram = lpsys.get('ram', {})
        for _, bank in ram.items():
            if core_key and core_key in bank:
                base_offset = bank.get(core_key, {}).get('offset')
                if base_offset is not None:
                    return base_offset + offset, base_offset + offset
            hcpu = bank.get('hcpu', {})
            base_offset = hcpu.get('offset', 0x20400000)
            return base_offset + offset, base_offset + offset
        return 0x20400000 + offset, 0x20400000 + offset

    # psram 别名处理：psram{x} 直接映射到 mpi{x}
    # 用于 exec 的地址使用 xip 地址
    psram_match = re.match(r'^psram(\d+)?$', region)
    if psram_match:
        # psram1 -> mpi1, psram2 -> mpi2, psram -> mpi1
        mpi_num = psram_match.group(1) or '1'
        mpi_name = f'mpi{mpi_num}'
        if mpi_name in mpi_config:
            mpi = mpi_config[mpi_name]
            base_offset = mpi.get('base', {}).get('offset', 0)
            sbus_addr = base_offset + offset
            if 'xip' in mpi:
                cbus_addr = mpi['xip'].get('offset', 0) + offset
            else:
                cbus_addr = sbus_addr
            return sbus_addr, cbus_addr

    logging.warning(f"Unknown region: {region}, using offset as address")
    return offset, offset


def _get_region_memory_type(region: str, chip_config: Dict[str, Any]) -> str:
    """Get memory type for a region.

    For mpi/psram regions, consult chip_config['memory_info'] by canonical mpi name.
    For RAM regions, returns 'ram'.
    """
    if region == 'hpsys_ram' or region.startswith('hpsys') or region == 'lpsys_ram' or region.startswith('lpsys'):
        return 'ram'

    # Normalize psram aliases to mpiN
    psram_match = re.match(r'^psram(\d+)?$', region)
    if psram_match:
        mpi_num = psram_match.group(1) or '1'
        region = f'mpi{mpi_num}'

    memory_info = chip_config.get('memory_info', {})
    info = memory_info.get(region, {})
    mtype = info.get('type')
    if mtype is not None:
        return str(mtype)

    # Default heuristic: when memory type is not specified for an mpi region,
    # treat it as NOR for backward compatibility (v1/v2 commonly use XIP
    # address as the "base" for flash regions).
    if re.match(r'^mpi\d+$', region):
        return 'nor'

    return ''


def _parse_ptab_file(path):
    """解析 ptab 文件（JSON 格式）"""
    with open(path) as f:
        return json.load(f, object_pairs_hook=OrderedDict)


def _parse_ptab_yaml_file(path):
    """解析 ptab 文件（YAML 格式）"""
    with open(path) as f:
        return yaml.safe_load(f)


def _is_yaml_file(path):
    """检查是否为 YAML 文件"""
    ext = os.path.splitext(path)[1].lower()
    return ext in ('.yaml', '.yml')


def _is_v3_yaml(data):
    """检查是否为 v3 格式的 YAML"""
    if isinstance(data, dict):
        version = data.get('version')
        if version is not None and str(version) == '3':
            return True
        # 如果有 partitions 字段，也认为是 v3
        if 'partitions' in data:
            return True
    return False


def load_ptab(path, fatal=False):
    """加载 ptab 文件

    支持格式：
    - v1/v2: JSON 格式的 ptab.json
    - v3: YAML 格式的 ptab.yaml

    Args:
        path: ptab 文件路径
        fatal: 解析错误时是否终止程序

    Returns:
        Ptab 或 PtabV3 对象
    """
    abspath = os.path.abspath(str(path))

    try:
        stat = os.stat(abspath)
        key = (abspath, stat.st_mtime_ns, stat.st_size)
        if key in _PTAB_CACHE:
            return _PTAB_CACHE[key]

        if _is_yaml_file(abspath):
            # YAML 格式
            data = _parse_ptab_yaml_file(abspath)
            if _is_v3_yaml(data):
                ptab_obj = PtabV3(abspath, data)
            else:
                # YAML 但不是 v3，转换为 v1/v2 兼容格式
                if isinstance(data, list):
                    ptab_obj = Ptab(abspath, data)
                else:
                    raise ValueError("Invalid YAML ptab format: expected dict with 'partitions' or list")
        else:
            # JSON 格式 (v1/v2)
            mems = _parse_ptab_file(abspath)
            ptab_obj = Ptab(abspath, mems)

    except (ValueError, yaml.YAMLError) as e:
        if fatal:
            if _is_yaml_file(abspath):
                print("ptab.yaml syntax error")
            else:
                print("ptab.json syntax error, might be caused by trailing comma of last item")
            print("Error message: {}".format(e))
            print("Please check file {}".format(abspath))
            raise SystemExit(1)
        raise

    _PTAB_CACHE[key] = ptab_obj
    return ptab_obj


def _get_depend(name):
    import building
    return building.GetDepend(name)


def convert_to_cbus_addr(addr, offset, core=None):
    if _get_depend("SOC_SF32LB55X"):
        return addr, offset
    if _get_depend("SOC_SF32LB56X"):
        if (addr >= 0x60000000) and (addr <= 0x6FFFFFFF):
            return addr - 0x50000000, offset
        return addr, offset
    if _get_depend("SOC_SF32LB58X"):
        cbus_addr = addr
        cbus_offset = offset
        if (addr >= 0x60000000) and (addr <= 0x6FFFFFFF):
            cbus_addr -= 0x50000000
        elif (addr >= 0x20000000) and (addr <= 0x2FFFFFFF) and core and core.lower() == "acpu":
            cbus_addr -= 0x20200000
            assert cbus_addr >= 0, "0x{:8X} is not a valid address for ACPU"
            cbus_offset -= 0x00200000
        return cbus_addr, cbus_offset
    if _get_depend("SOC_SF32LB52X"):
        if (addr >= 0x60000000) and (addr <= 0x6FFFFFFF):
            return addr - 0x50000000, offset
        return addr, offset
    raise Exception("unknown chip")


def add_default_regions(mems):
    if _get_depend("SOC_SF32LB55X"):
        _add_default_regions_55x(mems)
    elif _get_depend("SOC_SF32LB56X"):
        _add_default_regions_56x(mems)
    elif _get_depend("SOC_SF32LB58X"):
        _add_default_regions_58x(mems)
    elif _get_depend("SOC_SF32LB52X"):
        _add_default_regions_52x(mems)
    else:
        raise Exception("unknown chip")


def _add_default_regions_55x(mems):
    ftab_found = False
    flash1_mem = None
    for mem in mems:
        if 'flash1' == mem['mem']:
            flash1_mem = mem
        if "regions" not in mem:
            continue
        for region in mem['regions']:
            if "name" in region and 'ftab' == region['name']:
                ftab_found = True
        if ftab_found:
            break

    if not ftab_found:
        if not flash1_mem:
            flash1_mem = {
                "mem": "flash1",
                "base": "0x10000000",
                "regions": []
            }
            mems.insert(0, flash1_mem)

        if 'regions' not in flash1_mem:
            flash1_mem['regions'] = []

        ftab_region = {
            "offset": "0x00000000",
            "max_size": "0x00005000",
            "tags": [
                "FLASH_TABLE"
            ],
            "name": "ftab",
            "type": ["app_img", "app_exec"]
        }
        flash1_mem['regions'].insert(0, ftab_region)


def _add_default_regions_56x(mems):
    ftab_found = False
    bootloader_found = False
    flash5_mem = None
    for mem in mems:
        if 'flash5' == mem['mem']:
            flash5_mem = mem
        if "regions" not in mem:
            continue
        for region in mem['regions']:
            if "name" in region and 'ftab' == region['name']:
                ftab_found = True
            if "name" in region and 'bootloader' == region['name']:
                bootloader_found = True

        if ftab_found and bootloader_found:
            break

    if (not ftab_found) or (not bootloader_found):
        if not flash5_mem:
            flash5_mem = {
                "mem": "flash5",
                "base": "0x1C000000",
                "regions": []
            }
            mems.insert(0, flash5_mem)

        if 'regions' not in flash5_mem:
            flash5_mem['regions'] = []

        if not ftab_found:
            ftab_region = {
                "offset": "0x00000000",
                "max_size": "0x00004000",
                "tags": [
                    "FLASH_TABLE"
                ],
                "name": "ftab",
                "type": ["app_img", "app_exec"]
            }
            flash5_mem['regions'].insert(0, ftab_region)

        if not bootloader_found:
            bootloader_region = {
                "offset": "0x00020000",
                "max_size": "0x0000C000",
                "tags": [
                    "FLASH_BOOT_LOADER"
                ],
                "name": "bootloader",
                "type": ["app_img", "app_exec"]
            }
            flash5_mem['regions'].insert(0, bootloader_region)


def _add_default_regions_58x(mems):
    ftab_found = False
    bootloader_found = False
    flash5_mem = None
    for mem in mems:
        if 'flash5' == mem['mem']:
            flash5_mem = mem
        if "regions" not in mem:
            continue
        for region in mem['regions']:
            if "name" in region and 'ftab' == region['name']:
                ftab_found = True
            if "name" in region and 'bootloader' == region['name']:
                bootloader_found = True

        if ftab_found and bootloader_found:
            break

    if (not ftab_found) or (not bootloader_found):
        if not flash5_mem:
            flash5_mem = {
                "mem": "flash5",
                "base": "0x1C000000",
                "regions": []
            }
            mems.insert(0, flash5_mem)

        if 'regions' not in flash5_mem:
            flash5_mem['regions'] = []

        if not ftab_found:
            ftab_region = {
                "offset": "0x00000000",
                "max_size": "0x00005000",
                "tags": [
                    "FLASH_TABLE"
                ],
                "name": "ftab",
                "type": ["app_img", "app_exec"]
            }
            flash5_mem['regions'].insert(0, ftab_region)

        if not bootloader_found:
            bootloader_region = {
                "offset": "0x00020000",
                "max_size": "0x00020000",
                "tags": [
                    "FLASH_BOOT_LOADER"
                ],
                "name": "bootloader",
                "type": ["app_img", "app_exec"]
            }
            flash5_mem['regions'].insert(0, bootloader_region)


def _add_default_regions_52x(mems):
    ftab_found = False
    bootloader_exec_found = False
    bootloader_img_found = False
    bootloader_data_found = False
    boot_mem = None
    hpsys_ram_mem = None
    boot_dev_type = None

    for mem in mems:
        # guess boot_dev_type and boot_mem by memory name and address
        if "flash1" == mem['mem']:
            boot_mem = mem
            boot_dev_type = "nor"
        elif "flash2" == mem['mem']:
            boot_mem = mem
            if "0x12000000" == mem['base']:
                boot_dev_type = "nor"
            else:
                boot_dev_type = "nand"
        elif "sd1" == mem['mem']:
            boot_mem = mem
            boot_dev_type = 'sd'

        if "hpsys_ram" == mem['mem']:
            hpsys_ram_mem = mem
            continue

        for region in mem['regions']:
            if "name" in region and 'ftab' == region['name']:
                ftab_found = True
            if "name" in region and 'bootloader' == region['name']:
                bootloader_img_found = True

    for region in hpsys_ram_mem:
        if "name" in region and 'bootloader' == region['name'] and 'type' in region and "app_exec" in region['type']:
            bootloader_exec_found = True

        if "name" in region and 'bootloader' == region['name']:
            bootloader_data_found = True

    if not bootloader_exec_found:
        bootloader_region = {
            "offset": "0x00020000",
            "max_size": "0x00010000",
            "name": "bootloader",
            "type": ["app_exec"],
            "tags": ["FLASH_BOOT_LOADER"]
        }
        hpsys_ram_mem["regions"].insert(0, bootloader_region)

    if not bootloader_data_found:
        bootloader_region = {
            "offset": "0x00040000",
            "max_size": "0x00010000",
            "tags": ["BOOTLOADER_RAM_DATA"]
        }
        hpsys_ram_mem['regions'].insert(0, bootloader_region)

    if (not ftab_found) or (not bootloader_img_found):
        if not ftab_found:
            if "sd" == boot_dev_type:
                # MBR uses first 4096 bytes
                ftab_region = {
                    "offset": "0x00001000",
                    "max_size": "0x00008000",
                    "tags": ["FLASH_TABLE"],
                    "name": "ftab",
                    "type": ["app_img", "app_exec"]
                }
            else:
                ftab_region = {
                    "offset": "0x00000000",
                    "max_size": "0x00008000",
                    "tags": ["FLASH_TABLE"],
                    "name": "ftab",
                    "type": ["app_img", "app_exec"]
                }

            boot_mem['regions'].insert(0, ftab_region)

        if not bootloader_img_found:
            if "sd" == boot_dev_type:
                bootloader_region = {
                    "offset": "0x00011000",
                    "max_size": "0x00010000",
                    "tags": [],
                    "name": "bootloader",
                    "type": ["app_img"]
                }
            elif "nor" == boot_dev_type:
                bootloader_region = {
                    "offset": "0x00010000",
                    "max_size": "0x00010000",
                    "tags": [],
                    "name": "bootloader",
                    "type": ["app_img"]
                }
            elif "nand" == boot_dev_type:
                bootloader_region = {
                    "offset": "0x00080000",
                    "max_size": "0x00010000",
                    "tags": [],
                    "name": "bootloader",
                    "type": ["app_img"]
                }
            else:
                raise Exception(f"unknown type {boot_dev_type}")
            boot_mem['regions'].insert(0, bootloader_region)


def _get_ptab_path(env):
    return env.get('PARTITION_TABLE')


def get_ptab(env):
    path = _get_ptab_path(env)
    if not path:
        return None
    return load_ptab(path, fatal=True)


def _ptab_header_build(target, source, env):
    import resource
    src_file = str(source[0])
    target_file = str(target[0])
    output_dir = os.path.dirname(target_file)
    output_name = os.path.splitext(os.path.basename(target_file))[0]
    resource.GenPartitionTableHeaderFile(src_file, output_dir, output_name, env=env)


def generate(env):
    from SCons.Script import Builder
    from SCons.Action import Action

    env.AddMethod(get_ptab, "GetPtab")
    env.AddMethod(_get_ptab_path, "GetPtabPath")

    action = Action(_ptab_header_build, 'Generating $TARGET ...')
    bld = Builder(action=action)
    env.Append(BUILDERS={"PtabHeader": bld})


def exists(env):
    return True
