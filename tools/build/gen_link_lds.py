#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0

import ast
import os
import re
from typing import Any, Dict, List, Optional, Sequence, Tuple

from jinja2 import Environment, FileSystemLoader, StrictUndefined

import ptab as ptab_module


class LinkLdsError(RuntimeError):
    pass


def hex32(value: int) -> str:
    return '0x{:08X}'.format(value & 0xFFFFFFFF)


def _safe_eval_int(expr: str) -> int:
    """Evaluate a C-like integer expression safely.

    Supported:
    - hex/dec literals
    - +, -, *, /, //, %, <<, >>, |, &, ^
    - unary +/-
    - parentheses

    Note: Identifiers are not supported in this helper.
    """
    expr = (expr or '').strip()
    if not expr:
        raise LinkLdsError('Empty expression')

    # Strip outer parentheses to reduce noise like "((0x12000000))"
    while expr.startswith('(') and expr.endswith(')'):
        inner = expr[1:-1].strip()
        if not inner:
            break
        expr = inner

    tree = ast.parse(expr, mode='eval')

    def _eval(node: ast.AST) -> int:
        if isinstance(node, ast.Expression):
            return _eval(node.body)
        if isinstance(node, ast.Constant):
            if isinstance(node.value, bool):
                return int(node.value)
            if isinstance(node.value, int):
                return node.value
            raise LinkLdsError('Unsupported constant: {}'.format(type(node.value)))
        if isinstance(node, ast.UnaryOp) and isinstance(node.op, (ast.UAdd, ast.USub)):
            v = _eval(node.operand)
            return +v if isinstance(node.op, ast.UAdd) else -v
        if isinstance(node, ast.BinOp):
            left = _eval(node.left)
            right = _eval(node.right)
            op = node.op
            if isinstance(op, ast.Add):
                return left + right
            if isinstance(op, ast.Sub):
                return left - right
            if isinstance(op, ast.Mult):
                return left * right
            if isinstance(op, ast.Div):
                return int(left / right)
            if isinstance(op, ast.FloorDiv):
                return left // right
            if isinstance(op, ast.Mod):
                return left % right
            if isinstance(op, ast.LShift):
                return left << right
            if isinstance(op, ast.RShift):
                return left >> right
            if isinstance(op, ast.BitOr):
                return left | right
            if isinstance(op, ast.BitAnd):
                return left & right
            if isinstance(op, ast.BitXor):
                return left ^ right
            raise LinkLdsError('Unsupported binary operator: {}'.format(type(op)))
        raise LinkLdsError('Unsupported expression node: {}'.format(type(node)))

    return int(_eval(tree))


def _parse_define_map(header_path: str) -> Dict[str, str]:
    """Parse simple `#define NAME VALUE` lines into a raw string map.

    Only the last definition wins.
    """
    define_re = re.compile(r'^\s*#\s*define\s+([A-Za-z_]\w*)\s*(.*)$')
    out: Dict[str, str] = {}
    try:
        with open(header_path, 'r', encoding='utf-8', errors='ignore') as f:
            for line in f:
                m = define_re.match(line)
                if not m:
                    continue
                name = m.group(1)
                value = (m.group(2) or '').strip()
                # Drop trailing comments
                value = value.split('//', 1)[0].strip()
                out[name] = value
    except FileNotFoundError:
        return out
    return out


def _parse_rtconfig_defines(rtconfig_h_path: str) -> Dict[str, Any]:
    """Parse rtconfig.h into a dict usable by Jinja2.

    - `#define FOO 1` -> 1
    - `#define BAR` -> 1
    - non-numeric values kept as strings
    """
    raw = _parse_define_map(rtconfig_h_path)
    out: Dict[str, Any] = {}
    for k, v in raw.items():
        if v == '':
            out[k] = 1
            continue
        # Typical Kconfig output uses 1/0 or strings.
        try:
            out[k] = int(v, 0)
            continue
        except Exception:
            pass
        out[k] = v
    return out


def _infer_cmsis_dir_from_chip(chip: str) -> str:
    chip = (chip or '').upper()
    if chip.startswith('SF32LB52'):
        return 'sf32lb52x'
    raise LinkLdsError('Unsupported chip for Jinja2 link generation: {}'.format(chip))


def _load_mem_map_ints(sifli_sdk_root: str, cmsis_dir: str) -> Dict[str, int]:
    mem_map_path = os.path.join(sifli_sdk_root, 'drivers', 'cmsis', cmsis_dir, 'mem_map.h')
    wanted = {
        # Flash2 layout (used by HCPU link script)
        'QSPI2_MEM_BASE',
        'HCPU_FLASH2_IMG_SIZE',
        'HCPU_FLASH2_FONT_SIZE',
        # Bootloader runtime size
        'FLASH_BOOT_LOADER_SIZE',
        # Mailbox sizes
        'HPSYS_MBOX_BUF_SIZE',
        'LPSYS_MBOX_BUF_SIZE',
        # LCPU RAM view
        'LPSYS_RAM_BASE',
        'LPSYS_RAM_CBUS_BASE',
        'LPSYS_RAM_SIZE',
        'LCPU_RAM_CODE_SIZE',
    }

    raw = _parse_define_map(mem_map_path)
    out: Dict[str, int] = {}
    for name in wanted:
        if name not in raw:
            continue
        try:
            out[name] = _safe_eval_int(raw[name])
        except Exception as e:
            raise LinkLdsError('Failed to eval {} from {}: {} ({})'.format(name, mem_map_path, raw[name], e))
    return out


def _mpi_name_from_region(region: str) -> Optional[str]:
    region = (region or '').strip().lower()
    if not region:
        return None
    if region.startswith('mpi'):
        return region
    psram_match = re.match(r'^psram(\d+)?$', region)
    if psram_match:
        mpi_num = psram_match.group(1) or '1'
        return 'mpi{}'.format(mpi_num)
    return None


def _get_region_mem_type(region: str, chip_config: Dict[str, Any]) -> str:
    region = (region or '').strip().lower()
    if region == 'hpsys_ram' or region.startswith('hpsys') or region == 'lpsys_ram' or region.startswith('lpsys'):
        return 'ram'
    mpi_name = _mpi_name_from_region(region)
    if mpi_name:
        info = chip_config.get('memory_info', {}).get(mpi_name, {})
        mtype = (info.get('type') or '').lower()
        return mtype or 'nor'
    return ''


def _select_start_addr(mem_type: str, sbus_addr: int, cbus_addr: int) -> int:
    return cbus_addr if mem_type == 'nor' else sbus_addr


def _select_exec_addr(mem_type: str, sbus_addr: int, cbus_addr: int) -> int:
    return sbus_addr if mem_type in ('ram', 'nand', 'psram') else cbus_addr


def _get_hpsys_ram_hcpu(chip_config: Dict[str, Any]) -> Tuple[int, int]:
    ram = ((chip_config.get('ram') or {}).get('hpsys') or {}).get('ram') or {}
    base: Optional[int] = None
    total = 0
    for _, bank in ram.items():
        hcpu = bank.get('hcpu') or {}
        off = hcpu.get('offset')
        size = hcpu.get('size')
        if off is None or size is None:
            continue
        if base is None:
            base = int(off)
        total += int(size)
    if base is None:
        base = 0x20000000
    return base, total


def _get_lpsys_ram(chip_config: Dict[str, Any]) -> Tuple[int, int, int]:
    ram = ((chip_config.get('ram') or {}).get('lpsys') or {}).get('ram') or {}
    base_hcpu: Optional[int] = None
    base_lcpu: Optional[int] = None
    total: Optional[int] = None
    for _, bank in ram.items():
        hcpu = bank.get('hcpu') or {}
        lcpu = bank.get('lcpu') or {}
        if base_hcpu is None and hcpu.get('offset') is not None:
            base_hcpu = int(hcpu.get('offset'))
        if base_lcpu is None and lcpu.get('offset') is not None:
            base_lcpu = int(lcpu.get('offset'))
        if total is None and hcpu.get('size') is not None:
            total = int(hcpu.get('size'))
    if base_hcpu is None:
        base_hcpu = 0x20400000
    if base_lcpu is None:
        base_lcpu = 0x00400000
    if total is None:
        total = 0x6000
    return base_hcpu, base_lcpu, total


def _find_partition(partitions, *, ptype: Optional[str] = None, subtype: Optional[str] = None,
                    name: Optional[str] = None, core: Optional[str] = None) -> Optional[Dict[str, Any]]:
    for p in partitions:
        if not isinstance(p, dict):
            continue
        if ptype and p.get('type') != ptype:
            continue
        if subtype and p.get('subtype') != subtype:
            continue
        if name and (p.get('name') or '').lower() != name.lower():
            continue
        if core:
            pcore = (p.get('core') or 'HCPU').upper()
            if pcore != core.upper():
                continue
        return p
    return None


def _match_partition_by_alias(partitions, alias_base: str) -> Optional[Dict[str, Any]]:
    alias_base = (alias_base or '').strip().upper()
    if not alias_base:
        return None
    for p in partitions:
        aliases = p.get('aliases') or []
        for a in aliases:
            if str(a).strip().upper() == alias_base:
                return p
    return None


def compute_link_defines(ptab_obj, build_name: str, build_core: str, rtconfig_defines: Dict[str, Any]) -> Dict[str, Any]:
    if not ptab_obj or not hasattr(ptab_obj, 'is_v3') or not ptab_obj.is_v3():
        raise LinkLdsError('compute_link_defines only supports ptab v3')

    sifli_sdk_root = os.getenv('SIFLI_SDK') or os.getcwd()
    cmsis_dir = _infer_cmsis_dir_from_chip(getattr(ptab_obj, 'chip', ''))

    chip_config = ptab_obj.get_chip_config()
    partitions = ptab_obj.partitions

    mem_map_ints = _load_mem_map_ints(sifli_sdk_root, cmsis_dir)

    # Default RAM windows from SiliconSchema
    hpsys_base, hpsys_total = _get_hpsys_ram_hcpu(chip_config)
    lpsys_base_hcpu, lpsys_base_lcpu, lpsys_total = _get_lpsys_ram(chip_config)

    hpsys_mbox_size = int(mem_map_ints.get('HPSYS_MBOX_BUF_SIZE', 0x400))
    lpsys_mbox_size = int(mem_map_ints.get('LPSYS_MBOX_BUF_SIZE', 0x400))

    hcpu_ram_base = hpsys_base
    hcpu_ram_size = max(0, int(hpsys_total) - hpsys_mbox_size)
    hcpu_rom_ex_base = hcpu_ram_base + hcpu_ram_size

    lcpu_ram_base = lpsys_base_hcpu
    lcpu_ram_size = max(0, int(lpsys_total) - lpsys_mbox_size)
    lcpu_rom_base = lpsys_base_lcpu
    lcpu_rom_size = int(mem_map_ints.get('LCPU_RAM_CODE_SIZE', 0)) * 2

    # PSRAM window from ptab (optional)
    psram_part = (
        _match_partition_by_alias(partitions, 'PSRAM_DATA')
        or _find_partition(partitions, name='psram_data')
    )
    if psram_part:
        psram_region = psram_part.get('region', '')
        psram_offset = ptab_module.parse_size(psram_part.get('offset', 0))
        psram_size = ptab_module.parse_size(psram_part.get('size', 0))
        psram_sbus, psram_cbus = ptab_module.resolve_region_address(psram_region, psram_offset, chip_config, core=psram_part.get('core'))
        psram_mem_type = _get_region_mem_type(psram_region, chip_config)
        psram_base = _select_start_addr(psram_mem_type, psram_sbus, psram_cbus)
    else:
        psram_base = 0
        psram_size = 0

    # Pick ROM region for current build
    build_name = (build_name or '').strip().lower()
    build_core = (build_core or 'HCPU').strip().upper()

    rom_base = 0
    rom_size = 0

    if build_name == 'bootloader':
        boot_p = _find_partition(partitions, ptype='bootloader', core=build_core)
        if not boot_p:
            boot_p = _find_partition(partitions, ptype='bootloader')
        if not boot_p:
            raise LinkLdsError('bootloader partition not found in ptab v3')
        acc = boot_p.get('acc')
        if not isinstance(acc, dict):
            raise LinkLdsError('bootloader partition requires acc for link script')
        acc_region = str(acc.get('region', '')).strip()
        acc_offset = ptab_module.parse_size(acc.get('offset', 0))
        exec_sbus, exec_cbus = ptab_module.resolve_region_address(acc_region, acc_offset, chip_config, core=boot_p.get('core'))
        # NOTE: `acc` describes the execution address. Prefer CBUS/XIP view.
        rom_base = exec_cbus
        rom_size = int(mem_map_ints.get('FLASH_BOOT_LOADER_SIZE', 0))
        if rom_size <= 0:
            rom_size = ptab_module.parse_size(boot_p.get('size', 0))
    elif build_core == 'LCPU':
        rom_base = lcpu_rom_base
        rom_size = lcpu_rom_size
    else:
        # main/dfu or named images (HCPU)
        code_p = None
        if build_name == 'main':
            code_p = _find_partition(partitions, ptype='app', subtype='factory', core=build_core)
            if not code_p:
                code_p = _find_partition(partitions, ptype='app', subtype='factory')
        elif build_name == 'dfu':
            code_p = _find_partition(partitions, ptype='app', subtype='dfu', core=build_core)
            if not code_p:
                code_p = _find_partition(partitions, ptype='app', subtype='dfu')
        if not code_p and build_name:
            code_p = _find_partition(partitions, name=build_name)
        if not code_p:
            raise LinkLdsError('code partition not found for build: {}'.format(build_name))

        size = ptab_module.parse_size(code_p.get('size', 0))
        acc = code_p.get('acc')
        core = code_p.get('core')
        if isinstance(acc, dict):
            acc_region = str(acc.get('region', '')).strip()
            acc_offset = ptab_module.parse_size(acc.get('offset', 0))
            exec_sbus, exec_cbus = ptab_module.resolve_region_address(acc_region, acc_offset, chip_config, core=core)
            # NOTE: `acc` describes the execution address. Prefer CBUS/XIP view.
            rom_base = exec_cbus
        else:
            region = code_p.get('region', '')
            offset = ptab_module.parse_size(code_p.get('offset', 0))
            exec_sbus, exec_cbus = ptab_module.resolve_region_address(region, offset, chip_config, core=core)
            mem_type = _get_region_mem_type(region, chip_config)
            rom_base = _select_exec_addr(mem_type, exec_sbus, exec_cbus)
        rom_size = size

    # Pick RAM region for current build
    if build_name == 'bootloader':
        ram_part = (
            _match_partition_by_alias(partitions, 'BOOTLOADER_RAM_DATA')
            or _find_partition(partitions, name='bootloader_ram_data')
            or _find_partition(partitions, name='bootloader_ram')
        )
        if ram_part:
            region = ram_part.get('region', '')
            offset = ptab_module.parse_size(ram_part.get('offset', 0))
            size = ptab_module.parse_size(ram_part.get('size', 0))
            sbus, cbus = ptab_module.resolve_region_address(region, offset, chip_config, core=ram_part.get('core'))
            mem_type = _get_region_mem_type(region, chip_config)
            ram_base = _select_start_addr(mem_type, sbus, cbus)
            ram_size = size
        else:
            ram_base, ram_size = hcpu_ram_base, hcpu_ram_size
    elif build_core == 'LCPU':
        ram_base, ram_size = lcpu_ram_base, lcpu_ram_size
    else:
        ram_base, ram_size = hcpu_ram_base, hcpu_ram_size

    out: Dict[str, Any] = dict(rtconfig_defines or {})
    out['__ROM_BASE'] = int(rom_base)
    out['__ROM_SIZE'] = int(rom_size)
    out['__RAM_BASE'] = int(ram_base)
    out['__RAM_SIZE'] = int(ram_size)
    out['__ROM_EX_BASE'] = int(hcpu_rom_ex_base if build_core == 'HCPU' else 0)
    out['__ROM_EX_SIZE'] = 0
    out['__PSRAM_BASE'] = int(psram_base)
    out['__PSRAM_SIZE'] = int(psram_size)

    # int_res partitions: dedicated MEMORY regions and output sections
    reserved_memory_names = {'ROM', 'RAM', 'ROM_EX', 'PSRAM'}
    int_res_parts: List[Dict[str, Any]] = []
    for p in ptab_module.iter_int_res_partitions_v3(ptab_obj, core=build_core):
        name = (p.get('name') or '').strip()
        if not name:
            continue
        name_upper = name.upper()
        if name_upper in reserved_memory_names:
            raise LinkLdsError('int_res partition name conflicts with reserved MEMORY name: {}'.format(name_upper))

        region = (p.get('region') or '').strip()
        offset = ptab_module.parse_size(p.get('offset', 0))
        size = ptab_module.parse_size(p.get('size', 0))
        if size <= 0:
            continue
        base = ptab_module.get_download_addr_v3(region, offset, chip_config, core=p.get('core'))
        int_res_parts.append(
            {
                'name': name,
                'name_upper': name_upper,
                'base': int(base),
                'size': int(size),
            }
        )
    out['INT_RES_PARTS'] = int_res_parts
    return out


def render_link_lds(template_path: str, output_path: str, defines: Dict[str, Any], search_paths: Optional[Sequence[str]] = None) -> None:
    template_path = os.path.abspath(str(template_path))
    output_path = os.path.abspath(str(output_path))

    template_dir = os.path.dirname(template_path)
    sifli_sdk_root = os.getenv('SIFLI_SDK') or os.getcwd()

    loader_paths = list(search_paths or [])
    loader_paths = [os.path.abspath(p) for p in loader_paths if p]
    # Always search template dir first, then repo root.
    for p in (template_dir, sifli_sdk_root):
        ap = os.path.abspath(p)
        if ap not in loader_paths:
            loader_paths.append(ap)

    env = Environment(
        loader=FileSystemLoader(loader_paths),
        undefined=StrictUndefined,
        keep_trailing_newline=True,
        autoescape=False,
    )
    env.filters['hex32'] = hex32
    env.globals['hex32'] = hex32

    # Load by basename; includes are resolved via loader paths.
    template = env.get_template(os.path.basename(template_path))
    rendered = template.render(defines=defines)

    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    with open(output_path, 'w', encoding='utf-8', newline='\n') as f:
        f.write(rendered)
