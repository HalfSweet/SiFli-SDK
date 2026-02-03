#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0

"""
ptab v3 regression (offline, strict numeric compare)

This script compares key artifacts produced by legacy ptab.json (v1/v2) build
artifacts vs regenerated outputs from migrated ptab.yaml (v3):
- ptab.h (numeric macro values)
- link_copy.lds (numeric values of __ROM_BASE/__RAM_BASE/... assignments)
- ftab.bin (partition entries: base/size/xip_base/flags)

It does NOT require running full SCons builds (use prebuilt baseline artifacts).

Example:
  python3 tools/build/ptab_v3_regression.py --board sf32lb52-nano_52j --chip SF32LB52JUD6
"""

import argparse
import ast
import os
import re
import struct
import sys
import tempfile
import types
from typing import Any, Dict, List, Optional, Tuple


SEC_CONFIG_MAGIC = 0x53454346  # 'FCES'
PARTITION_ENTRY_COUNT = 16
PARTITION_ENTRY_STRUCT = struct.Struct('<IIII')


def _repo_root() -> str:
    sifli = os.getenv('SIFLI_SDK')
    if sifli:
        return os.path.abspath(sifli)
    return os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(allow_abbrev=False)
    parser.add_argument('--board', default='sf32lb52-nano_52j', help='Board name (without _hcpu suffix)')
    parser.add_argument('--chip', default='SF32LB52JUD6', help='Chip part number used for migrated ptab v3')
    parser.add_argument(
        '--baseline-build-dir',
        default=None,
        help='Baseline HCPU build dir (default: example/misc/button/project/build_<board>_hcpu)',
    )
    parser.add_argument('--keep-tmp', action='store_true', help='Keep generated files (print tmp dir)')
    return parser.parse_args()


class _EvalError(RuntimeError):
    pass


def _safe_eval_int(expr: str) -> int:
    expr = (expr or '').strip()
    if not expr:
        raise _EvalError('Empty expression')

    # Strip trailing semicolons and outer parentheses
    expr = expr.rstrip(';').strip()
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
            raise _EvalError('Unsupported constant: {}'.format(type(node.value)))
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
            raise _EvalError('Unsupported op: {}'.format(type(op)))
        raise _EvalError('Unsupported node: {}'.format(type(node)))

    return int(_eval(tree))


def _parse_c_defines(path: str) -> Dict[str, str]:
    define_re = re.compile(r'^\s*#\s*define\s+([A-Za-z_]\w*)\s+(.*)$')
    out: Dict[str, str] = {}
    with open(path, 'r', encoding='utf-8', errors='ignore') as f:
        for line in f:
            m = define_re.match(line)
            if not m:
                continue
            name = m.group(1)
            expr = (m.group(2) or '').strip()
            expr = expr.split('//', 1)[0].strip()
            out[name] = expr
    return out


def _eval_define(name: str, defs: Dict[str, str], cache: Dict[str, int]) -> int:
    if name in cache:
        return cache[name]
    if name not in defs:
        raise _EvalError('Unknown macro: {}'.format(name))
    expr = defs[name]

    # Simple reference case
    ref_match = re.fullmatch(r'\(?\s*([A-Za-z_]\w*)\s*\)?', expr)
    if ref_match and ref_match.group(1) in defs:
        v = _eval_define(ref_match.group(1), defs, cache)
        cache[name] = v
        return v

    # Replace identifiers with evaluated values
    tokens = set(re.findall(r'\b[A-Za-z_]\w*\b', expr))
    expanded = expr
    for tok in sorted(tokens, key=len, reverse=True):
        if tok not in defs:
            continue
        v = _eval_define(tok, defs, cache)
        expanded = re.sub(r'\b{}\b'.format(re.escape(tok)), str(v), expanded)

    v2 = _safe_eval_int(expanded)
    cache[name] = v2
    return v2


def _collect_macro_values(path: str) -> Dict[str, int]:
    defs = _parse_c_defines(path)
    cache: Dict[str, int] = {}
    out: Dict[str, int] = {}
    for k in defs.keys():
        try:
            out[k] = _eval_define(k, defs, cache)
        except Exception:
            # Skip non-numeric macros
            continue
    return out


def _parse_lds_assignments(path: str) -> Dict[str, int]:
    assign_re = re.compile(r'^\s*(__[A-Za-z0-9_]+)\s*=\s*(.+?);')
    out: Dict[str, int] = {}
    with open(path, 'r', encoding='utf-8', errors='ignore') as f:
        for line in f:
            m = assign_re.match(line)
            if not m:
                # stop early after header constants
                if out and line.strip().startswith('MEMORY'):
                    break
                continue
            name = m.group(1)
            expr = m.group(2).strip()
            try:
                out[name] = _safe_eval_int(expr)
            except Exception:
                continue
    return out


def _parse_ftab_partitions(path: str) -> List[Tuple[int, int, int, int]]:
    with open(path, 'rb') as f:
        data = f.read()
    if len(data) < 4 + PARTITION_ENTRY_STRUCT.size * PARTITION_ENTRY_COUNT:
        raise _EvalError('ftab.bin too small: {}'.format(len(data)))
    magic = struct.unpack_from('<I', data, 0)[0]
    if magic != SEC_CONFIG_MAGIC:
        raise _EvalError('Invalid ftab magic: 0x{:08X}'.format(magic))
    off = 4
    out: List[Tuple[int, int, int, int]] = []
    for _ in range(PARTITION_ENTRY_COUNT):
        base, size, xip_base, flags = PARTITION_ENTRY_STRUCT.unpack_from(data, off)
        out.append((base, size, xip_base, flags))
        off += PARTITION_ENTRY_STRUCT.size
    return out


def _write_generated_ptab_h(ptab_obj, env_name: str, core: str, out_path: str) -> None:
    root = _repo_root()
    sys.path.insert(0, os.path.join(root, 'tools', 'build'))
    # Inject rtconfig for GenPartitionTableHeaderContentV3
    rt = types.ModuleType('rtconfig')
    rt.CORE = core
    sys.modules['rtconfig'] = rt

    import resource  # type: ignore

    resource.InitIndentation()
    env = {'name': env_name}
    s = ''
    s += resource.MakeLine('#ifndef __PTAB__H__')
    s += resource.MakeLine('#define __PTAB__H__')
    s += resource.MakeLine('')
    s += resource.MakeLine('')
    s += resource.GenPartitionTableHeaderContentV3(env, ptab_obj)
    s += resource.MakeLine('')
    s += resource.MakeLine('')
    s += resource.MakeLine('')
    s += resource.MakeLine('#endif')

    with open(out_path, 'w', encoding='utf-8', newline='\n') as f:
        f.write(s)


def main() -> int:
    args = _parse_args()
    root = _repo_root()

    baseline_dir = args.baseline_build_dir
    if not baseline_dir:
        baseline_dir = os.path.join(root, 'example', 'misc', 'button', 'project', 'build_{}_hcpu'.format(args.board))
    baseline_dir = os.path.abspath(baseline_dir)

    baseline_main_ptab_h = os.path.join(baseline_dir, 'ptab.h')
    baseline_main_link = os.path.join(baseline_dir, 'link_copy.lds')
    baseline_ftab = os.path.join(baseline_dir, 'ftab', 'ftab.bin')
    baseline_main_bin = os.path.join(baseline_dir, 'main.bin')

    baseline_boot_dir = os.path.join(baseline_dir, 'bootloader')
    baseline_boot_ptab_h = os.path.join(baseline_boot_dir, 'ptab.h')
    baseline_boot_link = os.path.join(baseline_boot_dir, 'link_copy.lds')
    baseline_boot_bin = os.path.join(baseline_boot_dir, 'bootloader.bin')

    ptab_json = os.path.join(root, 'customer', 'boards', args.board, 'ptab.json')
    if not os.path.exists(ptab_json):
        print('ERROR: baseline ptab.json not found: {}'.format(ptab_json), file=sys.stderr)
        return 2

    # Prepare python imports from tools/build
    sys.path.insert(0, os.path.join(root, 'tools', 'build'))
    import ptab as ptab_module  # type: ignore
    import gen_ftab  # type: ignore
    import gen_link_lds  # type: ignore
    import migrate_ptab_to_v3  # type: ignore

    # Migrate to v3 YAML into tmp dir
    tmp_dir = tempfile.mkdtemp(prefix='ptab_v3_reg_')
    v3_yaml_path = os.path.join(tmp_dir, 'ptab.yaml')
    v3_dict = migrate_ptab_to_v3.migrate_ptab_json_to_v3(migrate_ptab_to_v3._load_ptab_json(ptab_json), args.chip)
    with open(v3_yaml_path, 'w', encoding='utf-8', newline='\n') as f:
        yaml_text = yaml_dump(v3_dict)
        f.write(yaml_text)

    ptab_obj = ptab_module.load_ptab(v3_yaml_path, fatal=True)

    # Generate v3 ptab.h (main + bootloader)
    gen_main_ptab_h = os.path.join(tmp_dir, 'ptab_main.h')
    gen_boot_ptab_h = os.path.join(tmp_dir, 'ptab_boot.h')
    _write_generated_ptab_h(ptab_obj, 'main', 'HCPU', gen_main_ptab_h)
    _write_generated_ptab_h(ptab_obj, 'bootloader', 'HCPU', gen_boot_ptab_h)

    # Compare ptab.h (numeric)
    baseline_main_macros = _collect_macro_values(baseline_main_ptab_h)
    gen_main_macros = _collect_macro_values(gen_main_ptab_h)
    baseline_boot_macros = _collect_macro_values(baseline_boot_ptab_h)
    gen_boot_macros = _collect_macro_values(gen_boot_ptab_h)

    key_macros = [
        'FLASH_TABLE_START_ADDR', 'FLASH_TABLE_SIZE', 'FLASH_TABLE_OFFSET',
        'HCPU_FLASH_CODE_START_ADDR', 'HCPU_FLASH_CODE_SIZE', 'HCPU_FLASH_CODE_OFFSET',
        'DFU_FLASH_CODE_START_ADDR', 'DFU_FLASH_CODE_SIZE', 'DFU_FLASH_CODE_OFFSET',
        'DFU_DOWNLOAD_REGION_START_ADDR', 'DFU_DOWNLOAD_REGION_SIZE', 'DFU_DOWNLOAD_REGION_OFFSET',
        'FS_REGION_START_ADDR', 'FS_REGION_SIZE', 'FS_REGION_OFFSET',
        'KVDB_DFU_REGION_START_ADDR', 'KVDB_DFU_REGION_SIZE', 'KVDB_DFU_REGION_OFFSET',
        'KVDB_BLE_REGION_START_ADDR', 'KVDB_BLE_REGION_SIZE', 'KVDB_BLE_REGION_OFFSET',
        'PSRAM_DATA_START_ADDR', 'PSRAM_DATA_SIZE', 'PSRAM_DATA_OFFSET',
        'FLASH_BOOT_LOADER_START_ADDR', 'FLASH_BOOT_LOADER_SIZE', 'FLASH_BOOT_LOADER_OFFSET',
        'BOOTLOADER_RAM_DATA_START_ADDR', 'BOOTLOADER_RAM_DATA_SIZE', 'BOOTLOADER_RAM_DATA_OFFSET',
        'CODE_START_ADDR', 'CODE_SIZE',
    ]

    def _check_macros(label: str, base: Dict[str, int], new: Dict[str, int]) -> None:
        for k in key_macros:
            if k not in base:
                continue
            if k not in new:
                raise _EvalError('{} missing macro: {}'.format(label, k))
            if int(base[k]) != int(new[k]):
                raise _EvalError('{} macro mismatch {}: 0x{:08X} != 0x{:08X}'.format(label, k, int(base[k]) & 0xFFFFFFFF, int(new[k]) & 0xFFFFFFFF))

    _check_macros('main', baseline_main_macros, gen_main_macros)
    _check_macros('bootloader', baseline_boot_macros, gen_boot_macros)

    # Generate v3 link_copy.lds (main + bootloader)
    hcpu_template = os.path.join(root, 'drivers', 'cmsis', 'sf32lb52x', 'Templates', 'gcc', 'hcpu', 'link.jinja2')
    boot_template = os.path.join(root, 'example', 'boot_loader', 'project', 'butterflmicro', 'ram_v2', 'link.jinja2')

    main_rtconfig_h = os.path.join(baseline_dir, 'rtconfig.h')
    boot_rtconfig_h = os.path.join(baseline_boot_dir, 'rtconfig.h')
    main_defines = gen_link_lds._parse_rtconfig_defines(main_rtconfig_h)
    boot_defines = gen_link_lds._parse_rtconfig_defines(boot_rtconfig_h)

    main_link_out = os.path.join(tmp_dir, 'link_main.lds')
    boot_link_out = os.path.join(tmp_dir, 'link_boot.lds')

    gen_link_lds.render_link_lds(
        hcpu_template,
        main_link_out,
        gen_link_lds.compute_link_defines(ptab_obj, 'main', 'HCPU', main_defines),
    )
    gen_link_lds.render_link_lds(
        boot_template,
        boot_link_out,
        gen_link_lds.compute_link_defines(ptab_obj, 'bootloader', 'HCPU', boot_defines),
    )

    lds_keys = [
        '__ROM_BASE', '__ROM_SIZE', '__RAM_BASE', '__RAM_SIZE',
        '__ROM_EX_BASE', '__ROM_EX_SIZE', '__PSRAM_BASE', '__PSRAM_SIZE',
        '__ROM2_BASE', '__ROM2_SIZE', '__ROM3_BASE', '__ROM3_SIZE',
    ]
    baseline_main_lds = _parse_lds_assignments(baseline_main_link)
    gen_main_lds = _parse_lds_assignments(main_link_out)
    for k in lds_keys:
        if k not in baseline_main_lds or k not in gen_main_lds:
            continue
        if int(baseline_main_lds[k]) != int(gen_main_lds[k]):
            raise _EvalError('main link mismatch {}: 0x{:08X} != 0x{:08X}'.format(k, int(baseline_main_lds[k]) & 0xFFFFFFFF, int(gen_main_lds[k]) & 0xFFFFFFFF))

    baseline_boot_lds = _parse_lds_assignments(baseline_boot_link)
    gen_boot_lds = _parse_lds_assignments(boot_link_out)
    for k in lds_keys:
        if k not in baseline_boot_lds or k not in gen_boot_lds:
            continue
        if int(baseline_boot_lds[k]) != int(gen_boot_lds[k]):
            raise _EvalError('bootloader link mismatch {}: 0x{:08X} != 0x{:08X}'.format(k, int(baseline_boot_lds[k]) & 0xFFFFFFFF, int(gen_boot_lds[k]) & 0xFFFFFFFF))

    # Generate v3 ftab.bin (main)
    boot_size = os.path.getsize(baseline_boot_bin) if os.path.exists(baseline_boot_bin) else 0x10000
    main_size = os.path.getsize(baseline_main_bin) if os.path.exists(baseline_main_bin) else 0x200000
    chip_config = ptab_obj.get_chip_config()
    ftab_data = gen_ftab.generate_ftab_binary(ptab_obj, chip_config, boot_size, main_size)
    gen_ftab_path = os.path.join(tmp_dir, 'ftab.bin')
    with open(gen_ftab_path, 'wb') as f:
        f.write(ftab_data)

    base_ftab_parts = _parse_ftab_partitions(baseline_ftab)
    gen_ftab_parts = _parse_ftab_partitions(gen_ftab_path)

    # Compare key partition entries (base/size/xip_base/flags)
    key_indices = [0, 1, 3, 4, 7, 8]
    for idx in key_indices:
        if base_ftab_parts[idx] != gen_ftab_parts[idx]:
            raise _EvalError('ftab entry {} mismatch: {} != {}'.format(idx, base_ftab_parts[idx], gen_ftab_parts[idx]))

    if args.keep_tmp:
        print('OK (tmp dir kept): {}'.format(tmp_dir))
    else:
        # Best-effort cleanup
        try:
            import shutil
            shutil.rmtree(tmp_dir)
        except Exception:
            pass
        print('OK')
    return 0


def yaml_dump(obj: Any) -> str:
    # Local import to avoid hard dependency when running under minimal envs.
    import yaml  # type: ignore
    from collections import OrderedDict

    class _OrderedDumper(yaml.SafeDumper):
        pass

    def _repr_ordered(dumper: yaml.Dumper, data: Dict[str, Any]):  # type: ignore
        return dumper.represent_mapping('tag:yaml.org,2002:map', data.items())

    _OrderedDumper.add_representer(dict, _repr_ordered)
    _OrderedDumper.add_representer(OrderedDict, _repr_ordered)
    return yaml.dump(obj, Dumper=_OrderedDumper, default_flow_style=False, allow_unicode=True, sort_keys=False, indent=2)


if __name__ == '__main__':
    raise SystemExit(main())
