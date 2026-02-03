#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0

"""
Migrate legacy ptab.json (v1/v2) to ptab.yaml (v3).

This tool focuses on keeping build compatibility:
- Converts regions to v3 `partitions` with `type/subtype/region/offset/size`.
- Converts exec-only regions (region has `exec` but no `img`) into `acc` on the
  corresponding `bootloader/app` partition.
- Uses legacy `tags` as `aliases` (dedup, case-insensitive) when needed.

Usage:
  python3 tools/build/migrate_ptab_to_v3.py -i customer/boards/<board>/ptab.json -o /tmp/ptab.yaml -c SF32LB52JUD6
"""

import argparse
import os
import json
import re
import sys
from collections import OrderedDict
from typing import Any, Dict, List, Optional, Tuple

import yaml


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Migrate ptab.json (v1/v2) to ptab.yaml (v3)',
        allow_abbrev=False,
    )
    parser.add_argument('-i', '--input', required=True, help='Input ptab.json path')
    parser.add_argument('-o', '--output', help='Output ptab.yaml path (default: stdout)')
    parser.add_argument(
        '-c', '--chip',
        default='SF32LB52JUD6',
        help='Chip part number for ptab v3 (default: SF32LB52JUD6)',
    )
    parser.add_argument('--no-header', action='store_true', help='Do not emit header comments')
    return parser.parse_args()


def _load_ptab_json(path: str) -> List[Dict[str, Any]]:
    with open(path, 'r', encoding='utf-8') as f:
        data = json.load(f, object_pairs_hook=OrderedDict)
    if not isinstance(data, list):
        raise ValueError('Invalid ptab.json: expected a list')
    return data


def _mem_to_region(mem_name: str) -> str:
    mem_name = (mem_name or '').strip().lower()
    mapping = {
        'flash1': 'mpi1',
        'flash2': 'mpi2',
        'flash3': 'mpi3',
        'flash4': 'mpi4',
        'flash5': 'mpi5',
        'psram1': 'psram1',
        'psram1_cbus': 'psram1',
        'psram2': 'psram2',
        'hpsys_ram': 'hpsys_ram',
        'lpsys_ram': 'lpsys_ram',
    }
    return mapping.get(mem_name, mem_name)


def _sanitize_name(name: str) -> str:
    name = (name or '').strip().lower()
    name = re.sub(r'[^a-z0-9_]', '_', name)
    if not name:
        return name
    if name[0].isdigit():
        name = 'p_' + name
    return name


def _dedup_aliases(aliases: List[str], canonical_name: str) -> List[str]:
    seen = set()
    out: List[str] = []
    canonical_upper = (canonical_name or '').upper()
    for a in aliases or []:
        a = str(a or '').strip()
        if not a:
            continue
        up = a.upper()
        if up == canonical_upper:
            continue
        if up in seen:
            continue
        seen.add(up)
        out.append(a)
    return out


def _infer_type_subtype_core(region: str, region_data: Dict[str, Any]) -> Tuple[str, Optional[str], Optional[str]]:
    tags = region_data.get('tags') or []
    tags = [str(t) for t in tags if str(t).strip()]
    img = str(region_data.get('img') or '').strip().lower()
    ftab = region_data.get('ftab') if isinstance(region_data.get('ftab'), dict) else {}
    ftab_name = str(ftab.get('name') or '').strip().lower()

    tags_upper = [t.upper() for t in tags]

    # ftab
    if 'FLASH_TABLE' in tags_upper or img == 'ftab' or ftab_name == 'ftab':
        return 'ftab', None, None

    # bootloader
    if img == 'bootloader' or ftab_name == 'bootloader' or 'FLASH_BOOT_LOADER' in tags_upper:
        return 'bootloader', None, 'HCPU'

    # app (factory/dfu)
    if 'DFU_FLASH_CODE' in tags_upper or img == 'dfu' or ftab_name == 'dfu':
        return 'app', 'dfu', 'HCPU'
    if 'HCPU_FLASH_CODE' in tags_upper or img == 'main' or ftab_name == 'main':
        return 'app', 'factory', 'HCPU'

    # filesystem / nvds
    if 'FS_REGION' in tags_upper:
        return 'data', 'filesystem', None
    if any(t.startswith('KVDB_') for t in tags_upper):
        return 'data', 'nvds', None

    # RAM-like regions
    if region in ('hpsys_ram', 'lpsys_ram') or region.startswith('psram'):
        return 'data', 'ram', None

    # default
    return 'data', 'raw', None


def _infer_partition_name(region_data: Dict[str, Any]) -> str:
    tags = region_data.get('tags') or []
    tags = [str(t) for t in tags if str(t).strip()]
    img = str(region_data.get('img') or '').strip().lower()
    ftab = region_data.get('ftab') if isinstance(region_data.get('ftab'), dict) else {}
    ftab_name = str(ftab.get('name') or '').strip().lower()

    # Prefer first tag as it maps to legacy macro base names.
    if tags:
        return _sanitize_name(tags[0])
    if ftab_name:
        return _sanitize_name(ftab_name)
    if img:
        return _sanitize_name(img)
    return ''


def migrate_ptab_json_to_v3(ptab_json: List[Dict[str, Any]], chip: str) -> Dict[str, Any]:
    # Map exec-only regions: exec_name -> {region, offset}
    exec_only: Dict[str, Dict[str, Any]] = {}

    # Collect all regions first
    regions_flat: List[Tuple[str, Dict[str, Any]]] = []
    for mem in ptab_json:
        if not isinstance(mem, dict):
            continue
        region = _mem_to_region(mem.get('mem'))
        for r in mem.get('regions') or []:
            if not isinstance(r, dict):
                continue
            regions_flat.append((region, r))

            exec_name = str(r.get('exec') or '').strip()
            img_name = str(r.get('img') or '').strip()
            if exec_name and not img_name:
                # exec-only region (acc source)
                exec_only[exec_name] = {
                    'region': region,
                    'offset': r.get('offset', '0'),
                }

    partitions: List[Dict[str, Any]] = []

    for region, r in regions_flat:
        # Skip exec-only regions; they'll be converted into acc
        if r.get('exec') and not r.get('img'):
            continue

        # Skip zero-sized regions
        try:
            if int(str(r.get('max_size', '0')), 0) == 0:
                continue
        except Exception:
            pass

        # Legacy ptab.json may contain overlapping "macro regions" in internal RAM.
        # For v3 we only keep non-overlapping partitions by default.
        tags = r.get('tags') or []
        tags_upper = [str(t).strip().upper() for t in tags if str(t).strip()]
        if region in ('hpsys_ram', 'lpsys_ram'):
            if any(t in (
                'HCPU_RAM_DATA',
                'HCPU_RO_DATA',
                'HPSYS_MBOX',
                'HCPU2LCPU_MB_CH2_BUF',
                'HCPU2LCPU_MB_CH1_BUF',
                'LCPU2HCPU_MB_CH1_BUF',
                'LCPU2HCPU_MB_CH2_BUF',
            ) for t in tags_upper):
                # Keep BOOTLOADER_RAM_DATA and LPSYS_RAM as they are typically non-overlapping.
                if 'BOOTLOADER_RAM_DATA' not in tags_upper and 'LPSYS_RAM' not in tags_upper:
                    continue

        name = _infer_partition_name(r)
        if not name:
            continue

        offset = r.get('offset', '0')
        size = r.get('max_size', '0')

        ptype, subtype, core = _infer_type_subtype_core(region, r)

        # Build aliases from legacy tags
        tags = [str(t) for t in tags if str(t).strip()]
        aliases = _dedup_aliases(tags, name)

        part: Dict[str, Any] = OrderedDict()
        part['name'] = name
        part['type'] = ptype
        if subtype:
            part['subtype'] = subtype
        part['region'] = region
        part['offset'] = offset
        part['size'] = size
        if core:
            part['core'] = core
        if aliases:
            part['aliases'] = aliases

        # Convert exec-only region into acc for bootloader/app partitions
        if ptype in ('bootloader', 'app'):
            # Determine image name to search for exec-only mapping.
            img_name = str(r.get('img') or '').strip()
            if img_name and img_name in exec_only:
                acc_src = exec_only[img_name]
                part['acc'] = OrderedDict([
                    ('region', acc_src.get('region', '')),
                    ('offset', acc_src.get('offset', '0')),
                ])

        partitions.append(part)

    # Auto-add calibration partition for common layouts (compat with legacy ftab)
    has_calibration = False
    flash_table_part: Optional[Dict[str, Any]] = None
    for p in partitions:
        if p.get('type') == 'ftab':
            flash_table_part = p
        if p.get('type') == 'data' and p.get('subtype') == 'calibration':
            has_calibration = True

    if flash_table_part and not has_calibration:
        try:
            flash_off = int(str(flash_table_part.get('offset', '0')), 0)
        except Exception:
            flash_off = 0
        try:
            flash_size = int(str(flash_table_part.get('size', '0')), 0)
        except Exception:
            flash_size = 0x8000
        cal_off = flash_off + flash_size
        cal_part: Dict[str, Any] = OrderedDict()
        cal_part['name'] = 'calibration'
        cal_part['type'] = 'data'
        cal_part['subtype'] = 'calibration'
        cal_part['region'] = flash_table_part.get('region', 'mpi2')
        cal_part['offset'] = '0x{:08X}'.format(cal_off)
        cal_part['size'] = '0x{:08X}'.format(0x2000)
        # Insert right after flash table for readability
        idx = partitions.index(flash_table_part) + 1
        partitions.insert(idx, cal_part)

    out: Dict[str, Any] = OrderedDict()
    out['version'] = 3
    out['chip'] = chip
    out['partitions'] = partitions
    return out


class _OrderedDumper(yaml.SafeDumper):
    pass


def _ordered_dict_representer(dumper: yaml.Dumper, data: OrderedDict) -> yaml.nodes.MappingNode:
    return dumper.represent_mapping('tag:yaml.org,2002:map', data.items())


_OrderedDumper.add_representer(OrderedDict, _ordered_dict_representer)


def main() -> int:
    args = _parse_args()

    ptab_json = _load_ptab_json(args.input)
    v3 = migrate_ptab_json_to_v3(ptab_json, args.chip)

    yaml_body = yaml.dump(
        v3,
        Dumper=_OrderedDumper,
        default_flow_style=False,
        allow_unicode=True,
        sort_keys=False,
        indent=2,
    )

    if not args.no_header:
        header = (
            '# ptab v3 - Partition Table\n'
            '# Migrated from: {}\n'
            '# Chip: {}\n'
            '#\n'
            '# Auto-generated. Please review and adjust as needed.\n\n'
        ).format(os.path.basename(args.input), args.chip)
        yaml_body = header + yaml_body

    if args.output:
        with open(args.output, 'w', encoding='utf-8', newline='\n') as f:
            f.write(yaml_body)
        print('Written to: {}'.format(args.output))
    else:
        sys.stdout.write(yaml_body)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
