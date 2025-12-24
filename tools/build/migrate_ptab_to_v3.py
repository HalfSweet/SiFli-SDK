#!/usr/bin/env python3
# Copyright (c) 2025 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0

"""
Migrate ptab.json (v1/v2) to ptab.yaml (v3) format.

Usage:
    python migrate_ptab_to_v3.py --input ptab.json --output ptab.yaml --chip SF32LB52
"""

from __future__ import annotations

import argparse
import json
import re
import sys
from collections import OrderedDict
from pathlib import Path
from typing import Dict, List, Optional

import yaml


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Migrate ptab.json (v1/v2) to ptab.yaml (v3)",
        allow_abbrev=False
    )
    parser.add_argument(
        "--input", "-i",
        required=True,
        help="Path to input ptab.json file"
    )
    parser.add_argument(
        "--output", "-o",
        help="Path to output ptab.yaml file (default: same dir, .yaml extension)"
    )
    parser.add_argument(
        "--chip", "-c",
        default="SF32LB52",
        help="Chip series name (default: SF32LB52)"
    )
    parser.add_argument(
        "--dry-run", "-n",
        action="store_true",
        help="Print output to stdout instead of writing file"
    )
    return parser.parse_args()


def load_v1v2_ptab(path: Path) -> tuple:
    """Load v1/v2 ptab.json and return (version, mems)"""
    with open(path) as f:
        data = json.load(f, object_pairs_hook=OrderedDict)

    if not isinstance(data, list):
        raise ValueError("Invalid ptab format: expected list")

    # Check for version header
    if data and isinstance(data[0], dict) and 'version' in data[0]:
        version = str(data[0]['version'])
        mems = data[1:]
    else:
        version = "1"
        mems = data

    return version, mems


def infer_region_from_mem(mem_name: str) -> str:
    """Infer v3 region name from v1/v2 mem name"""
    # Common mappings
    mappings = {
        'flash1': 'mpi1',
        'flash2': 'mpi2',
        'flash3': 'mpi3',
        'flash4': 'mpi4',
        'flash5': 'mpi5',
        'psram1': 'psram1',
        'psram1_cbus': 'psram1',
        'hpsys_ram': 'hpsys_ram',
        'lpsys_ram': 'lpsys_ram',
    }
    return mappings.get(mem_name, mem_name)


def infer_type_subtype(region_data: dict) -> tuple:
    """Infer type and subtype from v1/v2 region data"""
    name = region_data.get('name', '')
    tags = region_data.get('tags', [])
    ftab = region_data.get('ftab', {})
    type_list = region_data.get('type', [])

    ptype = None
    subtype = None

    # Check for ftab name
    ftab_name = ftab.get('name', '') if isinstance(ftab, dict) else ''

    if name == 'ftab' or ftab_name == 'ftab' or 'FLASH_TABLE' in tags:
        ptype = 'ptab'
        subtype = 'primary'
    elif name == 'bootloader' or ftab_name == 'bootloader' or 'FLASH_BOOT_LOADER' in tags:
        ptype = 'bootloader'
    elif name == 'dfu' or ftab_name == 'dfu' or 'DFU' in str(tags):
        ptype = 'app'
        subtype = 'dfu'
    elif name == 'main' or ftab_name == 'main':
        ptype = 'app'
        subtype = 'factory'
    elif 'app_img' in type_list or 'app_exec' in type_list:
        ptype = 'app'
        # Check for int_res indicators
        if any('IMG' in t or 'FONT' in t for t in tags):
            subtype = 'int_res'
        else:
            subtype = 'factory'
    elif any('FS_' in t or 'KVDB' in t for t in tags):
        ptype = 'data'
        subtype = 'nvds'
    elif any('RAM_DATA' in t for t in tags):
        ptype = 'data'
        subtype = 'ram'
    else:
        # Default to data for unknown
        ptype = 'data'

    return ptype, subtype


def convert_to_v3(version: str, mems: list, chip: str) -> dict:
    """Convert v1/v2 mems to v3 format"""
    v3 = {
        'version': 3,
        'chip': chip,
        'partitions': []
    }

    # Track which partitions we've seen (for exec regions)
    seen_partitions = {}

    for mem in mems:
        mem_name = mem.get('mem', '')
        region = infer_region_from_mem(mem_name)

        for region_data in mem.get('regions', []):
            tags = region_data.get('tags', [])
            name = region_data.get('name', '')
            type_list = region_data.get('type', [])

            # Skip empty regions with no meaningful data
            if not name and not tags and not region_data.get('ftab'):
                continue

            # Skip pure exec regions (they'll be merged with img regions)
            if 'app_exec' in type_list and 'app_img' not in type_list:
                # This is an exec-only region, find the corresponding img region
                if name and name in seen_partitions:
                    # Add exec_region/exec_offset to existing partition
                    partition = seen_partitions[name]
                    partition['exec_region'] = region
                    partition['exec_offset'] = region_data.get('offset', '0')
                continue

            # Infer type/subtype
            ptype, subtype = infer_type_subtype(region_data)

            # Generate name if missing
            if not name:
                if tags:
                    # Use first tag as name, convert to lowercase
                    name = tags[0].lower().replace('_start_addr', '').replace('_size', '')
                else:
                    name = f"unnamed_{len(v3['partitions'])}"

            # Clean up name for v3 format (lowercase, underscores only)
            name = re.sub(r'[^a-z0-9_]', '_', name.lower())
            if name[0].isdigit():
                name = 'p_' + name

            partition = {
                'name': name,
                'type': ptype,
                'region': region,
                'offset': region_data.get('offset', '0'),
                'size': region_data.get('max_size', '0'),
            }

            if subtype:
                partition['subtype'] = subtype

            # Check for exec address in ftab
            ftab = region_data.get('ftab', {})
            if isinstance(ftab, dict) and 'address' in ftab:
                if 'xip' in ftab['address'] and 'base' not in ftab['address']:
                    # This is a pure XIP region
                    pass

            # Add core if present
            if 'core' in region_data:
                partition['core'] = region_data['core']

            # Store for later reference
            if name:
                seen_partitions[name] = partition

            v3['partitions'].append(partition)

    return v3


class OrderedDumper(yaml.SafeDumper):
    """YAML dumper that preserves order and uses nice formatting"""
    pass


def str_representer(dumper, data):
    """Represent strings, using | for multiline"""
    if '\n' in data:
        return dumper.represent_scalar('tag:yaml.org,2002:str', data, style='|')
    return dumper.represent_scalar('tag:yaml.org,2002:str', data)


def ordered_dict_representer(dumper, data):
    return dumper.represent_mapping('tag:yaml.org,2002:map', data.items())


OrderedDumper.add_representer(str, str_representer)
OrderedDumper.add_representer(OrderedDict, ordered_dict_representer)


def main() -> None:
    args = parse_args()

    input_path = Path(args.input)
    if not input_path.exists():
        print(f"Error: Input file not found: {input_path}")
        sys.exit(1)

    # Load v1/v2 ptab
    version, mems = load_v1v2_ptab(input_path)
    print(f"Loaded ptab v{version} from {input_path}")

    # Convert to v3
    v3_data = convert_to_v3(version, mems, args.chip)
    print(f"Converted {len(v3_data['partitions'])} partitions")

    # Generate YAML output
    yaml_output = yaml.dump(
        v3_data,
        Dumper=OrderedDumper,
        default_flow_style=False,
        allow_unicode=True,
        sort_keys=False,
        indent=2
    )

    # Add header comment
    header = f"""\
# ptab v3 - Partition Table
# Migrated from: {input_path.name}
# Chip: {args.chip}
#
# This file was auto-generated. Please review and adjust as needed.

"""
    yaml_output = header + yaml_output

    if args.dry_run:
        print("\n--- Generated ptab.yaml ---")
        print(yaml_output)
    else:
        output_path = Path(args.output) if args.output else input_path.with_suffix('.yaml')
        output_path.write_text(yaml_output)
        print(f"Written to: {output_path}")


if __name__ == "__main__":
    main()
