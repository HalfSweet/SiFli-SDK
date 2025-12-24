#!/usr/bin/env python3
# Copyright (c) 2025 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0

"""
Validate ptab v3 format and constraints.

Usage:
    python validate_ptab_v3.py ptab.yaml
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path
from typing import List, Tuple

# Add tools/build to path
_TOOLS_BUILD_PATH = Path(__file__).parent
if str(_TOOLS_BUILD_PATH) not in sys.path:
    sys.path.insert(0, str(_TOOLS_BUILD_PATH))

import ptab as ptab_module


class ValidationError:
    def __init__(self, message: str, severity: str = "error"):
        self.message = message
        self.severity = severity  # "error" or "warning"

    def __str__(self):
        return f"[{self.severity.upper()}] {self.message}"


def validate_name(name: str, partition_idx: int) -> List[ValidationError]:
    """Validate partition name format"""
    errors = []

    if not name:
        errors.append(ValidationError(
            f"Partition {partition_idx}: 'name' is required"
        ))
        return errors

    # Must be lowercase letters, underscores, digits
    if not re.match(r'^[a-z][a-z0-9_]*$', name):
        if name[0].isdigit():
            errors.append(ValidationError(
                f"Partition '{name}': name cannot start with a digit"
            ))
        elif not name.islower() or not re.match(r'^[a-z0-9_]+$', name):
            errors.append(ValidationError(
                f"Partition '{name}': name must contain only lowercase letters, digits, and underscores"
            ))

    return errors


def validate_type(ptype: str, partition_name: str) -> List[ValidationError]:
    """Validate partition type"""
    errors = []
    valid_types = {'ptab', 'bootloader', 'app', 'data', 'code', 'ftab'}

    if not ptype:
        errors.append(ValidationError(
            f"Partition '{partition_name}': 'type' is required"
        ))
    elif ptype not in valid_types:
        errors.append(ValidationError(
            f"Partition '{partition_name}': unknown type '{ptype}', "
            f"valid types are: {', '.join(sorted(valid_types))}"
        ))

    return errors


def validate_region(region: str, partition_name: str, chip_config: dict) -> List[ValidationError]:
    """Validate region exists in chip config"""
    errors = []

    if not region:
        errors.append(ValidationError(
            f"Partition '{partition_name}': 'region' is required"
        ))
        return errors

    # Check if region exists
    mpi_config = chip_config.get('mpi', {})
    ram_config = chip_config.get('ram', {})

    known_regions = set(mpi_config.keys())
    known_regions.update(['hpsys_ram', 'lpsys_ram', 'psram1', 'psram'])

    if region not in known_regions and not region.startswith('mpi'):
        errors.append(ValidationError(
            f"Partition '{partition_name}': unknown region '{region}'",
            severity="warning"
        ))

    return errors


def validate_size(value, field_name: str, partition_name: str) -> List[ValidationError]:
    """Validate size/offset value"""
    errors = []

    try:
        parsed = ptab_module.parse_size(value)
        if parsed < 0:
            errors.append(ValidationError(
                f"Partition '{partition_name}': {field_name} cannot be negative"
            ))
    except (ValueError, TypeError) as e:
        errors.append(ValidationError(
            f"Partition '{partition_name}': invalid {field_name} value '{value}': {e}"
        ))

    return errors


def validate_exec_region_constraint(partitions: List[dict]) -> List[ValidationError]:
    """Validate: if exec_region exists, must have accelerate partition"""
    errors = []

    has_exec_region = False
    has_accelerate = False

    for p in partitions:
        if p.get('exec_region'):
            has_exec_region = True
        if p.get('type') == 'app' and p.get('subtype') == 'accelerate':
            has_accelerate = True

    if has_exec_region and not has_accelerate:
        errors.append(ValidationError(
            "exec_region is used but no partition with type='app' + subtype='accelerate' found"
        ))

    return errors


def validate_bootloader_unique(partitions: List[dict]) -> List[ValidationError]:
    """Validate: exactly one bootloader partition"""
    errors = []

    bootloaders = [p for p in partitions if p.get('type') == 'bootloader']

    if len(bootloaders) == 0:
        errors.append(ValidationError(
            "No partition with type='bootloader' found"
        ))
    elif len(bootloaders) > 1:
        names = [p.get('name', '?') for p in bootloaders]
        errors.append(ValidationError(
            f"Multiple bootloader partitions found: {', '.join(names)}"
        ))

    return errors


def validate_no_overlap(partitions: List[dict], chip_config: dict) -> List[ValidationError]:
    """Validate partitions don't overlap within same region"""
    errors = []

    # Group by region
    region_partitions = {}
    for p in partitions:
        region = p.get('region', '')
        if region not in region_partitions:
            region_partitions[region] = []

        try:
            offset = ptab_module.parse_size(p.get('offset', 0))
            size = ptab_module.parse_size(p.get('size', 0))
            region_partitions[region].append((
                p.get('name', '?'),
                offset,
                offset + size
            ))
        except (ValueError, TypeError):
            pass  # Already reported in validate_size

    # Check overlaps within each region
    for region, parts in region_partitions.items():
        parts.sort(key=lambda x: x[1])  # Sort by start offset

        for i in range(len(parts) - 1):
            name1, start1, end1 = parts[i]
            name2, start2, end2 = parts[i + 1]

            if end1 > start2:
                errors.append(ValidationError(
                    f"Partitions '{name1}' and '{name2}' overlap in region '{region}' "
                    f"(0x{start1:X}-0x{end1:X} vs 0x{start2:X}-0x{end2:X})"
                ))

    return errors


def validate_rom_index_deprecation(partitions: List[dict]) -> List[ValidationError]:
    """Warn about deprecated rom_index usage"""
    errors = []

    for p in partitions:
        if p.get('rom_index') is not None:
            errors.append(ValidationError(
                f"Partition '{p.get('name', '?')}': 'rom_index' is deprecated, "
                "please migrate to name-based section naming",
                severity="warning"
            ))

    return errors


def validate_ptab_v3(ptab_obj) -> List[ValidationError]:
    """Run all validations on a ptab v3 object"""
    errors = []

    if not ptab_obj.is_v3():
        errors.append(ValidationError("Not a v3 format ptab file"))
        return errors

    chip_config = ptab_obj.get_chip_config()
    partitions = ptab_obj.partitions

    # Per-partition validations
    for idx, p in enumerate(partitions):
        errors.extend(validate_name(p.get('name', ''), idx))
        errors.extend(validate_type(p.get('type', ''), p.get('name', f'partition_{idx}')))
        errors.extend(validate_region(p.get('region', ''), p.get('name', '?'), chip_config))
        errors.extend(validate_size(p.get('offset', 0), 'offset', p.get('name', '?')))
        errors.extend(validate_size(p.get('size', 0), 'size', p.get('name', '?')))

    # Global validations
    errors.extend(validate_exec_region_constraint(partitions))
    errors.extend(validate_bootloader_unique(partitions))
    errors.extend(validate_no_overlap(partitions, chip_config))
    errors.extend(validate_rom_index_deprecation(partitions))

    return errors


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Validate ptab v3 format",
        allow_abbrev=False
    )
    parser.add_argument(
        "ptab_file",
        help="Path to ptab.yaml file"
    )
    parser.add_argument(
        "--strict", "-s",
        action="store_true",
        help="Treat warnings as errors"
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    ptab_path = Path(args.ptab_file)
    if not ptab_path.exists():
        print(f"Error: File not found: {ptab_path}")
        sys.exit(1)

    # Load ptab
    try:
        ptab_obj = ptab_module.load_ptab(str(ptab_path), fatal=False)
    except Exception as e:
        print(f"Error loading ptab: {e}")
        sys.exit(1)

    # Validate
    errors = validate_ptab_v3(ptab_obj)

    # Report results
    error_count = sum(1 for e in errors if e.severity == "error")
    warning_count = sum(1 for e in errors if e.severity == "warning")

    for err in errors:
        print(err)

    print()
    if error_count == 0 and warning_count == 0:
        print(f"✓ {ptab_path}: Valid ptab v3 file")
        sys.exit(0)
    else:
        print(f"Found {error_count} error(s), {warning_count} warning(s)")
        if error_count > 0 or (args.strict and warning_count > 0):
            sys.exit(1)
        sys.exit(0)


if __name__ == "__main__":
    main()
