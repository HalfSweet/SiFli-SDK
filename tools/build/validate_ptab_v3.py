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
from typing import Dict, List, Optional, Tuple

import yaml

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
    valid_types = {'ftab', 'bootloader', 'app', 'data'}

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
    known_regions.update(['hpsys_ram', 'lpsys_ram', 'psram', 'psram1', 'psram2'])

    if region not in known_regions and not region.startswith('mpi') and not region.startswith('psram'):
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


def validate_acc(acc, partition_name: str, chip_config: dict, core: Optional[str]) -> List[ValidationError]:
    """Validate acc structure and address parsing."""
    errors = []

    if acc is None:
        return errors

    if not isinstance(acc, dict):
        errors.append(ValidationError(
            f"Partition '{partition_name}': 'acc' must be a dict with keys {{region, offset}}"
        ))
        return errors

    acc_region = acc.get('region', '')
    acc_offset = acc.get('offset', 0)

    if not acc_region:
        errors.append(ValidationError(
            f"Partition '{partition_name}': acc.region is required when acc is present"
        ))
        return errors

    errors.extend(validate_region(str(acc_region), f"{partition_name}.acc", chip_config))
    errors.extend(validate_size(acc_offset, 'acc.offset', partition_name))

    # Try resolving address (this may still succeed for unknown region but is useful for early errors)
    try:
        ptab_module.resolve_region_address(str(acc_region), ptab_module.parse_size(acc_offset), chip_config, core=core)
    except Exception as e:
        errors.append(ValidationError(
            f"Partition '{partition_name}': failed to resolve acc address: {e}"
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


def validate_ftab_unique(partitions: List[dict]) -> List[ValidationError]:
    """Validate: exactly one ftab partition"""
    errors = []
    ftabs = [p for p in partitions if p.get('type') == 'ftab']

    if len(ftabs) == 0:
        errors.append(ValidationError("No partition with type='ftab' found"))
    elif len(ftabs) > 1:
        names = [p.get('name', '?') for p in ftabs]
        errors.append(ValidationError(f"Multiple ftab partitions found: {', '.join(names)}"))

    return errors


def validate_factory_unique_by_core(partitions: List[dict]) -> List[ValidationError]:
    """Validate: at most one factory app per core."""
    errors = []
    seen: Dict[str, List[str]] = {}

    for p in partitions:
        if p.get('type') != 'app' or p.get('subtype') != 'factory':
            continue
        core = (p.get('core') or 'HCPU').upper()
        seen.setdefault(core, []).append(p.get('name', '?'))

    for core, names in seen.items():
        if len(names) > 1:
            errors.append(ValidationError(
                f"Multiple factory app partitions found for core {core}: {', '.join(names)}"
            ))

    return errors


def validate_name_unique(partitions: List[dict]) -> List[ValidationError]:
    errors = []
    seen: Dict[str, int] = {}
    for idx, p in enumerate(partitions):
        name = p.get('name', '')
        if not name:
            continue
        if name in seen:
            errors.append(ValidationError(
                f"Duplicate partition name '{name}' found at indices {seen[name]} and {idx}"
            ))
        else:
            seen[name] = idx
    return errors


def validate_aliases_conflict(partitions: List[dict]) -> List[ValidationError]:
    errors = []
    owner: Dict[str, str] = {}

    for p in partitions:
        pname = p.get('name', '?')
        aliases = p.get('aliases', []) or []
        if not isinstance(aliases, list):
            continue
        for a in aliases:
            a_norm = str(a).strip().upper()
            if not a_norm:
                continue
            if a_norm in owner and owner[a_norm] != pname:
                errors.append(ValidationError(
                    f"Alias macro base '{a_norm}' is used by multiple partitions: {owner[a_norm]} and {pname}"
                ))
            else:
                owner[a_norm] = pname

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

    # Header checks
    if not str(getattr(ptab_obj, 'version', '') or '') == '3':
        errors.append(ValidationError("ptab.yaml: version must be 3"))
    chip = getattr(ptab_obj, 'chip', '') or ''
    if not str(chip).strip():
        errors.append(ValidationError("ptab.yaml: 'chip' is required"))

    chip_config = ptab_obj.get_chip_config()
    partitions = ptab_obj.partitions

    errors.extend(validate_name_unique(partitions))
    errors.extend(validate_aliases_conflict(partitions))

    # Per-partition validations
    for idx, p in enumerate(partitions):
        errors.extend(validate_name(p.get('name', ''), idx))
        errors.extend(validate_type(p.get('type', ''), p.get('name', f'partition_{idx}')))
        errors.extend(validate_region(p.get('region', ''), p.get('name', '?'), chip_config))
        errors.extend(validate_size(p.get('offset', 0), 'offset', p.get('name', '?')))
        errors.extend(validate_size(p.get('size', 0), 'size', p.get('name', '?')))
        errors.extend(validate_acc(p.get('acc'), p.get('name', '?'), chip_config, p.get('core')))

    # Global validations
    errors.extend(validate_bootloader_unique(partitions))
    errors.extend(validate_ftab_unique(partitions))
    errors.extend(validate_factory_unique_by_core(partitions))

    # Bootloader must have acc (execution address)
    bootloaders = [p for p in partitions if p.get('type') == 'bootloader']
    if len(bootloaders) == 1:
        if not isinstance(bootloaders[0].get('acc'), dict):
            errors.append(ValidationError("bootloader partition must provide 'acc: {region, offset}'"))

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

    # v3 rejects legacy exec_region/exec_offset (acc-only)
    try:
        if ptab_path.suffix.lower() in ('.yaml', '.yml'):
            raw = yaml.safe_load(ptab_path.read_text(encoding='utf-8'))
            if isinstance(raw, dict) and str(raw.get('version', '') or '') == '3':
                parts = raw.get('partitions', [])
                if isinstance(parts, list):
                    for p in parts:
                        if not isinstance(p, dict):
                            continue
                        if 'exec_region' in p or 'exec_offset' in p:
                            name = p.get('name') or '?'
                            print(
                                "[ERROR] Partition '{}': exec_region/exec_offset is not supported in ptab v3; "
                                "use acc: {region, offset}.".format(name)
                            )
                            sys.exit(1)
    except SystemExit:
        raise
    except Exception:
        # Fall back to loader error reporting below (syntax errors, etc.)
        pass

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
