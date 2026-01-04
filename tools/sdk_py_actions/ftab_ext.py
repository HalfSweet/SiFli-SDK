# SPDX-FileCopyrightText: 2024-2025 SiFli Technologies Co., Ltd
# SPDX-License-Identifier: Apache-2.0

"""
Ftab (Flash Table) extension for sdk.py.

Provides commands to parse and display ftab.bin content in human-readable format.
"""

import json
import os
import struct
import sys
from enum import IntEnum
from pathlib import Path
from typing import Any, Dict, List, Optional

import click
from click.core import Context

from sdk_py_actions.errors import FatalError
from sdk_py_actions.global_options import global_options
from sdk_py_actions.tools import merge_action_lists, PropertyDict


class PartitionIndex(IntEnum):
    """Flash table partition indices"""
    FLASH_PARTITION_TABLE = 0
    CALIBRATION_TABLE = 1
    LCPU_IMAGE_PING = 2
    BOOTLOADER = 3
    HCPU_IMAGE = 4
    BOOT_PATCH = 5
    LCPU_IMAGE_PONG = 6
    BOOTLOADER_IMAGE_PONG = 7 # BCPU is bootloader
    HCPU_IMAGE_PONG = 8
    RAM_BOOT_PATCH = 9
    HCPU_IMAGE_RESERVE1 = 10
    HCPU_IMAGE_RESERVE2 = 11
    LCPU_IMAGE_RESERVE1 = 12
    LCPU_IMAGE_RESERVE2 = 13
    RESERVED1 = 14
    RESERVED2 = 15


class ImageFlag(IntEnum):
    """DFU image flags"""
    DFU_FLAG_ENC = 1
    DFU_FLAG_AUTO = 2
    DFU_FLAG_SINGLE = 4
    DFU_IMGHDR_KEY_OFFSET = 8
    DFU_FLAG_COMPRESS = 16


PARTITION_NAMES = {
    PartitionIndex.FLASH_PARTITION_TABLE: "ftab",
    PartitionIndex.CALIBRATION_TABLE: "calibration",
    PartitionIndex.LCPU_IMAGE_PING: "lcpu_image",
    PartitionIndex.LCPU_IMAGE_PONG: "lcpu_image_pong",
    PartitionIndex.BOOTLOADER: "bootloader",
    PartitionIndex.BOOTLOADER_IMAGE_PONG: "bootloader_pong",
    PartitionIndex.HCPU_IMAGE: "main",
    PartitionIndex.HCPU_IMAGE_PONG: "main_pong",
    PartitionIndex.BOOT_PATCH: "flash_boot_patch",
    PartitionIndex.RAM_BOOT_PATCH: "ram_boot_patch",
}

# Image Index corresponds to CORE_* enum:
# 0 = CORE_LCPU, 1 = CORE_BL, 2 = CORE_HCPU, 3 = CORE_BOOT
IMAGE_INDEX_NAMES = [
    "LCPU",
    "Bootloader",
    "HCPU",
    "Boot",
]


# Constants for ftab binary format
PARTITION_ENTRY_COUNT = 16
IMAGE_DESCRIPTION_COUNT = 14  # imgs[] array size = DFU_FLASH_PARTITION - 2
IMAGE_DESCRIPTION_SIZE = 512
PUBKEY_SIZE = 294
RESERVED_SIZE = 3542
IMAGE_INDEX_COUNT = 4
SEC_CONFIG_MAGIC = 0x53454346  # 'FCES'

# Image Description starts from Flash ID 2 (DFU_FLASH_IMG_LCPU)
# imgs[idx] corresponds to flash_id = idx + 2
IMAGE_DESC_FLASH_ID_OFFSET = 2

# Flash ID to name mapping (from dfu.h)
FLASH_ID_NAMES = {
    0: "SEC_CONFIG",
    1: "FACTORY_CAL",
    2: "LCPU",
    3: "Bootloader",
    4: "HCPU",
    5: "Boot",
    6: "LCPU2",
    7: "BCPU2",
    8: "HCPU2",
    9: "Boot2",
    10: "HCPU_EXT1",
    11: "HCPU_EXT2",
    12: "LCPU_EXT1",
    13: "LCPU_EXT2",
    14: "Reserved",
    15: "SINGLE",
    16: "Partition",
}


# Struct formats
PARTITION_INFO_STRUCT = struct.Struct("<IIII")  # base, size, xip_base, flags
IMAGE_DESC_HEADER_STRUCT = struct.Struct("<IHH")  # length, block_size, flags


def parse_flags(flags: int) -> List[str]:
    """Parse image flags into list of flag names"""
    flag_names = []
    if flags & ImageFlag.DFU_FLAG_ENC:
        flag_names.append("ENC")
    if flags & ImageFlag.DFU_FLAG_AUTO:
        flag_names.append("AUTO")
    if flags & ImageFlag.DFU_FLAG_SINGLE:
        flag_names.append("SINGLE")
    if flags & ImageFlag.DFU_IMGHDR_KEY_OFFSET:
        flag_names.append("KEY_OFFSET")
    if flags & ImageFlag.DFU_FLAG_COMPRESS:
        flag_names.append("COMPRESS")
    return flag_names


def parse_ftab_bin(data: bytes) -> Dict[str, Any]:
    """Parse ftab.bin binary data and return structured data.

    Args:
        data: Raw bytes of ftab.bin

    Returns:
        Dictionary containing parsed ftab data
    """
    if len(data) < 4:
        raise ValueError("ftab.bin too small")

    # Parse magic
    magic = struct.unpack("<I", data[0:4])[0]
    if magic != SEC_CONFIG_MAGIC:
        raise ValueError(f"Invalid magic: 0x{magic:08X}, expected 0x{SEC_CONFIG_MAGIC:08X}")

    offset = 4  # After magic

    # Parse partition table entries
    partitions = []
    for idx in range(PARTITION_ENTRY_COUNT):
        if offset + PARTITION_INFO_STRUCT.size > len(data):
            break

        base, size, xip_base, flags = PARTITION_INFO_STRUCT.unpack(
            data[offset:offset + PARTITION_INFO_STRUCT.size]
        )
        offset += PARTITION_INFO_STRUCT.size

        partition = {
            "index": idx,
            "name": PARTITION_NAMES.get(PartitionIndex(idx), f"reserved_{idx}"),
            "base": base,
            "size": size,
            "xip_base": xip_base,
            "flags": flags,
        }
        if size > 0:
            partition["end"] = base + size
        partitions.append(partition)

    # Skip public key and reserved areas
    offset += PUBKEY_SIZE + RESERVED_SIZE

    # Parse image description table
    # imgs[idx] corresponds to flash_id = idx + IMAGE_DESC_FLASH_ID_OFFSET
    image_descriptions = []
    for idx in range(IMAGE_DESCRIPTION_COUNT):
        if offset + IMAGE_DESC_HEADER_STRUCT.size > len(data):
            break

        length, block_size, flags = IMAGE_DESC_HEADER_STRUCT.unpack(
            data[offset:offset + IMAGE_DESC_HEADER_STRUCT.size]
        )

        flash_id = idx + IMAGE_DESC_FLASH_ID_OFFSET
        desc = {
            "index": idx,
            "flash_id": flash_id,
            "name": FLASH_ID_NAMES.get(flash_id, f"Reserved{flash_id}"),
            "length": length,
            "block_size": block_size,
            "flags": flags,
            "flags_str": parse_flags(flags),
            "valid": length != 0xFFFFFFFF and length != 0,
        }
        image_descriptions.append(desc)
        offset += IMAGE_DESCRIPTION_SIZE

    # Parse image index table
    image_indices = []
    for idx in range(IMAGE_INDEX_COUNT):
        if offset + 4 > len(data):
            break

        addr = struct.unpack("<I", data[offset:offset + 4])[0]
        index_entry = {
            "index": idx,
            "name": IMAGE_INDEX_NAMES[idx] if idx < len(IMAGE_INDEX_NAMES) else f"Index{idx}",
            "address": addr,
            "valid": addr != 0xFFFFFFFF and addr != 0,
        }
        image_indices.append(index_entry)
        offset += 4

    result = {
        "magic": magic,
        "magic_str": "FCES" if magic == SEC_CONFIG_MAGIC else "INVALID",
        "partitions": partitions,
        "image_descriptions": image_descriptions,
        "image_indices": image_indices,
        "file_size": len(data),
    }

    return result


def format_size(size: int) -> str:
    """Format size in human-readable format"""
    if size == 0:
        return "0"
    if size == 0xFFFFFFFF:
        return "-"
    if size >= 1024 * 1024:
        return f"{size / (1024 * 1024):.1f}MB"
    if size >= 1024:
        return f"{size / 1024:.1f}KB"
    return f"{size}B"


def print_ftab_table(ftab_data: Dict[str, Any]) -> None:
    """Print ftab data as rich tables.

    Args:
        ftab_data: Parsed ftab data from parse_ftab_bin
    """
    from rich.console import Console
    from rich.table import Table
    from rich.panel import Panel

    console = Console()

    # Header info
    console.print(f"\n[bold cyan]Flash Table (ftab.bin)[/bold cyan]")
    console.print(f"Magic: 0x{ftab_data['magic']:08X} ({ftab_data['magic_str']})")
    console.print(f"File Size: {ftab_data['file_size']} bytes")
    console.print()

    # ======== Partition Table ========
    table = Table(title="[bold]1. Partition Table Entries[/bold]", show_header=True, header_style="bold magenta")
    table.add_column("Index", style="cyan", justify="right")
    table.add_column("Name", style="green")
    table.add_column("Base Address", style="yellow", justify="right")
    table.add_column("Size", justify="right")
    table.add_column("Size (Hex)", style="dim", justify="right")
    table.add_column("XIP Base", style="yellow", justify="right")
    table.add_column("Flags", style="dim", justify="right")

    for p in ftab_data["partitions"]:
        if p["base"] == 0 and p["size"] == 0 and p["xip_base"] == 0:
            continue

        table.add_row(
            str(p["index"]),
            p["name"],
            f"0x{p['base']:08X}",
            format_size(p["size"]),
            f"0x{p['size']:08X}",
            f"0x{p['xip_base']:08X}" if p["xip_base"] else "-",
            f"0x{p['flags']:08X}" if p["flags"] else "-",
        )

    console.print(table)

    non_empty = [p for p in ftab_data["partitions"]
                 if p["base"] != 0 or p["size"] != 0 or p["xip_base"] != 0]
    console.print(f"[dim]Valid entries: {len(non_empty)}/{PARTITION_ENTRY_COUNT}[/dim]\n")

    # ======== Image Description Table ========
    table2 = Table(title="[bold]2. Image Description Table[/bold]", show_header=True, header_style="bold magenta")
    table2.add_column("Idx", style="dim", justify="right")
    table2.add_column("FlashID", style="cyan", justify="right")
    table2.add_column("Name", style="green")
    table2.add_column("Length", justify="right")
    table2.add_column("Length (Hex)", style="dim", justify="right")
    table2.add_column("Block Size", justify="right")
    table2.add_column("Flags", style="dim")
    table2.add_column("Status", style="bold")

    for desc in ftab_data["image_descriptions"]:
        status = "[green]Valid[/green]" if desc["valid"] else "[dim]Empty[/dim]"
        flags_str = ", ".join(desc["flags_str"]) if desc["flags_str"] else "-"

        table2.add_row(
            str(desc["index"]),
            str(desc["flash_id"]),
            desc["name"],
            format_size(desc["length"]),
            f"0x{desc['length']:08X}" if desc["length"] != 0xFFFFFFFF else "-",
            str(desc["block_size"]) if desc["block_size"] else "-",
            flags_str,
            status,
        )

    console.print(table2)

    valid_descs = [d for d in ftab_data["image_descriptions"] if d["valid"]]
    console.print(f"[dim]Valid entries: {len(valid_descs)}/{IMAGE_DESCRIPTION_COUNT}[/dim]\n")

    # ======== Image Index Table ========
    table3 = Table(title="[bold]3. Image Index Table[/bold]", show_header=True, header_style="bold magenta")
    table3.add_column("Index", style="cyan", justify="right")
    table3.add_column("Name", style="green")
    table3.add_column("Address", style="yellow", justify="right")
    table3.add_column("Status", style="bold")

    for idx_entry in ftab_data["image_indices"]:
        status = "[green]Valid[/green]" if idx_entry["valid"] else "[dim]Empty[/dim]"

        table3.add_row(
            str(idx_entry["index"]),
            idx_entry["name"],
            f"0x{idx_entry['address']:08X}" if idx_entry["valid"] else "-",
            status,
        )

    console.print(table3)

    valid_indices = [i for i in ftab_data["image_indices"] if i["valid"]]
    console.print(f"[dim]Valid entries: {len(valid_indices)}/{IMAGE_INDEX_COUNT}[/dim]\n")


def print_ftab_json(ftab_data: Dict[str, Any], pretty: bool = True) -> None:
    """Print ftab data as JSON.

    Args:
        ftab_data: Parsed ftab data from parse_ftab_bin
        pretty: Whether to use pretty formatting
    """
    # Convert to hex strings for readability
    output = {
        "magic": f"0x{ftab_data['magic']:08X}",
        "magic_str": ftab_data["magic_str"],
        "file_size": ftab_data["file_size"],
        "partitions": [],
        "image_descriptions": [],
        "image_indices": [],
    }

    # Partitions (skip empty)
    for p in ftab_data["partitions"]:
        if p["base"] == 0 and p["size"] == 0 and p["xip_base"] == 0:
            continue

        output["partitions"].append({
            "index": p["index"],
            "name": p["name"],
            "base": f"0x{p['base']:08X}",
            "size": p["size"],
            "size_hex": f"0x{p['size']:08X}",
            "xip_base": f"0x{p['xip_base']:08X}" if p["xip_base"] else None,
            "flags": p["flags"],
        })

    # Image descriptions
    for desc in ftab_data["image_descriptions"]:
        output["image_descriptions"].append({
            "index": desc["index"],
            "name": desc["name"],
            "length": desc["length"] if desc["length"] != 0xFFFFFFFF else None,
            "length_hex": f"0x{desc['length']:08X}" if desc["length"] != 0xFFFFFFFF else None,
            "block_size": desc["block_size"],
            "flags": desc["flags"],
            "flags_str": desc["flags_str"],
            "valid": desc["valid"],
        })

    # Image indices
    for idx_entry in ftab_data["image_indices"]:
        output["image_indices"].append({
            "index": idx_entry["index"],
            "name": idx_entry["name"],
            "address": f"0x{idx_entry['address']:08X}" if idx_entry["valid"] else None,
            "valid": idx_entry["valid"],
        })

    if pretty:
        print(json.dumps(output, indent=2))
    else:
        print(json.dumps(output))


def action_extensions(base_actions: Dict, project_path: str) -> Any:
    """Register ftab-related actions."""

    def ftab_dump_callback(
        target_name: str,
        ctx: Context,
        args: PropertyDict,
        ftab_path: str,
        output_format: str,
    ) -> None:
        """Parse and display ftab.bin content."""

        # Resolve ftab path
        if not ftab_path:
            # Try to find ftab.bin in build directories
            project_dir = args.project_dir
            possible_paths = list(Path(project_dir).glob("build_*/ftab.bin"))
            if not possible_paths:
                raise FatalError(
                    "No ftab.bin found. Please specify path with --path or build the project first."
                )
            ftab_path = str(possible_paths[0])
            print(f"Using: {ftab_path}")

        if not os.path.exists(ftab_path):
            raise FatalError(f"ftab.bin not found: {ftab_path}")

        # Read and parse
        with open(ftab_path, "rb") as f:
            data = f.read()

        try:
            ftab_data = parse_ftab_bin(data)
        except ValueError as e:
            raise FatalError(f"Failed to parse ftab.bin: {e}")

        # Output
        if output_format == "json":
            print_ftab_json(ftab_data, pretty=True)
        else:
            print_ftab_table(ftab_data)

    ftab_actions = {
        "actions": {
            "ftab-dump": {
                "callback": ftab_dump_callback,
                "help": "Parse and display ftab.bin content in human-readable format.",
                "options": global_options + [
                    {
                        "names": ["--path", "-p", "ftab_path"],
                        "help": "Path to ftab.bin file. If not specified, searches in build directories.",
                        "type": click.Path(exists=False),
                        "default": None,
                    },
                    {
                        "names": ["--format", "-f", "output_format"],
                        "help": "Output format: table (default) or json.",
                        "type": click.Choice(["table", "json"]),
                        "default": "table",
                    },
                ],
            },
        }
    }

    return ftab_actions
