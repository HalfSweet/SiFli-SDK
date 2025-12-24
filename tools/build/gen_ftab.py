#!/usr/bin/env python3
# Copyright (c) 2025 SiFli Technologies(Nanjing) Co., Ltd
# SPDX-License-Identifier: Apache-2.0

"""
Generate ftab.bin for SiFli platforms from ptab v3 files.

This script replaces the ftab subproject and directly generates the binary
flash table used by the bootloader.
"""

from __future__ import annotations

import argparse
import enum
import os
import struct
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple

# Add tools/build to path for ptab module
_TOOLS_BUILD_PATH = Path(__file__).parent
if str(_TOOLS_BUILD_PATH) not in sys.path:
    sys.path.insert(0, str(_TOOLS_BUILD_PATH))

import ptab as ptab_module


class PartitionIndex(enum.IntEnum):
    """Flash table partition indices"""
    FLASH_PARTITION_TABLE = 0
    CALIBRATION_TABLE = 1
    LCPU_IMAGE_PING = 2
    BOOTLOADER = 3
    HCPU_IMAGE = 4
    BOOT_PATCH = 5
    LCPU_IMAGE_PONG = 6
    BOOTLOADER_IMAGE_PONG = 7  # BCPU is bootloader
    HCPU_IMAGE_PONG = 8
    RAM_BOOT_PATCH = 9
    HCPU_IMAGE_RESERVE1 = 10
    HCPU_IMAGE_RESERVE2 = 11
    LCPU_IMAGE_RESERVE1 = 12
    LCPU_IMAGE_RESERVE2 = 13
    RESERVED1 = 14
    RESERVED2 = 15


class ImageFlag(enum.IntEnum):
    """DFU image flags"""
    DFU_FLAG_ENC = 1
    DFU_FLAG_AUTO = 2
    DFU_FLAG_SINGLE = 4
    DFU_IMGHDR_KEY_OFFSET = 8
    DFU_FLAG_COMPRESS = 16


# Constants for ftab binary format
PARTITION_ENTRY_COUNT = 16
IMAGE_DESCRIPTION_COUNT = 14
IMAGE_DESCRIPTION_SIZE = 512
PUBKEY_SIZE = 294
RESERVED_SIZE = 3542
IMAGE_INDEX_COUNT = 4
PARTITION_INFO_STRUCT = struct.Struct("<IIII")  # base, size, xip_base, flags
FLASH_TABLE_SIZE = 0x8000
SEC_CONFIG_MAGIC = 0x53454346  # 'FCES'


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate ftab.bin from ptab file",
        allow_abbrev=False
    )
    parser.add_argument(
        "--ptab", "-p",
        required=True,
        help="Path to ptab.json or ptab.yaml file"
    )
    parser.add_argument(
        "--output", "-o",
        required=True,
        help="Path to write the generated ftab.bin"
    )
    parser.add_argument(
        "--chip", "-c",
        help="Chip series (e.g., sf32lb52), auto-detected from ptab v3"
    )
    parser.add_argument(
        "--bootloader-size",
        type=lambda x: int(x, 0),
        default=0x10000,
        help="Bootloader binary size (default: 0x10000)"
    )
    parser.add_argument(
        "--main-size",
        type=lambda x: int(x, 0),
        default=0x200000,
        help="Main application binary size (default: 0x200000)"
    )
    return parser.parse_args()


def build_partition_table_entries(
    partitions: List[dict],
    chip_config: dict
) -> List[Tuple[int, int, int, int]]:
    """Build partition table entries from ptab partitions.

    对于 accelerate 分区：
    - base: 存储地址（flash）
    - xip_base: 执行地址（psram 的 xip 地址）
    - 下载时使用 base 地址，执行时使用 xip_base 地址

    Returns:
        List of (base, size, xip_base, flags) tuples
    """
    entries = [(0, 0, 0, 0)] * PARTITION_ENTRY_COUNT

    # Find key partitions
    ftab_partition = None
    calibration_partition = None
    bootloader_partition = None
    main_partition = None
    dfu_partition = None
    accelerate_partition = None

    for p in partitions:
        ptype = p.get('type', '')
        subtype = p.get('subtype', '')
        name = p.get('name', '')

        if ptype == 'ftab':
            ftab_partition = p
        elif ptype == 'data' and subtype == 'calibration':
            calibration_partition = p
        elif ptype == 'bootloader':
            bootloader_partition = p
        elif ptype == 'app' and subtype == 'factory':
            main_partition = p
        elif ptype == 'app' and subtype == 'dfu':
            dfu_partition = p
        elif ptype == 'app' and subtype == 'accelerate':
            accelerate_partition = p

    # Fill ftab entry
    if ftab_partition:
        size = ptab_module.parse_size(ftab_partition.get('size', 0))
        region = ftab_partition.get('region', '')
        offset = ptab_module.parse_size(ftab_partition.get('offset', 0))
        sbus_addr, cbus_addr = ptab_module.resolve_region_address(region, offset, chip_config)
        
        # Select base address based on memory type
        memory_info = chip_config.get('memory_info', {}).get(region, {})
        mem_type = memory_info.get('type', '').lower()
        base_addr = sbus_addr if mem_type in ('nand', 'psram', 'ram') else cbus_addr
        
        entries[PartitionIndex.FLASH_PARTITION_TABLE] = (base_addr, size, 0, 0)

    # Fill calibration entry
    if calibration_partition:
        size = ptab_module.parse_size(calibration_partition.get('size', 0))
        region = calibration_partition.get('region', '')
        offset = ptab_module.parse_size(calibration_partition.get('offset', 0))
        sbus_addr, cbus_addr = ptab_module.resolve_region_address(region, offset, chip_config)
        
        # Select base address based on memory type
        memory_info = chip_config.get('memory_info', {}).get(region, {})
        mem_type = memory_info.get('type', '').lower()
        base_addr = sbus_addr if mem_type in ('nand', 'psram', 'ram') else cbus_addr
        
        entries[PartitionIndex.CALIBRATION_TABLE] = (base_addr, size, 0, 0)

    # Fill bootloader entry
    if bootloader_partition:
        size = ptab_module.parse_size(bootloader_partition.get('size', 0))
        region = bootloader_partition.get('region', '')
        offset = ptab_module.parse_size(bootloader_partition.get('offset', 0))
        sbus_addr, cbus_addr = ptab_module.resolve_region_address(region, offset, chip_config)
        
        # Select base address based on memory type
        memory_info = chip_config.get('memory_info', {}).get(region, {})
        mem_type = memory_info.get('type', '').lower()
        base_addr = sbus_addr if mem_type in ('nand', 'psram', 'ram') else cbus_addr

        exec_region = bootloader_partition.get('exec_region', region)
        exec_offset = ptab_module.parse_size(bootloader_partition.get('exec_offset', offset))
        exec_sbus_addr, exec_cbus_addr = ptab_module.resolve_region_address(exec_region, exec_offset, chip_config)
        
        # XIP address: RAM/NAND use base, PSRAM/NOR use XIP
        exec_memory_info = chip_config.get('memory_info', {}).get(exec_region, {})
        exec_mem_type = exec_memory_info.get('type', '').lower()
        xip_addr = exec_sbus_addr if exec_mem_type in ('ram', 'nand') else exec_cbus_addr

        entries[PartitionIndex.BOOTLOADER] = (base_addr, size, xip_addr, 0)
        entries[PartitionIndex.BOOTLOADER_IMAGE_PONG] = (base_addr, size, xip_addr, 0)

    # Fill main entry
    # For main partition, if accelerate partition exists, use accelerate's xip address
    if main_partition:
        region = main_partition.get('region', '')
        offset = ptab_module.parse_size(main_partition.get('offset', 0))
        size = ptab_module.parse_size(main_partition.get('size', 0))
        sbus_addr, cbus_addr = ptab_module.resolve_region_address(region, offset, chip_config)
        
        # Select base address based on memory type
        memory_info = chip_config.get('memory_info', {}).get(region, {})
        mem_type = memory_info.get('type', '').lower()
        base_addr = sbus_addr if mem_type in ('nand', 'psram', 'ram') else cbus_addr

        # Check if accelerate partition is used for execution address
        if accelerate_partition:
            # accelerate partition: stored in flash, executed in psram
            # ftab's xip_base should be accelerate partition's xip address
            acc_region = accelerate_partition.get('region', '')
            acc_offset = ptab_module.parse_size(accelerate_partition.get('offset', 0))
            acc_sbus_addr, acc_cbus_addr = ptab_module.resolve_region_address(acc_region, acc_offset, chip_config)
            
            # XIP address for execution
            acc_memory_info = chip_config.get('memory_info', {}).get(acc_region, {})
            acc_mem_type = acc_memory_info.get('type', '').lower()
            xip_addr = acc_sbus_addr if acc_mem_type in ('ram', 'nand') else acc_cbus_addr
        elif main_partition.get('exec_region'):
            exec_region = main_partition.get('exec_region', region)
            exec_offset = ptab_module.parse_size(main_partition.get('exec_offset', offset))
            exec_sbus_addr, exec_cbus_addr = ptab_module.resolve_region_address(exec_region, exec_offset, chip_config)
            
            # XIP address: RAM/NAND use base, PSRAM/NOR use XIP
            exec_memory_info = chip_config.get('memory_info', {}).get(exec_region, {})
            exec_mem_type = exec_memory_info.get('type', '').lower()
            xip_addr = exec_sbus_addr if exec_mem_type in ('ram', 'nand') else exec_cbus_addr
        else:
            # Default XIP execution
            xip_sbus_addr, xip_cbus_addr = ptab_module.resolve_region_address(region, offset, chip_config)
            # Use base for NAND/RAM, XIP for PSRAM/NOR
            xip_addr = xip_sbus_addr if mem_type in ('ram', 'nand') else xip_cbus_addr

        entries[PartitionIndex.HCPU_IMAGE] = (base_addr, size, xip_addr, 0)
        entries[PartitionIndex.HCPU_IMAGE_PONG] = (base_addr, size, xip_addr, 0)

    return entries


def pack_partition_table(entries: List[Tuple[int, int, int, int]]) -> bytes:
    """Pack partition table entries to binary"""
    out = bytearray()
    for base, size, xip_base, flags in entries:
        out.extend(PARTITION_INFO_STRUCT.pack(base, size, xip_base, flags))
    return bytes(out)


def pack_image_description(length: int, used: bool) -> bytes:
    """Pack a single image description entry"""
    block_size = 512 if used else 0
    flags = ImageFlag.DFU_FLAG_AUTO if used else 0
    header = struct.pack("<IHH", length, block_size, flags)
    payload_len = IMAGE_DESCRIPTION_SIZE - len(header)
    return header + bytes(payload_len)


def build_image_descriptions(
    bootloader_size: int,
    main_size: int,
    dfu_size: int = 0
) -> bytes:
    """Build image description table

    Image description array indices correspond to:
    imgs[idx] = Flash ID (idx + 2)
    - imgs[0] = Flash ID 2 = DFU_FLASH_IMG_LCPU
    - imgs[1] = Flash ID 3 = DFU_FLASH_IMG_BL
    - imgs[2] = Flash ID 4 = DFU_FLASH_IMG_HCPU
    - imgs[3] = Flash ID 5 = DFU_FLASH_IMG_BOOT
    - imgs[4] = Flash ID 6 = DFU_FLASH_IMG_LCPU2
    - imgs[5] = Flash ID 7 = DFU_FLASH_IMG_BCPU2
    - imgs[6] = Flash ID 8 = DFU_FLASH_IMG_HCPU2
    ...
    """
    desc = bytearray()

    # Sizes array: index = Flash ID - 2
    sizes = [
        0xFFFFFFFF,      # [0] LCPU (not used in most configs)
        bootloader_size, # [1] Bootloader
        main_size,       # [2] HCPU (main app)
        0xFFFFFFFF,      # [3] Boot
        0xFFFFFFFF,      # [4] LCPU2
        0xFFFFFFFF,      # [5] BCPU2
        main_size,       # [6] HCPU2 (backup, same size as main)
        0xFFFFFFFF,      # [7] Boot2
    ]

    for idx in range(IMAGE_DESCRIPTION_COUNT):
        if idx < len(sizes):
            length = sizes[idx]
            used = length != 0xFFFFFFFF and length != 0
        else:
            length = 0xFFFFFFFF
            used = False
        desc.extend(pack_image_description(length, used))

    return bytes(desc)


def build_image_index_table(flash_base: int) -> bytes:
    """Build image index table

    Image Index entries point to image descriptions by CORE_* index:
    - entries[0] = CORE_LCPU → imgs[0] (Flash ID 2)
    - entries[1] = CORE_BL → imgs[1] (Flash ID 3)
    - entries[2] = CORE_HCPU → imgs[2] (Flash ID 4)
    - entries[3] = CORE_BOOT → imgs[3] (Flash ID 5)
    """
    base_offset = (
        flash_base
        + 4  # magic
        + PARTITION_ENTRY_COUNT * PARTITION_INFO_STRUCT.size
        + PUBKEY_SIZE
        + RESERVED_SIZE
    )

    # Index corresponds to CORE_* enum
    entries = [
        0xFFFFFFFF,                                   # [0] LCPU - not used
        base_offset + 1 * IMAGE_DESCRIPTION_SIZE,    # [1] Bootloader → imgs[1]
        base_offset + 2 * IMAGE_DESCRIPTION_SIZE,    # [2] HCPU → imgs[2]
        0xFFFFFFFF,                                   # [3] Boot - not used
    ]
    return struct.pack("<" + "I" * IMAGE_INDEX_COUNT, *entries)


def generate_ftab_binary(
    ptab_obj,
    chip_config: dict,
    bootloader_size: int,
    main_size: int
) -> bytes:
    """Generate complete ftab.bin content"""

    # Get partitions from v3 ptab
    partitions = ptab_obj.partitions

    # Get flash base for ftab (select based on memory type)
    flash_base = 0
    for p in partitions:
        ptype = p.get('type', '')
        if ptype == 'ftab':
            region = p.get('region', '')
            offset = ptab_module.parse_size(p.get('offset', 0))
            sbus_addr, cbus_addr = ptab_module.resolve_region_address(region, offset, chip_config)
            
            # Select base address based on memory type
            memory_info = chip_config.get('memory_info', {}).get(region, {})
            mem_type = memory_info.get('type', '').lower()
            flash_base = sbus_addr if mem_type in ('nand', 'psram', 'ram') else cbus_addr
            break

    # Build components
    entries = build_partition_table_entries(partitions, chip_config)
    partition_table = pack_partition_table(entries)
    image_descriptions = build_image_descriptions(bootloader_size, main_size)
    image_index_table = build_image_index_table(flash_base)

    # Assemble ftab binary
    blob = bytearray()
    blob.extend(struct.pack("<I", SEC_CONFIG_MAGIC))
    blob.extend(partition_table)
    blob.extend(bytes(PUBKEY_SIZE))
    blob.extend(bytes(RESERVED_SIZE))
    blob.extend(image_descriptions)
    blob.extend(image_index_table)

    return bytes(blob)


def main() -> None:
    args = parse_args()

    # Load ptab
    ptab_path = Path(args.ptab)
    if not ptab_path.exists():
        print(f"Error: ptab file not found: {ptab_path}")
        sys.exit(1)

    ptab_obj = ptab_module.load_ptab(str(ptab_path), fatal=True)

    # Only v3 format is supported
    if not ptab_obj.is_v3():
        print(f"Error: Only ptab v3 format is supported. Use ftab subproject for v1/v2.")
        sys.exit(1)

    # Get chip config
    chip_config = ptab_obj.get_chip_config()

    # Generate ftab binary
    ftab_binary = generate_ftab_binary(
        ptab_obj,
        chip_config,
        args.bootloader_size,
        args.main_size
    )

    # Write output
    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_bytes(ftab_binary)

    print(f"Generated ftab.bin: {output_path} ({len(ftab_binary)} bytes)")


if __name__ == "__main__":
    main()
