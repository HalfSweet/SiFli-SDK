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
    chip_config: dict,
    chip: Optional[str] = None,
) -> List[Tuple[int, int, int, int]]:
    """Build partition table entries from ptab partitions.

    v3 address rules:
    - entry.base uses the "download/storage" view:
      - NOR: XIP (cbus)
      - NAND/RAM/PSRAM: base (sbus)
    - entry.xip_base uses the "execution" view:
      - default: XIP (cbus)
      - RAM/NAND: base (sbus)
    - For app/bootloader partitions with `acc`, xip_base comes from acc.{region,offset}.

    Returns:
        List of (base, size, xip_base, flags) tuples
    """
    entries = [(0, 0, 0, 0)] * PARTITION_ENTRY_COUNT

    def _mpi_name_from_region(region: str) -> Optional[str]:
        if not region:
            return None
        if region.startswith('mpi'):
            return region
        if region.startswith('psram'):
            if region == 'psram':
                return 'mpi1'
            suffix = region.replace('psram', '')
            if suffix.isdigit():
                return f'mpi{suffix}'
        return None

    def _get_region_mem_type(region: str) -> str:
        if region == 'hpsys_ram' or region.startswith('hpsys') or region == 'lpsys_ram' or region.startswith('lpsys'):
            return 'ram'
        mpi_name = _mpi_name_from_region(region)
        if mpi_name:
            info = chip_config.get('memory_info', {}).get(mpi_name, {})
            mtype = (info.get('type') or '').lower()
            return mtype or 'nor'
        return ''

    def _select_download_addr(region: str, sbus_addr: int, cbus_addr: int) -> int:
        return cbus_addr if _get_region_mem_type(region) == 'nor' else sbus_addr

    def _select_exec_addr(region: str, sbus_addr: int, cbus_addr: int) -> int:
        mem_type = _get_region_mem_type(region)
        return sbus_addr if mem_type in ('ram', 'nand', 'psram') else cbus_addr

    def _get_flash_boot_loader_size_default() -> Optional[int]:
        try:
            sifli_sdk = os.getenv('SIFLI_SDK')
            if not sifli_sdk:
                # Fallback for offline tools
                sifli_sdk = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
            if not sifli_sdk:
                return None
            chip_u = (chip or '').upper()
            if chip_u.startswith('SF32LB52'):
                cmsis_dir = 'sf32lb52x'
            else:
                return None
            import gen_link_lds
            mem_ints = gen_link_lds._load_mem_map_ints(sifli_sdk, cmsis_dir)
            v = mem_ints.get('FLASH_BOOT_LOADER_SIZE')
            if v is None:
                return None
            v = int(v)
            return v if v > 0 else None
        except Exception:
            return None

    # Find key partitions
    ftab_partition = None
    calibration_partition = None
    bootloader_partition = None
    main_partition = None
    dfu_partition = None

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

    # Fill ftab entry
    if ftab_partition:
        size = ptab_module.parse_size(ftab_partition.get('size', 0))
        region = ftab_partition.get('region', '')
        offset = ptab_module.parse_size(ftab_partition.get('offset', 0))
        sbus_addr, cbus_addr = ptab_module.resolve_region_address(region, offset, chip_config)
        base_addr = _select_download_addr(region, sbus_addr, cbus_addr)
        
        entries[PartitionIndex.FLASH_PARTITION_TABLE] = (base_addr, size, 0, 0)

    # Fill calibration entry
    if calibration_partition:
        size = ptab_module.parse_size(calibration_partition.get('size', 0))
        region = calibration_partition.get('region', '')
        offset = ptab_module.parse_size(calibration_partition.get('offset', 0))
        sbus_addr, cbus_addr = ptab_module.resolve_region_address(region, offset, chip_config)
        base_addr = _select_download_addr(region, sbus_addr, cbus_addr)
        
        entries[PartitionIndex.CALIBRATION_TABLE] = (base_addr, size, 0, 0)

    # Fill bootloader entry
    if bootloader_partition:
        storage_size = ptab_module.parse_size(bootloader_partition.get('size', 0))
        size = _get_flash_boot_loader_size_default()
        if size is None:
            size = storage_size
        elif storage_size > size:
            size = storage_size
        region = bootloader_partition.get('region', '')
        offset = ptab_module.parse_size(bootloader_partition.get('offset', 0))
        sbus_addr, cbus_addr = ptab_module.resolve_region_address(region, offset, chip_config)
        base_addr = _select_download_addr(region, sbus_addr, cbus_addr)

        acc = bootloader_partition.get('acc')
        if not isinstance(acc, dict):
            raise ValueError("bootloader partition must provide acc for ftab generation")

        exec_region = str(acc.get('region', '')).strip()
        exec_offset = ptab_module.parse_size(acc.get('offset', 0))
        exec_sbus_addr, exec_cbus_addr = ptab_module.resolve_region_address(exec_region, exec_offset, chip_config)
        xip_addr = _select_exec_addr(exec_region, exec_sbus_addr, exec_cbus_addr)

        entries[PartitionIndex.BOOTLOADER] = (base_addr, size, xip_addr, 0)
        entries[PartitionIndex.BOOTLOADER_IMAGE_PONG] = (base_addr, size, xip_addr, 0)

    # Fill main entry
    if main_partition:
        region = main_partition.get('region', '')
        offset = ptab_module.parse_size(main_partition.get('offset', 0))
        size = ptab_module.parse_size(main_partition.get('size', 0))
        sbus_addr, cbus_addr = ptab_module.resolve_region_address(region, offset, chip_config)
        base_addr = _select_download_addr(region, sbus_addr, cbus_addr)

        acc = main_partition.get('acc')
        if isinstance(acc, dict):
            exec_region = str(acc.get('region', '')).strip()
            exec_offset = ptab_module.parse_size(acc.get('offset', 0))
            exec_sbus_addr, exec_cbus_addr = ptab_module.resolve_region_address(exec_region, exec_offset, chip_config)
            xip_addr = _select_exec_addr(exec_region, exec_sbus_addr, exec_cbus_addr)
        else:
            # No acc: allow XIP for NOR/PSRAM; NAND must provide acc
            mem_type = _get_region_mem_type(region)
            if mem_type == 'nand':
                raise ValueError("app.factory partition on NAND must provide acc for ftab generation")
            xip_addr = _select_exec_addr(region, sbus_addr, cbus_addr)

        entries[PartitionIndex.HCPU_IMAGE] = (base_addr, size, xip_addr, 0)
        entries[PartitionIndex.HCPU_IMAGE_PONG] = (base_addr, size, xip_addr, 0)

    # Fill dfu entry (optional)
    if dfu_partition:
        region = dfu_partition.get('region', '')
        offset = ptab_module.parse_size(dfu_partition.get('offset', 0))
        size = ptab_module.parse_size(dfu_partition.get('size', 0))
        sbus_addr, cbus_addr = ptab_module.resolve_region_address(region, offset, chip_config)
        base_addr = _select_download_addr(region, sbus_addr, cbus_addr)

        acc = dfu_partition.get('acc')
        if isinstance(acc, dict):
            exec_region = str(acc.get('region', '')).strip()
            exec_offset = ptab_module.parse_size(acc.get('offset', 0))
            exec_sbus_addr, exec_cbus_addr = ptab_module.resolve_region_address(exec_region, exec_offset, chip_config)
            xip_addr = _select_exec_addr(exec_region, exec_sbus_addr, exec_cbus_addr)
        else:
            mem_type = _get_region_mem_type(region)
            if mem_type == 'nand':
                raise ValueError("app.dfu partition on NAND must provide acc for ftab generation")
            xip_addr = _select_exec_addr(region, sbus_addr, cbus_addr)

        entries[PartitionIndex.LCPU_IMAGE_PING] = (base_addr, size, xip_addr, 0)
        entries[PartitionIndex.LCPU_IMAGE_PONG] = (base_addr, size, xip_addr, 0)

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
        dfu_size if dfu_size > 0 else 0xFFFFFFFF,  # [0] LCPU/DFU image
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


def build_image_index_table(flash_base: int, dfu_present: bool = False) -> bytes:
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
        base_offset + 0 * IMAGE_DESCRIPTION_SIZE if dfu_present else 0xFFFFFFFF,  # [0] LCPU/DFU
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

    # Get flash base for ftab (download/storage view)
    flash_base = 0
    for p in partitions:
        ptype = p.get('type', '')
        if ptype == 'ftab':
            region = p.get('region', '')
            offset = ptab_module.parse_size(p.get('offset', 0))
            sbus_addr, cbus_addr = ptab_module.resolve_region_address(region, offset, chip_config)
            # Reuse the same selection rule as partition table entries
            mpi_name = region if region.startswith('mpi') else None
            if region.startswith('psram'):
                mpi_name = 'mpi1' if region == 'psram' else 'mpi{}'.format(region.replace('psram', '') or '1')
            mem_type = (chip_config.get('memory_info', {}).get(mpi_name or region, {}).get('type') or 'nor').lower()
            flash_base = cbus_addr if mem_type == 'nor' else sbus_addr
            break

    # Build components
    entries = build_partition_table_entries(partitions, chip_config, getattr(ptab_obj, 'chip', None))
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
