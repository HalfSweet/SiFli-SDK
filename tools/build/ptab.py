import copy
import json
import logging
import os
from collections import OrderedDict

_PTAB_CACHE = {}


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

    def content_mems(self, clone=True):
        mems = self.mems[1:] if self._has_header else self.mems
        return copy.deepcopy(mems) if clone else mems

    def ensure_default_regions(self):
        mems = self.content_mems()
        if self.is_v2():
            add_default_regions(mems)
        return mems


def _parse_ptab_file(path):
    with open(path) as f:
        return json.load(f, object_pairs_hook=OrderedDict)


def load_ptab(path, fatal=False):
    abspath = os.path.abspath(str(path))
    try:
        stat = os.stat(abspath)
        key = (abspath, stat.st_mtime_ns, stat.st_size)
        if key in _PTAB_CACHE:
            return _PTAB_CACHE[key]
        mems = _parse_ptab_file(abspath)
    except ValueError as e:
        if fatal:
            print("ptab.json syntax error, might be caused by trailing comma of last item")
            print("Error message: {}".format(e))
            print("Please check file {}".format(abspath))
            raise SystemExit(1)
        raise

    ptab = Ptab(abspath, mems)
    _PTAB_CACHE[key] = ptab
    return ptab


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
