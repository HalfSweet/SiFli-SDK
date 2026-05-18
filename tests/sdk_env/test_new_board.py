#!/usr/bin/env python
#
# SPDX-FileCopyrightText: 2026 SiFli Technologies(Nanjing) Co., Ltd
#
# SPDX-License-Identifier: Apache-2.0

from __future__ import annotations

from pathlib import Path
from typing import Optional
import os
import sys
import tempfile
import unittest


ROOT = os.path.realpath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "tools"))

from sdk_py_actions.new_board import ChipVariant  # noqa: E402
from sdk_py_actions.new_board import MB  # noqa: E402
from sdk_py_actions.new_board import MemoryEntry  # noqa: E402
from sdk_py_actions.new_board import SAMPLE_BASE  # noqa: E402
from sdk_py_actions.new_board import Spec  # noqa: E402
from sdk_py_actions.new_board import build_confirmation_summary  # noqa: E402
from sdk_py_actions.new_board import normalize_spec  # noqa: E402
from sdk_py_actions.new_board import render_board_files  # noqa: E402


ASSETS_ROOT = Path(ROOT) / "tools" / "sdk_py_actions" / "new_board"
SCHEMA_PATH = ASSETS_ROOT / "schema" / "new-board.schema.json"
BASE_TEMPLATE_FILES = [
    "Kconfig.board",
    "board.h",
    "bsp_board.h",
    "bsp_init.c",
    "bsp_lcd_tp.c",
    "bsp_pinmux.c",
    "bsp_power.c",
]


class NewBoardRenderTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temp_dir = tempfile.TemporaryDirectory()
        self.root = Path(self.temp_dir.name).resolve()
        self.output_root = self.root / "output"
        self.output_root.mkdir()
        self.sdk_root = self.root / "sdk"
        self.sdk_root.mkdir()
        self.variant = ChipVariant(
            series="52",
            chip_dir="SF32LB52x",
            model_id="SF32LB52X",
            part_number="SF32LB52X",
            memory=(MemoryEntry("mpi1", "nor", 2 * MB),),
        )

    def tearDown(self) -> None:
        self.temp_dir.cleanup()

    def make_spec(
        self,
        base_name: str,
        generate_base: bool = False,
        series: str = "52",
        chip_model: str = "SF32LB52X",
        storage_type: str = "none",
        storage_size_mb: Optional[int] = None,
        storage_port: Optional[str] = None,
        storage_pinmux_type: Optional[str] = None,
    ) -> Spec:
        return Spec(
            series=series,
            chip_model=chip_model,
            storage_type=storage_type,
            storage_size_mb=storage_size_mb,
            storage_port=storage_port,
            board_name="demo_board",
            base_name=base_name,
            generate_base=generate_base,
            storage_pinmux_type=storage_pinmux_type,
        )

    def make_sample_base(self, series: str = "52", real_pinmux: bool = False) -> None:
        base_dir = self.sdk_root / SAMPLE_BASE[series]
        base_dir.mkdir(parents=True)
        for name in BASE_TEMPLATE_FILES:
            if real_pinmux and name == "bsp_pinmux.c":
                content = (Path(ROOT) / SAMPLE_BASE[series] / name).read_text(encoding="utf-8")
            else:
                content = f"// {name}\n"
            (base_dir / name).write_text(content, encoding="utf-8")

    def make_existing_base(self, base_dir: Path, base_kind: str = "base") -> None:
        base_dir.mkdir(parents=True)
        (base_dir / "Kconfig.board").write_text("config EXISTING_BASE\n", encoding="utf-8")
        if base_kind == "script":
            script_dir = base_dir / "script"
            script_dir.mkdir()
            (script_dir / "SConscript").write_text("# existing script base\n", encoding="utf-8")
        else:
            (base_dir / "SConscript.base").write_text("# existing base\n", encoding="utf-8")

    def render_top_kconfig(self, spec: Spec) -> str:
        rendered = render_board_files(
            spec=spec,
            variant=self.variant,
            sdk_root=self.sdk_root,
            output_root=self.output_root,
            assets_root=ASSETS_ROOT,
        )
        return rendered[self.output_root / spec.board_name / "Kconfig.board"]

    def render_top_sconscript(self, spec: Spec) -> str:
        rendered = render_board_files(
            spec=spec,
            variant=self.variant,
            sdk_root=self.sdk_root,
            output_root=self.output_root,
            assets_root=ASSETS_ROOT,
        )
        return rendered[self.output_root / spec.board_name / "SConscript"]

    def render_top_ptab(self, spec: Spec, variant: Optional[ChipVariant] = None) -> str:
        rendered = render_board_files(
            spec=spec,
            variant=variant or self.variant,
            sdk_root=self.sdk_root,
            output_root=self.output_root,
            assets_root=ASSETS_ROOT,
        )
        return rendered[self.output_root / spec.board_name / "ptab.yaml"]

    def test_generated_base_uses_relative_rsource(self) -> None:
        self.make_sample_base()

        content = self.render_top_kconfig(self.make_spec("demo_board_base", generate_base=True))

        self.assertEqual(content, 'rsource "../demo_board_base/Kconfig.board"\n')

    def test_generated_base_sconscript_uses_relative_path(self) -> None:
        self.make_sample_base()

        content = self.render_top_sconscript(self.make_spec("demo_board_base", generate_base=True))

        self.assertIn("SConscript('../demo_board_base/SConscript.base', variant_dir='base', duplicate=0)", content)

    def test_existing_base_name_uses_relative_rsource(self) -> None:
        self.make_existing_base(self.output_root / "shared_base")

        content = self.render_top_kconfig(self.make_spec("shared_base"))

        self.assertEqual(content, 'rsource "../shared_base/Kconfig.board"\n')

    def test_existing_base_name_sconscript_uses_relative_path(self) -> None:
        self.make_existing_base(self.output_root / "shared_base")

        content = self.render_top_sconscript(self.make_spec("shared_base"))

        self.assertIn("SConscript('../shared_base/SConscript.base', variant_dir='base', duplicate=0)", content)

    def test_existing_relative_base_path_uses_relative_rsource(self) -> None:
        self.make_existing_base(self.output_root / "bases" / "shared_base")

        content = self.render_top_kconfig(self.make_spec("bases/shared_base"))

        self.assertEqual(content, 'rsource "../bases/shared_base/Kconfig.board"\n')

    def test_existing_relative_script_base_sconscript_uses_relative_path(self) -> None:
        self.make_existing_base(self.output_root / "bases" / "shared_base", base_kind="script")

        content = self.render_top_sconscript(self.make_spec("bases/shared_base"))

        self.assertIn("SConscript('../bases/shared_base/script/SConscript', variant_dir='base', duplicate=0)", content)

    def test_existing_absolute_base_path_uses_source(self) -> None:
        external_base = self.root / "external" / "shared_base"
        self.make_existing_base(external_base)

        content = self.render_top_kconfig(self.make_spec(str(external_base)))

        self.assertEqual(content, f'source "{external_base.resolve().as_posix()}/Kconfig.board"\n')

    def test_existing_absolute_base_sconscript_uses_absolute_path(self) -> None:
        external_base = self.root / "external" / "shared_base"
        self.make_existing_base(external_base)

        content = self.render_top_sconscript(self.make_spec(str(external_base)))

        self.assertIn(
            f"SConscript('{external_base.resolve().as_posix()}/SConscript.base', variant_dir='base', duplicate=0)",
            content,
        )

    def test_52_sdmmc_ptab_uses_sdmmc1_region(self) -> None:
        self.make_sample_base()
        variant = ChipVariant(
            series="52",
            chip_dir="SF32LB52x",
            model_id="SF32LB52X",
            part_number="SF32LB52X",
            memory=(MemoryEntry("mpi1", "psram", 16 * MB),),
        )

        content = self.render_top_ptab(
            self.make_spec(
                "demo_board_base",
                generate_base=True,
                storage_type="sdmmc",
                storage_size_mb=16,
            ),
            variant=variant,
        )

        self.assertIn("  - sdmmc: sdmmc1\n    type: sd\n", content)
        self.assertIn("    region: sdmmc1\n", content)
        self.assertNotIn("mpi: mpi", content)

    def test_58_sdmmc_ptab_uses_sdmmc2_region(self) -> None:
        self.make_sample_base(series="58")
        variant = ChipVariant(
            series="58",
            chip_dir="SF32LB58x",
            model_id="SF32LB58X",
            part_number="SF32LB58X",
            memory=(MemoryEntry("mpi1", "psram", 32 * MB),),
        )

        content = self.render_top_ptab(
            self.make_spec(
                "demo_board_base",
                generate_base=True,
                series="58",
                chip_model="SF32LB58X",
                storage_type="sdmmc",
                storage_size_mb=16,
            ),
            variant=variant,
        )

        self.assertIn("  - sdmmc: sdmmc2\n    type: sd\n", content)
        self.assertIn("    region: sdmmc2\n", content)
        self.assertNotIn("mpi: mpi4", content)

    def test_58_sdmmc1_type1_uses_sdmmc1_region_and_jlink_suffix(self) -> None:
        self.make_existing_base(self.output_root / "shared_base")
        variant = ChipVariant(
            series="58",
            chip_dir="SF32LB58x",
            model_id="SF32LB58X",
            part_number="SF32LB58X",
            memory=(MemoryEntry("mpi1", "psram", 32 * MB),),
        )
        spec = self.make_spec(
            "shared_base",
            series="58",
            chip_model="SF32LB58X",
            storage_type="sdmmc",
            storage_size_mb=16,
            storage_port="sdmmc1",
            storage_pinmux_type="type1",
        )

        rendered = render_board_files(
            spec=spec,
            variant=variant,
            sdk_root=self.sdk_root,
            output_root=self.output_root,
            assets_root=ASSETS_ROOT,
        )

        ptab = rendered[self.output_root / spec.board_name / "ptab.yaml"]
        board_conf = rendered[self.output_root / spec.board_name / "hcpu" / "board.conf"]
        rtconfig = rendered[self.output_root / spec.board_name / "hcpu" / "rtconfig.py"]
        self.assertIn("  - sdmmc: sdmmc1\n    type: sd\n", ptab)
        self.assertIn("    region: sdmmc1\n", ptab)
        self.assertIn("CONFIG_BSP_USING_SDMMC1=y\n", board_conf)
        self.assertIn("CONFIG_SD_MAX_FREQ=48000000\n", board_conf)
        self.assertNotIn("CONFIG_BSP_USING_SDMMC2=y\n", board_conf)
        self.assertIn("JLINK_DEVICE = 'SF32LB58X_SD_TYPE1'\n", rtconfig)

    def test_generated_58_type1_base_specializes_storage_pinmux(self) -> None:
        self.make_sample_base(series="58", real_pinmux=True)
        variant = ChipVariant(
            series="58",
            chip_dir="SF32LB58x",
            model_id="SF32LB58X",
            part_number="SF32LB58X",
            memory=(MemoryEntry("mpi1", "psram", 32 * MB),),
        )
        spec = self.make_spec(
            "demo_board_base",
            generate_base=True,
            series="58",
            chip_model="SF32LB58X",
            storage_type="nand",
            storage_size_mb=128,
            storage_port="mpi4",
            storage_pinmux_type="type1",
        )

        rendered = render_board_files(
            spec=spec,
            variant=variant,
            sdk_root=self.sdk_root,
            output_root=self.output_root,
            assets_root=ASSETS_ROOT,
        )

        pinmux = rendered[self.output_root / spec.base_name / "bsp_pinmux.c"]
        rtconfig = rendered[self.output_root / spec.board_name / "hcpu" / "rtconfig.py"]
        self.assertIn("HAL_PIN_Set(PAD_PA39, MPI4_CLK, PIN_NOPULL, 1);", pinmux)
        self.assertIn("HAL_PIN_Set(PAD_PA49, GPIO_A49, PIN_PULLUP, 1);", pinmux)
        self.assertNotIn("HAL_PIN_Set(PAD_PA09, MPI4_CLK, PIN_NOPULL, 1);", pinmux)
        self.assertNotIn("HAL_PIN_Set(PAD_PA66, GPIO_A66, PIN_PULLUP, 1);", pinmux)
        self.assertIn("JLINK_DEVICE = 'SF32LB58X_NAND_TYPE1'\n", rtconfig)

    def test_normalize_spec_allows_absolute_existing_base_path(self) -> None:
        external_base = self.root / "external" / "shared_base"

        spec, _variant = normalize_spec(
            {
                "chip_series": "52",
                "chip_model": "SF32LB52X",
                "storage": {"type": "none"},
                "board_name": "demo_board",
                "generate_base": False,
                "base_name": str(external_base),
            },
            SCHEMA_PATH,
            {"52": [self.variant]},
        )

        self.assertEqual(spec.base_name, str(external_base))

    def test_normalize_58_pinmux_type_rules(self) -> None:
        variant = ChipVariant(
            series="58",
            chip_dir="SF32LB58x",
            model_id="SF32LB58X",
            part_number="SF32LB58X",
            memory=(MemoryEntry("mpi1", "psram", 32 * MB),),
        )

        with self.assertRaisesRegex(Exception, "storage_port=mpi4 requires"):
            normalize_spec(
                {
                    "chip_series": "58",
                    "chip_model": "SF32LB58X",
                    "storage": {"type": "nor", "size_mb": 16},
                    "storage_port": "mpi4",
                    "board_name": "demo_board",
                    "generate_base": False,
                    "base_name": "shared_base",
                },
                SCHEMA_PATH,
                {"58": [variant]},
            )

        with self.assertRaisesRegex(Exception, "pinmux_type is only supported"):
            normalize_spec(
                {
                    "chip_series": "58",
                    "chip_model": "SF32LB58X",
                    "storage": {"type": "sdmmc", "size_mb": 16, "pinmux_type": "type1"},
                    "storage_port": "sdmmc2",
                    "board_name": "demo_board",
                    "generate_base": False,
                    "base_name": "shared_base",
                },
                SCHEMA_PATH,
                {"58": [variant]},
            )

    def test_confirmation_summary_explains_generated_base(self) -> None:
        summary = build_confirmation_summary(
            series="52",
            chip_model="SF32LB52X",
            storage_type="none",
            storage_port=None,
            storage_size_mb=None,
            board_name="demo_board",
            base_name="demo_board_base",
            generate_base=True,
        )

        self.assertIn("[Base board]", summary)
        self.assertIn("  mode: create new base board", summary)
        self.assertIn("  base_name_or_path: demo_board_base", summary)

    def test_confirmation_summary_explains_absolute_existing_base(self) -> None:
        external_base = self.root / "external" / "shared_base"

        summary = build_confirmation_summary(
            series="52",
            chip_model="SF32LB52X",
            storage_type="nor",
            storage_port=None,
            storage_size_mb=16,
            board_name="demo_board",
            base_name=str(external_base),
            generate_base=False,
        )

        self.assertIn("  mode: reuse existing base board", summary)
        self.assertIn(f"  base_name_or_path: {external_base}", summary)


if __name__ == "__main__":
    unittest.main()
