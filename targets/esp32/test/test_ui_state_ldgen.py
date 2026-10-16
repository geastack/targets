#!/usr/bin/env python3
"""Exercise IDF's real ldgen and linker; requires IDF_PATH and ESP_TOOLCHAIN_PREFIX.

GEA_TEST_BUILD_DIR selects an existing ignored build directory. This links only
small section fixtures, not firmware, and never accesses a board.
"""

import io
import json
import os
from pathlib import Path
import re
import subprocess
import struct
import sys

idf = Path(os.environ["IDF_PATH"])
tool = os.environ["ESP_TOOLCHAIN_PREFIX"]
build = Path(os.environ["GEA_TEST_BUILD_DIR"]).resolve()
build.mkdir(parents=True, exist_ok=True)
fragment = Path(
    os.environ.get(
        "GEA_TEST_UI_STATE_FRAGMENT",
        Path(__file__).resolve().parents[1] / "ldfragments/gea_ui_state_external.lf",
    )
)
sys.path.insert(0, str(idf / "tools/ldgen"))
from ldgen.entity import EntityDB
from ldgen.fragments import parse_fragment_file
from ldgen.generation import Generation
from ldgen.linker_script import LinkerScript


class Config:
    def evaluate_expression(self, expression):
        # Only conditions in the real common/app fragments used by this test.
        assert expression in (
            "APP_BUILD_USE_FLASH_SECTIONS = y",
            "ESP_ALLOW_BSS_SEG_EXTERNAL_MEMORY = y",
        ), expression
        return True


def text_file(text, name):
    file = io.StringIO(text)
    file.name = name
    return file


def run(args):
    return subprocess.check_output(args, text=True, stderr=subprocess.STDOUT)


# Distinct objects exercise archive defaults, object overrides, symbol overrides
# and initialized data. Assembly fixes section spelling independently of the
# host/compiler's own small-data threshold.
def object_file(name, declarations):
    source = build / f"ui-placement-{name}.S"
    source.write_text(
        "\n".join(
            f'.section {section},"aw",@{kind}\n.balign 8\n.global {symbol}\n'
            f".type {symbol},@object\n{symbol}:\n{value}\n.size {symbol},.-{symbol}\n"
            for section, symbol, kind, value in declarations
        )
    )
    obj = build / f"{name}.c.obj"
    run([tool + "gcc", "-c", str(source), "-o", str(obj)])
    return obj


def states(owner):
    return [
        (f".bss.{owner}_bss", f"{owner}_bss", "nobits", ".zero 64"),
        (f".sbss.{owner}_small", f"{owner}_small", "nobits", ".zero 8"),
        (f".sbss2.{owner}_small2", f"{owner}_small2", "nobits", ".zero 4"),
        (f".data.{owner}_data", f"{owner}_data", "progbits", ".word 0x13572468"),
        (f".sdata.{owner}_sdata", f"{owner}_sdata", "progbits", ".word 0x24681357"),
    ]


objects = {
    "libgea_framework.a": [object_file("canvas", states("cache"))],
    "libgea_active_geatsc.a": [object_file("generated", states("app"))],
    "libmain.a": [
        object_file("frame_scheduler", states("scheduler")),
        object_file(
            "display",
            states("display")
            + [
                (".bss.flush_pool", "flush_pool", "nobits", ".zero 128"),
                (".sbss.flush_small", "flush_small", "nobits", ".zero 4"),
            ],
        ),
        object_file("board_driver", states("board")),
    ],
}
archives = []
entities = EntityDB()
for archive, members in objects.items():
    path = build / archive
    if path.exists():
        path.unlink()
    run([tool + "ar", "rcs", str(path), *map(str, members)])
    entities.add_sections_info(text_file(run([tool + "objdump", "-h", str(path)]), archive))
    archives.append(path)

# .sdata is normally swept by the IDF linker template, not by the common data
# section. Mirror that unchanged catch-all explicitly in this tiny template.
template = """SECTIONS {
 . = 0x10000;
 .dram : {
  mapping[dram0_data]
  mapping[dram0_bss]
  *(.sdata .sdata.*)
 }
 . = 0x20000;
 .external (NOLOAD) : {
  mapping[extern_ram]
 }
 . = 0x30000;
 .rtc (NOLOAD) : {
  mapping[rtc_force_fast]
 }
 /DISCARD/ : { *(.riscv.attributes .text .eh_frame) }
}"""
overrides = """
[scheme:test_rtc]
entries:
    bss -> rtc_force_fast
    common -> rtc_force_fast
[mapping:test_scheduler_internal]
archive: libmain.a
entries:
    frame_scheduler (test_rtc)
[mapping:test_flush_internal]
archive: libmain.a
entries:
    display:flush_pool (default)
    display:flush_small (default)
"""
results = []
for enabled in (False, True):
    generation = Generation()
    for source in (
        idf / "components/esp_common/common.lf",
        idf / "components/esp_common/soc.lf",
        idf / "components/esp_system/app.lf",
    ):
        generation.add_fragments_from_file(parse_fragment_file(str(source), Config()))
    if enabled:
        generation.add_fragments_from_file(parse_fragment_file(str(fragment), Config()))
    generation.add_fragments_from_file(
        parse_fragment_file(text_file(overrides, "overrides.lf"), Config())
    )
    script = LinkerScript(text_file(template, "sections.ld.in"))
    script.fill(generation.generate(entities, False))
    ld = build / f"ui-placement-{int(enabled)}.ld"
    with ld.open("w") as output:
        script.write(output)
    elf = build / f"ui-placement-{int(enabled)}.elf"
    run([tool + "ld", "-T", str(ld), "-o", str(elf), "--whole-archive", *map(str, archives)])
    symbols = {}
    for line in run([tool + "nm", "-n", str(elf)]).splitlines():
        match = re.match(r"([0-9a-f]+) [A-Za-z] (\w+)$", line)
        if match:
            symbols[match[2]] = int(match[1], 16)
    for owner in ("cache", "app", "display", "board", "scheduler"):
        for suffix in ("bss", "small", "small2", "data", "sdata"):
            symbol = f"{owner}_{suffix}"
            expected = 0x10000
            if owner == "scheduler" and suffix == "bss":
                expected = 0x30000
            elif (
                enabled
                and owner in ("cache", "app", "display")
                and suffix in ("bss", "small", "small2")
            ):
                expected = 0x20000
            assert expected <= symbols[symbol] < expected + 0x10000, (
                enabled,
                symbol,
                hex(symbols[symbol]),
                hex(expected),
            )
    data = build / f"ui-placement-{int(enabled)}.dram.bin"
    run([tool + "objcopy", "--dump-section", f".dram={data}", str(elf)])
    contents = data.read_bytes()
    for owner in ("cache", "app", "display", "board", "scheduler"):
        assert (
            struct.unpack_from("<I", contents, symbols[f"{owner}_data"] - 0x10000)[0] == 0x13572468
        )
        assert (
            struct.unpack_from("<I", contents, symbols[f"{owner}_sdata"] - 0x10000)[0] == 0x24681357
        )
    assert 0x10000 <= symbols["flush_pool"] < 0x20000
    assert 0x10000 <= symbols["flush_small"] < 0x20000
    results.append(
        {"external_enabled": enabled, "symbols": {k: hex(v) for k, v in symbols.items()}}
    )
(build / "ldgen-proof.json").write_text(json.dumps(results, indent=2) + "\n")
print(
    "IDF ldgen/link: archive, small-BSS, initialized data, RTC object and DMA-symbol overrides passed"
)
