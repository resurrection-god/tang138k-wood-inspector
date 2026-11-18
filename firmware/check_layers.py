"""Firmware layer check: a module depends only on modules strictly BELOW it.

The graph comes from the built object files firmware/Debug/src/app/*.o (nm): an
edge A -> B if A uses a function or data defined in B. These are the real
dependencies, not #include. Run after `make all`:

    python firmware/check_layers.py              # 0 = clean, otherwise the number of violations
    python firmware/check_layers.py --graph      # also print the whole graph
    python firmware/check_layers.py --selftest   # make sure the check can fail

A new application module must be added to LAYERS, otherwise the check refuses.
The layers and why they are so: CLAUDE.md, "Firmware".
"""
import glob
import os
import subprocess
import sys
import time
from collections import defaultdict

HERE = os.path.dirname(os.path.abspath(__file__))
OBJ = os.path.join(HERE, "Debug", "src", "app")
NM = r"C:\Andestech\AndeSight_RDS_v511\toolchains\nds32le-elf-mculib-v5\bin\riscv32-elf-nm.exe"
ENV = dict(os.environ, PATH=r"C:\Andestech\AndeSight_RDS_v511\cygwin\bin;" + os.environ.get("PATH", ""),
           HOME=r"C:\Andestech\AndeSight_RDS_v511\ice", SHELL="/bin/bash")

# Bottom up. Modules within a layer do not depend on each other.
LAYERS = [
    ["watchdog", "gvsp_gate", "spi_flash"],                # hardware WDT and its record; the delivery gate (pure); SPI flash
    ["board", "ethernet", "vmsp_protocol", "mailbox",    # board, MAC/PHY, framing, mailbox,
     "frame_udp", "gev_xml"],                              # udp_tx job in the FPGA (asks flash_ctl FEATURES); GenApi XML (data)
    ["i2c_bus"],                                           # I2C bus and multiplexer
    ["camera"],                                            # devices on the bus
    ["sync", "laser", "cam_trig", "mm_table", "nvstore", "ota"], # FPGA blocks via the mailbox; settings store; OTA
    ["video"],                                             # capture and the ring (camera + synchroniser)
    ["settings", "work_mode", "net_config"],               # storage; modes and their sets; address
    ["lcd_hud"],                                           # RGB666 LCD HUD over APB (Slot B status/log)
    ["hik_client"],                                        # Autonomous Hikrobot GigE Vision client -> lcd_hud
    ["vmsp_service", "console", "gvsp"],                   # protocol, UART console, GVSP frame scheduler
    ["gvcp"],                                              # GigE Vision control: calls VMSP handlers in-process
    ["main"],                                              # system assembly
]


def nm(path):
    for _ in range(30):                     # cygwin add_item sometimes fails at start-up
        r = subprocess.run([NM, path], capture_output=True, text=True, env=ENV)
        if r.returncode == 0:
            return r.stdout
        time.sleep(1)
    r.check_returncode()


def graph():
    defs, uses = {}, {}
    for o in sorted(glob.glob(os.path.join(OBJ, "*.o"))):
        m = os.path.basename(o)[:-2]
        d, u = set(), set()
        for line in nm(o).splitlines():
            p = line.split()
            if len(p) == 2 and p[0] == "U":
                u.add(p[1])
            elif len(p) == 3 and p[1] in "TDBRCGSV":   # G/S: RISC-V small data sections
                d.add(p[2])
        defs[m], uses[m] = d, u
    owner = {s: m for m, ds in defs.items() for s in ds}
    edges = defaultdict(lambda: defaultdict(set))
    for m, us in uses.items():
        for s in us:
            if s in owner and owner[s] != m:
                edges[m][owner[s]].add(s)
    return sorted(defs), edges


def check(mods, edges):
    layer = {m: i for i, ms in enumerate(LAYERS) for m in ms}
    bad = 0
    for m in mods:
        if m not in layer:
            print(f"НЕТ В ТАБЛИЦЕ СЛОЁВ: {m} — впишите модуль в LAYERS")
            bad += 1
    for a in mods:
        for b, syms in sorted(edges[a].items()):
            if a in layer and b in layer and layer[b] >= layer[a]:
                kind = "в своём слое" if layer[b] == layer[a] else "вверх"
                print(f"НАРУШЕНИЕ ({kind}): {a} (слой {layer[a]}) -> {b} (слой {layer[b]}): "
                      f"{', '.join(sorted(syms))}")
                bad += 1
    return bad


def main():
    if not glob.glob(os.path.join(OBJ, "*.o")):
        sys.exit(f"нет объектных файлов в {OBJ}: сначала make all")
    mods, edges = graph()
    if "--graph" in sys.argv:
        for m in mods:
            print(f"  {m:14s} -> " + ", ".join(f"{t}({len(s)})" for t, s in sorted(edges[m].items())))
    if "--selftest" in sys.argv:
        # The forbidden upward edge, as it was before 24.09.2026: settings calls work_mode.
        edges["settings"]["work_mode"].add("work_mode_get")
        n = check(mods, edges)
        print(f"selftest: подставлено 1 нарушение, найдено {n} — "
              + ("проверка работает" if n >= 1 else "ПРОВЕРКА СЛЕПА"))
        sys.exit(0 if n >= 1 else 1)
    n = check(mods, edges)
    print(f"слоёв {len(LAYERS)}, модулей {len(mods)}: " + ("чисто" if n == 0 else f"нарушений {n}"))
    sys.exit(n)


if __name__ == "__main__":
    main()
