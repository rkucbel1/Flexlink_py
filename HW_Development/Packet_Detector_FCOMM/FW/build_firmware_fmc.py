"""
build_firmware_fmc.py  (Flexlink_py/HW_Development/Packet_Detector_FCOMM/FW)

Builds the FMCOMMS3 firmware with the unified Vitis (2025.2) Python API, the
same way as ../../Packet_Detector/FW/build_firmware.py (whose platform fixes are copied below
unchanged: lwIP settings, PHY timeout, VFP save in the IRQ handler, rx-interrupt print, IRQ stack):
  1. platform from ../HW/build_fmcomms3/fmc_zed.xsa (standalone, Cortex-A9 #0) with lwIP
  2. application from the "lwIP Echo Server" template; echo.c is replaced by the server, and
     fmc_radio.c, fmc_ad9361_param.c, app_config.h are added. Two applications:
       FMC_APP=receiver (default)  fmc_pd_server.c: the FlexLink receiver pd_0 fed from the
                                   AD9361 (step 4; needs the step-4 bitstream)
       FMC_APP=capture             fmc_server.c: AD9361 capture only (step 3; any bitstream)
  3. ADI no-OS (AD9361 driver and the Xilinx platform layer) is compiled from its own checkout,
     through the template's UserConfig.cmake (USER_COMPILE_SOURCES / USER_INCLUDE_DIRECTORIES).
     The no-OS files are found by following the #include lines of our sources: every no-OS header
     that is used brings in the .c file of the same name (no_os_spi.h -> drivers/api/no_os_spi.c).
  4. build -> workspace/<app>/build/<app>.elf (fmc_pd_server or fmc_server)

    export NOOS_DIR=~/SiskinRF/no-OS        # default: next to Flexlink_py
    cd Flexlink_py/HW_Development/Packet_Detector_FCOMM/FW
    vitis -s build_firmware_fmc.py                  # receiver
    FMC_APP=capture vitis -s build_firmware_fmc.py  # capture server

The workspace folder is deleted and recreated on every run; edit the sources here.
"""
import glob
import os
import re
import shutil
import subprocess
import sys

import vitis

HERE = os.path.dirname(os.path.abspath(__file__))
WS = os.path.join(HERE, "workspace")
XSA = os.path.normpath(os.path.join(HERE, "..", "HW", "build_fmcomms3", "fmc_zed.xsa"))
NOOS = os.path.normpath(os.environ.get("NOOS_DIR", os.path.join(HERE, "..", "..", "..", "..", "no-OS")))
PLATFORM = "fmc_zed_platform"
DOMAIN = "standalone_ps7_cortexa9_0"
FMC_APP = os.environ.get("FMC_APP", "receiver")
if FMC_APP not in ("receiver", "capture"):
    sys.exit(f"FMC_APP must be receiver or capture, not {FMC_APP!r}")
APP = "fmc_pd_server" if FMC_APP == "receiver" else "fmc_server"
TEMPLATE = "lwip_echo_server"
LWIP = "lwip220"

# Our sources: the first replaces the template's echo.c
APP_SOURCES = [APP + ".c", "fmc_radio.c", "fmc_ad9361_param.c"]
APP_HEADERS = ["fmc_radio.h", "app_config.h"]

# no-OS: where its headers and sources are looked for (nothing else of no-OS is used)
NOOS_INC_DIRS = ["include", "drivers/platform/xilinx", "drivers/rf-transceiver/ad9361",
                 "drivers/axi_core/axi_adc_core", "drivers/axi_core/axi_dac_core"]
NOOS_SRC_DIRS = ["drivers/api", "util", "drivers/platform/xilinx", "drivers/rf-transceiver/ad9361",
                 "drivers/axi_core/axi_adc_core", "drivers/axi_core/axi_dac_core"]
# sources no header names: the AD9361 converter interface (declared in ad9361.h) and the Xilinx
# implementations of no_os_delay.h and no_os_axi_io.h
NOOS_EXTRA_SOURCES = ["drivers/rf-transceiver/ad9361/ad9361_conv.c",
                      "drivers/platform/xilinx/xilinx_delay.c",
                      "drivers/platform/xilinx/xilinx_axi_io.c"]
NOOS_DEFINES = ["XILINX_PLATFORM"]
HEAP_SIZE = "0x100000"              # the AD9361 driver allocates its state on the heap

# Board address when no DHCP server answers (direct cable to the PC). The template's
# default 192.168.1.10 clashes with home networks on 192.168.1.x, so main.c is patched.
BOARD_IP = (192, 168, 50, 10)
BOARD_GW = (192, 168, 50, 1)

# Ethernet link speed. "auto" (normal): lwIP restarts auto-negotiation and waits for the
# result. A fixed "1000" brought the link down ("Ethernet Link down" on the console): gigabit
# always needs auto-negotiation. The earlier "Auto negotiation error" came from the missing
# tick timer (XILTIMER_tick_timer below), not from the link. Use "100" only if the link
# drops to 100 Mbps (cable without four good pairs).
LINK_SPEED = "auto"
LINK_SPEED_VALUES = {"auto": "CONFIG_LINKSPEED_AUTODETECT", "1000": "CONFIG_LINKSPEED1000",
                     "100": "CONFIG_LINKSPEED100", "10": "CONFIG_LINKSPEED10"}

# lwIP settings for bulk transfers (MB-sized messages). They must satisfy lwIP's own
# sanity checks in init.c: TCP_SND_QUEUELEN = 16 * TCP_SND_BUF / TCP_MSS = 718 segments,
# so the segment pool (memp_n_tcp_seg) must hold at least that many. If the BSP still does
# not build with them, the script rebuilds with lwIP's defaults (works, only slower).
LWIP_CONFIG = {
    "lwip220_tcp_snd_buf": "65535",
    "lwip220_tcp_wnd": "65535",
    "lwip220_memp_n_tcp_seg": "1024",
    "lwip220_mem_size": "524288",
    "lwip220_memp_n_pbuf": "1024",
}

# Settings the lwip_echo_server template insists on (create_app_component checks them).
# Always applied, also in the fallback build. 2048 pool buffers of ~1.7 KB cover the
# 64 KB TCP window many times over.
TEMPLATE_CONFIG = [
    ("lwip220", "lwip220_dhcp", "true"),
    ("lwip220", "lwip220_lwip_dhcp_does_acd_check", "true"),
    ("lwip220", "lwip220_pbuf_pool_size", "2048"),
    ("xiltimer", "XILTIMER_en_interval_timer", "true"),
    # The 50 ms lwIP tick (TCP timers, DHCP timeout) needs a hardware timer. The default is
    # None (XTIMER_NO_TICK_TIMER): no interrupt ever fires and main.c waits for DHCP forever.
    ("xiltimer", "XILTIMER_tick_timer", "ps7_ttc_0"),
]


def build_platform(client, lwip_config):
    platform = client.create_platform_component(
        name=PLATFORM, hw_design=XSA, os="standalone", cpu="ps7_cortexa9_0",
        domain_name=DOMAIN, compiler="gcc")
    domain = platform.get_domain(name=DOMAIN)
    try:
        domain.set_lib(lib_name=LWIP)
    except Exception as exc:                           # the template may add it itself
        print(f"Note: set_lib({LWIP}): {exc}")
    settings = [(LWIP, p, v) for p, v in lwip_config.items()] + TEMPLATE_CONFIG
    # (the parameter is named "temac" but sets the PHY speed for the Zynq PS Ethernet too)
    settings.append((LWIP, "lwip220_temac_phy_link_speed", LINK_SPEED_VALUES[LINK_SPEED]))
    for lib, param, value in settings:
        try:
            domain.set_config(option="lib", param=param, value=value, lib_name=lib)
        except Exception as exc:
            print(f"Warning: {lib} setting {param}={value} not applied ({exc})")
    try:
        platform.build()
    except Exception as exc:
        print(f"Platform build error: {exc}")
    return platform


# lwIP's Marvell PHY routine (Zedboard: 88E1518) resets the PHY and waits only 5 x sleep(1)
# for auto-negotiation; with a direct cable to a PC that is too short ("Auto negotiation
# error", then the interface stays down). Patch the platform's copy of the source (the
# Xilinx install is not touched) and rebuild the platform.
PHY_WAIT_S = 20


def patch_phy_timeout(platform):
    vec_files = patch_irq_fpu_save()
    alloc_files = patch_rx_alloc_print()
    files = glob.glob(os.path.join(WS, PLATFORM, "**", "libsrc", "lwip220", "**", "xemacpsif_physpeed.c"),
                      recursive=True)
    if not files:
        print("Warning: xemacpsif_physpeed.c not found in the platform - PHY timeout not patched")
        platform.build()
        return
    patched = 0
    for path in files:
        with open(path) as f:
            text = f.read()
        start = text.find("static u32_t get_Marvell_phy_speed")
        end = text.find("static u32_t", start + 1)
        if start < 0 or end < 0:
            continue
        body = text[start:end]
        new_body = re.sub(r"timeout_counter == \d+\)", f"timeout_counter == {PHY_WAIT_S})", body)
        if new_body != body:
            with open(path, "w") as f:
                f.write(text[:start] + new_body + text[end:])
            patched += 1
    if patched == 0:
        print("Warning: Marvell timeout not found in xemacpsif_physpeed.c - not patched")
        platform.build()
        return
    print(f"Patched the Marvell PHY auto-negotiation timeout to {PHY_WAIT_S} s in {patched} file(s); rebuilding platform")
    try:
        platform.build()
    except Exception as exc:
        print(f"Platform rebuild error: {exc}")
    for path in vec_files:
        with open(path) as f:
            if FPU_FIXED not in f.read():
                print(f"Warning: the platform build restored the original {path} - VFP save NOT patched")
    for path in files:                                   # make sure the build did not regenerate it
        with open(path) as f:
            if f"timeout_counter == {PHY_WAIT_S})" not in f.read():
                print(f"Warning: the platform build restored the original {path} - timeout NOT patched")
    for path in alloc_files:
        with open(path) as f:
            if ALLOC_COUNTER not in f.read():
                print(f"Warning: the platform build restored the original {path} - pbuf-alloc print NOT removed")


# The standalone BSP's IRQ / FIQ entry (asm_vectors.S) saves the VFP/NEON registers only when
# FPU_HARD_FLOAT_ABI_ENABLED is set, and the 2025.2 SDT flow never sets it. The Ethernet
# interrupt handler uses them (XEmacPs_GetQxOffset copies its table with memcpy, which uses
# d0-d7), so an interrupt in the middle of a memcpy in lwIP or pd_server.c left 8 or 16 bytes of
# that table (0x404 0x408 / 0x624 0x628) in the copied data: about 0.13 % of the samples were
# wrong on LOAD and READ (firmware builds 4-8; build 3 probably escaped by its memory layout). Patch the
# platform's copy so the handlers always save d0-d7, d16-d31, FPSCR and FPEXC.
# The lwIP Ethernet driver (xemacpsif_dma.c) prints "unable to alloc pbuf in recv_handler" from
# the receive INTERRUPT each time the pbuf pool is empty. With the CoreBoard stream (60,000
# frames/s) one overload then never ends: each line blocks ~4 ms on the 115200-baud UART while
# frames keep arriving, and the board hangs (firmware build 20, mode 1). Replace the print with a
# counter; pd_server.c (build 22) reports it as rx_alloc_fail in UDP_STATS.
ALLOC_PRINT = re.compile(r'(?:xil_)?printf\s*\(\s*"unable to alloc pbuf in recv_handler\\r\\n"\s*\)\s*;')
ALLOC_COUNTER = "pd_rx_alloc_fail++; /* Flexlink: was a printf in the interrupt */"
ALLOC_DECL = "volatile unsigned int pd_rx_alloc_fail; /* Flexlink: pbuf pool empty in the rx interrupt */\n"


def patch_rx_alloc_print():
    files = glob.glob(os.path.join(WS, PLATFORM, "**", "libsrc", "lwip220", "**", "xemacpsif_dma.c"),
                      recursive=True)
    patched = []
    for path in files:
        with open(path) as f:
            text = f.read()
        new, n = ALLOC_PRINT.subn(ALLOC_COUNTER, text)
        if n == 0:
            continue
        if ALLOC_DECL not in new:                      # define the counter after the last #include
            last = [m.end() for m in re.finditer(r"^#include[^\n]*\n", new, re.M)]
            at = last[-1] if last else 0
            new = new[:at] + ALLOC_DECL + new[at:]
        with open(path, "w") as f:
            f.write(new)
        patched.append(path)
    if patched:
        print(f"Removed the rx-interrupt pbuf-alloc print in {len(patched)} file(s) (now a counter)")
    else:
        print(f"Warning: the pbuf-alloc print was not found in {len(files)} xemacpsif_dma.c file(s) - not patched")
    return patched


FPU_GUARD = "#if FPU_HARD_FLOAT_ABI_ENABLED"
FPU_FIXED = "#if 1 /* Flexlink: always save VFP state (was FPU_HARD_FLOAT_ABI_ENABLED) */"


def patch_irq_fpu_save():
    files = [f for f in glob.glob(os.path.join(WS, PLATFORM, "**", "asm_vectors.S"), recursive=True)
             if "cortexa9" in f]
    patched = 0
    for path in files:
        with open(path) as f:
            text = f.read()
        if FPU_GUARD in text:
            with open(path, "w") as f:
                f.write(text.replace(FPU_GUARD, FPU_FIXED))
            patched += 1
    print(f"Patched the IRQ/FIQ VFP save in {patched} of {len(files)} asm_vectors.S file(s)")
    return files


def check_irq_fpu_save(elf):
    """The vector code (start of .text) must contain vpush {d0-d7} (0xED2D0B10)."""
    with open(elf, "rb") as f:
        data = f.read()
    vpush = bytes.fromhex("100b2ded")
    # .text starts at 0x00100000 = file offset 0x1000 in this ELF; the handlers are in the first 1 KB
    ok = vpush in data[0x1000:0x1400]
    print("IRQ handler saves the VFP registers: OK" if ok else
          "*** WARNING: the IRQ handler does NOT save the VFP registers - LOAD/READ data will be "
          "corrupted (see patch_irq_fpu_save in build_firmware.py) ***")
    return ok


def patch_irq_stack(lscript):
    """The VFP save takes ~200 bytes of the 1 KB IRQ stack; give it 4 KB."""
    with open(lscript) as f:
        text = f.read()
    new, n = re.subn(r"(_IRQ_STACK_SIZE\s*:\s*)1024;", r"\g<1>4096;", text)
    if n:
        with open(lscript, "w") as f:
            f.write(new)
        print("IRQ stack: 4 KB")
    else:
        print(f"Warning: _IRQ_STACK_SIZE not found in {lscript} - IRQ stack left as it is")


def patch_board_ip(main_c):
    """Replace the template's fallback address 192.168.1.10 / gateway 192.168.1.1."""
    with open(main_c) as f:
        text = f.read()
    ip, gw = ", ".join(map(str, BOARD_IP)), ", ".join(map(str, BOARD_GW))
    new, n_ip = re.subn(r"192,\s*168,\s*1,\s*10\)", ip + ")", text)
    new, n_gw = re.subn(r"192,\s*168,\s*1,\s*1\)", gw + ")", new)
    new = new.replace("192.168.1.10", ".".join(map(str, BOARD_IP)))
    if n_ip == 0:
        print(f"Warning: fallback address not found in {main_c} - board keeps the template's address")
    with open(main_c, "w") as f:
        f.write(new)
    print(f"Board fallback address: {'.'.join(map(str, BOARD_IP))} ({n_ip} address, {n_gw} gateway lines patched)")


# ------------------------------------------------------------------------------------------
# no-OS sources: follow the #include "..." lines from our sources
# ------------------------------------------------------------------------------------------
INCLUDE_RE = re.compile(r'^\s*#\s*include\s*"([^"]+)"', re.M)


def noos_sources():
    headers = {}                                       # name -> path, first directory wins
    for d in NOOS_INC_DIRS + NOOS_SRC_DIRS:
        for p in sorted(glob.glob(os.path.join(NOOS, d, "*.h"))):
            headers.setdefault(os.path.basename(p), p)
    sources = {}
    for d in NOOS_SRC_DIRS:
        for p in sorted(glob.glob(os.path.join(NOOS, d, "*.c"))):
            sources.setdefault(os.path.basename(p)[:-2], p)

    todo = [os.path.join(HERE, f) for f in APP_SOURCES + APP_HEADERS]
    todo += [os.path.join(NOOS, f) for f in NOOS_EXTRA_SOURCES]
    seen, used = set(), []
    while todo:
        path = todo.pop()
        if path in seen:
            continue
        seen.add(path)
        if path.endswith(".c") and path.startswith(NOOS):
            used.append(path)
        with open(path, errors="replace") as f:
            text = f.read()
        for name in INCLUDE_RE.findall(text):
            base = os.path.basename(name)
            if base in headers and not os.path.isfile(os.path.join(HERE, base)):
                todo.append(headers[base])
                stem = base[:-2]
                if stem in sources:
                    todo.append(sources[stem])
    return sorted(used)


def noos_commit():
    try:
        return subprocess.run(["git", "-C", NOOS, "log", "-1", "--format=%h %cd", "--date=short"],
                              capture_output=True, text=True, timeout=10).stdout.strip()
    except Exception:
        return "?"


def cmake_list(items):
    return "\n".join(f'    "{i}"' for i in items)


def patch_user_config(path, sources, inc_dirs):
    with open(path) as f:
        text = f.read()
    repl = [
        (r"set\(USER_COMPILE_SOURCES\s*\n\)", f"set(USER_COMPILE_SOURCES\n{cmake_list(sources)}\n)"),
        (r"set\(USER_INCLUDE_DIRECTORIES\s*\n\)", f"set(USER_INCLUDE_DIRECTORIES\n{cmake_list(inc_dirs)}\n)"),
        (r'set\(USER_COMPILE_DEFINITIONS\s*\n""\s*\n\)', f"set(USER_COMPILE_DEFINITIONS\n{cmake_list(NOOS_DEFINES)}\n)"),
    ]
    for pattern, new in repl:
        text, n = re.subn(pattern, lambda m: new, text)
        if n != 1:
            sys.exit(f"UserConfig.cmake: pattern {pattern!r} not found - template changed? ({path})")
    with open(path, "w") as f:
        f.write(text)


def patch_heap(lscript):
    with open(lscript) as f:
        text = f.read()
    new, n = re.subn(r"(_HEAP_SIZE\s*=\s*DEFINED\(_HEAP_SIZE\)\s*\?\s*_HEAP_SIZE\s*:\s*)0x[0-9a-fA-F]+;",
                     r"\g<1>" + HEAP_SIZE + ";", text)
    if n:
        with open(lscript, "w") as f:
            f.write(new)
        print(f"Heap: {HEAP_SIZE}")
    else:
        print(f"Warning: _HEAP_SIZE not found in {lscript} - heap left as it is (the AD9361 driver needs ~100 KB)")


# no-OS's xilinx_delay.c includes xtime_l.h, which the SDT standalone BSP (2023.2+) may not have.
XTIME_SHIM = """/* xtime_l.h - written by build_firmware_fmc.py because this BSP has none (SDT flow).
 * no-OS xilinx_delay.c needs XTime, XTime_GetTime and COUNTS_PER_SECOND: the Cortex-A9 global
 * timer (CPU clock / 2), as the BSP's own xtime_l.h used to provide. */
#ifndef XTIME_H
#define XTIME_H
#include "xil_types.h"
#include "xil_io.h"
#include "xparameters.h"
typedef u64 XTime;
#if defined(XPAR_CPU_CORE_CLOCK_FREQ_HZ)
#define COUNTS_PER_SECOND (XPAR_CPU_CORE_CLOCK_FREQ_HZ / 2u)
#else
#define COUNTS_PER_SECOND 333333333u
#endif
static inline void XTime_GetTime(XTime *t)
{
    u32 hi, lo;
    do {
        hi = Xil_In32(0xF8F00204u);
        lo = Xil_In32(0xF8F00200u);
    } while (Xil_In32(0xF8F00204u) != hi);
    *t = ((u64)hi << 32) | lo;
}
#endif
"""


def bsp_has(header):
    return bool(glob.glob(os.path.join(WS, PLATFORM, "export", PLATFORM, "sw", DOMAIN, "**", header),
                          recursive=True))


def main():
    if not os.path.isfile(XSA):
        sys.exit(f"{XSA} not found - run the hardware build first (HW/build_hw_fmcomms3.tcl)")
    if not os.path.isdir(os.path.join(NOOS, "drivers", "rf-transceiver", "ad9361")):
        sys.exit(f"no-OS not found at {NOOS} - clone it there or set NOOS_DIR")
    for f in APP_SOURCES + APP_HEADERS:
        if not os.path.isfile(os.path.join(HERE, f)):
            sys.exit(f"{f} missing in {HERE}")
    print(f"Application: {APP} ({FMC_APP})")
    sources = noos_sources()
    inc_dirs = [os.path.join(NOOS, d) for d in NOOS_INC_DIRS]
    print(f"no-OS {noos_commit()} at {NOOS}: {len(sources)} source files")
    for s in sources:
        print("   ", os.path.relpath(s, NOOS))

    if os.path.isdir(WS):
        shutil.rmtree(WS)
    os.makedirs(WS)

    client = vitis.create_client()
    client.set_workspace(path=WS)
    try:
        xpfm = os.path.join(WS, PLATFORM, "export", PLATFORM, f"{PLATFORM}.xpfm")
        platform = build_platform(client, LWIP_CONFIG)
        if not os.path.isfile(xpfm):
            print("\n*** Platform build failed with the tuned lwIP settings - retrying with lwIP defaults ***\n")
            client.delete_component(name=PLATFORM)
            platform = build_platform(client, {})
        if not os.path.isfile(xpfm):
            sys.exit("Platform build failed - see workspace/_ide/logs/vitis.log (search for ERROR)")
        patch_phy_timeout(platform)
        for h in ("xspips.h", "xgpiops.h"):
            if not bsp_has(h):
                print(f"*** Warning: {h} not in the BSP - is SPI0 / GPIO enabled in the bitstream's PS? ***")

        app = client.create_app_component(name=APP, platform=xpfm, domain=DOMAIN, template=TEMPLATE)

        src = os.path.join(WS, APP, "src")
        for f in APP_SOURCES + APP_HEADERS:
            shutil.copy(os.path.join(HERE, f), src)
        if not bsp_has("xtime_l.h"):
            with open(os.path.join(src, "xtime_l.h"), "w") as f:
                f.write(XTIME_SHIM)
            print("BSP has no xtime_l.h: wrote a global-timer version for no-OS xilinx_delay.c")

        # the template's CMakeLists.txt lists its sources by name: echo.c -> ours
        cmake = os.path.join(src, "CMakeLists.txt")
        with open(cmake) as f:
            text = f.read()
        if "collect (PROJECT_LIB_SOURCES echo.c)" not in text:
            sys.exit(f"echo.c not listed in {cmake} as expected - template changed?")
        ours = "\n".join(f"collect (PROJECT_LIB_SOURCES {s})" for s in APP_SOURCES)
        with open(cmake, "w") as f:
            f.write(text.replace("collect (PROJECT_LIB_SOURCES echo.c)", ours))
        if os.path.isfile(os.path.join(src, "echo.c")):
            os.remove(os.path.join(src, "echo.c"))

        patch_user_config(os.path.join(src, "UserConfig.cmake"), sources, inc_dirs + [src])
        patch_board_ip(os.path.join(src, "main.c"))
        lscript = os.path.join(src, "lscript.ld")
        patch_irq_stack(lscript)
        patch_heap(lscript)

        app.build()
        elf = os.path.join(WS, APP, "build", f"{APP}.elf")
        if os.path.isfile(elf):
            check_irq_fpu_save(elf)
            print(f"Done: {elf}")
        else:
            sys.exit(f"Build failed - no {APP}.elf; see the compiler messages above")
    finally:
        vitis.dispose()


main()
