"""
build_firmware.py  (Flexlink_py/HW_Development/Packet_Detector/FW)

Builds the Zedboard firmware with the unified Vitis (2025.2) Python API:
  1. platform from ../HW/build/pd_zed.xsa (standalone, Cortex-A9 #0) with the lwIP library
  2. application from the "lwIP Echo Server" template (its main.c sets up Ethernet,
     DHCP with fallback to BOARD_IP = 192.168.50.10, and the main loop)
  3. the template's echo.c is replaced by pd_server.c
  4. build -> workspace/pd_server/build/pd_server.elf

    cd Flexlink_py/HW_Development/Packet_Detector/FW
    vitis -s build_firmware.py

The workspace folder is deleted and recreated on every run; edit pd_server.c here, not in
the workspace. To work in the IDE afterwards: vitis -w workspace
"""
import glob
import os
import re
import shutil
import sys

import vitis

HERE = os.path.dirname(os.path.abspath(__file__))
WS = os.path.join(HERE, "workspace")
XSA = os.path.normpath(os.path.join(HERE, "..", "HW", "build", "pd_zed.xsa"))
PLATFORM = "pd_zed_platform"
DOMAIN = "standalone_ps7_cortexa9_0"
APP = "pd_server"
TEMPLATE = "lwip_echo_server"
LWIP = "lwip220"

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
    files = glob.glob(os.path.join(WS, PLATFORM, "**", "libsrc", "lwip220", "**", "xemacpsif_physpeed.c"),
                      recursive=True)
    if not files:
        print("Warning: xemacpsif_physpeed.c not found in the platform - PHY timeout not patched")
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
        return
    print(f"Patched the Marvell PHY auto-negotiation timeout to {PHY_WAIT_S} s in {patched} file(s); rebuilding platform")
    try:
        platform.build()
    except Exception as exc:
        print(f"Platform rebuild error: {exc}")
    for path in files:                                   # make sure the build did not regenerate it
        with open(path) as f:
            if f"timeout_counter == {PHY_WAIT_S})" not in f.read():
                print(f"Warning: the platform build restored the original {path} - timeout NOT patched")


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


def main():
    if not os.path.isfile(XSA):
        sys.exit(f"{XSA} not found - run the hardware build first (HW/build_hw.tcl)")
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

        app = client.create_app_component(name=APP, platform=xpfm, domain=DOMAIN, template=TEMPLATE)

        # Swap the template's echo.c for pd_server.c: the template's CMakeLists.txt lists its
        # sources by name, so rename the entry there and remove echo.c.
        src = os.path.join(WS, APP, "src")
        shutil.copy(os.path.join(HERE, "pd_server.c"), src)
        cmake = os.path.join(src, "CMakeLists.txt")
        with open(cmake) as f:
            text = f.read()
        if "echo.c" in text:
            with open(cmake, "w") as f:
                f.write(text.replace("echo.c", "pd_server.c"))
            if os.path.isfile(os.path.join(src, "echo.c")):
                os.remove(os.path.join(src, "echo.c"))
        else:
            print(f"Warning: echo.c not listed in {cmake} - add pd_server.c to the sources by hand")

        patch_board_ip(os.path.join(src, "main.c"))

        app.build()
        elf = os.path.join(WS, APP, "build", f"{APP}.elf")
        print(f"Done: {elf}" if os.path.isfile(elf) else "Build finished - look for pd_server.elf under workspace/")
    finally:
        vitis.dispose()


main()
