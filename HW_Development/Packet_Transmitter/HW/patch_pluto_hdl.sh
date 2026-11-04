#!/usr/bin/env bash
# patch_pluto_hdl.sh - add the Flexlink transmitter to a plutosdr-fw checkout (its hdl submodule).
#
#   ./patch_pluto_hdl.sh ~/plutosdr-fw        # then: cd ~/plutosdr-fw && make
#
# Copies RTL/*.v, ptx_pluto_bd.tcl and ptx_pluto.xdc to hdl/projects/pluto/ptx/ (rtl/ subfolder for the RTL) and
# appends "source ptx/ptx_pluto_bd.tcl" to hdl/projects/pluto/system_bd.tcl (once). Run it again
# after changing the RTL. plutosdr-fw master expects Vivado 2025.1; with 2025.2 set
# ADI_IGNORE_VERSION_CHECK=1 (adi_project_xilinx.tcl stops on a version mismatch otherwise).
set -euo pipefail
FW=${1:?usage: $0 <plutosdr-fw folder>}
HERE=$(cd "$(dirname "$0")" && pwd)
PRJ=$FW/hdl/projects/pluto
[ -f "$PRJ/system_bd.tcl" ] || { echo "no $PRJ/system_bd.tcl - is $FW a plutosdr-fw checkout (git clone --recursive)?"; exit 1; }
mkdir -p "$PRJ/ptx/rtl"
cp "$HERE"/../RTL/ptx_pluto.v "$HERE"/../RTL/ptx_core.v "$HERE"/../RTL/ptx_enc.v "$HERE"/../RTL/ptx_ifft.v "$HERE"/../RTL/ptx_roms.v "$PRJ/ptx/rtl/"
cp "$HERE/ptx_pluto_bd.tcl" "$HERE/ptx_pluto.xdc" "$PRJ/ptx/"
grep -q "ptx/ptx_pluto_bd.tcl" "$PRJ/system_bd.tcl" || printf '\n# Flexlink transmitter\nsource ptx/ptx_pluto_bd.tcl\n' >> "$PRJ/system_bd.tcl"
echo "Patched $PRJ (ptx/ and system_bd.tcl). Build: cd $FW && ADI_IGNORE_VERSION_CHECK=1 make"
