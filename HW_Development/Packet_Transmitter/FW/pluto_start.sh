#!/usr/bin/env bash
# pluto_start.sh - start the Flexlink transmitter server on the Pluto. Run it after every power
# cycle of the Pluto (the server lives in the Pluto's /tmp, which is RAM).
#
#   bash pluto_start.sh                 Pluto at 192.168.2.1
#   bash pluto_start.sh 192.168.3.1     another address
#
# Needs FW/ptx_server next to this script: built with "make -f ptx.mk pluto" on the PC that has
# plutosdr-fw, and copied to the others (it is not in git).
# Asks for the Pluto's root password (analog) once; with sshpass installed (sudo apt install
# sshpass) it does not ask at all (PLUTO_PASS=... for another password).
# The Pluto's SSH key changes at every boot: it is not checked or stored for this connection.
set -uo pipefail

PLUTO=${1:-${PLUTO:-192.168.2.1}}
HERE=$(cd "$(dirname "$0")" && pwd)
BIN=$HERE/ptx_server
EXPECT_FW=${EXPECT_FW:-v0.39-18-g3b68}          # the Flexlink transmitter firmware (pluto.frm)
CTL=/tmp/ptx-ssh-$(id -un)-$PLUTO

SSH_OPTS=(-o StrictHostKeyChecking=no -o UserKnownHostsFile=/dev/null -o LogLevel=ERROR
          -o ConnectTimeout=5 -o ControlMaster=auto -o ControlPath="$CTL" -o ControlPersist=60)
if command -v sshpass >/dev/null 2>&1; then
    SSH=(sshpass -p "${PLUTO_PASS:-analog}" ssh "${SSH_OPTS[@]}")
else
    SSH=(ssh "${SSH_OPTS[@]}")
    echo "(the Pluto's password is analog; install sshpass to skip this)"
fi
pluto() { "${SSH[@]}" "root@$PLUTO" "$@"; }
finish() { ssh -o ControlPath="$CTL" -O exit "root@$PLUTO" >/dev/null 2>&1; }
trap finish EXIT
fail() { echo "ERROR: $*" >&2; exit 1; }

# 1. the server program
[ -s "$BIN" ] || fail "$BIN not found. Copy FW/ptx_server from the PC that built it (it is not in git),
       or build it: make -f ptx.mk pluto FW=~/plutosdr-fw"

# 2. the Pluto on the network
ping -c 1 -W 2 "$PLUTO" >/dev/null 2>&1 || fail "no answer from $PLUTO. Is the Pluto plugged in and booted
       (about 30 s, LED1 blinking slowly)? Is its USB network up (nmcli device; ip -br addr)?"

# 3. its firmware
FW=$(pluto 'iio_attr -C fw_version 2>/dev/null') || fail "ssh to root@$PLUTO failed"
FW=${FW##*: }
echo "Pluto $PLUTO, firmware $FW"
case "$FW" in
    "$EXPECT_FW"*) ;;
    *) echo "WARNING: not the Flexlink transmitter firmware ($EXPECT_FW...). If this Pluto was never
         flashed with pluto.frm: copy it to the PlutoSDR drive, sync, then eject /dev/sdX
         (not udisksctl power-off), and wait for it to reboot. Starting the server anyway." ;;
esac

# 4. copy the server (under a new name, so a running one does not block it) and start it
pluto 'cat > /tmp/ptx_server.new && chmod +x /tmp/ptx_server.new && mv -f /tmp/ptx_server.new /tmp/ptx_server' \
    < "$BIN" || fail "copying ptx_server to the Pluto failed"
pluto 'killall ptx_server 2>/dev/null; sleep 0.3
       nohup /tmp/ptx_server > /tmp/ptx_server.log 2>&1 < /dev/null &
       sleep 1; cat /tmp/ptx_server.log' > /tmp/ptx_server_start.$$ 2>&1
LOG=$(cat /tmp/ptx_server_start.$$); rm -f /tmp/ptx_server_start.$$
echo "$LOG"

# 5. listening?
if echo "$LOG" | grep -q "listening on port"; then
    echo "OK: the transmitter server is running on $PLUTO."
else
    fail "the server did not start (log above; also: ssh root@$PLUTO cat /tmp/ptx_server.log)"
fi
