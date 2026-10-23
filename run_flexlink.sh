#!/usr/bin/env bash
# Starts the Flexlink app on Linux and macOS.
# The first run makes a virtual environment in .venv and installs what the app needs;
# later runs just start the app.
set -e
cd "$(dirname "$0")"

PY="${PYTHON:-python3}"
if ! command -v "$PY" >/dev/null 2>&1; then
    echo "Python 3 not found. Install Python 3.10 or newer (https://www.python.org/downloads/)."
    exit 1
fi

if [ ! -x .venv/bin/python ]; then
    echo "First start: creating .venv and installing numpy, scipy, h5py, matplotlib, FreeSimpleGUI ..."
    "$PY" -m venv .venv
    .venv/bin/python -m pip install --upgrade pip
    .venv/bin/python -m pip install -e .
fi

if ! .venv/bin/python -c "import tkinter" >/dev/null 2>&1; then
    echo "Tkinter is missing (the app's window toolkit)."
    echo "  Debian/Ubuntu: sudo apt install python3-tk"
    echo "  Fedora:        sudo dnf install python3-tkinter"
    echo "  macOS (Homebrew Python): brew install python-tk"
    exit 1
fi

exec .venv/bin/python App/Flexlinksimulation.py "$@"
