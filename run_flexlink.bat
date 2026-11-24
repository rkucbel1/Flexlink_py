@echo off
rem Starts the Flexlink app on Windows (double-click this file).
rem The first run makes a virtual environment in .venv and installs what the app needs;
rem later runs just start the app.
setlocal
cd /d "%~dp0"

if not exist ".venv\Scripts\python.exe" (
    where py >nul 2>nul
    if errorlevel 1 (
        echo Python not found. Install Python 3.10 or newer from https://www.python.org/downloads/
        echo and tick "Add python.exe to PATH" in the installer.
        pause
        exit /b 1
    )
    echo First start: creating .venv and installing numpy, scipy, h5py, matplotlib, FreeSimpleGUI ...
    py -3 -m venv .venv || goto :failed
    ".venv\Scripts\python.exe" -m pip install --upgrade pip || goto :failed
    ".venv\Scripts\python.exe" -m pip install -e . || goto :failed
)

".venv\Scripts\python.exe" App\Simulation\Flexlinksimulation.py %*
exit /b %errorlevel%

:failed
echo Installation failed - see the messages above.
pause
exit /b 1
