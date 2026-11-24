@echo off
setlocal enabledelayedexpansion
cd /d "%~dp0"

echo ======================================================================
echo  XEROTON - Environment Setup (Windows 11)
echo ======================================================================

:: Locate Icarus Verilog
if exist "C:\msys64\ucrt64\bin\iverilog.exe" set "PATH=C:\msys64\ucrt64\bin;%PATH%"
if exist "C:\iverilog\bin\iverilog.exe" set "PATH=C:\iverilog\bin;%PATH%"

where iverilog >nul 2>nul
if errorlevel 1 (
    echo [ERROR] Icarus Verilog ^(iverilog^) not found in PATH.
    echo         Install from https://bleyer.org/icarus/ or via MSYS2:
    echo           pacman -S mingw-w64-ucrt-x86_64-iverilog mingw-w64-ucrt-x86_64-gtkwave
    exit /b 1
) else (
    echo [OK] Found Icarus Verilog:
    iverilog -V | findstr /B "Icarus"
)

:: Check local conda-forge environment first (prebuilt Windows PyBullet)
if exist "%~dp0.env\python.exe" (
    echo [OK] Project Python+PyBullet environment already exists at "%~dp0.env"
    "%~dp0.env\python.exe" --version
    "%~dp0.env\python.exe" -c "import pybullet, pytest; print('[OK] pybullet and pytest ready')"
    if errorlevel 1 (
        echo [ERROR] .env exists but pybullet or pytest import failed.
        exit /b 1
    )
    exit /b 0
)

:: Try creating .env via micromamba (no Visual C++ Build Tools needed)
where micromamba >nul 2>nul
if not errorlevel 1 (
    echo [INFO] Creating local Python 3.11 + PyBullet environment via micromamba...
    set "MAMBA_ROOT_PREFIX=C:\m"
    micromamba create -y -p "%~dp0.env" -c conda-forge python=3.11 pybullet pytest
    if errorlevel 1 (
        echo [ERROR] micromamba environment creation failed.
        exit /b 1
    )
    echo [OK] Setup complete!
    exit /b 0
)

:: Fallback: system Python + pip
where python >nul 2>nul
if errorlevel 1 (
    echo [ERROR] Python not found. Install Python 3.10+ or micromamba.
    exit /b 1
)
echo [INFO] Installing Python dependencies with pip...
python -m pip install -r requirements.txt
if errorlevel 1 (
    echo [ERROR] pip install failed. Note: On Windows without Visual C++ Build Tools,
    echo         install micromamba ^(winget install Mamba.Micromamba^) and re-run setup.bat.
    exit /b 1
)
echo [OK] Setup complete!
exit /b 0
