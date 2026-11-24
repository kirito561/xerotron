@echo off
setlocal
cd /d "%~dp0"

:: Select Python interpreter (prefer local conda-forge .env with prebuilt PyBullet)
set "PY_EXE="
if exist "%~dp0.env\python.exe" set "PY_EXE=%~dp0.env\python.exe"
if not defined PY_EXE (
    where python >nul 2>nul && set "PY_EXE=python"
)
if not defined PY_EXE (
    where py >nul 2>nul && set "PY_EXE=py"
)
if not defined PY_EXE (
    echo [ERROR] Python not found. Run setup.bat first.
    exit /b 1
)

if not exist "simulation\joint_targets.csv" (
    echo [ERROR] simulation\joint_targets.csv is missing. Run run_verilog.bat first.
    exit /b 1
)

echo [SIM] Launching PyBullet 3D simulation driven by Verilog CSV...
"%PY_EXE%" python\main.py %*
if errorlevel 1 (
    echo [ERROR] Python/PyBullet simulation exited with an error.
    exit /b 1
)
exit /b 0
