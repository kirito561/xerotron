@echo off
setlocal
cd /d "%~dp0"

echo ======================================================================
echo  XEROTON - Full Automated Verilog + PyBullet Pipeline
echo ======================================================================

:: 1. Check Python availability
set "PY_EXE="
if exist "%~dp0.env\python.exe" set "PY_EXE=%~dp0.env\python.exe"
if not defined PY_EXE (
    where python >nul 2>nul && set "PY_EXE=python"
)
if not defined PY_EXE (
    where py >nul 2>nul && set "PY_EXE=py"
)
if not defined PY_EXE (
    echo [ERROR] Step 1 failed: Python is not available. Run setup.bat first.
    exit /b 1
)
echo [STEP 1/8] Python detected: "%PY_EXE%"
"%PY_EXE%" --version

:: 2. Check Icarus Verilog availability
if exist "C:\msys64\ucrt64\bin\iverilog.exe" set "PATH=C:\msys64\ucrt64\bin;%PATH%"
if exist "C:\iverilog\bin\iverilog.exe" set "PATH=C:\iverilog\bin;%PATH%"
where iverilog >nul 2>nul
if errorlevel 1 (
    echo [ERROR] Step 2 failed: iverilog not found in PATH.
    exit /b 1
)
where vvp >nul 2>nul
if errorlevel 1 (
    echo [ERROR] Step 2 failed: vvp not found in PATH.
    exit /b 1
)
echo [STEP 2/8] Icarus Verilog detected.

:: 3. Check required Python packages
echo [STEP 3/8] Checking Python packages ^(pybullet, pytest^)...
"%PY_EXE%" -c "import pybullet, pytest; print('[OK] pybullet + pytest imported successfully')"
if errorlevel 1 (
    echo [ERROR] Step 3 failed: Missing required Python packages. Run setup.bat first.
    exit /b 1
)

:: 4. Compile all Verilog modules
echo [STEP 4/8] Compiling Verilog modules...
call build_verilog.bat
if errorlevel 1 (
    echo [ERROR] Step 4 failed: Verilog compilation error.
    exit /b 1
)

:: 5 & 6. Run testbench and generate CSV + VCD
echo [STEP 5-6/8] Running Verilog testbench to generate CSV and VCD...
call run_verilog.bat
if errorlevel 1 (
    echo [ERROR] Step 5-6 failed: Verilog simulation failed.
    exit /b 1
)

:: 7. Validate CSV and VCD data integrity
echo [STEP 7/8] Validating generated CSV joint data and VCD waveform...
"%PY_EXE%" -m pytest tests\test_data_validation.py -v
if errorlevel 1 (
    echo [ERROR] Step 7 failed: Generated CSV/VCD failed validation checks.
    exit /b 1
)

:: 8. Launch PyBullet simulation
echo [STEP 8/8] Launching PyBullet 3D simulation...
call run_python_simulation.bat %*
if errorlevel 1 (
    echo [ERROR] Step 8 failed: PyBullet simulation encountered an error.
    exit /b 1
)

echo ======================================================================
echo  XEROTON pipeline finished successfully!
echo ======================================================================
exit /b 0
