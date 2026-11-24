@echo off
setlocal
cd /d "%~dp0"

if exist "C:\msys64\ucrt64\bin\vvp.exe" set "PATH=C:\msys64\ucrt64\bin;%PATH%"
if exist "C:\iverilog\bin\vvp.exe" set "PATH=C:\iverilog\bin;%PATH%"

where vvp >nul 2>nul
if errorlevel 1 (
    echo [ERROR] vvp not found. Install Icarus Verilog and add it to PATH.
    exit /b 1
)

if not exist "simulation\xeroton_sim.vvp" (
    echo [INFO] Compiled binary not found; running build_verilog.bat first...
    call build_verilog.bat
    if errorlevel 1 exit /b 1
)

echo [RUN] Executing XEROTON Verilog testbench...
vvp simulation\xeroton_sim.vvp > simulation\verilog_sim.log 2>&1
set "VVP_EXIT=%errorlevel%"
type simulation\verilog_sim.log

if not "%VVP_EXIT%"=="0" (
    echo [ERROR] vvp execution failed with exit code %VVP_EXIT%.
    exit /b 1
)

findstr /C:"ALL VERILOG TESTS PASSED!" simulation\verilog_sim.log >nul
if errorlevel 1 (
    echo [ERROR] Verilog testbench reported assertion failures. Check simulation\verilog_sim.log.
    exit /b 1
)

if not exist "simulation\joint_targets.csv" (
    echo [ERROR] Expected output simulation\joint_targets.csv was not generated.
    exit /b 1
)
if not exist "simulation\xeroton.vcd" (
    echo [ERROR] Expected waveform simulation\xeroton.vcd was not generated.
    exit /b 1
)

echo [OK] Verilog simulation complete. Generated simulation\joint_targets.csv and simulation\xeroton.vcd.
exit /b 0
