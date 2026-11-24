@echo off
setlocal
cd /d "%~dp0"

if exist "C:\msys64\ucrt64\bin\iverilog.exe" set "PATH=C:\msys64\ucrt64\bin;%PATH%"
if exist "C:\iverilog\bin\iverilog.exe" set "PATH=C:\iverilog\bin;%PATH%"

where iverilog >nul 2>nul
if errorlevel 1 (
    echo [ERROR] iverilog not found. Install Icarus Verilog and add it to PATH.
    exit /b 1
)

if not exist "simulation" mkdir "simulation"

echo [BUILD] Compiling XEROTON Verilog RTL and verification testbench...
iverilog -g2012 -Wall -s tb_xeroton -o simulation\xeroton_sim.vvp ^
    rtl\servo_pwm.v ^
    rtl\safety_controller.v ^
    rtl\gait_controller.v ^
    rtl\xeroton_top.v ^
    tb\tb_xeroton.v

if errorlevel 1 (
    echo [ERROR] Verilog verification testbench compilation failed.
    exit /b 1
)

echo [BUILD] Compiling live interactive Verilog command runner...
iverilog -g2012 -Wall -s tb_live_runner -o simulation\xeroton_live.vvp ^
    rtl\servo_pwm.v ^
    rtl\safety_controller.v ^
    rtl\gait_controller.v ^
    rtl\xeroton_top.v ^
    tb\tb_live_runner.v

if errorlevel 1 (
    echo [ERROR] Live Verilog runner compilation failed.
    exit /b 1
)

echo [OK] Compiled successfully: simulation\xeroton_sim.vvp and simulation\xeroton_live.vvp
exit /b 0
