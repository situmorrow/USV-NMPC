@echo off
REM =======================================================================
REM  Auto-start Helper untuk Mini PC di Kapal Autonomous (USV)
REM  Menjalankan Kendali Pelacakan Waypoint NMPC & Jembatan Serial ESP32-S3
REM =======================================================================

set COM_PORT=COM16
set BAUD=115200
set RUDDER_MODE=nmpc
set PRINT_MODE=nmpc
set YAW_OFFSET=180.0
set R_TRAN=3.0
set SURGE_SPEED=0.6114
set SHIP_LENGTH=1.0107
set USB_WAIT_SEC=10

cd /d "%~dp0"

if not exist "build\read_write_serial.exe" (
  if not exist "read_write_serial.exe" (
    echo [ERROR] read_write_serial.exe tidak ditemukan di %~dp0 atau build\
    pause
    exit /b 1
  )
)

set EXE_PATH=read_write_serial.exe
if exist "build\read_write_serial.exe" set EXE_PATH=build\read_write_serial.exe

echo [INFO] Menunggu USB serial siap selama %USB_WAIT_SEC% detik...
timeout /t %USB_WAIT_SEC% /nobreak >nul

:retry
echo [INFO] Menjalankan %EXE_PATH% pada %COM_PORT% mode %RUDDER_MODE%...
%EXE_PATH% --port %COM_PORT% --baud %BAUD% --rudder-mode %RUDDER_MODE% --print %PRINT_MODE% --yaw-offset %YAW_OFFSET% --r-tran %R_TRAN% --surge-speed %SURGE_SPEED% --ship-length %SHIP_LENGTH%
echo.
echo [WARN] Program berhenti. Restart otomatis dalam 5 detik...
timeout /t 5 /nobreak >nul
goto retry
