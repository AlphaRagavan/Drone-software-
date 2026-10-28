@echo off
title NIDAR AirMouse Ground Control Station (GCS)
color 0A
echo ===================================================================
echo  NIDAR AirMouse Ground Control Station (GCS) - Windows 10/11
echo  Platform: Acer Nitro V15 / Any Windows 10 or 11 Desktop
echo  Mode: 100% Offline, Zero Connectivity, Isolated 5GHz Wi-Fi
echo ===================================================================
echo.
echo Launching Standalone GCS Desktop Application...
python "%~dp0gcs_app.py" %*
if %ERRORLEVEL% NEQ 0 (
    echo.
    echo [!] An error occurred while launching GCS.
    pause
)
