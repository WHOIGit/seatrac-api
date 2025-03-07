:: This script was originally written in PowerShell, but Windows is annoying
:: about running PowerShell scripts by default.

@echo off
setlocal

:: Check if Python is installed
where python >nul 2>nul
if %ERRORLEVEL% neq 0 (
    echo Python not found.
    exit /b 1
)

:: Get path to frida.exe
for /f "delims=" %%I in ('python -c "import sysconfig; print(sysconfig.get_path('scripts', sysconfig.get_preferred_scheme('user')))" 2^>nul') do set "scriptsDir=%%I"
set "frida=%scriptsDir%\frida.exe"
if not exist "%frida%" (
    echo Frida not found.
    exit /b 1
)

:: Get path to frida-sslkeylog.js
set "script=%~dp0%frida-sslkeylog.js"

:: Start Frida
"%frida%" -l "%script%" "C:\Program Files\SeaTrac\dashboard\Dashboard.exe"
