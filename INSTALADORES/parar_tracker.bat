@echo off
chcp 65001 >nul
color 0C
title VR Tracker - Encerrando

echo.
echo ╔════════════════════════════════════════════════════════════════════╗
echo ║           VR TRACKER - ENCERRANDO PROCESSOS                       ║
echo ╚════════════════════════════════════════════════════════════════════╝
echo.

echo 🔍 Procurando processos Python do tracker...
echo.

REM Matar todos os processos Python rodando tracker_control_center.py
for /f "tokens=2" %%a in ('tasklist /FI "IMAGENAME eq python.exe" /FO LIST ^| find "PID:"') do (
    wmic process where "ProcessId=%%a AND CommandLine LIKE '%%tracker_control_center%%'" delete >nul 2>&1
)

REM Método alternativo caso o WMIC falhe
taskkill /F /FI "WINDOWTITLE eq *tracker_control_center*" >nul 2>&1
taskkill /F /FI "IMAGENAME eq python.exe" /FI "MEMUSAGE gt 50000" >nul 2>&1

echo ✅ Processos encerrados!
echo.
echo ════════════════════════════════════════════════════════════════════
echo.
echo Esta janela será fechada em 3 segundos...
timeout /t 3 >nul

exit
