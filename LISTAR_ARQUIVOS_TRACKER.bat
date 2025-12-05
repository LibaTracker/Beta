@echo off
chcp 65001 >nul
color 0A
title VR Tracker - Listando Arquivos

cls
echo.
echo ╔════════════════════════════════════════════════════════════════════════╗
echo ║           📋 LISTANDO ARQUIVOS DA PASTA C:\tracker                   ║
echo ╚════════════════════════════════════════════════════════════════════════╝
echo.
echo Este script vai gerar uma lista completa de todos os arquivos e pastas
echo dentro de "C:\tracker" e suas subpastas.
echo.
echo Pressione qualquer tecla para começar...
pause >nul
echo.

set "OUTPUT_FILE=%CD%\lista_arquivos_tracker.txt"

echo Gerando lista de arquivos...
echo.

REM Listar arquivos e pastas recursivamente
dir /s /b /a "%CD%" > "%OUTPUT_FILE%"

echo    ✅ Lista gerada em: %OUTPUT_FILE%
echo.
echo ════════════════════════════════════════════════════════════════════════
echo.
echo Pressione qualquer tecla para fechar...
pause >nul
exit
