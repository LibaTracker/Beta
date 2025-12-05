@echo off
chcp 65001 >nul
color 0B
title VR Tracker - Instalação do Cliente

mode con: cols=80 lines=30

:INICIO
cls
echo.
echo ╔════════════════════════════════════════════════════════════════════════════╗
echo ║             🎮  VR TRACKER - INSTALAÇÃO DO CLIENTE  🎮                     ║
echo ╚════════════════════════════════════════════════════════════════════════════╝
echo.
echo   Bem-vindo ao assistente de instalação!
echo.
echo   Pressione qualquer tecla para começar...
pause >nul

REM ============================================================================
REM                           ETAPA 1: VERIFICAR PYTHON
REM ============================================================================
:ETAPA1
cls
call :DESENHAR_HEADER
echo.
echo   ┌─────────────────────────────────────────────────────────────────────┐
echo   │  ETAPA 1/4: Verificando Python                                      │
echo   └─────────────────────────────────────────────────────────────────────┘
echo.
call :DESENHAR_BARRA 0

echo   🔍 Procurando instalação do Python...
timeout /t 1 >nul

python --version >nul 2>&1
if errorlevel 1 (
    call :DESENHAR_BARRA 25
    echo.
    echo   ❌ Python não encontrado!
    echo.
    echo   📥 Iniciando download do Python 3.11...
    echo.

    powershell -Command "& {[Net.ServicePointManager]::SecurityProtocol = [Net.SecurityProtocolType]::Tls12; Invoke-WebRequest -Uri 'https://www.python.org/ftp/python/3.11.9/python-3.11.9-amd64.exe' -OutFile '%TEMP%\python-installer.exe'}"

    if errorlevel 1 (
        echo.
        echo   ❌ Falha no download!
        echo.
        echo   📥 Instale manualmente: https://www.python.org/downloads/
        echo.
        goto FIM_COM_ERRO
    )

    call :DESENHAR_BARRA 50
    echo.
    echo   ✅ Download concluído!
    echo   🚀 Executando instalador...
    echo.

    "%TEMP%\python-installer.exe" /passive InstallAllUsers=1 PrependPath=1 Include_test=0
    timeout /t 5 >nul

    call :DESENHAR_BARRA 75
    echo.
    echo   ✅ Python instalado!
    echo.
    echo ════════════════════════════════════════════════════════════════════
    echo   ⚠️  REINICIE ESTE INSTALADOR para atualizar o PATH
    echo ════════════════════════════════════════════════════════════════════
    echo.
    goto FIM_NORMAL
)

call :DESENHAR_BARRA 100
echo.
echo   ✅ Python encontrado!
for /f "tokens=*" %%i in ('python --version 2^>^&1') do echo   📦 %%i
timeout /t 2 >nul

REM ============================================================================
REM                        ETAPA 2: INSTALAR DEPENDÊNCIAS
REM ============================================================================
:ETAPA2
cls
call :DESENHAR_HEADER
echo.
echo   ┌─────────────────────────────────────────────────────────────────────┐
echo   │  ETAPA 2/4: Instalando Dependências Python                          │
echo   └─────────────────────────────────────────────────────────────────────┘
echo.

echo   📦 Atualizando pip...
call :DESENHAR_BARRA 0
timeout /t 1 >nul

python -m pip install --upgrade pip --quiet --disable-pip-version-check >nul 2>&1

call :DESENHAR_BARRA 30
echo   📦 Instalando NumPy...
timeout /t 1 >nul

python -m pip install numpy --quiet --disable-pip-version-check >nul 2>&1
if errorlevel 1 (
    echo.
    echo   ⚠️  Instalação silenciosa falhou. Tentando com output...
    python -m pip install numpy
    if errorlevel 1 (
        echo.
        echo ════════════════════════════════════════════════════════════════════
        echo   ❌ Erro ao instalar NumPy!
        echo ════════════════════════════════════════════════════════════════════
        echo.
        goto FIM_COM_ERRO
    )
)

call :DESENHAR_BARRA 65
echo   📦 Instalando Requests...
timeout /t 1 >nul

python -m pip install requests --quiet --disable-pip-version-check >nul 2>&1

call :DESENHAR_BARRA 100
echo.
echo   ✅ Todas as dependências instaladas!
timeout /t 2 >nul

REM ============================================================================
REM                       ETAPA 3: VERIFICAR ARQUIVOS
REM ============================================================================
:ETAPA3
cls
call :DESENHAR_HEADER
echo.
echo   ┌─────────────────────────────────────────────────────────────────────┐
echo   │  ETAPA 3/4: Verificando Arquivos                                     │
echo   └─────────────────────────────────────────────────────────────────────┘
echo.

set PROGRESS=0
set TOTAL=3
set ERRO=0

call :CHECK_FILE "tracker_control_center.py" "Servidor Web"
call :CHECK_FILE "INICIAR_TRACKER.bat" "Script de Inicialização"
call :CHECK_FILE "PARAR_TRACKER.bat" "Script de Parada"

if %ERRO% GTR 0 (
    echo.
    echo ════════════════════════════════════════════════════════════════════
    echo   ❌ Faltam %ERRO% arquivo(s)!
    echo ════════════════════════════════════════════════════════════════════
    echo.
    goto FIM_COM_ERRO
)

echo.
echo   ✅ Todos os arquivos encontrados!
timeout /t 2 >nul

REM ============================================================================
REM                    ETAPA 4: CONFIGURAR FIREWALL (SEM PERGUNTA)
REM ============================================================================
:ETAPA4
cls
call :DESENHAR_HEADER
echo.
echo   ┌─────────────────────────────────────────────────────────────────────┐
echo   │  ETAPA 4/4: Configurando Firewall                                    │
echo   └─────────────────────────────────────────────────────────────────────┘
echo.
call :DESENHAR_BARRA 0

echo   🔓 Abrindo porta UDP 5005...
timeout /t 1 >nul

netsh advfirewall firewall delete rule name="VR Tracker UDP" >nul 2>&1
netsh advfirewall firewall add rule name="VR Tracker UDP" dir=in action=allow protocol=UDP localport=5005 >nul 2>&1

if errorlevel 1 (
    call :DESENHAR_BARRA 50
    echo.
    echo   ⚠️  Execute como ADMINISTRADOR para configurar firewall.
    timeout /t 2 >nul
) else (
    call :DESENHAR_BARRA 100
    echo.
    echo   ✅ Firewall configurado!
    timeout /t 2 >nul
)

REM ============================================================================
REM                           CRIAR CONFIGURAÇÃO
REM ============================================================================
:CRIAR_CONFIG
cls
call :DESENHAR_HEADER
echo.
echo   📝 Criando estrutura...
echo.

if not exist "calibration_profiles" mkdir calibration_profiles
if not exist "logs" mkdir logs
if not exist "temp" mkdir temp

if not exist "network_config.json" (
    (
        echo {
        echo   "udp_port": 5005,
        echo   "listen_address": "0.0.0.0"
        echo }
    ) > network_config.json
)

echo   ✅ Configuração criada!
timeout /t 1 >nul

REM ============================================================================
REM                        INSTALAÇÃO CONCLUÍDA
REM ============================================================================
:CONCLUIDO
cls
call :DESENHAR_HEADER
echo.
echo   ╔═══════════════════════════════════════════════════════════════════════╗
echo   ║              ✅  INSTALAÇÃO CONCLUÍDA COM SUCESSO!  ✅                ║
echo   ╚═══════════════════════════════════════════════════════════════════════╝
echo.
echo   🎯 PRÓXIMOS PASSOS:
echo.
echo   1️⃣  Execute: INICIAR_TRACKER.bat
echo   2️⃣  Sincronize o tracker na aba "Rede"
echo   3️⃣  Faça a calibração
echo.
echo ════════════════════════════════════════════════════════════════════════
echo.
echo   🚀 Deseja iniciar o sistema agora? (S/N)
echo.
set /p INICIAR="   Resposta: "

if /i "%INICIAR%"=="S" (
    if exist "INICIAR_TRACKER.bat" (
        start "" "INICIAR_TRACKER.bat"
        echo   ✅ Sistema iniciado!
        timeout /t 2 >nul
    )
)

goto FIM_NORMAL

:FIM_COM_ERRO
echo ════════════════════════════════════════════════════════════════════════
echo.
echo   Pressione qualquer tecla para fechar...
pause >nul
exit /b 1

:FIM_NORMAL
echo.
echo   ✨ Instalação finalizada!
echo.
echo   Pressione qualquer tecla para fechar...
pause >nul
exit /b 0

REM ============================================================================
REM                              FUNÇÕES
REM ============================================================================

:DESENHAR_HEADER
echo ╔════════════════════════════════════════════════════════════════════════════╗
echo ║                      VR TRACKER - INSTALAÇÃO DO CLIENTE                    ║
echo ╚════════════════════════════════════════════════════════════════════════════╝
goto :eof

:DESENHAR_BARRA
setlocal enabledelayedexpansion
set /a PERCENT=%1
set /a FILLED=%PERCENT%/2
set /a EMPTY=50-%FILLED%

set "BARRA="
for /l %%i in (1,1,%FILLED%) do set "BARRA=!BARRA!█"
for /l %%i in (1,1,%EMPTY%) do set "BARRA=!BARRA!░"

echo   [!BARRA!] %PERCENT%%%
endlocal
goto :eof

:CHECK_FILE
setlocal
set /a PROGRESS+=1
set /a PERCENT=(%PROGRESS%*100)/%TOTAL%

if exist %1 (
    echo   [%PROGRESS%/%TOTAL%] ✅ %~2
) else (
    echo   [%PROGRESS%/%TOTAL%] ❌ %~2 - FALTANDO
    set /a ERRO+=1
)
call :DESENHAR_BARRA %PERCENT%
timeout /t 1 >nul
endlocal & set ERRO=%ERRO%
goto :eof
