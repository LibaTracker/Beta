@echo off
setlocal enabledelayedexpansion

echo [LibaTracker] Instalador do driver - inicio.

rem PASTA DE ORIGEM (onde estão os arquivos do driver)
set "SRC_ROOT=C:\libatracker"

if not exist "%SRC_ROOT%" (
  echo Erro: pasta de origem nao encontrada: %SRC_ROOT%
  echo Coloque os arquivos do driver em %SRC_ROOT% (ou em uma subpasta) e reexecute.
  pause
  exit /b 1
)

rem Procurar pasta que contenha driver.vrdrivermanifest ou driver*.dll
set "FOUND_SRC="
for /r "%SRC_ROOT%" %%F in (driver.vrdrivermanifest driver*.dll *.dll *.json *.jsonc) do (
  rem pega a pasta do arquivo encontrado
  set "FOUND_SRC=%%~dpF"
  goto :found_src
)
:found_src

if "%FOUND_SRC%"=="" (
  rem se não achou por nome, usa a raiz
  set "FOUND_SRC=%SRC_ROOT%"
)

echo Pasta fonte selecionada: "%FOUND_SRC%"

rem localizar Steam
echo Procurando instalacao do Steam...
for /f "tokens=2* delims=    " %%A in ('reg query "HKLM\SOFTWARE\Wow6432Node\Valve\Steam" /v InstallPath 2^>nul ^| find "InstallPath"') do set STEAMPATH=%%B
if "%STEAMPATH%"=="" (
    for /f "tokens=2* delims=    " %%A in ('reg query "HKCU\Software\Valve\Steam" /v SteamPath 2^>nul ^| find "SteamPath"') do set STEAMPATH=%%B
)
if "%STEAMPATH%"=="" (
  if exist "C:\Program Files (x86)\Steam" set STEAMPATH=C:\Program Files (x86)\Steam
)
if "%STEAMPATH%"=="" (
  if exist "C:\Program Files\Steam" set STEAMPATH=C:\Program Files\Steam
)

if "%STEAMPATH%"=="" (
  echo Nao foi possivel localizar a instalacao do Steam automaticamente.
  echo Informe o caminho manualmente ou coloque o Steam no local padrao.
  pause
  exit /b 1
)

echo Steam encontrado em: "%STEAMPATH%"

rem determinar pasta SteamVR\drivers
set "STEAMVR_DRIVERS=%STEAMPATH%\steamapps\common\SteamVR\drivers"
if not exist "%STEAMVR_DRIVERS%" (
  rem tentar alternativa
  if exist "%STEAMPATH%\common\SteamVR\drivers" set "STEAMVR_DRIVERS=%STEAMPATH%\common\SteamVR\drivers"
)

if not exist "%STEAMVR_DRIVERS%" (
  echo Pasta SteamVR\drivers nao encontrada. Certifique-se de que o SteamVR esteja instalado.
  pause
  exit /b 1
)

set "TARGET_FOLDER=libatracker"
set "TARGET_PATH=%STEAMVR_DRIVERS%\%TARGET_FOLDER%"

echo Criando pasta destino: "%TARGET_PATH%"
mkdir "%TARGET_PATH%" 2>nul

echo Copiando arquivos de "%FOUND_SRC%" para "%TARGET_PATH%" ...
xcopy /Y /E /I "%FOUND_SRC%\*" "%TARGET_PATH%\" >nul
if %ERRORLEVEL% neq 0 (
  echo Erro ao copiar arquivos. Execute este script como Administrador.
  pause
  exit /b 1
)

echo Copia concluida com sucesso.

rem tenta fechar o Steam (se estiver rodando) para forcar recarregamento
echo Reiniciando Steam (se estiver em execucao)...
taskkill /IM steam.exe /F >nul 2>&1
timeout /t 2 >nul

echo Iniciando Steam...
start "" "%STEAMPATH%\steam.exe"

echo Instalacao concluida.
echo Abra o SteamVR e verifique se o dispositivo 'libatracker' aparece em Devices -> Tracked Devices.
pause
endlocal
