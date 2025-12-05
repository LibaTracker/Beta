@echo off
SETLOCAL

REM Define o diretório base do projeto (onde este .bat está)
SET "PROJECT_DIR=%~dp0"
CD /D "%PROJECT_DIR%"

REM Ativa o ambiente virtual
CALL "%PROJECT_DIR%venv\Scripts\activate"
IF %ERRORLEVEL% NEQ 0 (
    ECHO ❌ Falha ao ativar ambiente virtual. Certifique-se de ter executado 'install_vjoy_and_env.bat' primeiro.
    PAUSE
    EXIT /B 1
)

REM Inicia o tracker_control_center.py
ECHO.
ECHO Iniciando Central de Controle do VR Tracker...
"%PROJECT_DIR%venv\Scripts\python.exe" "%PROJECT_DIR%tracker_control_center.py"

REM Desativa o ambiente virtual ao sair (opcional, mas boa pratica)
deactivate

ENDLOCAL
