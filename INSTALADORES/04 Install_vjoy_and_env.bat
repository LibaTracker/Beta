@echo off
SETLOCAL

REM Define o diretório base do projeto (onde este .bat está)
SET "PROJECT_DIR=%~dp0"
CD /D "%PROJECT_DIR%"

ECHO.
ECHO ======================================================
ECHO  Configuracao do Ambiente Python e vJoy para VR Tracker
ECHO ======================================================
ECHO.

REM 1. Verifica se o Python esta instalado
python --version >nul 2>&1
IF %ERRORLEVEL% NEQ 0 (
    ECHO ❌ Python nao encontrado. Por favor, instale Python 3.x (com pip) e adicione-o ao PATH.
    ECHO    Recomendado: https://www.python.org/downloads/
    PAUSE
    EXIT /B 1
)
ECHO ✅ Python encontrado.

REM 2. Cria ou ativa o ambiente virtual
ECHO.
ECHO Criando/Ativando ambiente virtual...
IF NOT EXIST "venv" (
    python -m venv venv
    IF %ERRORLEVEL% NEQ 0 (
        ECHO ❌ Falha ao criar ambiente virtual.
        PAUSE
        EXIT /B 1
    )
    ECHO ✅ Ambiente virtual 'venv' criado.
) ELSE (
    ECHO ✅ Ambiente virtual 'venv' ja existe.
)

REM Ativa o ambiente virtual
CALL "%PROJECT_DIR%venv\Scripts\activate"
IF %ERRORLEVEL% NEQ 0 (
    ECHO ❌ Falha ao ativar ambiente virtual.
    PAUSE
    EXIT /B 1
)
ECHO ✅ Ambiente virtual ativado.

REM 3. Instala as dependencias do requirements.txt
ECHO.
ECHO Instalando dependencias do requirements.txt...
IF EXIST "requirements.txt" (
    pip install -r requirements.txt
    IF %ERRORLEVEL% NEQ 0 (
        ECHO ❌ Falha ao instalar dependencias.
        ECHO    Verifique sua conexao com a internet ou o conteudo de requirements.txt.
        PAUSE
        EXIT /B 1
    )
    ECHO ✅ Dependencias instaladas com sucesso.
) ELSE (
    ECHO ⚠️  requirements.txt nao encontrado. Pulando instalacao de dependencias.
)

REM 4. Instala o pyvjoy (se nao estiver ja no requirements.txt e for necessario)
REM Se pyvjoy ja estiver no requirements.txt, esta etapa pode ser redundante.
REM Mantemos aqui como um fallback ou para garantir que seja instalado.
ECHO.
ECHO Verificando e instalando pyvjoy...
pip show pyvjoy >nul 2>&1
IF %ERRORLEVEL% NEQ 0 (
    pip install pyvjoy
    IF %ERRORLEVEL% NEQ 0 (
        ECHO ❌ Falha ao instalar pyvjoy.
        ECHO    Verifique se o vJoy esta instalado corretamente no seu sistema.
        PAUSE
        EXIT /B 1
    )
    ECHO ✅ pyvjoy instalado com sucesso.
) ELSE (
    ECHO ✅ pyvjoy ja esta instalado.
)

REM 5. Instala o driver vJoy (se ainda nao estiver instalado)
ECHO.
ECHO Verificando e instalando driver vJoy...
REM Este comando assume que o instalador do vJoy esta em um local conhecido
REM ou que o usuario o executara manualmente.
REM Para automatizar, voce precisaria do instalador do vJoy e de comandos silenciosos.
REM Por enquanto, apenas instruimos o usuario.
ECHO Por favor, certifique-se de que o driver vJoy esta instalado no seu sistema.
ECHO Voce pode baixa-lo em: https://github.com/jshafer817/vJoy/releases
ECHO.
ECHO Pressione qualquer tecla para continuar apos verificar/instalar o vJoy...
PAUSE >nul

ECHO.
ECHO ======================================================
ECHO  Configuracao Concluida!
ECHO ======================================================
ECHO.
ECHO Para iniciar a Central de Controle do VR Tracker, execute:
ECHO "%PROJECT_DIR%venv\Scripts\python.exe" "%PROJECT_DIR%tracker_control_center.py"
ECHO.
ECHO Ou simplesmente execute 'start_tracker_control_center.bat'
ECHO.

ENDLOCAL
