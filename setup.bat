@echo off
setlocal enabledelayedexpansion

REM Set Python version variable
set PYTHON_VERSION=py -3.14

echo ==================================
echo PadSuite Project Setup
echo ==================================
echo.

REM Check if the specified Python version is installed
%PYTHON_VERSION% --version >nul 2>nul
if %ERRORLEVEL% NEQ 0 (
    echo ERROR: %PYTHON_VERSION% is not installed.
    echo Please install %PYTHON_VERSION% first: https://www.python.org/ftp/python/3.14.5/python-3.14.5-amd64.exe
    echo.
    pause
    exit /b 1
)

echo %PYTHON_VERSION% detected:
%PYTHON_VERSION% --version
echo.

REM Check if requirements.txt exists in the current directory
if exist "requirements.txt" (
    echo.
    echo --- Setting up virtual environment in current directory ---

    REM Create virtual environment if it doesn't exist
    if not exist ".\.venv" (
        echo Creating virtual environment with %PYTHON_VERSION%
        %PYTHON_VERSION% -m venv ".\.venv"
    )

    REM Upgrade pip
    call ".\.venv\Scripts\python.exe" -m pip install --upgrade pip

    REM Install dependencies
    call ".\.venv\Scripts\python.exe" -m pip install -r "requirements.txt"

    echo Finished setting up virtual environment
    )
)

echo.
echo ==================================
echo Setup complete
echo ==================================
pause