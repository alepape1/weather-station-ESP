@echo off
cd /d "%~dp0"
python flasher_gui.py
if errorlevel 1 (
    echo.
    echo Error al lanzar la app. Asegurate de tener Python instalado.
    pause
)
