@echo off
setlocal
cd /d "%~dp0.."

if not exist "server\.venv\Scripts\python.exe" (
  echo Missing server virtual environment. Run server\start-server.bat once first.
  exit /b 1
)

REM Tunnel is the only public entry point. Keep FastAPI off all LAN interfaces.
"server\.venv\Scripts\python.exe" -m uvicorn server.app:app --host 127.0.0.1 --port 8080
