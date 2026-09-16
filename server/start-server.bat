@echo off
REM Double-click this file to run the ESP32 telemetry collector.
REM First run creates server\.venv and installs the dependencies.
setlocal
cd /d "%~dp0.."

if not exist "server\.venv\Scripts\python.exe" (
  echo [1/2] Creating virtual environment...
  py -3 -m venv "server\.venv"
  if errorlevel 1 goto :error
  echo [2/2] Installing dependencies...
  "server\.venv\Scripts\python.exe" -m pip install --upgrade pip -q
  "server\.venv\Scripts\python.exe" -m pip install -r "server\requirements.txt"
  if errorlevel 1 goto :error
)

echo.
echo Telemetry collector starting...
echo.
echo   History page : http://localhost:8080/
echo   From the LAN : http://YOUR-PC-IP:8080/   (find it with: ipconfig)
echo.
echo Keep this window open. Press Ctrl+C to stop.
echo.

REM Open the history page a few seconds later, once uvicorn is listening.
start "" cmd /c "timeout /t 3 /nobreak >nul & start http://localhost:8080/"

"server\.venv\Scripts\python.exe" -m uvicorn server.app:app --host 0.0.0.0 --port 8080
goto :eof

:error
echo.
echo Setup failed - see the messages above.
pause
