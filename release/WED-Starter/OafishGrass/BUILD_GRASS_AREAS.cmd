@echo off
setlocal
set "HERE=%~dp0"
set "SCENERY=%HERE%.."
where py >nul 2>nul
if %errorlevel%==0 (
  py -3 "%HERE%ogr_extract_wed.py" "%SCENERY%"
) else (
  python "%HERE%ogr_extract_wed.py" "%SCENERY%"
)
if errorlevel 1 (
  echo.
  echo [OGR] GAGAL. Pastikan earth.wed.xml sudah disimpan dari WED dan Python tersedia.
  pause
  exit /b 1
)
echo.
echo [OGR] OK. ogr_areas.json sudah dibuat dari mapping WED.
pause
