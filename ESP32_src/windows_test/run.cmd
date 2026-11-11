@echo off
setlocal
chcp 65001 >nul
py -3 -c "import sys; assert sys.version_info >= (3,9)" >nul 2>&1
if not errorlevel 1 (
  py -3 "%~dp0windows_test.py" %*
  goto finished
)
python -c "import sys; assert sys.version_info >= (3,9)" >nul 2>&1
if not errorlevel 1 (
  python "%~dp0windows_test.py" %*
  goto finished
)
echo Please install Python 3.9+ from python.org with pip and PATH enabled.
pause
exit /b 1
:finished
set "test_exit=%errorlevel%"
if not "%test_exit%"=="0" echo Test stopped with exit code %test_exit%.
pause
exit /b %test_exit%
