@echo off

set DOWNLOADS_DIR=%USERPROFILE%\Downloads
set PATH=^
%DOWNLOADS_DIR%\arduino-cli_0.34.2_Windows_64bit;

if not exist "%DOWNLOADS_DIR%\serial-locate-v1.0.0\serial-locate.exe" (
cd /d "%DOWNLOADS_DIR%" && ^
%SystemRoot%\System32\curl.exe "https://github.com/dirkarnez/serial-locate/releases/download/v1.0.0/serial-locate-v1.0.0.zip" -L -O &&^
C:\Windows\System32\WindowsPowerShell\v1.0\powershell.exe -command "Expand-Archive -LiteralPath 'serial-locate-v1.0.0.zip' -DestinationPath 'serial-locate-v1.0.0'"
) else (
	echo serial-locate found
)


if not exist %DOWNLOADS_DIR%\arduino-cli_0.34.2_Windows_64bit\arduino-cli.exe (
cd %DOWNLOADS_DIR% && ^
%SystemRoot%\System32\curl.exe "https://github.com/arduino/arduino-cli/releases/download/0.34.2/arduino-cli_0.34.2_Windows_64bit.zip" -L -O &&^
C:\Windows\System32\WindowsPowerShell\v1.0\powershell.exe -command "Expand-Archive -LiteralPath 'arduino-cli_0.34.2_Windows_64bit.zip' -DestinationPath 'arduino-cli_0.34.2_Windows_64bit'"
) else (
echo arduino-cli found 
)

arduino-cli.exe core download arduino:avr &&^
arduino-cli.exe core install arduino:avr &&^
echo installing arduino:avr

for /f "tokens=*" %%a in (
'%DOWNLOADS_DIR%\serial-locate-v1.0.0\serial-locate.exe --vid=2341 --pid=0043 --usb=true'
) do (
    set output=%%a
)

echo %output%

arduino-cli.exe upload -p %output% --fqbn  arduino:avr:uno --input-file=EIE3105_ATmega328P_Application.hex
pause
