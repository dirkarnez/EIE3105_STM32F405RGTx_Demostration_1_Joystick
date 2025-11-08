REM run as Administrator
@echo off

set DOWNLOADS_DIR=%USERPROFILE%\Downloads
set DOWNLOADS_DIR_LINUX=%DOWNLOADS_DIR:\=/%

set SOFTWARES_DIR=D:\Softwares
set PROGRAM_FILES=C:\"Program Files"

SET PATH=^
%DOWNLOADS_DIR%\STM32CubeIDE-portable-v1.6.1\plugins\com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.1.400.202404281720\tools\bin;^
%SOFTWARES_DIR%\STM32CubeIDE-portable-v1.6.1\plugins\com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.1.400.202404281720\tools\bin;^
%PROGRAM_FILES%\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin;

cd .ci && ( (  STM32_Programmer_CLI.exe -c port=SWD freq=4000 -w RobotCar.elf -s && echo ok ) || echo failed ) && pause
