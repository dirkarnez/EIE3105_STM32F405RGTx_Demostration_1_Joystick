atmega328p-adc
==============
### GCC
- [Optimize Options (Using the GNU Compiler Collection (GCC))](https://gcc.gnu.org/onlinedocs/gcc/Optimize-Options.html)

### Notes
- GoogleTest does not support C project, therefore
    - Force CMake to compile C source (`main.c`) with as C++ source:
        - `set_source_files_properties("main.c" PROPERTIES LANGUAGE CXX)`
### Reference
- [AlexWills37/ATmega-Music-Automation: September 2021 - Firmware for the ATmega 328p that allows you to play progammable music with a piezo buzzer](https://github.com/AlexWills37/ATmega-Music-Automation)
- [elmohrimedabdessalam/ADC-AVR: C library for Analog to Digital Converter module of ATmega328 MCU](https://github.com/elmohrimedabdessalam/ADC-AVR)
- [Google-Developers-Sohag/AVR-Sandbox: Dig deeper into AVR-WORLD](https://github.com/Google-Developers-Sohag/AVR-Sandbox)
- [Software-Hardware-Codesign/AVR-Sandbox: Dig deeper into the AVR-WORLD using the 8-bit mcu avr devices on a special shell environment.](https://github.com/Software-Hardware-Codesign/AVR-Sandbox)
- [Software-Hardware-Codesign/ShiftAvr: A modern implementation of avr-core and GNU libraries for avr microcontroller units purely written in C.](https://github.com/Software-Hardware-Codesign/ShiftAvr)
- [aleksa012po/C_Arduino_Electronic_Piano_Keyboard: Arduino Electronic Piano Keyboard written in ANSI C, not finished, idea is to use one input analog pin, and 8 buttons with some resistors, ideally for ATinny or some ESP](https://github.com/aleksa012po/C_Arduino_Electronic_Piano_Keyboard)
- https://github.com/unalfaruk/ATMega328P_Examples/blob/master/TIMER1_EX/TIMER1_EX/main.c
- [git-aniket/ATMega328P: This repository contains all work done on the ATMega328P viz PID position control, PWM, ADC among others](https://github.com/git-aniket/ATMega328P)
- [enderslash1010/ATmega328p: Repo for bare-metal programming with ATmega328p](https://github.com/enderslash1010/ATmega328p)
- [aleksa012po/C_Arduino_Generate_Sound_Two_Projects: C_Arduino_Generisanje_Zvuka - Arduino Assembler, Sound Generation. Connect the amplifier with the transistor to the output pin. Use timers to change frequencies and change zeros and ones.](https://github.com/aleksa012po/C_Arduino_Generate_Sound_Two_Projects)
- [BaseMax/TestAVR](https://github.com/BaseMax/TestAVR)
- [ramonauble/sp: an open source synthesizer & sequencer, based on the Atmega328p.](https://github.com/ramonauble/sp)
- [aleksa012po/C_I2C_Hardware_Arduino_EEPROM: I2C program for addressing hardware Arduino EEPROM written in ANSI C](https://github.com/aleksa012po/C_I2C_Hardware_Arduino_EEPROM)
- [aleksa012po/C_Arduino__ADC: Arduino ADC. Install a voltage divider at the input. Change the value of the resistor and read the states of the AD converter](https://github.com/aleksa012po/C_Arduino__ADC)
- [aleksa012po/C_Blink_2sec_Delay: Arduino Blink LED, delay 2 sec, written in ANSI C](https://github.com/aleksa012po/C_Blink_2sec_Delay)
- [aleksa012po/C_PWM_for_controling_LED_using_Interrupt: Simple program written in ANSI C for arduino, uses PWM and Interrupt for controlling LED diode](https://github.com/aleksa012po/C_PWM_for_controling_LED_using_Interrupt)
- [aleksa012po/C_Interrupts: Interrupts written in ANSI C](https://github.com/aleksa012po/C_Interrupts)
- [ibrhmkrt/MicroballisticHedefMermiAlgilama: Embedded Systems, C , Avr-C](https://github.com/ibrhmkrt/MicroballisticHedefMermiAlgilama)
- [aleksa012po/C_Timer0_With_ISR: Simple program written in ANSI C for arduino, I use Timer1 with ISR](https://github.com/aleksa012po/C_Timer0_With_ISR)
- [aleksa012po/C_Arduino_Electronic_Piano_Keyboard: Arduino Electronic Piano Keyboard written in ANSI C, not finished, idea is to use one input analog pin, and 8 buttons with some resistors, ideally for ATinny or some ESP](https://github.com/aleksa012po/C_Arduino_Electronic_Piano_Keyboard)
