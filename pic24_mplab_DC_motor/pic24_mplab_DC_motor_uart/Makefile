# MPLAB IDE generated this makefile for use with GNU make.
# Project: motor driver.mcp
# Date: Wed May 21 22:12:20 2014

AS = xc16-as.exe
CC = xc16-gcc.exe
LD = xc16-ld.exe
AR = xc16-ar.exe
HX = xc16-bin2hex.exe
RM = rm

motor\ driver.hex : motor\ driver.cof
	$(HX) "motor driver.cof" -omf=coff

motor\ driver.cof : main.o
	$(CC) -omf=coff -mcpu=24FJ48GA002 "main.o" -o"motor driver.cof" -Wl,-L"D:\Program Files\Microchip\xc16\v1.21\lib",--script="p24FJ48GA002.gld",--defsym=__MPLAB_BUILD=1,-Map="motor driver.map",--report-mem

main.o : p24fj48ga002.h main.c
	$(CC) -omf=coff -mcpu=24FJ48GA002 -x c -c "main.c" -o"main.o" -I"D:\Program Files\Microchip\xc16\v1.21\inc" -g -Wall

clean : 
	$(RM) "main.o" "motor driver.cof" "motor driver.hex"

