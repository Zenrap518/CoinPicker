SHELL=cmd
CC=arm-none-eabi-gcc
AS=arm-none-eabi-as
LD=arm-none-eabi-ld
CCFLAGS=-mcpu=cortex-m0 -mthumb -g -DSTM32L051xx
#MAKEFLAGS += -B

# Search for the path of the right libraries.  Works only on Windows.
GCCPATH=$(subst \bin\arm-none-eabi-gcc.exe,\,$(shell where $(CC)))
LIBPATH1=$(subst \libgcc.a,,$(shell dir /s /b "$(GCCPATH)*libgcc.a" | find "v6-m"))
LIBPATH1=D:\CrossIDE\gcc-arm-none-eabi-10.3-2021.10-win32\gcc-arm-none-eabi-10.3-2021.10\lib\gcc\arm-none-eabi\10.3.1\thumb\v6-m\nofp
LIBPATH2=$(subst \libc_nano.a,,$(shell dir /s /b "$(GCCPATH)*libc_nano.a" | find "v6-m"))
LIBPATH2=D:\CrossIDE\gcc-arm-none-eabi-10.3-2021.10-win32\gcc-arm-none-eabi-10.3-2021.10\arm-none-eabi\lib\thumb\v6-m\nofp
LIBSPEC=-L"$(LIBPATH1)" -L"$(LIBPATH2)"

OBJS= main.o wait.o serial.o startup.o newlib_stubs.o UART2.o md.o

PORTN=$(shell type COMPORT.inc)
BUILD_DIR = build
VPATH = $(BUILD_DIR)

# For smaller hex file remove '-u _printf_float' below, SIGNIFICANTLY reduces file size (16kb reduction)
main.elf : $(BUILD_DIR) $(OBJS)
	$(LD) $(addprefix $(BUILD_DIR)/, $(OBJS) ) $(LIBSPEC) -Os  -nostdlib -lnosys -lgcc -T Common/LDscripts/stm32l051xx.ld --cref -Map $(BUILD_DIR)/main.map -o $(BUILD_DIR)/main.elf
	arm-none-eabi-objcopy -O ihex $(BUILD_DIR)/main.elf $(BUILD_DIR)/main.hex
	@echo.
	@echo Success!
	@echo.

main.o: main.c
	$(CC) -c $(CCFLAGS) main.c -o $(BUILD_DIR)/main.o

wait.o: wait.c
	$(CC) -c $(CCFLAGS) wait.c -o $(BUILD_DIR)/wait.o

startup.o: Common/Source/startup.c
	$(CC) -c $(CCFLAGS) -DUSE_USART1 Common/Source/startup.c -o $(BUILD_DIR)/startup.o

serial.o: Common/Source/serial.c
	$(CC) -c $(CCFLAGS) Common/Source/serial.c -o $(BUILD_DIR)/serial.o

newlib_stubs.o: Common/Source/newlib_stubs.c
	$(CC) -c $(CCFLAGS) Common/Source/newlib_stubs.c -o $(BUILD_DIR)/newlib_stubs.o

UART2.o: UART2.c
	$(CC) -c $(CCFLAGS) UART2.c -o $(BUILD_DIR)/UART2.o

md.o: md.c
	$(CC) -c $(CCFLAGS) md.c -o $(BUILD_DIR)/md.o

clean: 
	@del /q $(BUILD_DIR)\*.* 2>NUL
	@echo Success!
	@echo.
	
Flash_Load:
	@taskkill /f /im putty.exe /t /fi "status eq running" > NUL
	@echo stm32flash\stm32flash -w build/main.hex -b 230400  -v -R -g  0x0 ^^>sflash.bat
	@ stm32flash\BO230\BO230 -b >>sflash.bat
	@sflash.bat
	@echo cmd /c start putty.exe -sercfg 115200,8,n,1,N -serial ^^>sputty.bat
	@ stm32flash\BO230\BO230 -r >>sputty.bat
	@sputty


putty:
	@taskkill /f /im putty.exe /t /fi "status eq running" > NUL
	@echo cmd /c start putty.exe -sercfg 115200,8,n,1,N -serial ^^>sputty.bat
	@ stm32flash\BO230\BO230 -r >>sputty.bat
	@sputty
	


