SHELL=cmd
CC = xc32-gcc
OBJCPY = xc32-bin2hex
ARCH = -mprocessor=32MX130F064B
CCFLAGS = -mips16 -g -x c -mprocessor=32MX130F064B -MMD  -DXPRJ_default=default -legacy-libc
OBJS = remote.o lcd.o UART.o
BUILD_DIR=build
VPATH = $(BUILD_DIR)
PORTN=$(shell type COMPORT.inc)


remote.elf: $(BUILD_DIR) $(OBJS)
	$(CC) $(ARCH) -o $(BUILD_DIR)/remote.elf $(addprefix $(BUILD_DIR)/, $(OBJS) ) -mips16 -DXPRJ_default=default -legacy-libc -Wl,-Map=$(BUILD_DIR)/remote.map
	$(OBJCPY) $(BUILD_DIR)/remote.elf
	@echo.
	@echo Success!
	@echo.
   
remote.o: remote.c
	$(CC) -c $(CCFLAGS) remote.c -o $(BUILD_DIR)/remote.o

lcd.o: lcd.c
	$(CC) -c $(CCFLAGS) lcd.c -o $(BUILD_DIR)/lcd.o

UART.o: UART.c
	$(CC) -c $(CCFLAGS) UART.c -o $(BUILD_DIR)/UART.o

clean:
	@del /q $(BUILD_DIR)\*.* 2>NUL
	@echo Success!
	@echo.
	
LoadFlash:
	@taskkill /f /im putty.exe /t /fi "status eq running" > NUL
	pro32 -p $(BUILD_DIR)/remote.hex
	cmd /c start putty.exe -serial $(PORTN) -sercfg 115200,8,n,1,N

putty:
	@taskkill /f /im putty.exe /t /fi "status eq running" > NUL
	cmd /c start putty.exe -serial $(PORTN) -sercfg 115200,8,n,1,N
