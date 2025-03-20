SHELL=cmd
CC = xc32-gcc
OBJCPY = xc32-bin2hex
ARCH = -mprocessor=32MX130F064B
OBJ = remote.o lcd.o
PORTN=$(shell type COMPORT.inc)

remote.elf: $(OBJ)
	$(CC) $(ARCH) -o remote.elf $(OBJ) -mips16 -DXPRJ_default=default -legacy-libc -Wl,-Map=remote.map
	$(OBJCPY) remote.elf
	@echo Success!
   
remote.o: remote.c lcd.h
	$(CC) -mips16 -g -x c -c $(ARCH) -MMD -o remote.o remote.c -DXPRJ_default=default -legacy-libc

lcd.o: lcd.c lcd.h
	$(CC) -mips16 -g -x c -c $(ARCH) -MMD -o lcd.o lcd.c -DXPRJ_default=default -legacy-libc

clean:
	@del *.o *.elf *.hex *.map *.d 2>NUL
	
LoadFlash:
	@taskkill /f /im putty.exe /t /fi "status eq running" > NUL
	pro32 -p remote.hex
	cmd /c start putty.exe -serial $(PORTN) -sercfg 115200,8,n,1,N

putty:
	@taskkill /f /im putty.exe /t /fi "status eq running" > NUL
	cmd /c start putty.exe -serial $(PORTN) -sercfg 115200,8,n,1,N

Picture:
	@cmd /c start Pictures\PIC32MX130_LCD.jpg

dummy: remote.hex remote.map
	$(CC) --version
