@echo off
::This file was created automatically by CrossIDE to compile with C51.
C:
cd "\Users\Ryan\Desktop\ELEC_291_codes\Project2\"
"D:\CrossIDE\Call51\Bin\c51.exe" --use-stdout  "C:\Users\Ryan\Desktop\ELEC_291_codes\Project2\lcd.c"
if not exist hex2mif.exe goto done
if exist lcd.ihx hex2mif lcd.ihx
if exist lcd.hex hex2mif lcd.hex
:done
echo done
echo Crosside_Action Set_Hex_File C:\Users\Ryan\Desktop\ELEC_291_codes\Project2\lcd.hex
