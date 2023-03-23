@echo off
::This file was created automatically by CrossIDE to compile with C51.
C:
cd "\Users\peter\Documents\ELEC291\Project2\Project-2-Tracking-Robot\doc\"
"C:\CrossIDE\Call51\Bin\c51.exe" --use-stdout  "C:\Users\peter\Documents\ELEC291\Project2\Project-2-Tracking-Robot\doc\remote.c"
if not exist hex2mif.exe goto done
if exist remote.ihx hex2mif remote.ihx
if exist remote.hex hex2mif remote.hex
:done
echo done
echo Crosside_Action Set_Hex_File C:\Users\peter\Documents\ELEC291\Project2\Project-2-Tracking-Robot\doc\remote.hex
