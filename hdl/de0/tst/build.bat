@echo off
copy %1.mac %pf_tmp%\%1.mac >>NUL
echo macro hd2:%1.mac /list:hd2:%1.lst /object:hd2:%1.obj >%pf_tmp%\build.com
echo link hd2:%1.obj /execute:hd2:%1.lda /lda >>%pf_tmp%\build.com
%pf_ecc_pdp11%\pdp11.exe @hd2:build.com
srec_cat %pf_tmp%\%1.lda -dec_binary -o %pf_tmp%\%1.bin -binary
move vt52.log %pf_tmp%\vt52.log
rem fc /b %pf_tmp%\%1.bin original\%1.bin > %pf_tmp%\%1.a
srec_cat %pf_tmp%\%1.bin -binary -fill 0x00 0x0000 0x4000 -byte-swap 2 -o %pf_tmp%\%1.mem --VMem 16
srec_cat %pf_tmp%\%1.bin -binary -fill 0x00 0x0000 0x4000 -byte-swap 2 -o %pf_tmp%\%1.hex -Intel
srec_cat %pf_tmp%\%1.bin -binary -fill 0x00 0x0000 0x4000 -byte-swap 2 -o %pf_tmp%\%1.mif -Memory_Initialization_File 16 -obs=2
copy %pf_tmp%\%1.mem ..\sim\test.mem
copy %pf_tmp%\%1.mif ..\sim\rtl\de0_test.mif
copy %pf_tmp%\%1.mif ..\rtl\de0_test.mif
@echo on
