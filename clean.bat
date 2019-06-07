@echo off
if exist .\hdl\de0 call :proj de0
if exist .\hdl\vm1max call :proj vm1max
if exist .\hdl\vm2max call :proj vm2max
if exist .\hdl\vm3max call :proj vm3max
exit

:proj
if exist .\hdl\%1\out				rd  .\hdl\%1\out /s /q
if exist .\hdl\%1\*.jdi 			rd  .\hdl\%1\*.jdi /s /q
if exist .\hdl\%1\db 				rd  .\hdl\%1\db /s /q
if exist .\hdl\%1\incremental_db 		rd  .\hdl\%1\incremental_db /s /q
if exist .\hdl\%1\greybox_tmp	 		rd  .\hdl\%1\greybox_tmp /s /q
if exist .\hdl\%1\sim\rtl_work		 	rd  .\hdl\%1\sim\rtl_work /s /q
if exist .\hdl\%1\sim\*.mif			del .\hdl\%1\sim\*.mif /s /q
if exist .\hdl\%1\sim\*.hex			del .\hdl\%1\sim\*.hex /s /q
if exist .\hdl\%1\sim\*rtl_verilog.do		del .\hdl\%1\sim\*rtl_verilog.do /s /q

del *.qws *.sdo *.vo *.qip *.sft *.wlf *.jdi *.ver *.mem *.xrf *.bak msim_transcript. do modelsim.ini *.rpt /q /s

for /r "./hdl/%~1" %%9 in (*.v) do atxt32 %%9 %%9 -s3 -f
exit /b

