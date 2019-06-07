onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate -group DE0 /tb1/clk50
add wave -noupdate -group DE0 /tb1/button
add wave -noupdate -group DE0 /tb1/switch
add wave -noupdate -group DE0 /tb1/hex0
add wave -noupdate -group DE0 /tb1/hex1
add wave -noupdate -group DE0 /tb1/hex2
add wave -noupdate -group DE0 /tb1/hex3
add wave -noupdate -group DE0 /tb1/led
add wave -noupdate -group DE0 /tb1/uart_txd
add wave -noupdate -group DE0 /tb1/uart_rxd
add wave -noupdate -group DE0 /tb1/uart_cts
add wave -noupdate -group DE0 /tb1/uart_rts
add wave -noupdate -group DE0 /tb1/dram_dq
add wave -noupdate -group DE0 /tb1/dram_addr
add wave -noupdate -group DE0 /tb1/dram_ldqm
add wave -noupdate -group DE0 /tb1/dram_udqm
add wave -noupdate -group DE0 /tb1/dram_we_n
add wave -noupdate -group DE0 /tb1/dram_cas_n
add wave -noupdate -group DE0 /tb1/dram_ras_n
add wave -noupdate -group DE0 /tb1/dram_cs_n
add wave -noupdate -group DE0 /tb1/dram_ba
add wave -noupdate -group DE0 /tb1/dram_clk
add wave -noupdate -group DE0 /tb1/dram_cke
add wave -noupdate -group DE0 /tb1/fl_dq
add wave -noupdate -group DE0 /tb1/fl_addr
add wave -noupdate -group DE0 /tb1/fl_we_n
add wave -noupdate -group DE0 /tb1/fl_rst_n
add wave -noupdate -group DE0 /tb1/fl_oe_n
add wave -noupdate -group DE0 /tb1/fl_ce_n
add wave -noupdate -group DE0 /tb1/fl_wp_n
add wave -noupdate -group DE0 /tb1/fl_byte_n
add wave -noupdate -group DE0 /tb1/fl_rb
add wave -noupdate -group DE0 /tb1/lcd_blig
add wave -noupdate -group DE0 /tb1/lcd_rw
add wave -noupdate -group DE0 /tb1/lcd_en
add wave -noupdate -group DE0 /tb1/lcd_rs
add wave -noupdate -group DE0 /tb1/lcd_data
add wave -noupdate -group DE0 /tb1/sd_dat0
add wave -noupdate -group DE0 /tb1/sd_dat3
add wave -noupdate -group DE0 /tb1/sd_cmd
add wave -noupdate -group DE0 /tb1/sd_clk
add wave -noupdate -group DE0 /tb1/sd_wp_n
add wave -noupdate -group DE0 /tb1/ps2_kbdat
add wave -noupdate -group DE0 /tb1/ps2_kbclk
add wave -noupdate -group DE0 /tb1/ps2_msdat
add wave -noupdate -group DE0 /tb1/ps2_msclk
add wave -noupdate -group DE0 /tb1/vga_hs
add wave -noupdate -group DE0 /tb1/vga_vs
add wave -noupdate -group DE0 /tb1/vga_r
add wave -noupdate -group DE0 /tb1/vga_g
add wave -noupdate -group DE0 /tb1/vga_b
add wave -noupdate -group DE0 /tb1/gpio0_clkin
add wave -noupdate -group DE0 /tb1/gpio0_clkout
add wave -noupdate -group DE0 /tb1/gpio0_d
add wave -noupdate -group DE0 /tb1/gpio1_clkin
add wave -noupdate -group DE0 /tb1/gpio1_clkout
add wave -noupdate -group DE0 /tb1/gpio1_d
add wave -noupdate -radix octal /tb1/nAD
add wave -noupdate -radix octal /tb1/AD_in
add wave -noupdate -radix octal /tb1/AD_out
add wave -noupdate -radix octal /tb1/AD_vec
add wave -noupdate /tb1/AD_oe
add wave -noupdate /tb1/nINIT
add wave -noupdate /tb1/nSYNC
add wave -noupdate /tb1/nDIN
add wave -noupdate /tb1/nDOUT
add wave -noupdate /tb1/nWTBT
add wave -noupdate /tb1/nBSY
add wave -noupdate /tb1/nRPLY
add wave -noupdate /tb1/nVIRQ
add wave -noupdate -expand /tb1/nIRQ
add wave -noupdate /tb1/nIAKO
add wave -noupdate /tb1/CLK
add wave -noupdate /tb1/nACLO
add wave -noupdate /tb1/nDCLO
add wave -noupdate /tb1/de0_top/wb_clk
add wave -noupdate /tb1/de0_top/wb_cyc
add wave -noupdate /tb1/de0_top/wb_ack
add wave -noupdate /tb1/de0_top/wb_adr
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {2403641 ps} 0}
quietly wave cursor active 1
configure wave -namecolwidth 40
configure wave -valuecolwidth 40
configure wave -justifyvalue left
configure wave -signalnamewidth 1
configure wave -snapdistance 10
configure wave -datasetprefix 0
configure wave -rowmargin 4
configure wave -childrowmargin 2
configure wave -gridoffset 0
configure wave -gridperiod 1
configure wave -griddelta 40
configure wave -timeline 0
configure wave -timelineunits ns
update
WaveRestoreZoom {0 ps} {3899659 ps}
