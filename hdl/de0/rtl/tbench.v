//
// Copyright (c) 2014 by 1801BM1@gmail.com
//
// Testbench for the 1801BM1 replica, native QBUS version
//______________________________________________________________________________
//
`include "config.v"

module tb1();

reg         clk50;
reg [2:0]   button;
reg [9:0]   switch;
wire [7:0]  hex0, hex1, hex2, hex3;
wire [9:0]  led;
                                 //
wire        uart_txd;            // UART transmitter
reg         uart_rxd;            // UART receiver
wire        uart_cts;            // UART clear to send
reg         uart_rts;            // UART request to send
                                 //
wire [15:0] dram_dq;             // SDRAM data bus 16 bits
wire [12:0] dram_addr;           // SDRAM address bus 13 bits
wire        dram_ldqm;           // SDRAM low-byte data mask
wire        dram_udqm;           // SDRAM high-byte data mask
wire        dram_we_n;           // SDRAM write enable
wire        dram_cas_n;          // SDRAM column address strobe
wire        dram_ras_n;          // SDRAM row address strobe
wire        dram_cs_n;           // SDRAM chip select
wire [1:0]  dram_ba;             // SDRAM bank address
wire        dram_clk;            // SDRAM clock
wire        dram_cke;            // SDRAM clock enable
                                 //
wire [15:0] fl_dq;               // FLASH data bus 15 Bits
wire [21:0] fl_addr;             // FLASH address bus 22 Bits
wire        fl_we_n;             // FLASH write enable
wire        fl_rst_n;            // FLASH reset
wire        fl_oe_n;             // FLASH output enable
wire        fl_ce_n;             // FLASH chip enable
wire        fl_wp_n;             // FLASH hardware write protect
wire        fl_byte_n;           // FLASH selects 8/16-bit mode
wire        fl_rb;               // FLASH ready/busy
                                 //
wire        lcd_blig;            // LCD back light ON/OFF
wire        lcd_rw;              // LCD read/write select, 0 = write, 1 = read
wire        lcd_en;              // LCD enable
wire        lcd_rs;              // LCD command/data select, 0 = command, 1 = data
wire [7:0]  lcd_data;            // LCD data bus 8 bits
                                 //
wire           sd_dat0;          // SD Card Data 0
wire           sd_dat3;          // SD Card Data 3
wire           sd_cmd;           // SD Card Command Signal
wire           sd_clk;           // SD Card Clock
wire           sd_wp_n;          // SD Card Write Protect
                                 //
wire           ps2_kbdat;        // PS2 Keyboard Data
wire           ps2_kbclk;        // PS2 Keyboard Clock
wire           ps2_msdat;        // PS2 Mouse Data
wire           ps2_msclk;        // PS2 Mouse Clock
                                 //
wire           vga_hs;           // VGA H_SYNC
wire           vga_vs;           // VGA V_SYNC
wire [3:0]     vga_r;            // VGA Red[3:0]
wire [3:0]     vga_g;            // VGA Green[3:0]
wire [3:0]     vga_b;            // VGA Blue[3:0]
                                 //
wire [1:0]     gpio0_clkin;      // GPIO Connection 0 Clock In Bus
wire [1:0]     gpio0_clkout;     // GPIO Connection 0 Clock Out Bus
wire [31:0]    gpio0_d;          // GPIO Connection 0 Data Bus
                                 //
wire [1:0]     gpio1_clkin;      // GPIO Connection 1 Clock In Bus
wire [1:0]     gpio1_clkout;     // GPIO Connection 1 Clock Out Bus
wire [31:0]    gpio1_d;          // GPIO Connection 1 Data Bus

tri1  [15:0]   nAD;
reg   [15:0]   AD_in;
reg   [15:0]   AD_out;
reg   [15:0]   AD_vec;
reg            AD_oe;

reg            nINIT;
reg            nSYNC;
reg            nDIN;
reg            nDOUT;
reg            nWTBT;
reg            nBSY;
tri1           nRPLY;
tri1           nVIRQ;
tri1  [3:1]    nIRQ;
reg            nIAKO;
wire           CLK;
tri1           nACLO;
tri1           nDCLO;

initial
begin
   clk50 = 0;
   forever
      begin
         #10 clk50 = 0;
         #10 clk50 = 1;
      end
end

initial
begin
   button = 3'b111;
   uart_rxd = 1'b1;
   uart_rts = 1'b0;
end

initial
begin
   #`CONFIG_SIM_TIME_LIMIT $stop;
end

`ifdef CONFIG_SIM_DEBUG_TTY
always @(posedge lcd_en)
begin
#2
   $display("tty: %03O", lcd_data);
end
`endif

always @(posedge lcd_rs)
begin
   $display("Access to shadow register");
   $stop;
end

de0 de0_top(
   .de0_clock_50(clk50),
   .de0_clock_50_2(clk50),
   .de0_button(button),
   .de0_sw(switch),
   .de0_hex0(hex0),
   .de0_hex1(hex1),
   .de0_hex2(hex2),
   .de0_hex3(hex3),
   .de0_led(led),

   .de0_uart_txd(uart_txd),
   .de0_uart_rxd(uart_rxd /*uart_txd*/),
   .de0_uart_cts(uart_cts),
   .de0_uart_rts(uart_rts),

   .de0_dram_dq(dram_dq),
   .de0_dram_addr(dram_addr),
   .de0_dram_ldqm(dram_ldqm),
   .de0_dram_udqm(dram_udqm),
   .de0_dram_we_n(dram_we_n),
   .de0_dram_cas_n(dram_cas_n),
   .de0_dram_ras_n(dram_ras_n),
   .de0_dram_cs_n(dram_cs_n),
   .de0_dram_ba(dram_ba),
   .de0_dram_clk(dram_clk),
   .de0_dram_cke(dram_cke),

   .de0_fl_dq(fl_dq),
   .de0_fl_addr(fl_addr),
   .de0_fl_we_n(fl_we_n),
   .de0_fl_rst_n(fl_rst_n),
   .de0_fl_oe_n(fl_oe_n),
   .de0_fl_ce_n(fl_ce_n),
   .de0_fl_wp_n(fl_wp_n),
   .de0_fl_byte_n(fl_byte_n),
   .de0_fl_rb(fl_rb),

   .de0_lcd_blig(lcd_blig),
   .de0_lcd_rw(lcd_rw),
   .de0_lcd_en(lcd_en),
   .de0_lcd_rs(lcd_rs),
   .de0_lcd_data(lcd_data),

   .de0_sd_dat0(sd_dat0),
   .de0_sd_dat3(sd_dat3),
   .de0_sd_cmd(sd_cmd),
   .de0_sd_clk(sd_clk),
   .de0_sd_wp_n(sd_wp_n),

   .de0_ps2_kbdat(ps2_kbdat),
   .de0_ps2_kbclk(ps2_kbclk),
   .de0_ps2_msdat(ps2_msdat),
   .de0_ps2_msclk(ps2_msclk),

   .de0_vga_hs(vga_hs),
   .de0_vga_vs(vga_vs),
   .de0_vga_r(vga_r),
   .de0_vga_g(vga_g),
   .de0_vga_b(vga_b),

   .de0_gpio0_clkin(gpio0_clkin),
   .de0_gpio0_clkout(gpio0_clkout),
   .de0_gpio0_d(gpio0_d),

   .de0_gpio1_clkin(gpio1_clkin),
   .de0_gpio1_clkout(gpio1_clkout),
   .de0_gpio1_d(gpio1_d)
);

assign   gpio1_d[25] = nSYNC;
assign   gpio1_d[26] = nDIN;
assign   gpio1_d[27] = nDOUT;
assign   gpio1_d[28] = nWTBT;
assign   gpio1_d[29] = nINIT;
assign   gpio1_d[23] = nBSY;
assign   gpio1_clkin[0] = 1;
assign   gpio1_clkin[1] = nIAKO;

assign   nRPLY   = gpio1_d[24];
assign   nVIRQ   = gpio1_d[22];
assign   nIRQ[1] = gpio1_d[17];
assign   nIRQ[2] = gpio1_d[20];
assign   nIRQ[3] = gpio1_d[21];
assign   CLK     = gpio1_clkout[0];
assign   nACLO   = gpio1_clkout[1];
assign   nDCLO   = gpio1_d[14];

assign   gpio1_d[19:18] = nAD[15:14];
assign   gpio1_d[13:0]  = nAD[13:0];
assign   nAD      = AD_oe ? AD_in[15:0] : 16'hZZZZ;

initial
begin
   AD_out   = 0;
   AD_vec   = 0;
   AD_in    = 0;
   AD_oe    = 0;
   nBSY     = 1;
   nIAKO    = 1;
   nDOUT    = 1;
   nDIN     = 1;
   nSYNC    = 1;
   nINIT    = 0;
#(`CONFIG_SIM_CLOCK_HPERIOD*24)
   nINIT    = 1;
#(`CONFIG_SIM_CLOCK_HPERIOD*4)
   nINIT    = 1;

   qbus_read(16'O177716);
   qbus_read(16'O000000);

   qbus_read(16'O177560);
   qbus_write(16'O177560, 16'O000100);

   qbus_read(16'O177564);
   qbus_write(16'O177564, 16'O000101);

   qbus_read(16'O176560);
   qbus_write(16'O176560, 16'O000100);

   qbus_read(16'O176564);
   qbus_write(16'O176564, 16'O000100);

/*
forever
begin
   if (~nVIRQ)
   begin
      qbus_inta();
      case(AD_vec)
         16'O000060:
         begin
            qbus_read(16'O177562);
            $display("RX0: %06O", AD_out);
         end
         16'O000064:
         begin
            qbus_write(16'O177566, sym0);
            $display("TX0: %06O", sym0);
            sym0 = sym0 + 16'O000001;
         end
         16'O000360:
         begin
            qbus_read(16'O176562);
            $display("RX1: %06O", AD_out);
         end
         16'O000364:
         begin
            qbus_write(16'O176566, sym1);
            $display("TX1: %06O", sym1);
            sym1 = sym1 + 16'O000001;
         end
         default:
         begin
            $display("Invalid vector %O", AD_vec);
         end
      endcase
   end
   else
   begin
#(`SIM_CONFIG_CLOCK_HPERIOD*8);
   end
end
*/
end

task qbus_write
(
   input [15:0]  addr,
   input [15:0]  data
);
begin
@ (negedge CLK)
   nBSY  = 1;
   nIAKO = 1;
   nSYNC = 1;
   nDIN  = 1;
   nDOUT = 1;
   AD_oe = 0;

@ (negedge CLK)
   nBSY  = 0;
   AD_oe = 1;
   AD_in = ~addr;
   nWTBT = 0;
@ (posedge CLK)
@ (negedge CLK)
   nSYNC = 0;
@ (posedge CLK)
   nWTBT = 1;
   AD_in = ~data;
   nDOUT = 0;
@ (negedge nRPLY);
@ (posedge CLK);
   nSYNC = 1;
   nDOUT = 1;
@ (posedge nRPLY);
#(`CONFIG_SIM_CLOCK_HPERIOD);
   AD_oe = 0;
   nBSY  = 1;
@ (posedge CLK);
end
endtask

task qbus_read
(
   input [15:0]  addr
);
begin
@ (negedge CLK)
   nBSY  = 1;
   nIAKO = 1;
   nSYNC = 1;
   nDIN  = 1;
   nDOUT = 1;
   AD_oe = 0;
   nWTBT = 1;

@ (negedge CLK)
   nBSY  = 0;
   AD_oe = 1;
   AD_in = ~addr;
@ (posedge CLK)
@ (negedge CLK)
   nSYNC = 0;
@ (posedge CLK)
   AD_oe = 0;
   nDIN  = 0;
@ (negedge nRPLY);
@ (posedge CLK);
   AD_out = ~nAD;
   nSYNC = 1;
   nDIN  = 1;
@ (posedge nRPLY);
#(`CONFIG_SIM_CLOCK_HPERIOD);
   AD_oe = 0;
   nBSY  = 1;
@ (posedge CLK);
end
endtask

task qbus_inta();
begin
   nBSY  = 1;
   nIAKO = 1;
   nSYNC = 1;
   nDIN  = 1;
   nDOUT = 1;
   AD_oe = 0;
#(`CONFIG_SIM_CLOCK_HPERIOD);
   nSYNC = 0;
#(`CONFIG_SIM_CLOCK_HPERIOD);
   nIAKO = 0;
   nDIN  = 0;
@ (negedge nRPLY);
#(`CONFIG_SIM_CLOCK_HPERIOD);
   AD_vec = ~nAD;
   nSYNC = 1;
   nIAKO = 1;
   nDIN  = 1;
@ (posedge nRPLY);
#(`CONFIG_SIM_CLOCK_HPERIOD);
   nBSY  = 1;
end
endtask

//_____________________________________________________________________________
//
endmodule


