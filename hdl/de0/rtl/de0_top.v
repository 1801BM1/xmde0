//
// Copyright (c) 2014-2015 by 1801BM1@gmail.com
//
// Top module for DE0 board based system
//______________________________________________________________________________
//
`include "config.v"

module de0_reset
(
   input    clk,
   input    button,
   input    plock,
   output   dclo,
   output   aclo
);
localparam DCLO_COUNTER_WIDTH = log2(`DE0_DCLO_WIDTH_CLK);
localparam ACLO_COUNTER_WIDTH = log2(`DE0_ACLO_DELAY_CLK);

reg [DCLO_COUNTER_WIDTH-1:0] dclo_cnt;
reg [ACLO_COUNTER_WIDTH-1:0] aclo_cnt;
reg [1:0] reset;
reg aclo_out, dclo_out;

assign dclo = dclo_out;
assign aclo = aclo_out;

always @(posedge clk)
begin
   //
   // Resolve metastability issues
   //
   reset[0] <= button | plock;
   reset[1] <= reset[0];

   if (reset[1])
   begin
      dclo_cnt    <= 0;
      aclo_cnt    <= 0;
      aclo_out    <= 1'b1;
      dclo_out    <= 1'b1;
   end
   else
   begin
      //
      // Count the DCLO pulse
      //
      if (dclo_cnt != `DE0_DCLO_WIDTH_CLK)
         dclo_cnt <= dclo_cnt + 1'b1;
      else
         dclo_out <= 1'b0;

      //
      // After DCLO completion start count the ACLO pulse
      //
      if (~dclo_out)
         if (aclo_cnt != `DE0_ACLO_DELAY_CLK)
            aclo_cnt <= aclo_cnt + 1'b1;
         else
            aclo_out <= 1'b0;
   end
end

function integer log2(input integer value);
   begin
      for (log2=0; value>0; log2=log2+1)
         value = value >> 1;
   end
endfunction
endmodule


//______________________________________________________________________________
//
module de0_birq
(
   input    clk,
   input    ena_ms,
   input    button,
   output   irq
);
reg [4:0] cnt = 5'b00000;
reg out = 1'b0;

assign irq = out;
always @(posedge clk)
begin
   if (button)
      cnt <= 5'b00000;
   else
   if (ena_ms)
      if (cnt != 5'b11111)
         cnt <= cnt + 5'b00001;

   out <= (cnt != 5'b11111);
end
endmodule

//______________________________________________________________________________
//
// Provides the 1 microseconds and 1 milliseconds strobes
//
module de0_timer #(parameter REFCLK=100000000)
(
   input       clk,
   input       rst,
   output reg  ena_us,
   output reg  ena_ms,
   output reg  irq_50
);
localparam US_COUNTER_WIDTH = log2(REFCLK/1000000);
localparam MS_COUNTER_WIDTH = log2(1000);

reg   [US_COUNTER_WIDTH-1:0] count_us;
reg   [MS_COUNTER_WIDTH-1:0] count_ms;
reg   [4:0] count_50;

initial
begin
   count_us = 0;
   count_ms = 0;
   count_50 = 0;
end

always @(posedge clk, posedge rst)
begin
   if (rst)
      begin
         ena_us   <= 1'b0;
         ena_ms   <= 1'b0;
         count_us <= 0;
         count_ms <= 0;
         count_50 <= 0;
         irq_50   <= 1'b0;
      end
   else
      begin
         //
         // One microsecond interval counter
         //
         if (count_us == ((REFCLK/1000000)-1))
            begin
               ena_us <= 1'b1;
               count_us <= 0;
            end
         else
            begin
               ena_us <= 1'b0;
               count_us <= count_us + 1'b1;
            end
         //
         // One millisecond interval counter
         //
         if (ena_us)
            if (count_ms == (1000-1))
               begin
                  ena_ms <= 1'b1;
                  count_ms <= 0;
               end
            else
               begin
                  ena_ms <= 1'b0;
                  count_ms <= count_ms + 1'b1;
               end
         else
            ena_ms <= 1'b0;
         //
         // 50Hz timer output
         //
         if (ena_ms)
            if (count_50 == 19)
            begin
               irq_50 <= 1'b0;
               count_50 <= 0;
            end
            else
            begin
               if (count_50 == 9)
                  irq_50 <= 1'b1;

               count_50 <= count_50 + 1'b1;
            end
      end
end

function integer log2(input integer value);
   begin
      for (log2=0; value>0; log2=log2+1)
         value = value >> 1;
   end
endfunction
endmodule

//______________________________________________________________________________
//
// Vectored interrupt controller
//
module vic_wb #(parameter N=1)
(
   input                wb_clk_i,   // system clock
   input                wb_rst_i,   // peripheral reset
   output reg           wb_irq_o,   // vectored interrupt request
   output reg  [15:0]   wb_dat_o,   // interrupt vector output
   input                wb_stb_i,   // interrupt vector strobe
   output reg           wb_ack_o,   // interrupt vector acknowledgement
                                    //
   input    [N*16-1:0]  ivec,       // interrupt vector values
   input       [N-1:0]  ireq,       // interrupt request lines
   output reg  [N-1:0]  iack        // interrupt acknowledgements
);
localparam W = log2(N);
//
// Contains the current interrupt number with highest priority or 1's if no any
//
reg   [W-1:0] nvec;
integer i;

always @(posedge wb_clk_i or posedge wb_rst_i)
begin
   if (wb_rst_i)
   begin
      wb_ack_o <= 1'b0;
      wb_dat_o <= 16'O000000;
      iack     <= 0;
      nvec     <= {(W){1'b1}};
   end
   else
   begin
      wb_ack_o <= wb_stb_i & wb_irq_o & ~wb_ack_o;
      wb_irq_o <= ~(&nvec) & ~wb_ack_o;

      for (i=N-1; i>=0; i=i-1)
         iack[i] <= (nvec == i) & ireq[i] & wb_stb_i & wb_irq_o & ~iack[i];

      if (wb_stb_i & ~wb_ack_o & wb_irq_o)
         wb_dat_o <= trunc_w16(ivec >> (nvec*16));

      if (~wb_stb_i)
      begin
         nvec <= {(W){1'b1}};
         for (i=N-1; i>=0; i=i-1)
            if (ireq[i]) nvec <= trunc_int(i);
      end
      else
         if (wb_ack_o)
            nvec <= {(W){1'b1}};
   end
end

function integer log2(input integer value);
   for (log2=0; value>0; log2=log2+1)
      value = value >> 1;
endfunction

function [15:0] trunc_w16(input [N*16-1:0] value);
   trunc_w16 = value[15:0];
endfunction

function [W-1:0] trunc_int(input integer value);
   trunc_int = value[W-1:0];
endfunction
endmodule

//______________________________________________________________________________
//
// Initialized RAM block - 4K x 16
//
module mem_wb
(
   input          wb_clk_i,
   input  [15:0]  wb_adr_i,
   input  [15:0]  wb_dat_i,
   output [15:0]  wb_dat_o,
   input          wb_cyc_i,
   input          wb_we_i,
   input  [1:0]   wb_sel_i,
   input          wb_stb_i,
   output         wb_ack_o
);
wire [1:0] ena;
reg [1:0]ack;
//
//de0_ram8k ram(
// .address(wb_adr_i[12:1]),
// .byteena(ena),
// .clock(wb_clk_i),
// .data(wb_dat_i),
// .rden(~wb_we_i & wb_cyc_i & wb_stb_i),
// .wren( wb_we_i & wb_cyc_i & wb_stb_i),
// .q(wb_dat_o));
//
de0_ram16k ram(
   .address(wb_adr_i[13:1]),
   .byteena(ena),
   .clock(wb_clk_i),
   .data(wb_dat_i),
   .rden(~wb_we_i & wb_cyc_i & wb_stb_i),
   .wren( wb_we_i & wb_cyc_i & wb_stb_i),
   .q(wb_dat_o));

assign ena = wb_we_i ? wb_sel_i : 2'b11;
assign wb_ack_o = wb_cyc_i & wb_stb_i & (ack[1] | wb_we_i);
always @ (posedge wb_clk_i)
begin
   ack[0] <= wb_cyc_i & wb_stb_i;
   ack[1] <= wb_cyc_i & ack[0];
end
endmodule

//______________________________________________________________________________
//
// Top project module - instantiates the DE0 board itself
//
module de0
(
   input          de0_clock_50,        // clock input 50 MHz
   input          de0_clock_50_2,      // clock input 50 MHz
                                       //
   input    [2:0] de0_button,          // push button[2:0]
                                       //
   input    [9:0] de0_sw,              // DPDT toggle switch[9:0]
   output   [7:0] de0_hex0,            // seven segment digit 0
   output   [7:0] de0_hex1,            // seven segment digit 1
   output   [7:0] de0_hex2,            // seven segment digit 2
   output   [7:0] de0_hex3,            // seven segment digit 3
   output   [9:0] de0_led,             // LED green[9:0]
                                       //
   output         de0_uart_txd,        // UART transmitter
   input          de0_uart_rxd,        // UART receiver
   output         de0_uart_cts,        // UART clear to send (inverted)
   input          de0_uart_rts,        // UART request to send  (inverted)
                                       //
   inout   [15:0] de0_dram_dq,         // SDRAM data bus 16 bits
   output  [12:0] de0_dram_addr,       // SDRAM address bus 13 bits
   output         de0_dram_ldqm,       // SDRAM low-byte data mask
   output         de0_dram_udqm,       // SDRAM high-byte data mask
   output         de0_dram_we_n,       // SDRAM write enable
   output         de0_dram_cas_n,      // SDRAM column address strobe
   output         de0_dram_ras_n,      // SDRAM row address strobe
   output         de0_dram_cs_n,       // SDRAM chip select
   output   [1:0] de0_dram_ba,         // SDRAM bank address
   output         de0_dram_clk,        // SDRAM clock
   output         de0_dram_cke,        // SDRAM clock enable
                                       //
   inout   [15:0] de0_fl_dq,           // FLASH data bus 15 Bits
   output  [21:0] de0_fl_addr,         // FLASH address bus 22 Bits
   output         de0_fl_we_n,         // FLASH write enable
   output         de0_fl_rst_n,        // FLASH reset
   output         de0_fl_oe_n,         // FLASH output enable
   output         de0_fl_ce_n,         // FLASH chip enable
   output         de0_fl_wp_n,         // FLASH hardware write protect
   output         de0_fl_byte_n,       // FLASH selects 8/16-bit mode
   input          de0_fl_rb,           // FLASH ready/busy
                                       //
   output         de0_lcd_blig,        // LCD back light ON/OFF
   output         de0_lcd_rw,          // LCD read/write select, 0 = write, 1 = read
   output         de0_lcd_en,          // LCD enable
   output         de0_lcd_rs,          // LCD command/data select, 0 = command, 1 = data
   inout    [7:0] de0_lcd_data,        // LCD data bus 8 bits
                                       //
   inout          de0_sd_dat0,         // SD Card Data 0
   inout          de0_sd_dat3,         // SD Card Data 3
   inout          de0_sd_cmd,          // SD Card Command Signal
   output         de0_sd_clk,          // SD Card Clock
   input          de0_sd_wp_n,         // SD Card Write Protect
                                       //
   inout          de0_ps2_kbdat,       // PS2 Keyboard Data
   inout          de0_ps2_kbclk,       // PS2 Keyboard Clock
   inout          de0_ps2_msdat,       // PS2 Mouse Data
   inout          de0_ps2_msclk,       // PS2 Mouse Clock
                                       //
   output         de0_vga_hs,          // VGA H_SYNC
   output         de0_vga_vs,          // VGA V_SYNC
   output   [3:0] de0_vga_r,           // VGA Red[3:0]
   output   [3:0] de0_vga_g,           // VGA Green[3:0]
   output   [3:0] de0_vga_b,           // VGA Blue[3:0]
                                       //
   input    [1:0] de0_gpio0_clkin,     // GPIO Connection 0 Clock In Bus
   output   [1:0] de0_gpio0_clkout,    // GPIO Connection 0 Clock Out Bus
   inout   [31:0] de0_gpio0_d,         // GPIO Connection 0 Data Bus
                                       //
   input    [1:0] de0_gpio1_clkin,     // GPIO Connection 1 Clock In Bus
   output   [1:0] de0_gpio1_clkout,    // GPIO Connection 1 Clock Out Bus
   inout   [31:0] de0_gpio1_d          // GPIO Connection 1 Data Bus
);

//______________________________________________________________________________
//
wire        clk50;                     // 50 MHz clock source entry
wire        sys_clk_p;                 // system positive clock (all buses)
wire        sys_clk_n;                 // system negative clock (180 phase shift)
wire        sys_init;                  // peripheral reset
wire        sys_plock;                 //
wire        ena_us, ena_ms;            //
wire        i50Hz, iBut;               //
                                       //
wire        wb_clk;                    //
wire [15:0] wb_adr;                    // master address out bus
wire [15:0] wb_out;                    // master data out bus
wire [15:0] wb_mux;                    // master data in bus
wire        wb_cyc;                    // master wishbone cycle
wire        wb_we;                     // master wishbone direction
wire [1:0]  wb_sel;                    // master wishbone byte election
wire        wb_stb;                    // master wishbone strobe
wire        wb_ack;                    // master wishbone acknowledgement
                                       //
wire        vm_istb;                   //
wire        vm_iack;                   //
wire [15:0] vm_ivec;                   //
wire [2:0]  mx_stb;                    //
wire [2:0]  mx_ack;                    // system wishbone data mux
wire [15:0] mx_dat[2:0];               //
                                       //
wire [2:1]  vm_sel;                    //
wire [15:0] vm_in14;                   //
reg  [15:0] vm_reg0, vm_reg1;          //
                                       //
wire        vm_init_out;               //
wire        vm_dclo_in;                //
wire        vm_aclo_in;                //
wire        vm_virq;                   //
wire [3:1]  vm_irq;                    //
                                       //
wire        tx_irq, tx_ack;            //
wire        rx_irq, rx_ack;            //
wire [31:0] baud;                      //
                                       //
reg [15:0]  st_ad;                     // SignalTap II section
reg         st_qclk;                   //
reg         st_din, st_dout;           //
reg         st_sync, st_wtbt;          //
reg         st_aclo, st_dclo;          //
reg         st_iako, st_rply;          //
reg         st_led, st_ena;            //
reg         st_irq, st_sel, st_ar;     //
wire        st_clk;                    //
                                       //
`ifdef _CONFIG_VM3_                    //
reg         st_ta, st_umap, st_hltm;   //
`endif                                 //

reg tin;
//______________________________________________________________________________
//
assign st_clk = wb_clk;

always @(posedge st_clk)
begin
`ifdef _CONFIG_VM1_
   st_ad    <= ~{de0_gpio1_d[19:18], de0_gpio1_d[13:0]};
   st_din   <= de0_gpio1_d[26];
   st_dout  <= de0_gpio1_d[27];
   st_sync  <= de0_gpio1_d[25];
   st_wtbt  <= de0_gpio1_d[28];
   st_aclo  <= de0_gpio1_clkout[1];
   st_dclo  <= de0_gpio1_d[14];
   st_iako  <= de0_gpio1_clkin[1];
   st_rply  <= de0_gpio1_d[24];
   st_ar    <= de0_gpio1_d[23]; // true reply from CPU pin
   st_sel   <= 1'b1;
`endif

`ifdef _CONFIG_VM2_
   st_ad[0]    <= ~de0_gpio1_d[18];
   st_ad[1]    <= ~de0_gpio1_d[12];
   st_ad[2]    <= ~de0_gpio1_d[10];
   st_ad[3]    <= ~de0_gpio1_d[8];
   st_ad[4]    <= ~de0_gpio1_d[6];
   st_ad[5]    <= ~de0_gpio1_d[4];
   st_ad[6]    <= ~de0_gpio1_d[2];
   st_ad[7]    <= ~de0_gpio1_d[0];
   st_ad[8]    <= ~de0_gpio1_d[1];
   st_ad[9]    <= ~de0_gpio1_d[3];
   st_ad[10]   <= ~de0_gpio1_d[5];
   st_ad[11]   <= ~de0_gpio1_d[7];
   st_ad[12]   <= ~de0_gpio1_d[9];
   st_ad[13]   <= ~de0_gpio1_d[11];
   st_ad[14]   <= ~de0_gpio1_d[13];
   st_ad[15]   <= ~de0_gpio1_d[19];

   st_din   <= de0_gpio1_d[26];
   st_dout  <= de0_gpio1_d[27];
   st_sync  <= de0_gpio1_d[25];
   st_wtbt  <= de0_gpio1_d[28];
   st_aclo  <= de0_gpio1_clkout[1];
   st_dclo  <= de0_gpio1_d[14];
   st_iako  <= de0_gpio1_clkin[1];
   st_rply  <= de0_gpio1_d[24];
   st_sel   <= de0_gpio1_d[21];
   st_ar    <= de0_gpio1_d[23];
`endif

`ifdef _CONFIG_VM3_
   st_ad[0]    <= ~de0_gpio1_d[19];
   st_ad[1]    <= ~de0_gpio1_d[18];
   st_ad[2]    <= ~de0_gpio1_d[13];
   st_ad[3]    <= ~de0_gpio1_d[12];
   st_ad[4]    <= ~de0_gpio1_d[11];
   st_ad[5]    <= ~de0_gpio1_d[10];
   st_ad[6]    <= ~de0_gpio1_d[9];
   st_ad[7]    <= ~de0_gpio1_d[8];
   st_ad[8]    <= ~de0_gpio1_d[7];
   st_ad[9]    <= ~de0_gpio1_d[6];
   st_ad[10]   <= ~de0_gpio1_d[5];
   st_ad[11]   <= ~de0_gpio1_d[4];
   st_ad[12]   <= ~de0_gpio1_d[3];
   st_ad[13]   <= ~de0_gpio1_d[2];
   st_ad[14]   <= ~de0_gpio1_d[1];
   st_ad[15]   <= ~de0_gpio1_d[0];

   st_ta       <= de0_gpio1_d[14];
   st_sel      <= de0_gpio1_d[15];
   st_hltm     <= de0_gpio1_d[16];
   st_umap     <= de0_gpio1_d[17];

   st_rply  <= de0_gpio1_d[24];
   st_sync  <= de0_gpio1_d[25];
   st_din   <= de0_gpio1_d[26];
   st_dout  <= de0_gpio1_d[27];
   st_wtbt  <= de0_gpio1_d[28];
   st_iako  <= de0_gpio1_clkin[1];

   st_aclo  <= ~vm_aclo_in;
   st_dclo  <= ~vm_dclo_in;
   st_ar    <= st_ta & st_hltm & st_umap;
`endif

   st_qclk  <= de0_gpio1_clkout[0];
   st_ena   <= de0_gpio1_d[31];
   st_irq   <= ~vm_irq[2]; // ~vm_virq;
   st_led   <= &st_ad & st_qclk & st_din & st_dout & st_sync & st_wtbt & st_ar & st_clk
              & st_aclo & st_dclo & st_iako & st_rply & st_ena & st_irq & st_sel;
end
//______________________________________________________________________________
//
assign      sys_init = vm_init_out;
assign      baud = 921600/115200-1;

assign      vm_irq[1] = 1'b0;
assign      vm_irq[2] = de0_sw[0] ? i50Hz : iBut;
assign      vm_irq[3] = 1'b0;

de0_reset reset
(
   .clk(clk50),
   .button(~de0_button[2]),
   .plock(~sys_plock),
   .dclo(vm_dclo_in),
   .aclo(vm_aclo_in)
);

//______________________________________________________________________________
//
// Clock section
//
assign wb_clk  = sys_clk_p;
assign clk50   = de0_clock_50_2;

de0_pll100 corepll
//de0_pll_zbd corepll
(
   .inclk0(clk50),
   .c0(sys_clk_p),
   .c1(sys_clk_n),
   .locked(sys_plock)
);

//______________________________________________________________________________
//
`ifdef CONFIG_SYS_CLOCK
defparam timer.REFCLK = `CONFIG_SYS_CLOCK;
`endif

de0_timer timer
(
   .clk(wb_clk),
   .rst(vm_dclo_in),
   .ena_us(ena_us),
   .ena_ms(ena_ms),
   .irq_50(i50Hz)
);

de0_birq birq
(
   .clk(wb_clk),
   .ena_ms(ena_ms),
   .button(~de0_button[1]),
   .irq(iBut)
);

//______________________________________________________________________________
//
// CPU instantiation
//
`ifdef _CONFIG_VM1_
vm1_mod cpu
(
   .vm_clk_p(sys_clk_p),               // positive processor clock
   .vm_clk_n(sys_clk_n),               // negative processor clock
   .vm_clk_slow(1'b0),                 // slow clock sim mode
   .vm_clk_ena(1'b1),                  // slow clock strobe
   .vm_clk_tve(1'b1),                  // VE-timer clock enable
   .vm_clk_sp(1'b0),                   // external pin SP clock
                                       //
   .vm_pa(2'b00),                      // processor number
   .vm_init_in(1'b0),                  // peripheral reset
   .vm_init_out(vm_init_out),          // peripheral reset
   .vm_dclo(vm_dclo_in),               // processor reset
   .vm_aclo(vm_aclo_in),               // power fail notificaton
   .vm_irq(vm_irq),                    // radial interrupt requests
   .vm_virq(vm_virq),                  // vectored interrupt request
                                       //
   .wbm_gnt_i(1'b1),                   // master wishbone granted
   .wbm_adr_o(wb_adr),                 // master wishbone address
   .wbm_dat_o(wb_out),                 // master wishbone data output
   .wbm_dat_i(wb_mux),                 // master wishbone data input
   .wbm_cyc_o(wb_cyc),                 // master wishbone cycle
   .wbm_we_o(wb_we),                   // master wishbone direction
   .wbm_sel_o(wb_sel),                 // master wishbone byte election
   .wbm_stb_o(wb_stb),                 // master wishbone strobe
   .wbm_ack_i(wb_ack),                 // master wishbone acknowledgement
                                       //
   .wbi_dat_i(vm_ivec),                // interrupt vector input
   .wbi_stb_o(vm_istb),                // interrupt vector strobe
   .wbi_ack_i(vm_iack),                // interrupt vector acknowledgement
                                       //
   .wbs_adr_i(wb_adr[3:0]),            // slave wishbone address
   .wbs_dat_i(wb_out),                 // slave wishbone data input
   .wbs_cyc_i(wb_cyc),                 // slave wishbone cycle
   .wbs_we_i(wb_we),                   // slave wishbone direction
   .wbs_stb_i(mx_stb[0]),              // slave wishbone strobe
   .wbs_ack_o(mx_ack[0]),              // slave wishbone acknowledgement
   .wbs_dat_o(mx_dat[0]),              // slave wishbone data output
                                       //
   .vm_reg14(vm_in14),                 // register 177714 data input
   .vm_reg16(16'o000000),              // register 177716 data input
   .vm_sel(vm_sel),                    // register select outputs
                                       //
   .vm_gpio(de0_gpio1_d),              //
   .vm_gpio_in(de0_gpio1_clkin),       //
   .vm_gpio_out(de0_gpio1_clkout)      //
);
`endif

`ifdef _CONFIG_VM2_
vm2_mod cpu
(
   .vm_clk_p(sys_clk_p),               // positive processor clock
   .vm_clk_n(sys_clk_n),               // negative processor clock
   .vm_clk_slow(1'b0),                 // slow clock sim mode
   .vm_clk_ena(1'b1),                  // slow clock strobe
   .vm_clk_tve(1'b1),                  // VE-timer clock enable
   .vm_clk_sp(1'b0),                   // external pin SP clock
                                       //
   .vm_pa(2'b00),                      // processor number
   .vm_init_in(1'b0),                  // peripheral reset
   .vm_init_out(vm_init_out),          // peripheral reset
   .vm_dclo(vm_dclo_in),               // processor reset
   .vm_aclo(vm_aclo_in),               // power fail notificaton
   .vm_irq(vm_irq),                    // radial interrupt requests
   .vm_virq(vm_virq),                  // vectored interrupt request
                                       //
   .wbm_gnt_i(1'b1),                   // master wishbone granted
   .wbm_adr_o(wb_adr),                 // master wishbone address
   .wbm_dat_o(wb_out),                 // master wishbone data output
   .wbm_dat_i(wb_mux),                 // master wishbone data input
   .wbm_cyc_o(wb_cyc),                 // master wishbone cycle
   .wbm_we_o(wb_we),                   // master wishbone direction
   .wbm_sel_o(wb_sel),                 // master wishbone byte election
   .wbm_stb_o(wb_stb),                 // master wishbone strobe
   .wbm_ack_i(wb_ack),                 // master wishbone acknowledgement
                                       //
   .wbi_dat_i(vm_ivec),                // interrupt vector input
   .wbi_stb_o(vm_istb),                // interrupt vector strobe
   .wbi_ack_i(vm_iack),                // interrupt vector acknowledgement
                                       //
   .wbs_adr_i(wb_adr[3:0]),            // slave wishbone address
   .wbs_dat_i(wb_out),                 // slave wishbone data input
   .wbs_cyc_i(wb_cyc),                 // slave wishbone cycle
   .wbs_we_i(wb_we),                   // slave wishbone direction
   .wbs_stb_i(mx_stb[0]),              // slave wishbone strobe
   .wbs_ack_o(mx_ack[0]),              // slave wishbone acknowledgement
   .wbs_dat_o(mx_dat[0]),              // slave wishbone data output
                                       //
   .vm_reg14(vm_in14),                 // register 177714 data input
   .vm_reg16(16'o000000),              // register 177716 data input
   .vm_sel(vm_sel),                    // register select outputs
                                       //
   .vm_gpio(de0_gpio1_d),              //
   .vm_gpio_in(de0_gpio1_clkin),       //
   .vm_gpio_out(de0_gpio1_clkout)      //
);
`endif

`ifdef _CONFIG_VM3_
vm3_mod cpu
(
   .vm_clk_p(sys_clk_p),               // positive processor clock
   .vm_clk_n(sys_clk_n),               // negative processor clock
   .vm_clk_slow(1'b0),                 // slow clock sim mode
   .vm_clk_ena(1'b1),                  // slow clock strobe
   .vm_clk_tve(1'b1),                  // VE-timer clock enable
   .vm_clk_sp(1'b0),                   // external pin SP clock
                                       //
   .vm_pa(2'b00),                      // processor number
   .vm_init_in(1'b0),                  // peripheral reset
   .vm_init_out(vm_init_out),          // peripheral reset
   .vm_dclo(vm_dclo_in),               // processor reset
   .vm_aclo(vm_aclo_in),               // power fail notificaton
   .vm_irq(vm_irq),                    // radial interrupt requests
   .vm_virq(vm_virq),                  // vectored interrupt request
                                       //
   .wbm_gnt_i(1'b1),                   // master wishbone granted
   .wbm_adr_o(wb_adr),                 // master wishbone address
   .wbm_dat_o(wb_out),                 // master wishbone data output
   .wbm_dat_i(wb_mux),                 // master wishbone data input
   .wbm_cyc_o(wb_cyc),                 // master wishbone cycle
   .wbm_we_o(wb_we),                   // master wishbone direction
   .wbm_sel_o(wb_sel),                 // master wishbone byte election
   .wbm_stb_o(wb_stb),                 // master wishbone strobe
   .wbm_ack_i(wb_ack),                 // master wishbone acknowledgement
                                       //
   .wbi_dat_i(vm_ivec),                // interrupt vector input
   .wbi_stb_o(vm_istb),                // interrupt vector strobe
   .wbi_ack_i(vm_iack),                // interrupt vector acknowledgement
                                       //
   .wbs_adr_i(wb_adr[3:0]),            // slave wishbone address
   .wbs_dat_i(wb_out),                 // slave wishbone data input
   .wbs_cyc_i(wb_cyc),                 // slave wishbone cycle
   .wbs_we_i(wb_we),                   // slave wishbone direction
   .wbs_stb_i(mx_stb[0]),              // slave wishbone strobe
   .wbs_ack_o(mx_ack[0]),              // slave wishbone acknowledgement
   .wbs_dat_o(mx_dat[0]),              // slave wishbone data output
                                       //
   .vm_reg14(vm_in14),                 // register 177714 data input
   .vm_reg16(16'o000000),              // register 177716 data input
   .vm_sel(vm_sel),                    // register select outputs
                                       //
   .vm_gpio(de0_gpio1_d),              //
   .vm_gpio_in(de0_gpio1_clkin),       //
   .vm_gpio_out(de0_gpio1_clkout)      //
);
`endif

//______________________________________________________________________________
//
mem_wb mem(
   .wb_clk_i(wb_clk),
   .wb_adr_i(wb_adr),
   .wb_dat_i(wb_out),
   .wb_dat_o(mx_dat[1]),
   .wb_cyc_i(wb_cyc),
   .wb_we_i(wb_we),
   .wb_sel_i(wb_sel),
   .wb_stb_i(mx_stb[1]),
   .wb_ack_o(mx_ack[1])
);

//______________________________________________________________________________
//
`ifdef CONFIG_SYS_CLOCK
defparam uart.REFCLK = `CONFIG_SYS_CLOCK;
`endif

uart_wb uart
(
   .wb_clk_i(wb_clk),
   .wb_rst_i(sys_init),
   .wb_adr_i(wb_adr[2:0]),
   .wb_dat_i(wb_out),
   .wb_dat_o(mx_dat[2]),
   .wb_cyc_i(wb_cyc),
   .wb_we_i(wb_we),
   .wb_stb_i(mx_stb[2]),
   .wb_ack_o(mx_ack[2]),

   .tx_dat_o(de0_uart_txd),
   .tx_cts_i(de0_uart_rts),
   .rx_dat_i(de0_uart_rxd),
   .rx_dtr_o(de0_uart_cts),

   .tx_irq_o(tx_irq),
   .tx_ack_i(tx_ack),
   .rx_irq_o(rx_irq),
   .rx_ack_i(rx_ack),

   .cfg_bdiv(baud[15:0]),
   .cfg_nbit(2'b11),
   .cfg_nstp(1'b1),
   .cfg_pena(1'b0),
   .cfg_podd(1'b0)
);

vic_wb #(.N(2)) vic
(
   .wb_clk_i(wb_clk),
   .wb_rst_i(vm_dclo_in),
   .wb_irq_o(vm_virq),
   .wb_dat_o(vm_ivec),
   .wb_stb_i(vm_istb),
   .wb_ack_o(vm_iack),
   .ivec({16'o000064, 16'o000060}),
   .ireq({tx_irq, rx_irq}),
   .iack({tx_ack, rx_ack})
);

//______________________________________________________________________________
//
assign mx_stb[0]  = wb_stb & wb_cyc & (wb_adr[15:4] == (16'o177700 >> 4));
assign mx_stb[1]  = wb_stb & wb_cyc & (wb_adr[15:14] == 2'o0);
assign mx_stb[2]  = wb_stb & wb_cyc & (wb_adr[15:3] == (16'o177560 >> 3));

assign wb_ack     = mx_ack[0] | mx_ack[1] | mx_ack[2];
assign wb_mux     = (mx_stb[0] ? mx_dat[0] : 16'o000000)
                  | (mx_stb[1] ? mx_dat[1] : 16'o000000)
                  | (mx_stb[2] ? mx_dat[2] : 16'o000000);
//______________________________________________________________________________
//
//
// Simulation stop flag and console
//
assign de0_lcd_rs    = wb_stb & wb_cyc & ((wb_adr == 16'o177676) | (wb_adr == 16'o177674));
assign de0_lcd_data  = wb_out[7:0];
assign de0_lcd_en    = (wb_adr == 16'o177566) & wb_stb & wb_we & wb_ack;

//______________________________________________________________________________
//
// 7-segment display registers and switches
//
assign de0_hex0      = ~vm_reg0[7:0];
assign de0_hex1      = ~vm_reg0[15:8];
assign de0_hex2      = ~vm_reg1[7:0];
assign de0_hex3      = ~vm_reg1[15:8];

always @(posedge wb_clk)
begin
   if (sys_init)
   begin
      vm_reg0 <= 16'o000000;
      vm_reg1 <= 16'o000000;
   end
   else
   begin
`ifdef _CONFIG_VM3_
      if (vm_sel[2] & wb_we) vm_reg0 <= wb_out;
      if (vm_sel[1] & wb_we) vm_reg1 <= wb_out;
`else
      if (vm_sel[2] & wb_we & ~wb_adr[0]) vm_reg0 <= wb_out;
      if (vm_sel[2] & wb_we &  wb_adr[0]) vm_reg1 <= wb_out;
`endif
   end
end

assign vm_in14[2:0]     = de0_button;
assign vm_in14[12:3]    = de0_sw;
assign vm_in14[15:13]   = de0_button;

//______________________________________________________________________________
//
// Temporary and debug assignments
//
/*
initial de0_dram_we_n = 1'b0;
always @(posedge sys_clk_p)
begin
   de0_dram_we_n  <= ~de0_dram_we_n;
   tin <= de0_gpio0_d[0];
end
assign   de0_dram_udqm  = ~sys_clk_p;
*/
assign   de0_dram_dq    = 16'hzzzz;
assign   de0_dram_addr  = 13'h0000;
assign   de0_dram_ldqm  = 1'b0;
assign   de0_dram_udqm  = 1'b0;
assign   de0_dram_we_n  = 1'b1;
assign   de0_dram_cas_n = 1'b1;
assign   de0_dram_ras_n = 1'b1;
assign   de0_dram_cs_n  = 1'b1;
assign   de0_dram_ba[0] = 1'b0;
assign   de0_dram_ba[1] = 1'b0;
assign   de0_dram_clk   = wb_clk;
assign   de0_dram_cke   = 1'b1;

assign   de0_fl_dq      = 16'hzzzz;
assign   de0_fl_addr    = 22'hzzzzzz;
assign   de0_fl_we_n    = 1'b1;
assign   de0_fl_rst_n   = 1'b0;
assign   de0_fl_oe_n    = 1'b1;
assign   de0_fl_ce_n    = 1'b1;
assign   de0_fl_wp_n    = 1'b0;
assign   de0_fl_byte_n  = 1'b1;

// assign   de0_lcd_data   = 8'hzz;
assign   de0_lcd_blig   = 1'b0;
assign   de0_lcd_rw     = 1'b0;
// assign   de0_lcd_en     = 1'b0;
// assign   de0_lcd_rs     = 1'b0;

assign   de0_sd_clk     = 1'b0;
assign   de0_sd_dat0    = 1'hz;
assign   de0_sd_dat3    = 1'hz;
assign   de0_sd_cmd     = 1'hz;

assign   de0_ps2_kbdat  = 1'hz;
assign   de0_ps2_kbclk  = 1'hz;
assign   de0_ps2_msdat  = 1'hz;
assign   de0_ps2_msclk  = 1'hz;

assign   de0_vga_hs     = 1'b0;
assign   de0_vga_vs     = 1'b0;
assign   de0_vga_r      = 4'h0;
assign   de0_vga_g      = 4'h0;
assign   de0_vga_b      = 4'h0;

// assign de0_hex0      = 8'hzz;
// assign de0_hex1      = 8'hzz;
// assign de0_hex2      = 8'hzz;
// assign de0_hex3      = 8'hzz;
assign de0_led[8:1]     = 7'hzz;
assign de0_led[0]       = de0_sw[0];
assign de0_led[9]       = st_led;

assign   de0_gpio0_clkout  = 2'b00;
assign   de0_gpio0_d       = 32'hzzzzzzzz;

//
// assign   de0_gpio1_clkout  = 2'b00;
// assign   de0_gpio1_d       = 32'hzzzzzzzz;
//
//______________________________________________________________________________
//
endmodule
