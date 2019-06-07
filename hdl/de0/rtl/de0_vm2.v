//
// Copyright (c) 2014-2015 by 1801BM1@gmail.com
//______________________________________________________________________________
//
// Real VM2 module thunk from Wishbone compatible version of 1801VM2 processor
//
`include "config.v"

module vm2_mod
(
   //
   // Processor core clock section:
   //    - vm_clk_p     - processor core positive clock, also feeds the wishbone buses
   //    - vm_clk_n     - processor core negative clock, should be vm_clk_p 180 degree phase shifted
   //    - vm_clk_ena   - slow clock simulation strobe, enables clock at vm_clk_p
   //    - vm_clk_tve   - VE-timer clock enable strobe, enables clock at vm_clk_p
   //    - vm_clk_slow  - clock mode selector, enables clock slowdown simulation,
   //                     the external I/O cycles is launched with rate of vm_clk_ena
   //
   input          vm_clk_p,         // positive clock
   input          vm_clk_n,         // negative clock
   input          vm_clk_ena,       // slow clock enable
   input          vm_clk_tve,       // VE-timer clock enable
   input          vm_clk_sp,        // external pin SP clock
   input          vm_clk_slow,      // slow clock sim mode
                                    //
   input  [1:0]   vm_pa,            // processor number
   input          vm_init_in,       // peripheral reset input
   output         vm_init_out,      // peripheral reset output
                                    //
   input          vm_dclo,          // processor reset
   input          vm_aclo,          // power fail notificaton
   input  [3:1]   vm_irq,           // radial interrupt requests
   input          vm_virq,          // vectored interrupt request
                                    //
   input          wbm_gnt_i,        // master wishbone granted
   output [15:0]  wbm_adr_o,        // master wishbone address
   output [15:0]  wbm_dat_o,        // master wishbone data output
   input  [15:0]  wbm_dat_i,        // master wishbone data input
   output         wbm_cyc_o,        // master wishbone cycle
   output         wbm_we_o,         // master wishbone direction
   output [1:0]   wbm_sel_o,        // master wishbone byte election
   output         wbm_stb_o,        // master wishbone strobe
   input          wbm_ack_i,        // master wishbone acknowledgement
                                    //
   input  [15:0]  wbi_dat_i,        // interrupt vector input
   output         wbi_stb_o,        // interrupt vector strobe
   input          wbi_ack_i,        // interrupt vector acknowledgement
                                    //
   input  [3:0]   wbs_adr_i,        // slave wishbone address
   input  [15:0]  wbs_dat_i,        // slave wishbone data input
   output [15:0]  wbs_dat_o,        // slave wishbone data output
   input          wbs_cyc_i,        // slave wishbone cycle
   input          wbs_we_i,         // slave wishbone direction
   input          wbs_stb_i,        // slave wishbone strobe
   output         wbs_ack_o,        // slave wishbone acknowledgement
                                    //
   input  [15:0]  vm_reg14,         // register 177714 data
   input  [15:0]  vm_reg16,         // register 177716 data
   output [2:1]   vm_sel,           // register select outputs
                                    //
   inout  [31:0]  vm_gpio,          // external GPIO
   inout  [1:0]   vm_gpio_in,       //
   output [1:0]   vm_gpio_out       //
);

//______________________________________________________________________________
//
reg   [31:20]  vm_gpio_t0;          //
reg   [31:20]  vm_gpio_t1;          //
wire  [15:0]   vm_data;             //
                                    //
reg   [7:0]    qbus_div;            //
reg            qbus_clk;            //
reg            qbus_rise;           //
reg            qbus_fall;           //
reg   [2:0]    qbus_irq;            //
reg            qbus_rply;           //
reg            qbus_aclo;           //
reg            qbus_dclo;           //
wire           qbus_ar;             //
                                    //
reg            rply;                //
reg   [15:0]   addr;                //
reg   [15:0]   data_in;             //
reg   [15:0]   data_out;            //
                                    //
wire           din, dout, wtbt;     //
wire           sync, iako, init;    //
wire           dmgo, sel;           //
reg   [2:0]    wedge;               //
reg            redge, iedge;        //
reg            ioe, doe, roe;       //
                                    //
reg   [15:0]   wbs_d;               // slave wishbone data register
reg            wbs_r, wbs_w;        //
reg            wbs_a;               //
wire           wbs_a_rc;            //
                                    //
reg            wb_we, wb_cyc;       //
reg            wb_stb, wbi_stb;     //
reg   [1:0]    wb_sel;              //
reg   [1:0]    wlock;               //
//______________________________________________________________________________
//
// Clock prescaler
//
initial
begin
   qbus_div = 8'h00;
   rply     = 1'b0;

   wb_cyc   = 1'b0;
   wb_we    = 1'b0;
   wb_sel   = 2'b00;
   wb_stb   = 1'b0;
   wbi_stb  = 1'b0;

   ioe      = 1'b0;
   doe      = 1'b0;
   roe      = 1'b0;
   wlock    = 2'b00;
end
//
// vm_clk_n       - ignored
// vm_clk_ena     - ignored
// vm_clk_tve     - ignored
// vm_clk_sp      - ignored
// vm_clk_slow    - ignored
// vm_pa          - ignored
// vm_init_in     - ignored
//
always @(posedge vm_clk_p)
begin
   if (qbus_div == `VM2_CLOCK_LOW)
   begin
      qbus_div  <= 8'h00;
      qbus_clk  <= 1'b0;
   end
   else
   begin
      if (qbus_div == `VM2_CLOCK_HIGH)
         qbus_clk  <= 1'b1;

      qbus_div  <= qbus_div + 8'h01;
   end

   qbus_rise <= (qbus_div == `VM2_CLOCK_HIGH);
   qbus_fall <= (qbus_div == `VM2_CLOCK_LOW);
end

//
// Input latches
//
always @(posedge vm_clk_p)
begin
   //
   // Resolve metastability issues
   //
   vm_gpio_t0 <= {vm_gpio_in, vm_gpio[29:20]};
   vm_gpio_t1 <= vm_gpio_t0;
end

assign sel  = ~vm_gpio_t1[21];
assign sync = ~vm_gpio_t1[25];
assign din  = ~vm_gpio_t1[26];
assign dout = ~vm_gpio_t1[27];
assign wtbt = ~vm_gpio_t1[28];
assign init = ~vm_gpio_t1[29];
assign dmgo = ~vm_gpio_t1[30];
assign iako = ~vm_gpio_t1[31];

assign vm_gpio_out[0]   = qbus_clk;
assign vm_gpio_out[1]   = ~qbus_aclo;
assign vm_gpio[14]      = ~qbus_dclo;
assign vm_gpio[15]      = 1'b1;           // sack
assign vm_gpio[16]      = 1'b1;           // dmr
assign vm_gpio[17]      = ~qbus_irq[2];   // evnt
assign vm_gpio[20]      = ~qbus_irq[1];   // halt
assign vm_gpio[22]      = ~qbus_irq[0];   // virq
assign vm_gpio[23]      = ~qbus_ar;       // ar
assign vm_gpio[24]      = ~rply;          // ~qbus_rply;
assign qbus_ar          = sync & ~qbus_rply & ~din & ~dout;

assign vm_gpio[30]      = 1'b0;  // nEXENA
assign vm_gpio[31]      = ~ioe;  // nEXDIN

assign vm_gpio[0]       = (ioe & doe) ? ~data_in[7]  : 1'bZ;
assign vm_gpio[1]       = (ioe & doe) ? ~data_in[8]  : 1'bZ;
assign vm_gpio[2]       = (ioe & doe) ? ~data_in[6]  : 1'bZ;
assign vm_gpio[3]       = (ioe & doe) ? ~data_in[9]  : 1'bZ;
assign vm_gpio[4]       = (ioe & doe) ? ~data_in[5]  : 1'bZ;
assign vm_gpio[5]       = (ioe & doe) ? ~data_in[10] : 1'bZ;
assign vm_gpio[6]       = (ioe & doe) ? ~data_in[4]  : 1'bZ;
assign vm_gpio[7]       = (ioe & doe) ? ~data_in[11] : 1'bZ;
assign vm_gpio[8]       = (ioe & doe) ? ~data_in[3]  : 1'bZ;
assign vm_gpio[9]       = (ioe & doe) ? ~data_in[12] : 1'bZ;
assign vm_gpio[10]      = (ioe & doe) ? ~data_in[2]  : 1'bZ;
assign vm_gpio[11]      = (ioe & doe) ? ~data_in[13] : 1'bZ;
assign vm_gpio[12]      = (ioe & doe) ? ~data_in[1]  : 1'bZ;
assign vm_gpio[13]      = (ioe & doe) ? ~data_in[14] : 1'bZ;
assign vm_gpio[18]      = (ioe & doe) ? ~data_in[0]  : 1'bZ;
assign vm_gpio[19]      = (ioe & doe) ? ~data_in[15] : 1'bZ;

assign vm_data[0]       = vm_gpio[18];
assign vm_data[1]       = vm_gpio[12];
assign vm_data[2]       = vm_gpio[10];
assign vm_data[3]       = vm_gpio[8];
assign vm_data[4]       = vm_gpio[6];
assign vm_data[5]       = vm_gpio[4];
assign vm_data[6]       = vm_gpio[2];
assign vm_data[7]       = vm_gpio[0];
assign vm_data[8]       = vm_gpio[1];
assign vm_data[9]       = vm_gpio[3];
assign vm_data[10]      = vm_gpio[5];
assign vm_data[11]      = vm_gpio[7];
assign vm_data[12]      = vm_gpio[9];
assign vm_data[13]      = vm_gpio[11];
assign vm_data[14]      = vm_gpio[13];
assign vm_data[15]      = vm_gpio[19];

always @(posedge vm_clk_p)
begin
   roe <= ioe & ~din;
   doe <= ioe & din;
   if ((din & rply) | (din & ~sync & sel))
      ioe <= 1'b1;
   else
      if (roe)
         ioe <= 1'b0;
end
//
// Outputs synchronization at CLK transition
//
always @(posedge vm_clk_p)
begin
   if (qbus_rise)
   begin
      qbus_aclo <= vm_aclo;
      qbus_dclo <= vm_dclo;
      qbus_irq  <= {vm_irq[2:1], vm_virq};
   end

   if (qbus_fall)
   begin
      qbus_rply <= rply;
   end
end

always @(posedge vm_clk_p)
begin
   wedge[0] <= dout & sync;
   wedge[1] <= wedge[0];
   wedge[2] <= wedge[1];

   redge    <= din & sync;
   iedge    <= din & iako;

   if (~sync) addr <= ~vm_data;
   if (~wedge[2] & wedge[1]) data_out <= ~vm_data;

   if (wbm_ack_i & sync & (din | dout))
      rply <= 1'b1;
   else
      if (wbi_ack_i & iako & din)
         rply <= 1'b1;
      else
         if (~din & ~dout)
            rply <= 1'b0;
end

assign vm_init_out      = init;
assign wbm_adr_o        = addr;
assign wbm_dat_o        = data_out;
assign wbm_cyc_o        = wb_cyc;
assign wbm_we_o         = wb_we;
assign wbm_sel_o        = wb_sel;
assign wbm_stb_o        = wb_stb;
assign wbi_stb_o        = wbi_stb;

always @(posedge vm_clk_p)
begin
   if (vm_aclo & vm_dclo) wlock[0] <= 1'b1;

   if (wbm_stb_o & wbm_ack_i) data_in <= wbm_dat_i;
   if (wbi_stb_o & wbi_ack_i) data_in <= wbi_dat_i;
   if (~sync & din & sel)     data_in <= vm_reg16; // addressless read

   //
   // Start interrupt vector fetch
   //
   if (~iedge & iako & din)
      wbi_stb <= 1'b1;
   else
      if (~iako | ~din | wbi_ack_i)
         wbi_stb <= 1'b0;

   if (~sync & din & sel & ~vm_aclo & ~vm_dclo)
         wlock[1] <= 1'b1;

   if (~wedge[2] & wedge[1] & ~vm_dclo & (wlock == 2'b11))
   begin
      wb_cyc      <= 1'b1;
      wb_we       <= 1'b1;
      wb_stb      <= 1'b1;
      wb_sel[0]   <= ~wtbt | ~addr[0];
      wb_sel[1]   <= ~wtbt | addr[0];
   end
   if (~redge & sync & din)
   begin
      wb_cyc      <= 1'b1;
      wb_we       <= 1'b0;
      wb_stb      <= 1'b1;
      wb_sel      <= 2'b11;
   end
   if ((~sync & ~din & ~dout) | wbm_ack_i)
   begin
      wb_cyc <= 1'b0;
      wb_stb <= 1'b0;
      wb_we  <= 1'b0;
      wb_sel <= 2'b00;
   end
end

//______________________________________________________________________________
//
assign   vm_sel[2]   = wbs_stb_i & (wbs_adr_i[3:1] == 3'b110) & wbs_a;  // 177714
assign   vm_sel[1]   = wbs_stb_i & (wbs_adr_i[3:1] == 3'b111) & wbs_a;  // 177716

assign   wbs_dat_o   = wbs_d;
assign   wbs_ack_o   = wbs_a;
assign   wbs_a_rc    = wbs_cyc_i & wbs_stb_i & (wbs_adr_i[3:2] == 2'b11)
                     & ((~wbs_we_i & ~wbs_r) | (wbs_we_i & ~wbs_w));

always @(posedge vm_clk_p)
begin
   wbs_a <= wbs_a_rc;
   wbs_r <= wbs_cyc_i & ((wbs_stb_i & ~wbs_we_i) | wbs_r);
   wbs_w <= wbs_cyc_i & ((wbs_stb_i & wbs_we_i)  | wbs_w);

   if (wbs_stb_i & ~wbs_we_i)
      case(wbs_adr_i[3:1])
         3'b110:  wbs_d <= vm_reg14;                     // 177714
         3'b111:  wbs_d <= vm_reg16;                     // 177716
         default: wbs_d <= 16'o000000;
      endcase
end
endmodule
