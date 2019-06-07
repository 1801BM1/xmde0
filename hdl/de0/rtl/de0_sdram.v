//
// Copyright (c) 2014-2015 by 1801BM1@gmail.com
//
// SDR SDRAM Controller
//______________________________________________________________________________
//
`include "config.v"

module de0_sdram
#(parameter
   REFCLK            = 100000000,   // Reference clock requency in Hz
   SDRAM_DATA_SIZE   = 16,          // Data bus width
   SDRAM_ADDR_SIZE   = 24,          // Address bus width
   SDRAM_ROW_SIZE    = 13,          // Row address width
   SDRAM_COL_SIZE    = 9,           // Column address width
   SDRAM_BANK_SIZE   = 2,           // Bank select lines
   SDRAM_BURST_LEN   = 8,           // Maximum burst length
                                    //
   SDRAM_TRC         = 7,           // ROW cycle time (ticks)
   SDRAM_TRAS        = 5,           // ROW active time
   SDRAM_TRCD        = 2,           // RAS to CAS delay
   SDRAM_TCL         = 2,           // CAS latency
   SDRAM_TRP         = 2,           // ROW precharge time
   SDRAM_TDPL        = 2,           // Write Data to Precharge
   SDRAM_TREF        = 780,         // Refresh period (ticks)
   SDRAM_THRF        = 80           // High priority refresh
)
(
   input                            mc_clk_i,         // system bus clock (sdram one should be phase aligned)
   input                            mc_clk_us_i,      // microseconds clock enable
   input                            mc_pwron_i,       // power-up reset, can be applied once
                                                      //
   output [SDRAM_DATA_SIZE-1:0]     mc_dat_o,         // system hub data bus
   input  [SDRAM_DATA_SIZE-1:0]     mc_dat_i,         // system hub data bus
   input  [SDRAM_DATA_SIZE/8-1:0]   mc_sel_i,         // byte lane mask (meaningful for write operations)
   input  [SDRAM_ADDR_SIZE-1:0]     mc_adr_i,         // system hub address
   input                            mc_stb_i,         // command strobe
   input                            mc_rfsh_i,        // refresh command
   input                            mc_wreq_i,        // read/write command
   input  [2:0]                     mc_blen_i,        // command burst length
   output reg                       mc_done_o,        // command completed
   output reg                       mc_ack_o,         // data transfer acknowledment
   output reg                       mc_rdy_o,         // controller ready for command
                                                      //
   output reg                       mc_lref_o,        // refresh request low priority
   output reg                       mc_href_o,        // refresh request high priority
                                                      //
   inout  [SDRAM_DATA_SIZE-1:0]     sdram_dat_io,     // DRAM data bus
   output [SDRAM_ROW_SIZE-1:0]      sdram_adr_o,      // DRAM address bus
   output [SDRAM_BANK_SIZE-1:0]     sdram_bank_o,     // DRAM bank selection
   output [SDRAM_DATA_SIZE/8-1:0]   sdram_dqm_o,      // DRAM write data mask
   output                           sdram_cke_o,      // DRAM clock enable
   output                           sdram_cas_n,      // DRAM column address strobe
   output                           sdram_ras_n,      // DRAM row address strobe
   output                           sdram_we_n,       // DRAM write enable
   output                           sdram_cs_n        // DRAM chip select
);
//______________________________________________________________________________
//
// SDRAM command table         CS RAS CAS WE
//
localparam  SDRAM_CMD_DESL    = 4'b1111,        // device deselected
            SDRAM_CMD_NOP     = 4'b0111,        // no operation
            SDRAM_CMD_MRS     = 4'b0000,        // mode register set
            SDRAM_CMD_ACT     = 4'b0011,        // bank activate
            SDRAM_CMD_READ    = 4'b0101,        // read (autoprecharge)
            SDRAM_CMD_WRITE   = 4'b0100,        // write (autoprecharge)
            SDRAM_CMD_PRE     = 4'b0010,        // precharge (bank/all)
            SDRAM_CMD_BST     = 4'b0110,        // burst stop
            SDRAM_CMD_RFSH    = 4'b0001;        // auto refresh


localparam  MODE_REGISTER = 12'b000000110001;

localparam  INIT_IDLE            = 3'b000,
              INIT_WAIT_200us    = 3'b001,
              INIT_INIT_PRE      = 3'b010,
              INIT_WAIT_PRE      = 3'b011,
              INIT_MODE_REG      = 3'b100,
              INIT_WAIT_MODE_REG = 3'b101,
              INIT_DONE_ST       = 3'b110;

localparam IDLE_ST           = 4'b0000,
              REFRESH_ST      = 4'b0001,
              REFRESH_WAIT_ST = 4'b0010,
              ACT_ST          = 4'b0011,
              WAIT_ACT_ST     = 4'b0100,
              WRITE0_ST       = 4'b0101,
              WRITE1_ST       = 4'b0110,
              WRITE_PRE_ST    = 4'b0111,
              READ0_ST        = 4'b1000,
              READ1_ST        = 4'b1001,
              READ2_ST        = 4'b1010,
              READ3_ST        = 4'b1011,
              READ4_ST        = 4'b1100,
              READ_PRE_ST     = 4'b1101,
              PRE_ST          = 4'b1110,
              WAIT_PRE_ST     = 4'b1111;

localparam REF_COUNTER_WIDTH = log2(SDRAM_TREF);

//______________________________________________________________________________
//
reg   [SDRAM_DATA_SIZE-1:0]         rdat_o;           // write data register
reg   [SDRAM_DATA_SIZE-1:0]         rdat_i;           // read data register
reg   [SDRAM_ROW_SIZE-1:0]          radr;             // address register
reg   [SDRAM_BANK_SIZE-1:0]         rbank;            // bank selection
reg   [SDRAM_DATA_SIZE/8-1:0]       rdqm;             // write data mask
reg   [3:0]                         rcmd;             // memory command register
reg                                 rdena;            // data output enable
                                                      //
reg   [REF_COUNTER_WIDTH-1:0]       ref_cnt;          //
reg                                 ref_hrq, ref_lrq; //
//______________________________________________________________________________
//
assign   sdram_cke_o    = 1'b1;
assign   sdram_dat_io   = rdena ? rdat_o : {SDRAM_DATA_SIZE {1'bZ}};
assign   sdram_adr_o    = radr;
assign   sdram_bank_o   = rbank;
assign   sdram_dqm_o    = rdqm;

assign   sdram_cs_n     = rcmd[3];
assign   sdram_ras_n    = rcmd[2];
assign   sdram_cas_n    = rcmd[1];
assign   sdram_we_n     = rcmd[0];

always @(posedge mc_clk_i)
begin
end
/*
power up seq
read / write /refresh
*/
function integer log2(input integer value);
   begin
      for (log2=0; value>0; log2=log2+1)
         value = value >> 1;
   end
endfunction
endmodule
