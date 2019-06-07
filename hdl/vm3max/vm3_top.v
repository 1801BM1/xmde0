//
// Copyright (c) 2015 by 1801BM1@gmail.com
//______________________________________________________________________________
//
module vm3_top
(
   output         nBHE,          // high byte enable
   output         nBLE,          // low byte enable
   output         nMDIR,         // translation direction
   input[1:0]     nCFG,          // board configuration
   input[1:0]     nMRSV,         // service signals
                                 //
   input          nMRPLY,        // connector side: RPLY
   output         nMIAKO,        // connector side: IAKO
   output         nMDMGO,        // connector side: DMGO
   output[21:16]  nMAC,          // muxed address/control
                                 //
   output         nMSYNC,        // connector side: SYNC
   output         nMDIN,         // connector side: DIN
   output         nMDOUT,        // connector side: DOUT
   output         nMWTBT,        // connector side: WTBT
   output         nMINIT,        // connector side: INIT
                                 //
   output         nDCLO,         // DC low, output to both CPU
   output         nACLO,         // AC low, output to both CPU
   output         nSACK,         // processor side: SACK
   output         nDMR,          // processor side: DMR
   output         nRPLY,         // processor side: RPLY
                                 //
   output         nSSYNC,        // processor side: SSYNC
   input[21:16]   nA,            // processor side: A16-21
   input          nBS,           // processor side: BS
   input          nTA,           // processor side: TA
   input          nSEL,          // processor side: SEL
   input          nUMAP,         // processor side: UMAP
   input          nHLTM,         // processor side: HLTM
   input          nINIT,         // processor side: INIT
   input          nDOUT,         // processor side: DOUT
   input          nDIN,          // processor side: DIN
   input          nWTBT,         // processor side: WTBT
   input          nSYNC,         // processor side: SYNC
   input          nIAKO,         // processor side: IAKO
   input          nDMGO,         // processor side: DMGO
                                 //
   input[1:0]     MXIN,          // mux input
   input          MXSTB,         // mux strobe
   input          MCLK,          // clock input
   output         CLC,           // clock output for CPU
                                 //
   output         nHALT,         // processor side: HALT
   output         nEVNT,         // processor side: EVNT
   output[3:0]    nIRQ           // processor side: IRQx
);

//______________________________________________________________________________
//
reg[5:0] shr;
reg[7:0] mux;
reg[1:0] dmr;
wire[2:0] delay;

wire     nEXDIN   = nMRSV[1];
wire     nEXMUX   = nMRSV[0];

//______________________________________________________________________________
//
// Simple level translation
//
assign   CLC      = MCLK;
assign   nMSYNC   = nSYNC;
assign   nMDIN    = nDIN;
assign   nMDOUT   = nDOUT;
assign   nMWTBT   = nWTBT;
assign   nMINIT   = nINIT;
assign   nMIAKO   = nIAKO;
assign   nMDMGO   = nDMGO;

assign   nRPLY    = nMRPLY | (nDIN & nDOUT);
assign   nSSYNC   = ~nCFG[0] & nSYNC;

//______________________________________________________________________________
//
// Address/control multiplexer
//
assign   nMAC[21:16] = nEXMUX ? nA[21:16] : {nA[21], nUMAP, nHLTM, nSEL, nBS, nTA};

//______________________________________________________________________________
//
// Serial loadable register
//
assign   nDCLO    = mux[7] & nCFG[1];
assign   nACLO    = mux[6];
assign   nHALT    = mux[5];
assign   nEVNT    = delay[2]; // mux[4];
assign   nIRQ     = mux[3:0];

lcell dl0(.in(mux[4]), .out(delay[0]));
lcell dl1(.in(delay[0]), .out(delay[1]));
lcell dl2(.in(delay[1]), .out(delay[2]));

assign   nDMR     = dmr[0];
assign   nSACK    = dmr[1];

always @(posedge MCLK)
begin
   if (MXSTB)
      mux <= {shr[5:0], MXIN[1:0]};
   else
      shr <= {shr[3:0], MXIN[1:0]};
end

always @(negedge MCLK) dmr <= MXIN;

//______________________________________________________________________________
//
// Data/Adress buffers control
//
assign   nBHE  = nDIN & ~nEXDIN;
assign   nBLE  = nDIN & ~nEXDIN;
assign   nMDIR = ~nEXDIN;

endmodule
