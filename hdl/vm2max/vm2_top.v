//
// Copyright (c) 2015 by 1801BM1@gmail.com
//______________________________________________________________________________
//
module vm2_top
(
   output      nBHE,          // high byte enable
   output      nBLE,          // low byte enable
   output      nMDIR,         // translation direction
   input[1:0]  nCFG,          // board configuration
   input[1:0]  nMRSV,         // service signals
                              //
   input       nMDCLO,        // DC low, input from main board
   input       nMACLO,        // AC low, input from main board
   input       nMSACK,        // system acknowledgement
   input       nMDMR,         // direct memory request
   input       nMVIRQ,        // vectored IRQ input
   input       nMHALT,        // HALT interrupt request
   input       nMEVNT,        // timer interrupt request
   input       nMRPLY,        // connector side: RPLY
   output      nMIAKO,        // connector side: IAKO
   output      nMDMGO,        // connector side: DMGO
                              //
   input       nMAR,          // connector side: AR
   output      nMSEL,         // connector side: SEL
   output      nMSYNC,        // connector side: SYNC
   output      nMDIN,         // connector side: DIN
   output      nMDOUT,        // connector side: DOUT
   output      nMWTBT,        // connector side: WTBT
   output      nMINIT,        // connector side: INIT
                              //
   output      nDCLO,         // DC low, output to both CPU
   output      nACLO,         // AC low, output to both CPU
   output      nSACK,         // processor side: SACK
   output      nDMR,          // processor side: DMR
   output      nRPLY,         // processor side: RPLY
   output      nWAKI,         // processor side: WAKI
                              //
   output      nAR,           // processor side: AR
   input       nSEL,          // processor side: SEL
   input       nWRQ,          // processor side: WRQ
   input       nINIT,         // processor side: INIT
   input       nDOUT,         // processor side: DOUT
   input       nDIN,          // processor side: DIN
   input       nWTBT,         // processor side: WTBT
   input       nSYNC,         // processor side: SYNC
   input       nIAKO,         // processor side: IAKO
   input       nDMGO,         // processor side: DMGO
                              //
   input       MCLK,          // clock input
   input       CLCO,          // processor core clock
   output      CLC,           // clock output for CPU
                              //
   output      nVIRQ,         // processor side: VIRQ
   output      nHALT,         // processor side: HALT
   output      nEVNT          // processor side: EVNT
);

//______________________________________________________________________________
//
wire     nEXDIN   = nMRSV[1];
wire     nEXENA   = nMRSV[0];

//______________________________________________________________________________
//
// Simple level translation
//
assign   CLC      = MCLK;
assign   nDCLO    = nMDCLO;
assign   nACLO    = nMACLO;

assign   nMSEL    = nSEL;
assign   nMSYNC   = nSYNC;
assign   nMDIN    = nDIN;
assign   nMDOUT   = nDOUT;
assign   nMWTBT   = nWTBT;
assign   nMINIT   = nINIT;
assign   nMIAKO   = nIAKO;
assign   nMDMGO   = nDMGO;

//______________________________________________________________________________
//
assign   nVIRQ    = nMVIRQ;
assign   nHALT    = nMHALT;
assign   nEVNT    = nMEVNT;
assign   nWAKI    = nWRQ;

assign   nAR      = nMAR;
assign   nSACK    = nMSACK;
assign   nDMR     = nMDMR;
assign   nRPLY    = nMRPLY;

//______________________________________________________________________________
//
// Data/Adress buffers control
//
assign   nBHE  = nEXENA | (nDIN & ~nEXDIN);
assign   nBLE  = nEXENA | (nDIN & ~nEXDIN);
assign   nMDIR = ~nEXDIN;

endmodule
