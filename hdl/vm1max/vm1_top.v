//
// Copyright (c) 2013 by 1801BM1@gmail.com
//______________________________________________________________________________
//
module vm1_top
(
   output      nBHE,          // high byte enable
   output      nBLE,          // low byte enable
   output      nMDIR,         // translation direction
   input[3:0]  nCFG,          // board configuration
   input[1:0]  nMRSV,         // service signals
                              //
   input       nMDCLO,        // DC low, input from main board
   input       nMACLO,        // AC low, input from main board
   input       nMSACK,        // system acknowledgement
   input       nMDMR,         // direct memory request
   input       nMVIRQ,        // vectored IRQ input
   input[3:1]  nMIRQ,         // fixed IRQ inputs
   input       nMRPLY,        // connector side: RPLY
   output      nMIAKO,        // connector side: IAKO
   output      nMDMGO,        // connector side: DMGO
                              //
   output      nMBSY,         // connector side: BSY
   output      nMSYNC,        // connector side: SYNC
   output      nMDIN,         // connector side: DIN
   output      nMDOUT,        // connector side: DOUT
   output      nMWTBT,        // connector side: WTBT
   output      nMINIT,        // connector side: INIT
                              //
   output      nDCLO,         // DC low, output to both CPU
   output      nACLO,         // AC low, output to both CPU
   output      nSACK,         // SACK - open drain
   output      nDMR,          // DMR - open drain
   inout       nRPLY,         // RPLY - open drain
                              //
   input       nBSY,          // processor side: BSY
   input       nINIT,         // processor side: INIT
   input       nDOUT,         // processor side: DOUT
   input       nDIN,          // processor side: DIN
   input       nWTBT,         // processor side: WTBT
   input       nSYNC,         // processor side: SYNC
                              //
   input       CLK,           // clock input
   output      CLK_MASTER,    // clock output for master CPU
   output      CLK_SLAVE,     // clock outpit for slave CPU
                              //
   output[3:1] nIRQ_MASTER,   // IRQ outputs to master CPU
   output      nVIRQ_MASTER,  // master CPU vectored IRQ
   input       nIAKO_MASTER,  //
   input[2:1]  nSEL_MASTER,   // master SELx
   input       nDMGO_MASTER,  //
                              //
   output[3:1] nIRQ_SLAVE,    // IRQ outputs to slave CPU
   output      nVIRQ_SLAVE,   // slave CPU vectored IRQ
   input       nIAKO_SLAVE,   //
   input[2:1]  nSEL_SLAVE,    // slave SELx
   input       nDMGO_SLAVE,   //
   output      nDMGI_SLAVE    //
);

//______________________________________________________________________________
//
wire     MULTI_CPU = ~nCFG[0];
wire     SLAVE_IRQ = ~nCFG[1];
wire     nEXDIN    = nMRSV[1];
wire     nEXENA    = nMRSV[0];

//______________________________________________________________________________
//
// Simple level translation
//
assign   CLK_MASTER        = CLK;
assign   CLK_SLAVE         = CLK;

assign   nDCLO             = nMDCLO;
assign   nACLO             = nMACLO;
assign   nMBSY             = nCFG[3] ? nBSY : nRPLY;
assign   nMSYNC            = nSYNC;
assign   nMDIN             = nDIN;
assign   nMDOUT            = nDOUT;
assign   nMWTBT            = nWTBT;
assign   nMINIT            = nINIT;

//______________________________________________________________________________
//
// IRQ demultiplexer
// Slave processor (CPU number != 0) does not serve vectored interrupts
//
// If nCFG[1] is high (jumper is removed) the master CPU processes IRQs
// If nCFG[1] is low (jumper is set) the slave CPU processes IRQs
//
assign   nVIRQ_MASTER      =  nMVIRQ;
assign   nIRQ_MASTER[1]    =  SLAVE_IRQ | nMIRQ[1];
assign   nIRQ_MASTER[2]    =  SLAVE_IRQ | nMIRQ[2];
assign   nIRQ_MASTER[3]    =  SLAVE_IRQ | nMIRQ[3];
assign   nVIRQ_SLAVE       = 1'b1;
assign   nIRQ_SLAVE[1]     = ~SLAVE_IRQ | nMIRQ[1];
assign   nIRQ_SLAVE[2]     = ~SLAVE_IRQ | nMIRQ[2];
assign   nIRQ_SLAVE[3]     = ~SLAVE_IRQ | nMIRQ[3];
assign   nMIAKO            = nIAKO_MASTER;

//______________________________________________________________________________
//
assign   nMDMGO            = ~MULTI_CPU & nDMGO_MASTER
                           |  MULTI_CPU & nDMGO_SLAVE;
assign   nDMGI_SLAVE       = ~MULTI_CPU | nDMGO_MASTER;

//______________________________________________________________________________
//
// Data/Adress buffers control
//
assign   nBHE  = nEXENA | (nDIN & ~nEXDIN);
assign   nBLE  = nEXENA | (nDIN & ~nEXDIN);
assign   nMDIR = ~nEXDIN;

//______________________________________________________________________________
//
assign   nRPLY = nSYNC ? nMRPLY | (nDIN & nDOUT): (nMRPLY ? 1'bz : (nDIN & nDOUT));
assign   nSACK = nMSACK ? 1'bz : 1'b0;
assign   nDMR  = nMDMR ? 1'bz : 1'b0;

endmodule
