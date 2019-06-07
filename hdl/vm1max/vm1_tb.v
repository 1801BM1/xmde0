//
// Copyright (c) 2013 by VSO Team
//______________________________________________________________________________
//
`timescale 1ns / 10ps

module vm1_tb();

wire  nBHE;
wire  nBLE;
wire  nMDIR;
reg   [3:0] nCFG;
reg   [1:0] nMRSV;

reg   nMDCLO;
reg   nMACLO;
reg   nMSACK;
reg   nMDMR;
reg   nMVIRQ;
reg   [3:1] nMIRQ;
reg   nMRPLY;
wire  nMIAKO;
wire  nMDMGO;

wire  nMBSY;
wire  nMSYNC;
wire  nMDIN;
wire  nMDOUT;
wire  nMWTBT;
wire  nMINIT;

wire  nDCLO;
wire  nACLO;
tri1  nSACK;
tri1  nDMR;
tri1  nRPLY;

reg   nBSY;
reg   nINIT;
reg   nDOUT;
reg   nDIN;
reg   nWTBT;
reg   nSYNC;

reg   CLK;
wire  CLK_MASTER;
wire  CLK_SLAVE;
reg   [2:1] nSEL_MASTER;
reg   [2:1] nSEL_SLAVE;

wire  nVIRQ_MASTER;
wire  [3:1] nIRQ_MASTER;
wire  nVIRQ_SLAVE;
wire  [3:1] nIRQ_SLAVE;

reg   nIAKO_MASTER;
reg   nIAKO_SLAVE;
reg   nDMGO_MASTER;
reg   nDMGO_SLAVE;
wire  nDMGI_SLAVE;

vm1_top vm_tb(
   nBHE,
   nBLE,
   nMDIR,
   nCFG,
   nMRSV,
   nMDCLO,
   nMACLO,
   nMSACK,
   nMDMR,
   nMVIRQ,
   nMIRQ,
   nMRPLY,
   nMIAKO,
   nMDMGO,
   nMBSY,
   nMSYNC,
   nMDIN,
   nMDOUT,
   nMWTBT,
   nMINIT,
   nDCLO,
   nACLO,
   nSACK,
   nDMR,
   nRPLY,
   nBSY,
   nINIT,
   nDOUT,
   nDIN,
   nWTBT,
   nSYNC,
   CLK,
   CLK_MASTER,
   CLK_SLAVE,
   nSEL_MASTER,
   nSEL_SLAVE,
   nVIRQ_MASTER,
   nIRQ_MASTER,
   nVIRQ_SLAVE,
   nIRQ_SLAVE,
   nIAKO_MASTER,
   nIAKO_SLAVE,
   nDMGO_MASTER,
   nDMGO_SLAVE,
   nDMGI_SLAVE
);

initial
begin
         CLK = 0;
#100     forever #100 CLK = ~CLK;
end

initial
begin
         nCFG = 4'hF;
#1000    nCFG = 4'hC;
#1000    $stop;
end

initial
begin
         nMRSV    = 2'b11;
         nMSACK   = 1;
         nMDMR    = 1;
         nMVIRQ   = 1;
         nMIRQ    = 3'b111;
         nMRPLY   = 1;
         nBSY     = 1;
         nINIT    = 1;
         nDOUT    = 1;
         nDIN     = 1;
         nWTBT    = 1;
         nSYNC    = 1;

         nSEL_MASTER    = 2'b11;
         nSEL_SLAVE     = 2'b11;
         nIAKO_MASTER   = 1;
         nIAKO_SLAVE    = 1;
         nDMGO_MASTER   = 1;
         nDMGO_SLAVE    = 1;

         nMDCLO = 0;
         nMACLO = 0;
#200     nMRSV  = 2'b00;
#10      nSYNC  = 0;
#10      nDIN   = 0;
#10      nMRPLY = 0;
#20      nDIN   = 1;
#10      nMRPLY = 1;
#10      nSYNC  = 1;
#10      nMRSV  = 2'b11;
end

initial
begin
#200     nMDCLO = 1;
         nMACLO = 1;
#200     nMDCLO = 0;
         nMACLO = 0;
#2000    $stop;
end
endmodule

