//
//Copyright (C) 1991-2012 Altera Corporation
//Your use of Altera Corporation's design tools, logic functions
//and other software and tools, and its AMPP partner logic
//functions, and any output files from any of the foregoing
//(including device programming or simulation files), and any
//associated documentation or information are expressly subject
//to the terms and conditions of the Altera Program License
//Subscription Agreement, Altera MegaCore Function License
//Agreement, or other applicable license agreement, including,
//without limitation, that your use is for the sole purpose of
//programming logic devices manufactured by Altera and sold by
//Altera or its authorized distributors.  Please refer to the
//applicable agreement for further details.
//
// synopsys translate_off
`timescale 1 ps / 1 ps
// synopsys translate_on
module de0_pll50 (
   inclk0,
   c0,
   c1,
   locked);

   input   inclk0;
   output     c0;
   output     c1;
   output     locked;

   wire [4:0] sub_wire0;
   wire  sub_wire2;
   wire [0:0] sub_wire6 = 1'h0;
   wire [0:0] sub_wire3 = sub_wire0[0:0];
   wire [1:1] sub_wire1 = sub_wire0[1:1];
   wire  c1 = sub_wire1;
   wire  locked = sub_wire2;
   wire  c0 = sub_wire3;
   wire  sub_wire4 = inclk0;
   wire [1:0] sub_wire5 = {sub_wire6, sub_wire4};

   altpll   altpll_component (
            .inclk (sub_wire5),
            .clk (sub_wire0),
            .locked (sub_wire2),
            .activeclock (),
            .areset (1'b0),
            .clkbad (),
            .clkena ({6{1'b1}}),
            .clkloss (),
            .clkswitch (1'b0),
            .configupdate (1'b0),
            .enable0 (),
            .enable1 (),
            .extclk (),
            .extclkena ({4{1'b1}}),
            .fbin (1'b1),
            .fbmimicbidir (),
            .fbout (),
            .fref (),
            .icdrclk (),
            .pfdena (1'b1),
            .phasecounterselect ({4{1'b1}}),
            .phasedone (),
            .phasestep (1'b1),
            .phaseupdown (1'b1),
            .pllena (1'b1),
            .scanaclr (1'b0),
            .scanclk (1'b0),
            .scanclkena (1'b1),
            .scandata (1'b0),
            .scandataout (),
            .scandone (),
            .scanread (1'b0),
            .scanwrite (1'b0),
            .sclkout0 (),
            .sclkout1 (),
            .vcooverrange (),
            .vcounderrange ());
   defparam
      altpll_component.bandwidth_type = "AUTO",
      altpll_component.clk0_divide_by = 1,
      altpll_component.clk0_duty_cycle = 50,
      altpll_component.clk0_multiply_by = 1,
      altpll_component.clk0_phase_shift = "0",
      altpll_component.clk1_divide_by = 1,
      altpll_component.clk1_duty_cycle = 50,
      altpll_component.clk1_multiply_by = 1,
      altpll_component.clk1_phase_shift = "10000",
      altpll_component.compensate_clock = "CLK0",
      altpll_component.inclk0_input_frequency = 20000,
      altpll_component.intended_device_family = "Cyclone III",
      altpll_component.lpm_hint = "CBX_MODULE_PREFIX=de0_pll50",
      altpll_component.lpm_type = "altpll",
      altpll_component.operation_mode = "NORMAL",
      altpll_component.pll_type = "AUTO",
      altpll_component.port_activeclock = "PORT_UNUSED",
      altpll_component.port_areset = "PORT_UNUSED",
      altpll_component.port_clkbad0 = "PORT_UNUSED",
      altpll_component.port_clkbad1 = "PORT_UNUSED",
      altpll_component.port_clkloss = "PORT_UNUSED",
      altpll_component.port_clkswitch = "PORT_UNUSED",
      altpll_component.port_configupdate = "PORT_UNUSED",
      altpll_component.port_fbin = "PORT_UNUSED",
      altpll_component.port_inclk0 = "PORT_USED",
      altpll_component.port_inclk1 = "PORT_UNUSED",
      altpll_component.port_locked = "PORT_USED",
      altpll_component.port_pfdena = "PORT_UNUSED",
      altpll_component.port_phasecounterselect = "PORT_UNUSED",
      altpll_component.port_phasedone = "PORT_UNUSED",
      altpll_component.port_phasestep = "PORT_UNUSED",
      altpll_component.port_phaseupdown = "PORT_UNUSED",
      altpll_component.port_pllena = "PORT_UNUSED",
      altpll_component.port_scanaclr = "PORT_UNUSED",
      altpll_component.port_scanclk = "PORT_UNUSED",
      altpll_component.port_scanclkena = "PORT_UNUSED",
      altpll_component.port_scandata = "PORT_UNUSED",
      altpll_component.port_scandataout = "PORT_UNUSED",
      altpll_component.port_scandone = "PORT_UNUSED",
      altpll_component.port_scanread = "PORT_UNUSED",
      altpll_component.port_scanwrite = "PORT_UNUSED",
      altpll_component.port_clk0 = "PORT_USED",
      altpll_component.port_clk1 = "PORT_USED",
      altpll_component.port_clk2 = "PORT_UNUSED",
      altpll_component.port_clk3 = "PORT_UNUSED",
      altpll_component.port_clk4 = "PORT_UNUSED",
      altpll_component.port_clk5 = "PORT_UNUSED",
      altpll_component.port_clkena0 = "PORT_UNUSED",
      altpll_component.port_clkena1 = "PORT_UNUSED",
      altpll_component.port_clkena2 = "PORT_UNUSED",
      altpll_component.port_clkena3 = "PORT_UNUSED",
      altpll_component.port_clkena4 = "PORT_UNUSED",
      altpll_component.port_clkena5 = "PORT_UNUSED",
      altpll_component.port_extclk0 = "PORT_UNUSED",
      altpll_component.port_extclk1 = "PORT_UNUSED",
      altpll_component.port_extclk2 = "PORT_UNUSED",
      altpll_component.port_extclk3 = "PORT_UNUSED",
      altpll_component.self_reset_on_loss_lock = "ON",
      altpll_component.width_clock = 5;


endmodule

// ============================================================
// CNX file retrieval info
// ============================================================
// Retrieval info: PRIVATE: ACTIVECLK_CHECK STRING "0"
// Retrieval info: PRIVATE: BANDWIDTH STRING "1.000"
// Retrieval info: PRIVATE: BANDWIDTH_FEATURE_ENABLED STRING "1"
// Retrieval info: PRIVATE: BANDWIDTH_FREQ_UNIT STRING "MHz"
// Retrieval info: PRIVATE: BANDWIDTH_PRESET STRING "Low"
// Retrieval info: PRIVATE: BANDWIDTH_USE_AUTO STRING "1"
// Retrieval info: PRIVATE: BANDWIDTH_USE_PRESET STRING "0"
// Retrieval info: PRIVATE: CLKBAD_SWITCHOVER_CHECK STRING "0"
// Retrieval info: PRIVATE: CLKLOSS_CHECK STRING "0"
// Retrieval info: PRIVATE: CLKSWITCH_CHECK STRING "0"
// Retrieval info: PRIVATE: CNX_NO_COMPENSATE_RADIO STRING "0"
// Retrieval info: PRIVATE: CREATE_CLKBAD_CHECK STRING "0"
// Retrieval info: PRIVATE: CREATE_INCLK1_CHECK STRING "0"
// Retrieval info: PRIVATE: CUR_DEDICATED_CLK STRING "c0"
// Retrieval info: PRIVATE: CUR_FBIN_CLK STRING "c0"
// Retrieval info: PRIVATE: DEVICE_SPEED_GRADE STRING "Any"
// Retrieval info: PRIVATE: DIV_FACTOR0 NUMERIC "1"
// Retrieval info: PRIVATE: DIV_FACTOR1 NUMERIC "1"
// Retrieval info: PRIVATE: DUTY_CYCLE0 STRING "50.00000000"
// Retrieval info: PRIVATE: DUTY_CYCLE1 STRING "50.00000000"
// Retrieval info: PRIVATE: EFF_OUTPUT_FREQ_VALUE0 STRING "50.000000"
// Retrieval info: PRIVATE: EFF_OUTPUT_FREQ_VALUE1 STRING "50.000000"
// Retrieval info: PRIVATE: EXPLICIT_SWITCHOVER_COUNTER STRING "0"
// Retrieval info: PRIVATE: EXT_FEEDBACK_RADIO STRING "0"
// Retrieval info: PRIVATE: GLOCKED_COUNTER_EDIT_CHANGED STRING "1"
// Retrieval info: PRIVATE: GLOCKED_FEATURE_ENABLED STRING "0"
// Retrieval info: PRIVATE: GLOCKED_MODE_CHECK STRING "0"
// Retrieval info: PRIVATE: GLOCK_COUNTER_EDIT NUMERIC "1048575"
// Retrieval info: PRIVATE: HAS_MANUAL_SWITCHOVER STRING "1"
// Retrieval info: PRIVATE: INCLK0_FREQ_EDIT STRING "50.000"
// Retrieval info: PRIVATE: INCLK0_FREQ_UNIT_COMBO STRING "MHz"
// Retrieval info: PRIVATE: INCLK1_FREQ_EDIT STRING "100.000"
// Retrieval info: PRIVATE: INCLK1_FREQ_EDIT_CHANGED STRING "1"
// Retrieval info: PRIVATE: INCLK1_FREQ_UNIT_CHANGED STRING "1"
// Retrieval info: PRIVATE: INCLK1_FREQ_UNIT_COMBO STRING "MHz"
// Retrieval info: PRIVATE: INTENDED_DEVICE_FAMILY STRING "Cyclone III"
// Retrieval info: PRIVATE: INT_FEEDBACK__MODE_RADIO STRING "1"
// Retrieval info: PRIVATE: LOCKED_OUTPUT_CHECK STRING "1"
// Retrieval info: PRIVATE: LONG_SCAN_RADIO STRING "1"
// Retrieval info: PRIVATE: LVDS_MODE_DATA_RATE STRING "Not Available"
// Retrieval info: PRIVATE: LVDS_MODE_DATA_RATE_DIRTY NUMERIC "0"
// Retrieval info: PRIVATE: LVDS_PHASE_SHIFT_UNIT0 STRING "deg"
// Retrieval info: PRIVATE: LVDS_PHASE_SHIFT_UNIT1 STRING "deg"
// Retrieval info: PRIVATE: MIG_DEVICE_SPEED_GRADE STRING "Any"
// Retrieval info: PRIVATE: MIRROR_CLK0 STRING "0"
// Retrieval info: PRIVATE: MIRROR_CLK1 STRING "0"
// Retrieval info: PRIVATE: MULT_FACTOR0 NUMERIC "1"
// Retrieval info: PRIVATE: MULT_FACTOR1 NUMERIC "1"
// Retrieval info: PRIVATE: NORMAL_MODE_RADIO STRING "1"
// Retrieval info: PRIVATE: OUTPUT_FREQ0 STRING "100.00000000"
// Retrieval info: PRIVATE: OUTPUT_FREQ1 STRING "100.00000000"
// Retrieval info: PRIVATE: OUTPUT_FREQ_MODE0 STRING "0"
// Retrieval info: PRIVATE: OUTPUT_FREQ_MODE1 STRING "0"
// Retrieval info: PRIVATE: OUTPUT_FREQ_UNIT0 STRING "MHz"
// Retrieval info: PRIVATE: OUTPUT_FREQ_UNIT1 STRING "MHz"
// Retrieval info: PRIVATE: PHASE_RECONFIG_FEATURE_ENABLED STRING "1"
// Retrieval info: PRIVATE: PHASE_RECONFIG_INPUTS_CHECK STRING "0"
// Retrieval info: PRIVATE: PHASE_SHIFT0 STRING "0.00000000"
// Retrieval info: PRIVATE: PHASE_SHIFT1 STRING "180.00000000"
// Retrieval info: PRIVATE: PHASE_SHIFT_STEP_ENABLED_CHECK STRING "0"
// Retrieval info: PRIVATE: PHASE_SHIFT_UNIT0 STRING "deg"
// Retrieval info: PRIVATE: PHASE_SHIFT_UNIT1 STRING "deg"
// Retrieval info: PRIVATE: PLL_ADVANCED_PARAM_CHECK STRING "0"
// Retrieval info: PRIVATE: PLL_ARESET_CHECK STRING "0"
// Retrieval info: PRIVATE: PLL_AUTOPLL_CHECK NUMERIC "1"
// Retrieval info: PRIVATE: PLL_ENHPLL_CHECK NUMERIC "0"
// Retrieval info: PRIVATE: PLL_FASTPLL_CHECK NUMERIC "0"
// Retrieval info: PRIVATE: PLL_FBMIMIC_CHECK STRING "0"
// Retrieval info: PRIVATE: PLL_LVDS_PLL_CHECK NUMERIC "0"
// Retrieval info: PRIVATE: PLL_PFDENA_CHECK STRING "0"
// Retrieval info: PRIVATE: PLL_TARGET_HARCOPY_CHECK NUMERIC "0"
// Retrieval info: PRIVATE: PRIMARY_CLK_COMBO STRING "inclk0"
// Retrieval info: PRIVATE: RECONFIG_FILE STRING "de0_pll50"
// Retrieval info: PRIVATE: SACN_INPUTS_CHECK STRING "0"
// Retrieval info: PRIVATE: SCAN_FEATURE_ENABLED STRING "1"
// Retrieval info: PRIVATE: SELF_RESET_LOCK_LOSS STRING "1"
// Retrieval info: PRIVATE: SHORT_SCAN_RADIO STRING "0"
// Retrieval info: PRIVATE: SPREAD_FEATURE_ENABLED STRING "0"
// Retrieval info: PRIVATE: SPREAD_FREQ STRING "50.000"
// Retrieval info: PRIVATE: SPREAD_FREQ_UNIT STRING "KHz"
// Retrieval info: PRIVATE: SPREAD_PERCENT STRING "0.500"
// Retrieval info: PRIVATE: SPREAD_USE STRING "0"
// Retrieval info: PRIVATE: SRC_SYNCH_COMP_RADIO STRING "0"
// Retrieval info: PRIVATE: STICKY_CLK0 STRING "1"
// Retrieval info: PRIVATE: STICKY_CLK1 STRING "1"
// Retrieval info: PRIVATE: SWITCHOVER_COUNT_EDIT NUMERIC "1"
// Retrieval info: PRIVATE: SWITCHOVER_FEATURE_ENABLED STRING "1"
// Retrieval info: PRIVATE: SYNTH_WRAPPER_GEN_POSTFIX STRING "0"
// Retrieval info: PRIVATE: USE_CLK0 STRING "1"
// Retrieval info: PRIVATE: USE_CLK1 STRING "1"
// Retrieval info: PRIVATE: USE_CLKENA0 STRING "0"
// Retrieval info: PRIVATE: USE_CLKENA1 STRING "0"
// Retrieval info: PRIVATE: USE_MIL_SPEED_GRADE NUMERIC "0"
// Retrieval info: PRIVATE: ZERO_DELAY_RADIO STRING "0"
// Retrieval info: LIBRARY: altera_mf altera_mf.altera_mf_components.all
// Retrieval info: CONSTANT: BANDWIDTH_TYPE STRING "AUTO"
// Retrieval info: CONSTANT: CLK0_DIVIDE_BY NUMERIC "1"
// Retrieval info: CONSTANT: CLK0_DUTY_CYCLE NUMERIC "50"
// Retrieval info: CONSTANT: CLK0_MULTIPLY_BY NUMERIC "1"
// Retrieval info: CONSTANT: CLK0_PHASE_SHIFT STRING "0"
// Retrieval info: CONSTANT: CLK1_DIVIDE_BY NUMERIC "1"
// Retrieval info: CONSTANT: CLK1_DUTY_CYCLE NUMERIC "50"
// Retrieval info: CONSTANT: CLK1_MULTIPLY_BY NUMERIC "1"
// Retrieval info: CONSTANT: CLK1_PHASE_SHIFT STRING "10000"
// Retrieval info: CONSTANT: COMPENSATE_CLOCK STRING "CLK0"
// Retrieval info: CONSTANT: INCLK0_INPUT_FREQUENCY NUMERIC "20000"
// Retrieval info: CONSTANT: INTENDED_DEVICE_FAMILY STRING "Cyclone III"
// Retrieval info: CONSTANT: LPM_TYPE STRING "altpll"
// Retrieval info: CONSTANT: OPERATION_MODE STRING "NORMAL"
// Retrieval info: CONSTANT: PLL_TYPE STRING "AUTO"
// Retrieval info: CONSTANT: PORT_ACTIVECLOCK STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_ARESET STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_CLKBAD0 STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_CLKBAD1 STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_CLKLOSS STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_CLKSWITCH STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_CONFIGUPDATE STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_FBIN STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_INCLK0 STRING "PORT_USED"
// Retrieval info: CONSTANT: PORT_INCLK1 STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_LOCKED STRING "PORT_USED"
// Retrieval info: CONSTANT: PORT_PFDENA STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_PHASECOUNTERSELECT STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_PHASEDONE STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_PHASESTEP STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_PHASEUPDOWN STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_PLLENA STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_SCANACLR STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_SCANCLK STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_SCANCLKENA STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_SCANDATA STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_SCANDATAOUT STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_SCANDONE STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_SCANREAD STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_SCANWRITE STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_clk0 STRING "PORT_USED"
// Retrieval info: CONSTANT: PORT_clk1 STRING "PORT_USED"
// Retrieval info: CONSTANT: PORT_clk2 STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_clk3 STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_clk4 STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_clk5 STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_clkena0 STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_clkena1 STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_clkena2 STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_clkena3 STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_clkena4 STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_clkena5 STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_extclk0 STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_extclk1 STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_extclk2 STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_extclk3 STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: SELF_RESET_ON_LOSS_LOCK STRING "ON"
// Retrieval info: CONSTANT: WIDTH_CLOCK NUMERIC "5"
// Retrieval info: USED_PORT: @clk 0 0 5 0 OUTPUT_CLK_EXT VCC "@clk[4..0]"
// Retrieval info: USED_PORT: c0 0 0 0 0 OUTPUT_CLK_EXT VCC "c0"
// Retrieval info: USED_PORT: c1 0 0 0 0 OUTPUT_CLK_EXT VCC "c1"
// Retrieval info: USED_PORT: inclk0 0 0 0 0 INPUT_CLK_EXT GND "inclk0"
// Retrieval info: USED_PORT: locked 0 0 0 0 OUTPUT GND "locked"
// Retrieval info: CONNECT: @inclk 0 0 1 1 GND 0 0 0 0
// Retrieval info: CONNECT: @inclk 0 0 1 0 inclk0 0 0 0 0
// Retrieval info: CONNECT: c0 0 0 0 0 @clk 0 0 1 0
// Retrieval info: CONNECT: c1 0 0 0 0 @clk 0 0 1 1
// Retrieval info: CONNECT: locked 0 0 0 0 @locked 0 0 0 0
// Retrieval info: GEN_FILE: TYPE_NORMAL de0_pll50.v TRUE
// Retrieval info: GEN_FILE: TYPE_NORMAL de0_pll50.ppf TRUE
// Retrieval info: GEN_FILE: TYPE_NORMAL de0_pll50.inc FALSE
// Retrieval info: GEN_FILE: TYPE_NORMAL de0_pll50.cmp FALSE
// Retrieval info: GEN_FILE: TYPE_NORMAL de0_pll50.bsf FALSE
// Retrieval info: GEN_FILE: TYPE_NORMAL de0_pll50_inst.v FALSE
// Retrieval info: GEN_FILE: TYPE_NORMAL de0_pll50_bb.v TRUE
// Retrieval info: LIB_FILE: altera_mf
// Retrieval info: CBX_MODULE_PREFIX: ON


//Copyright (C) 1991-2012 Altera Corporation
//Your use of Altera Corporation's design tools, logic functions
//and other software and tools, and its AMPP partner logic
//functions, and any output files from any of the foregoing
//(including device programming or simulation files), and any
//associated documentation or information are expressly subject
//to the terms and conditions of the Altera Program License
//Subscription Agreement, Altera MegaCore Function License
//Agreement, or other applicable license agreement, including,
//without limitation, that your use is for the sole purpose of
//programming logic devices manufactured by Altera and sold by
//Altera or its authorized distributors.  Please refer to the
//applicable agreement for further details.


// synopsys translate_off
`timescale 1 ps / 1 ps
// synopsys translate_on
module de0_pll100 (
   inclk0,
   c0,
   c1,
   locked);

   input   inclk0;
   output     c0;
   output     c1;
   output     locked;

   wire [4:0] sub_wire0;
   wire  sub_wire2;
   wire [0:0] sub_wire6 = 1'h0;
   wire [0:0] sub_wire3 = sub_wire0[0:0];
   wire [1:1] sub_wire1 = sub_wire0[1:1];
   wire  c1 = sub_wire1;
   wire  locked = sub_wire2;
   wire  c0 = sub_wire3;
   wire  sub_wire4 = inclk0;
   wire [1:0] sub_wire5 = {sub_wire6, sub_wire4};

   altpll   altpll_component (
            .inclk (sub_wire5),
            .clk (sub_wire0),
            .locked (sub_wire2),
            .activeclock (),
            .areset (1'b0),
            .clkbad (),
            .clkena ({6{1'b1}}),
            .clkloss (),
            .clkswitch (1'b0),
            .configupdate (1'b0),
            .enable0 (),
            .enable1 (),
            .extclk (),
            .extclkena ({4{1'b1}}),
            .fbin (1'b1),
            .fbmimicbidir (),
            .fbout (),
            .fref (),
            .icdrclk (),
            .pfdena (1'b1),
            .phasecounterselect ({4{1'b1}}),
            .phasedone (),
            .phasestep (1'b1),
            .phaseupdown (1'b1),
            .pllena (1'b1),
            .scanaclr (1'b0),
            .scanclk (1'b0),
            .scanclkena (1'b1),
            .scandata (1'b0),
            .scandataout (),
            .scandone (),
            .scanread (1'b0),
            .scanwrite (1'b0),
            .sclkout0 (),
            .sclkout1 (),
            .vcooverrange (),
            .vcounderrange ());
   defparam
      altpll_component.bandwidth_type = "AUTO",
      altpll_component.clk0_divide_by = 1,
      altpll_component.clk0_duty_cycle = 50,
      altpll_component.clk0_multiply_by = 2,
      altpll_component.clk0_phase_shift = "0",
      altpll_component.clk1_divide_by = 1,
      altpll_component.clk1_duty_cycle = 50,
      altpll_component.clk1_multiply_by = 2,
      altpll_component.clk1_phase_shift = "5000",
      altpll_component.compensate_clock = "CLK0",
      altpll_component.inclk0_input_frequency = 20000,
      altpll_component.intended_device_family = "Cyclone III",
      altpll_component.lpm_hint = "CBX_MODULE_PREFIX=de0_pll100",
      altpll_component.lpm_type = "altpll",
      altpll_component.operation_mode = "NORMAL",
      altpll_component.pll_type = "AUTO",
      altpll_component.port_activeclock = "PORT_UNUSED",
      altpll_component.port_areset = "PORT_UNUSED",
      altpll_component.port_clkbad0 = "PORT_UNUSED",
      altpll_component.port_clkbad1 = "PORT_UNUSED",
      altpll_component.port_clkloss = "PORT_UNUSED",
      altpll_component.port_clkswitch = "PORT_UNUSED",
      altpll_component.port_configupdate = "PORT_UNUSED",
      altpll_component.port_fbin = "PORT_UNUSED",
      altpll_component.port_inclk0 = "PORT_USED",
      altpll_component.port_inclk1 = "PORT_UNUSED",
      altpll_component.port_locked = "PORT_USED",
      altpll_component.port_pfdena = "PORT_UNUSED",
      altpll_component.port_phasecounterselect = "PORT_UNUSED",
      altpll_component.port_phasedone = "PORT_UNUSED",
      altpll_component.port_phasestep = "PORT_UNUSED",
      altpll_component.port_phaseupdown = "PORT_UNUSED",
      altpll_component.port_pllena = "PORT_UNUSED",
      altpll_component.port_scanaclr = "PORT_UNUSED",
      altpll_component.port_scanclk = "PORT_UNUSED",
      altpll_component.port_scanclkena = "PORT_UNUSED",
      altpll_component.port_scandata = "PORT_UNUSED",
      altpll_component.port_scandataout = "PORT_UNUSED",
      altpll_component.port_scandone = "PORT_UNUSED",
      altpll_component.port_scanread = "PORT_UNUSED",
      altpll_component.port_scanwrite = "PORT_UNUSED",
      altpll_component.port_clk0 = "PORT_USED",
      altpll_component.port_clk1 = "PORT_USED",
      altpll_component.port_clk2 = "PORT_UNUSED",
      altpll_component.port_clk3 = "PORT_UNUSED",
      altpll_component.port_clk4 = "PORT_UNUSED",
      altpll_component.port_clk5 = "PORT_UNUSED",
      altpll_component.port_clkena0 = "PORT_UNUSED",
      altpll_component.port_clkena1 = "PORT_UNUSED",
      altpll_component.port_clkena2 = "PORT_UNUSED",
      altpll_component.port_clkena3 = "PORT_UNUSED",
      altpll_component.port_clkena4 = "PORT_UNUSED",
      altpll_component.port_clkena5 = "PORT_UNUSED",
      altpll_component.port_extclk0 = "PORT_UNUSED",
      altpll_component.port_extclk1 = "PORT_UNUSED",
      altpll_component.port_extclk2 = "PORT_UNUSED",
      altpll_component.port_extclk3 = "PORT_UNUSED",
      altpll_component.self_reset_on_loss_lock = "OFF",
      altpll_component.width_clock = 5;


endmodule
// ============================================================
// CNX file retrieval info
// ============================================================
// Retrieval info: PRIVATE: ACTIVECLK_CHECK STRING "0"
// Retrieval info: PRIVATE: BANDWIDTH STRING "1.000"
// Retrieval info: PRIVATE: BANDWIDTH_FEATURE_ENABLED STRING "1"
// Retrieval info: PRIVATE: BANDWIDTH_FREQ_UNIT STRING "MHz"
// Retrieval info: PRIVATE: BANDWIDTH_PRESET STRING "Low"
// Retrieval info: PRIVATE: BANDWIDTH_USE_AUTO STRING "1"
// Retrieval info: PRIVATE: BANDWIDTH_USE_PRESET STRING "0"
// Retrieval info: PRIVATE: CLKBAD_SWITCHOVER_CHECK STRING "0"
// Retrieval info: PRIVATE: CLKLOSS_CHECK STRING "0"
// Retrieval info: PRIVATE: CLKSWITCH_CHECK STRING "0"
// Retrieval info: PRIVATE: CNX_NO_COMPENSATE_RADIO STRING "0"
// Retrieval info: PRIVATE: CREATE_CLKBAD_CHECK STRING "0"
// Retrieval info: PRIVATE: CREATE_INCLK1_CHECK STRING "0"
// Retrieval info: PRIVATE: CUR_DEDICATED_CLK STRING "c0"
// Retrieval info: PRIVATE: CUR_FBIN_CLK STRING "c0"
// Retrieval info: PRIVATE: DEVICE_SPEED_GRADE STRING "8"
// Retrieval info: PRIVATE: DIV_FACTOR0 NUMERIC "2"
// Retrieval info: PRIVATE: DIV_FACTOR1 NUMERIC "2"
// Retrieval info: PRIVATE: DUTY_CYCLE0 STRING "50.00000000"
// Retrieval info: PRIVATE: DUTY_CYCLE1 STRING "50.00000000"
// Retrieval info: PRIVATE: EFF_OUTPUT_FREQ_VALUE0 STRING "100.000000"
// Retrieval info: PRIVATE: EFF_OUTPUT_FREQ_VALUE1 STRING "100.000000"
// Retrieval info: PRIVATE: EXPLICIT_SWITCHOVER_COUNTER STRING "0"
// Retrieval info: PRIVATE: EXT_FEEDBACK_RADIO STRING "0"
// Retrieval info: PRIVATE: GLOCKED_COUNTER_EDIT_CHANGED STRING "1"
// Retrieval info: PRIVATE: GLOCKED_FEATURE_ENABLED STRING "0"
// Retrieval info: PRIVATE: GLOCKED_MODE_CHECK STRING "0"
// Retrieval info: PRIVATE: GLOCK_COUNTER_EDIT NUMERIC "1048575"
// Retrieval info: PRIVATE: HAS_MANUAL_SWITCHOVER STRING "1"
// Retrieval info: PRIVATE: INCLK0_FREQ_EDIT STRING "50.000"
// Retrieval info: PRIVATE: INCLK0_FREQ_UNIT_COMBO STRING "MHz"
// Retrieval info: PRIVATE: INCLK1_FREQ_EDIT STRING "100.000"
// Retrieval info: PRIVATE: INCLK1_FREQ_EDIT_CHANGED STRING "1"
// Retrieval info: PRIVATE: INCLK1_FREQ_UNIT_CHANGED STRING "1"
// Retrieval info: PRIVATE: INCLK1_FREQ_UNIT_COMBO STRING "MHz"
// Retrieval info: PRIVATE: INTENDED_DEVICE_FAMILY STRING "Cyclone III"
// Retrieval info: PRIVATE: INT_FEEDBACK__MODE_RADIO STRING "1"
// Retrieval info: PRIVATE: LOCKED_OUTPUT_CHECK STRING "1"
// Retrieval info: PRIVATE: LONG_SCAN_RADIO STRING "1"
// Retrieval info: PRIVATE: LVDS_MODE_DATA_RATE STRING "Not Available"
// Retrieval info: PRIVATE: LVDS_MODE_DATA_RATE_DIRTY NUMERIC "0"
// Retrieval info: PRIVATE: LVDS_PHASE_SHIFT_UNIT0 STRING "deg"
// Retrieval info: PRIVATE: LVDS_PHASE_SHIFT_UNIT1 STRING "deg"
// Retrieval info: PRIVATE: MIG_DEVICE_SPEED_GRADE STRING "Any"
// Retrieval info: PRIVATE: MIRROR_CLK0 STRING "0"
// Retrieval info: PRIVATE: MIRROR_CLK1 STRING "0"
// Retrieval info: PRIVATE: MULT_FACTOR0 NUMERIC "4"
// Retrieval info: PRIVATE: MULT_FACTOR1 NUMERIC "4"
// Retrieval info: PRIVATE: NORMAL_MODE_RADIO STRING "1"
// Retrieval info: PRIVATE: OUTPUT_FREQ0 STRING "100.00000000"
// Retrieval info: PRIVATE: OUTPUT_FREQ1 STRING "100.00000000"
// Retrieval info: PRIVATE: OUTPUT_FREQ_MODE0 STRING "0"
// Retrieval info: PRIVATE: OUTPUT_FREQ_MODE1 STRING "0"
// Retrieval info: PRIVATE: OUTPUT_FREQ_UNIT0 STRING "MHz"
// Retrieval info: PRIVATE: OUTPUT_FREQ_UNIT1 STRING "MHz"
// Retrieval info: PRIVATE: PHASE_RECONFIG_FEATURE_ENABLED STRING "1"
// Retrieval info: PRIVATE: PHASE_RECONFIG_INPUTS_CHECK STRING "0"
// Retrieval info: PRIVATE: PHASE_SHIFT0 STRING "0.00000000"
// Retrieval info: PRIVATE: PHASE_SHIFT1 STRING "180.00000000"
// Retrieval info: PRIVATE: PHASE_SHIFT_STEP_ENABLED_CHECK STRING "0"
// Retrieval info: PRIVATE: PHASE_SHIFT_UNIT0 STRING "deg"
// Retrieval info: PRIVATE: PHASE_SHIFT_UNIT1 STRING "deg"
// Retrieval info: PRIVATE: PLL_ADVANCED_PARAM_CHECK STRING "0"
// Retrieval info: PRIVATE: PLL_ARESET_CHECK STRING "0"
// Retrieval info: PRIVATE: PLL_AUTOPLL_CHECK NUMERIC "1"
// Retrieval info: PRIVATE: PLL_ENHPLL_CHECK NUMERIC "0"
// Retrieval info: PRIVATE: PLL_FASTPLL_CHECK NUMERIC "0"
// Retrieval info: PRIVATE: PLL_FBMIMIC_CHECK STRING "0"
// Retrieval info: PRIVATE: PLL_LVDS_PLL_CHECK NUMERIC "0"
// Retrieval info: PRIVATE: PLL_PFDENA_CHECK STRING "0"
// Retrieval info: PRIVATE: PLL_TARGET_HARCOPY_CHECK NUMERIC "0"
// Retrieval info: PRIVATE: PRIMARY_CLK_COMBO STRING "inclk0"
// Retrieval info: PRIVATE: RECONFIG_FILE STRING "de0_pll100.mif"
// Retrieval info: PRIVATE: SACN_INPUTS_CHECK STRING "0"
// Retrieval info: PRIVATE: SCAN_FEATURE_ENABLED STRING "1"
// Retrieval info: PRIVATE: SELF_RESET_LOCK_LOSS STRING "0"
// Retrieval info: PRIVATE: SHORT_SCAN_RADIO STRING "0"
// Retrieval info: PRIVATE: SPREAD_FEATURE_ENABLED STRING "0"
// Retrieval info: PRIVATE: SPREAD_FREQ STRING "50.000"
// Retrieval info: PRIVATE: SPREAD_FREQ_UNIT STRING "KHz"
// Retrieval info: PRIVATE: SPREAD_PERCENT STRING "0.500"
// Retrieval info: PRIVATE: SPREAD_USE STRING "0"
// Retrieval info: PRIVATE: SRC_SYNCH_COMP_RADIO STRING "0"
// Retrieval info: PRIVATE: STICKY_CLK0 STRING "1"
// Retrieval info: PRIVATE: STICKY_CLK1 STRING "1"
// Retrieval info: PRIVATE: SWITCHOVER_COUNT_EDIT NUMERIC "1"
// Retrieval info: PRIVATE: SWITCHOVER_FEATURE_ENABLED STRING "1"
// Retrieval info: PRIVATE: SYNTH_WRAPPER_GEN_POSTFIX STRING "0"
// Retrieval info: PRIVATE: USE_CLK0 STRING "1"
// Retrieval info: PRIVATE: USE_CLK1 STRING "1"
// Retrieval info: PRIVATE: USE_CLKENA0 STRING "0"
// Retrieval info: PRIVATE: USE_CLKENA1 STRING "0"
// Retrieval info: PRIVATE: USE_MIL_SPEED_GRADE NUMERIC "0"
// Retrieval info: PRIVATE: ZERO_DELAY_RADIO STRING "0"
// Retrieval info: LIBRARY: altera_mf altera_mf.altera_mf_components.all
// Retrieval info: CONSTANT: BANDWIDTH_TYPE STRING "AUTO"
// Retrieval info: CONSTANT: CLK0_DIVIDE_BY NUMERIC "1"
// Retrieval info: CONSTANT: CLK0_DUTY_CYCLE NUMERIC "50"
// Retrieval info: CONSTANT: CLK0_MULTIPLY_BY NUMERIC "2"
// Retrieval info: CONSTANT: CLK0_PHASE_SHIFT STRING "0"
// Retrieval info: CONSTANT: CLK1_DIVIDE_BY NUMERIC "1"
// Retrieval info: CONSTANT: CLK1_DUTY_CYCLE NUMERIC "50"
// Retrieval info: CONSTANT: CLK1_MULTIPLY_BY NUMERIC "2"
// Retrieval info: CONSTANT: CLK1_PHASE_SHIFT STRING "5000"
// Retrieval info: CONSTANT: COMPENSATE_CLOCK STRING "CLK0"
// Retrieval info: CONSTANT: INCLK0_INPUT_FREQUENCY NUMERIC "20000"
// Retrieval info: CONSTANT: INTENDED_DEVICE_FAMILY STRING "Cyclone III"
// Retrieval info: CONSTANT: LPM_TYPE STRING "altpll"
// Retrieval info: CONSTANT: OPERATION_MODE STRING "NORMAL"
// Retrieval info: CONSTANT: PLL_TYPE STRING "AUTO"
// Retrieval info: CONSTANT: PORT_ACTIVECLOCK STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_ARESET STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_CLKBAD0 STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_CLKBAD1 STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_CLKLOSS STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_CLKSWITCH STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_CONFIGUPDATE STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_FBIN STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_INCLK0 STRING "PORT_USED"
// Retrieval info: CONSTANT: PORT_INCLK1 STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_LOCKED STRING "PORT_USED"
// Retrieval info: CONSTANT: PORT_PFDENA STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_PHASECOUNTERSELECT STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_PHASEDONE STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_PHASESTEP STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_PHASEUPDOWN STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_PLLENA STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_SCANACLR STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_SCANCLK STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_SCANCLKENA STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_SCANDATA STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_SCANDATAOUT STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_SCANDONE STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_SCANREAD STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_SCANWRITE STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_clk0 STRING "PORT_USED"
// Retrieval info: CONSTANT: PORT_clk1 STRING "PORT_USED"
// Retrieval info: CONSTANT: PORT_clk2 STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_clk3 STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_clk4 STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_clk5 STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_clkena0 STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_clkena1 STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_clkena2 STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_clkena3 STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_clkena4 STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_clkena5 STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_extclk0 STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_extclk1 STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_extclk2 STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_extclk3 STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: SELF_RESET_ON_LOSS_LOCK STRING "OFF"
// Retrieval info: CONSTANT: WIDTH_CLOCK NUMERIC "5"
// Retrieval info: USED_PORT: @clk 0 0 5 0 OUTPUT_CLK_EXT VCC "@clk[4..0]"
// Retrieval info: USED_PORT: c0 0 0 0 0 OUTPUT_CLK_EXT VCC "c0"
// Retrieval info: USED_PORT: c1 0 0 0 0 OUTPUT_CLK_EXT VCC "c1"
// Retrieval info: USED_PORT: inclk0 0 0 0 0 INPUT_CLK_EXT GND "inclk0"
// Retrieval info: USED_PORT: locked 0 0 0 0 OUTPUT GND "locked"
// Retrieval info: CONNECT: @inclk 0 0 1 1 GND 0 0 0 0
// Retrieval info: CONNECT: @inclk 0 0 1 0 inclk0 0 0 0 0
// Retrieval info: CONNECT: c0 0 0 0 0 @clk 0 0 1 0
// Retrieval info: CONNECT: c1 0 0 0 0 @clk 0 0 1 1
// Retrieval info: CONNECT: locked 0 0 0 0 @locked 0 0 0 0
// Retrieval info: GEN_FILE: TYPE_NORMAL de0_pll100.v TRUE
// Retrieval info: GEN_FILE: TYPE_NORMAL de0_pll100.ppf TRUE
// Retrieval info: GEN_FILE: TYPE_NORMAL de0_pll100.inc FALSE
// Retrieval info: GEN_FILE: TYPE_NORMAL de0_pll100.cmp FALSE
// Retrieval info: GEN_FILE: TYPE_NORMAL de0_pll100.bsf FALSE
// Retrieval info: GEN_FILE: TYPE_NORMAL de0_pll100_inst.v FALSE
// Retrieval info: GEN_FILE: TYPE_NORMAL de0_pll100_bb.v TRUE
// Retrieval info: LIB_FILE: altera_mf
// Retrieval info: CBX_MODULE_PREFIX: ON

// synopsys translate_off
`timescale 1 ps / 1 ps
// synopsys translate_on
module de0_pll_zbd(
   inclk0,
   c0,
   c1,
   locked);

   input   inclk0;
   output     c0;
   output     c1;
   output     locked;

   wire [4:0] sub_wire0;
   wire  sub_wire2;
   wire [0:0] sub_wire6 = 1'h0;
   wire [0:0] sub_wire3 = sub_wire0[0:0];
   wire [1:1] sub_wire1 = sub_wire0[1:1];
   wire  c1 = sub_wire1;
   wire  locked = sub_wire2;
   wire  c0 = sub_wire3;
   wire  sub_wire4 = inclk0;
   wire [1:0] sub_wire5 = {sub_wire6, sub_wire4};

   altpll   altpll_component (
            .inclk (sub_wire5),
            .clk (sub_wire0),
            .locked (sub_wire2),
            .activeclock (),
            .areset (1'b0),
            .clkbad (),
            .clkena ({6{1'b1}}),
            .clkloss (),
            .clkswitch (1'b0),
            .configupdate (1'b0),
            .enable0 (),
            .enable1 (),
            .extclk (),
            .extclkena ({4{1'b1}}),
            .fbin (1'b1),
            .fbmimicbidir (),
            .fbout (),
            .fref (),
            .icdrclk (),
            .pfdena (1'b1),
            .phasecounterselect ({4{1'b1}}),
            .phasedone (),
            .phasestep (1'b1),
            .phaseupdown (1'b1),
            .pllena (1'b1),
            .scanaclr (1'b0),
            .scanclk (1'b0),
            .scanclkena (1'b1),
            .scandata (1'b0),
            .scandataout (),
            .scandone (),
            .scanread (1'b0),
            .scanwrite (1'b0),
            .sclkout0 (),
            .sclkout1 (),
            .vcooverrange (),
            .vcounderrange ());
   defparam
      altpll_component.bandwidth_type = "AUTO",
      altpll_component.clk0_divide_by = 1,
      altpll_component.clk0_duty_cycle = 50,
      altpll_component.clk0_multiply_by = 2,
      altpll_component.clk0_phase_shift = "0",
      altpll_component.clk1_divide_by = 1,
      altpll_component.clk1_duty_cycle = 50,
      altpll_component.clk1_multiply_by = 2,
      altpll_component.clk1_phase_shift = "5000",
      altpll_component.compensate_clock = "CLK0",
      altpll_component.inclk0_input_frequency = 20000,
      altpll_component.intended_device_family = "Cyclone III",
      altpll_component.lpm_hint = "CBX_MODULE_PREFIX=a",
      altpll_component.lpm_type = "altpll",
      altpll_component.operation_mode = "ZERO_DELAY_BUFFER",
      altpll_component.pll_type = "AUTO",
      altpll_component.port_activeclock = "PORT_UNUSED",
      altpll_component.port_areset = "PORT_UNUSED",
      altpll_component.port_clkbad0 = "PORT_UNUSED",
      altpll_component.port_clkbad1 = "PORT_UNUSED",
      altpll_component.port_clkloss = "PORT_UNUSED",
      altpll_component.port_clkswitch = "PORT_UNUSED",
      altpll_component.port_configupdate = "PORT_UNUSED",
      altpll_component.port_fbin = "PORT_UNUSED",
      altpll_component.port_inclk0 = "PORT_USED",
      altpll_component.port_inclk1 = "PORT_UNUSED",
      altpll_component.port_locked = "PORT_USED",
      altpll_component.port_pfdena = "PORT_UNUSED",
      altpll_component.port_phasecounterselect = "PORT_UNUSED",
      altpll_component.port_phasedone = "PORT_UNUSED",
      altpll_component.port_phasestep = "PORT_UNUSED",
      altpll_component.port_phaseupdown = "PORT_UNUSED",
      altpll_component.port_pllena = "PORT_UNUSED",
      altpll_component.port_scanaclr = "PORT_UNUSED",
      altpll_component.port_scanclk = "PORT_UNUSED",
      altpll_component.port_scanclkena = "PORT_UNUSED",
      altpll_component.port_scandata = "PORT_UNUSED",
      altpll_component.port_scandataout = "PORT_UNUSED",
      altpll_component.port_scandone = "PORT_UNUSED",
      altpll_component.port_scanread = "PORT_UNUSED",
      altpll_component.port_scanwrite = "PORT_UNUSED",
      altpll_component.port_clk0 = "PORT_USED",
      altpll_component.port_clk1 = "PORT_USED",
      altpll_component.port_clk2 = "PORT_UNUSED",
      altpll_component.port_clk3 = "PORT_UNUSED",
      altpll_component.port_clk4 = "PORT_UNUSED",
      altpll_component.port_clk5 = "PORT_UNUSED",
      altpll_component.port_clkena0 = "PORT_UNUSED",
      altpll_component.port_clkena1 = "PORT_UNUSED",
      altpll_component.port_clkena2 = "PORT_UNUSED",
      altpll_component.port_clkena3 = "PORT_UNUSED",
      altpll_component.port_clkena4 = "PORT_UNUSED",
      altpll_component.port_clkena5 = "PORT_UNUSED",
      altpll_component.port_extclk0 = "PORT_UNUSED",
      altpll_component.port_extclk1 = "PORT_UNUSED",
      altpll_component.port_extclk2 = "PORT_UNUSED",
      altpll_component.port_extclk3 = "PORT_UNUSED",
      altpll_component.self_reset_on_loss_lock = "ON",
      altpll_component.width_clock = 5;


endmodule

// ============================================================
// CNX file retrieval info
// ============================================================
// Retrieval info: PRIVATE: ACTIVECLK_CHECK STRING "0"
// Retrieval info: PRIVATE: BANDWIDTH STRING "1.000"
// Retrieval info: PRIVATE: BANDWIDTH_FEATURE_ENABLED STRING "1"
// Retrieval info: PRIVATE: BANDWIDTH_FREQ_UNIT STRING "MHz"
// Retrieval info: PRIVATE: BANDWIDTH_PRESET STRING "Low"
// Retrieval info: PRIVATE: BANDWIDTH_USE_AUTO STRING "1"
// Retrieval info: PRIVATE: BANDWIDTH_USE_PRESET STRING "0"
// Retrieval info: PRIVATE: CLKBAD_SWITCHOVER_CHECK STRING "0"
// Retrieval info: PRIVATE: CLKLOSS_CHECK STRING "0"
// Retrieval info: PRIVATE: CLKSWITCH_CHECK STRING "0"
// Retrieval info: PRIVATE: CNX_NO_COMPENSATE_RADIO STRING "0"
// Retrieval info: PRIVATE: CREATE_CLKBAD_CHECK STRING "0"
// Retrieval info: PRIVATE: CREATE_INCLK1_CHECK STRING "0"
// Retrieval info: PRIVATE: CUR_DEDICATED_CLK STRING "c0"
// Retrieval info: PRIVATE: CUR_FBIN_CLK STRING "c0"
// Retrieval info: PRIVATE: DEVICE_SPEED_GRADE STRING "6"
// Retrieval info: PRIVATE: DIV_FACTOR0 NUMERIC "2"
// Retrieval info: PRIVATE: DIV_FACTOR1 NUMERIC "2"
// Retrieval info: PRIVATE: DUTY_CYCLE0 STRING "50.00000000"
// Retrieval info: PRIVATE: DUTY_CYCLE1 STRING "50.00000000"
// Retrieval info: PRIVATE: EFF_OUTPUT_FREQ_VALUE0 STRING "100.000000"
// Retrieval info: PRIVATE: EFF_OUTPUT_FREQ_VALUE1 STRING "100.000000"
// Retrieval info: PRIVATE: EXPLICIT_SWITCHOVER_COUNTER STRING "0"
// Retrieval info: PRIVATE: EXT_FEEDBACK_RADIO STRING "0"
// Retrieval info: PRIVATE: GLOCKED_COUNTER_EDIT_CHANGED STRING "1"
// Retrieval info: PRIVATE: GLOCKED_FEATURE_ENABLED STRING "0"
// Retrieval info: PRIVATE: GLOCKED_MODE_CHECK STRING "0"
// Retrieval info: PRIVATE: GLOCK_COUNTER_EDIT NUMERIC "1048575"
// Retrieval info: PRIVATE: HAS_MANUAL_SWITCHOVER STRING "1"
// Retrieval info: PRIVATE: INCLK0_FREQ_EDIT STRING "50.000"
// Retrieval info: PRIVATE: INCLK0_FREQ_UNIT_COMBO STRING "MHz"
// Retrieval info: PRIVATE: INCLK1_FREQ_EDIT STRING "100.000"
// Retrieval info: PRIVATE: INCLK1_FREQ_EDIT_CHANGED STRING "1"
// Retrieval info: PRIVATE: INCLK1_FREQ_UNIT_CHANGED STRING "1"
// Retrieval info: PRIVATE: INCLK1_FREQ_UNIT_COMBO STRING "MHz"
// Retrieval info: PRIVATE: INTENDED_DEVICE_FAMILY STRING "Cyclone III"
// Retrieval info: PRIVATE: INT_FEEDBACK__MODE_RADIO STRING "1"
// Retrieval info: PRIVATE: LOCKED_OUTPUT_CHECK STRING "1"
// Retrieval info: PRIVATE: LONG_SCAN_RADIO STRING "1"
// Retrieval info: PRIVATE: LVDS_MODE_DATA_RATE STRING "Not Available"
// Retrieval info: PRIVATE: LVDS_MODE_DATA_RATE_DIRTY NUMERIC "0"
// Retrieval info: PRIVATE: LVDS_PHASE_SHIFT_UNIT0 STRING "deg"
// Retrieval info: PRIVATE: LVDS_PHASE_SHIFT_UNIT1 STRING "deg"
// Retrieval info: PRIVATE: MIG_DEVICE_SPEED_GRADE STRING "Any"
// Retrieval info: PRIVATE: MIRROR_CLK0 STRING "0"
// Retrieval info: PRIVATE: MIRROR_CLK1 STRING "0"
// Retrieval info: PRIVATE: MULT_FACTOR0 NUMERIC "4"
// Retrieval info: PRIVATE: MULT_FACTOR1 NUMERIC "4"
// Retrieval info: PRIVATE: NORMAL_MODE_RADIO STRING "0"
// Retrieval info: PRIVATE: OUTPUT_FREQ0 STRING "100.00000000"
// Retrieval info: PRIVATE: OUTPUT_FREQ1 STRING "100.00000000"
// Retrieval info: PRIVATE: OUTPUT_FREQ_MODE0 STRING "0"
// Retrieval info: PRIVATE: OUTPUT_FREQ_MODE1 STRING "0"
// Retrieval info: PRIVATE: OUTPUT_FREQ_UNIT0 STRING "MHz"
// Retrieval info: PRIVATE: OUTPUT_FREQ_UNIT1 STRING "MHz"
// Retrieval info: PRIVATE: PHASE_RECONFIG_FEATURE_ENABLED STRING "1"
// Retrieval info: PRIVATE: PHASE_RECONFIG_INPUTS_CHECK STRING "0"
// Retrieval info: PRIVATE: PHASE_SHIFT0 STRING "0.00000000"
// Retrieval info: PRIVATE: PHASE_SHIFT1 STRING "180.00000000"
// Retrieval info: PRIVATE: PHASE_SHIFT_STEP_ENABLED_CHECK STRING "0"
// Retrieval info: PRIVATE: PHASE_SHIFT_UNIT0 STRING "deg"
// Retrieval info: PRIVATE: PHASE_SHIFT_UNIT1 STRING "deg"
// Retrieval info: PRIVATE: PLL_ADVANCED_PARAM_CHECK STRING "0"
// Retrieval info: PRIVATE: PLL_ARESET_CHECK STRING "0"
// Retrieval info: PRIVATE: PLL_AUTOPLL_CHECK NUMERIC "1"
// Retrieval info: PRIVATE: PLL_ENHPLL_CHECK NUMERIC "0"
// Retrieval info: PRIVATE: PLL_FASTPLL_CHECK NUMERIC "0"
// Retrieval info: PRIVATE: PLL_FBMIMIC_CHECK STRING "0"
// Retrieval info: PRIVATE: PLL_LVDS_PLL_CHECK NUMERIC "0"
// Retrieval info: PRIVATE: PLL_PFDENA_CHECK STRING "0"
// Retrieval info: PRIVATE: PLL_TARGET_HARCOPY_CHECK NUMERIC "0"
// Retrieval info: PRIVATE: PRIMARY_CLK_COMBO STRING "inclk0"
// Retrieval info: PRIVATE: RECONFIG_FILE STRING "de0_pll_zbd.mif"
// Retrieval info: PRIVATE: SACN_INPUTS_CHECK STRING "0"
// Retrieval info: PRIVATE: SCAN_FEATURE_ENABLED STRING "1"
// Retrieval info: PRIVATE: SELF_RESET_LOCK_LOSS STRING "1"
// Retrieval info: PRIVATE: SHORT_SCAN_RADIO STRING "0"
// Retrieval info: PRIVATE: SPREAD_FEATURE_ENABLED STRING "0"
// Retrieval info: PRIVATE: SPREAD_FREQ STRING "50.000"
// Retrieval info: PRIVATE: SPREAD_FREQ_UNIT STRING "KHz"
// Retrieval info: PRIVATE: SPREAD_PERCENT STRING "0.500"
// Retrieval info: PRIVATE: SPREAD_USE STRING "0"
// Retrieval info: PRIVATE: SRC_SYNCH_COMP_RADIO STRING "0"
// Retrieval info: PRIVATE: STICKY_CLK0 STRING "1"
// Retrieval info: PRIVATE: STICKY_CLK1 STRING "1"
// Retrieval info: PRIVATE: SWITCHOVER_COUNT_EDIT NUMERIC "1"
// Retrieval info: PRIVATE: SWITCHOVER_FEATURE_ENABLED STRING "1"
// Retrieval info: PRIVATE: SYNTH_WRAPPER_GEN_POSTFIX STRING "0"
// Retrieval info: PRIVATE: USE_CLK0 STRING "1"
// Retrieval info: PRIVATE: USE_CLK1 STRING "1"
// Retrieval info: PRIVATE: USE_CLKENA0 STRING "0"
// Retrieval info: PRIVATE: USE_CLKENA1 STRING "0"
// Retrieval info: PRIVATE: USE_MIL_SPEED_GRADE NUMERIC "0"
// Retrieval info: PRIVATE: ZERO_DELAY_RADIO STRING "1"
// Retrieval info: LIBRARY: altera_mf altera_mf.altera_mf_components.all
// Retrieval info: CONSTANT: BANDWIDTH_TYPE STRING "AUTO"
// Retrieval info: CONSTANT: CLK0_DIVIDE_BY NUMERIC "1"
// Retrieval info: CONSTANT: CLK0_DUTY_CYCLE NUMERIC "50"
// Retrieval info: CONSTANT: CLK0_MULTIPLY_BY NUMERIC "2"
// Retrieval info: CONSTANT: CLK0_PHASE_SHIFT STRING "0"
// Retrieval info: CONSTANT: CLK1_DIVIDE_BY NUMERIC "1"
// Retrieval info: CONSTANT: CLK1_DUTY_CYCLE NUMERIC "50"
// Retrieval info: CONSTANT: CLK1_MULTIPLY_BY NUMERIC "2"
// Retrieval info: CONSTANT: CLK1_PHASE_SHIFT STRING "5000"
// Retrieval info: CONSTANT: COMPENSATE_CLOCK STRING "CLK0"
// Retrieval info: CONSTANT: INCLK0_INPUT_FREQUENCY NUMERIC "20000"
// Retrieval info: CONSTANT: INTENDED_DEVICE_FAMILY STRING "Cyclone III"
// Retrieval info: CONSTANT: LPM_TYPE STRING "altpll"
// Retrieval info: CONSTANT: OPERATION_MODE STRING "ZERO_DELAY_BUFFER"
// Retrieval info: CONSTANT: PLL_TYPE STRING "AUTO"
// Retrieval info: CONSTANT: PORT_ACTIVECLOCK STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_ARESET STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_CLKBAD0 STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_CLKBAD1 STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_CLKLOSS STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_CLKSWITCH STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_CONFIGUPDATE STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_FBIN STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_INCLK0 STRING "PORT_USED"
// Retrieval info: CONSTANT: PORT_INCLK1 STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_LOCKED STRING "PORT_USED"
// Retrieval info: CONSTANT: PORT_PFDENA STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_PHASECOUNTERSELECT STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_PHASEDONE STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_PHASESTEP STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_PHASEUPDOWN STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_PLLENA STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_SCANACLR STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_SCANCLK STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_SCANCLKENA STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_SCANDATA STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_SCANDATAOUT STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_SCANDONE STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_SCANREAD STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_SCANWRITE STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_clk0 STRING "PORT_USED"
// Retrieval info: CONSTANT: PORT_clk1 STRING "PORT_USED"
// Retrieval info: CONSTANT: PORT_clk2 STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_clk3 STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_clk4 STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_clk5 STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_clkena0 STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_clkena1 STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_clkena2 STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_clkena3 STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_clkena4 STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_clkena5 STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_extclk0 STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_extclk1 STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_extclk2 STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: PORT_extclk3 STRING "PORT_UNUSED"
// Retrieval info: CONSTANT: SELF_RESET_ON_LOSS_LOCK STRING "ON"
// Retrieval info: CONSTANT: WIDTH_CLOCK NUMERIC "5"
// Retrieval info: USED_PORT: @clk 0 0 5 0 OUTPUT_CLK_EXT VCC "@clk[4..0]"
// Retrieval info: USED_PORT: c0 0 0 0 0 OUTPUT_CLK_EXT VCC "c0"
// Retrieval info: USED_PORT: c1 0 0 0 0 OUTPUT_CLK_EXT VCC "c1"
// Retrieval info: USED_PORT: inclk0 0 0 0 0 INPUT_CLK_EXT GND "inclk0"
// Retrieval info: USED_PORT: locked 0 0 0 0 OUTPUT GND "locked"
// Retrieval info: CONNECT: @inclk 0 0 1 1 GND 0 0 0 0
// Retrieval info: CONNECT: @inclk 0 0 1 0 inclk0 0 0 0 0
// Retrieval info: CONNECT: c0 0 0 0 0 @clk 0 0 1 0
// Retrieval info: CONNECT: c1 0 0 0 0 @clk 0 0 1 1
// Retrieval info: CONNECT: locked 0 0 0 0 @locked 0 0 0 0
// Retrieval info: GEN_FILE: TYPE_NORMAL de0_pll_zbd.v TRUE
// Retrieval info: GEN_FILE: TYPE_NORMAL de0_pll_zbd.ppf TRUE
// Retrieval info: GEN_FILE: TYPE_NORMAL de0_pll_zbd.inc FALSE
// Retrieval info: GEN_FILE: TYPE_NORMAL de0_pll_zbd.cmp FALSE
// Retrieval info: GEN_FILE: TYPE_NORMAL de0_pll_zbd.bsf FALSE
// Retrieval info: GEN_FILE: TYPE_NORMAL de0_pll_zbd_inst.v FALSE
// Retrieval info: GEN_FILE: TYPE_NORMAL de0_pll_zbd_bb.v TRUE
// Retrieval info: LIB_FILE: altera_mf
// Retrieval info: CBX_MODULE_PREFIX: ON

// synopsys translate_off
`timescale 1 ps / 1 ps
// synopsys translate_on
module de0_ram8k (
   address,
   byteena,
   clock,
   data,
   rden,
   wren,
   q);

   input [11:0]  address;
   input [1:0]  byteena;
   input   clock;
   input [15:0]  data;
   input   rden;
   input   wren;
   output   [15:0]  q;
`ifndef ALTERA_RESERVED_QIS
// synopsys translate_off
`endif
   tri1  [1:0]  byteena;
   tri1    clock;
   tri1    rden;
`ifndef ALTERA_RESERVED_QIS
// synopsys translate_on
`endif

   wire [15:0] sub_wire0;
   wire [15:0] q = sub_wire0[15:0];

   altsyncram  altsyncram_component (
            .address_a (address),
            .byteena_a (byteena),
            .clock0 (clock),
            .data_a (data),
            .wren_a (wren),
            .rden_a (rden),
            .q_a (sub_wire0),
            .aclr0 (1'b0),
            .aclr1 (1'b0),
            .address_b (1'b1),
            .addressstall_a (1'b0),
            .addressstall_b (1'b0),
            .byteena_b (1'b1),
            .clock1 (1'b1),
            .clocken0 (1'b1),
            .clocken1 (1'b1),
            .clocken2 (1'b1),
            .clocken3 (1'b1),
            .data_b (1'b1),
            .eccstatus (),
            .q_b (),
            .rden_b (1'b1),
            .wren_b (1'b0));
   defparam
      altsyncram_component.byte_size = 8,
      altsyncram_component.clock_enable_input_a = "BYPASS",
      altsyncram_component.clock_enable_output_a = "BYPASS",
      altsyncram_component.init_file = "./rtl/de0_test.mif",
      altsyncram_component.intended_device_family = "Cyclone III",
      altsyncram_component.lpm_hint = "ENABLE_RUNTIME_MOD=NO",
      altsyncram_component.lpm_type = "altsyncram",
      altsyncram_component.numwords_a = 4096,
      altsyncram_component.operation_mode = "SINGLE_PORT",
      altsyncram_component.outdata_aclr_a = "NONE",
      altsyncram_component.outdata_reg_a = "CLOCK0",
      altsyncram_component.power_up_uninitialized = "FALSE",
      altsyncram_component.ram_block_type = "M9K",
      altsyncram_component.read_during_write_mode_port_a = "DONT_CARE",
      altsyncram_component.widthad_a = 12,
      altsyncram_component.width_a = 16,
      altsyncram_component.width_byteena_a = 2;


endmodule

// ============================================================
// CNX file retrieval info
// ============================================================
// Retrieval info: PRIVATE: ADDRESSSTALL_A NUMERIC "0"
// Retrieval info: PRIVATE: AclrAddr NUMERIC "0"
// Retrieval info: PRIVATE: AclrByte NUMERIC "0"
// Retrieval info: PRIVATE: AclrData NUMERIC "0"
// Retrieval info: PRIVATE: AclrOutput NUMERIC "0"
// Retrieval info: PRIVATE: BYTE_ENABLE NUMERIC "1"
// Retrieval info: PRIVATE: BYTE_SIZE NUMERIC "8"
// Retrieval info: PRIVATE: BlankMemory NUMERIC "0"
// Retrieval info: PRIVATE: CLOCK_ENABLE_INPUT_A NUMERIC "0"
// Retrieval info: PRIVATE: CLOCK_ENABLE_OUTPUT_A NUMERIC "0"
// Retrieval info: PRIVATE: Clken NUMERIC "0"
// Retrieval info: PRIVATE: DataBusSeparated NUMERIC "1"
// Retrieval info: PRIVATE: IMPLEMENT_IN_LES NUMERIC "0"
// Retrieval info: PRIVATE: INIT_FILE_LAYOUT STRING "PORT_A"
// Retrieval info: PRIVATE: INIT_TO_SIM_X NUMERIC "0"
// Retrieval info: PRIVATE: INTENDED_DEVICE_FAMILY STRING "Cyclone III"
// Retrieval info: PRIVATE: JTAG_ENABLED NUMERIC "0"
// Retrieval info: PRIVATE: JTAG_ID STRING "NONE"
// Retrieval info: PRIVATE: MAXIMUM_DEPTH NUMERIC "0"
// Retrieval info: PRIVATE: MIFfilename STRING "./rtl/de0_test.mif"
// Retrieval info: PRIVATE: NUMWORDS_A NUMERIC "4096"
// Retrieval info: PRIVATE: RAM_BLOCK_TYPE NUMERIC "2"
// Retrieval info: PRIVATE: READ_DURING_WRITE_MODE_PORT_A NUMERIC "2"
// Retrieval info: PRIVATE: RegAddr NUMERIC "1"
// Retrieval info: PRIVATE: RegData NUMERIC "1"
// Retrieval info: PRIVATE: RegOutput NUMERIC "1"
// Retrieval info: PRIVATE: SYNTH_WRAPPER_GEN_POSTFIX STRING "0"
// Retrieval info: PRIVATE: SingleClock NUMERIC "1"
// Retrieval info: PRIVATE: UseDQRAM NUMERIC "1"
// Retrieval info: PRIVATE: WRCONTROL_ACLR_A NUMERIC "0"
// Retrieval info: PRIVATE: WidthAddr NUMERIC "12"
// Retrieval info: PRIVATE: WidthData NUMERIC "16"
// Retrieval info: PRIVATE: rden NUMERIC "1"
// Retrieval info: LIBRARY: altera_mf altera_mf.altera_mf_components.all
// Retrieval info: CONSTANT: BYTE_SIZE NUMERIC "8"
// Retrieval info: CONSTANT: CLOCK_ENABLE_INPUT_A STRING "BYPASS"
// Retrieval info: CONSTANT: CLOCK_ENABLE_OUTPUT_A STRING "BYPASS"
// Retrieval info: CONSTANT: INIT_FILE STRING "./rtl/de0_test.mif"
// Retrieval info: CONSTANT: INTENDED_DEVICE_FAMILY STRING "Cyclone III"
// Retrieval info: CONSTANT: LPM_HINT STRING "ENABLE_RUNTIME_MOD=NO"
// Retrieval info: CONSTANT: LPM_TYPE STRING "altsyncram"
// Retrieval info: CONSTANT: NUMWORDS_A NUMERIC "4096"
// Retrieval info: CONSTANT: OPERATION_MODE STRING "SINGLE_PORT"
// Retrieval info: CONSTANT: OUTDATA_ACLR_A STRING "NONE"
// Retrieval info: CONSTANT: OUTDATA_REG_A STRING "CLOCK0"
// Retrieval info: CONSTANT: POWER_UP_UNINITIALIZED STRING "FALSE"
// Retrieval info: CONSTANT: RAM_BLOCK_TYPE STRING "M9K"
// Retrieval info: CONSTANT: READ_DURING_WRITE_MODE_PORT_A STRING "DONT_CARE"
// Retrieval info: CONSTANT: WIDTHAD_A NUMERIC "12"
// Retrieval info: CONSTANT: WIDTH_A NUMERIC "16"
// Retrieval info: CONSTANT: WIDTH_BYTEENA_A NUMERIC "2"
// Retrieval info: USED_PORT: address 0 0 12 0 INPUT NODEFVAL "address[11..0]"
// Retrieval info: USED_PORT: byteena 0 0 2 0 INPUT VCC "byteena[1..0]"
// Retrieval info: USED_PORT: clock 0 0 0 0 INPUT VCC "clock"
// Retrieval info: USED_PORT: data 0 0 16 0 INPUT NODEFVAL "data[15..0]"
// Retrieval info: USED_PORT: q 0 0 16 0 OUTPUT NODEFVAL "q[15..0]"
// Retrieval info: USED_PORT: rden 0 0 0 0 INPUT VCC "rden"
// Retrieval info: USED_PORT: wren 0 0 0 0 INPUT NODEFVAL "wren"
// Retrieval info: CONNECT: @address_a 0 0 12 0 address 0 0 12 0
// Retrieval info: CONNECT: @byteena_a 0 0 2 0 byteena 0 0 2 0
// Retrieval info: CONNECT: @clock0 0 0 0 0 clock 0 0 0 0
// Retrieval info: CONNECT: @data_a 0 0 16 0 data 0 0 16 0
// Retrieval info: CONNECT: @rden_a 0 0 0 0 rden 0 0 0 0
// Retrieval info: CONNECT: @wren_a 0 0 0 0 wren 0 0 0 0
// Retrieval info: CONNECT: q 0 0 16 0 @q_a 0 0 16 0
// Retrieval info: GEN_FILE: TYPE_NORMAL de0_ram8k.v TRUE
// Retrieval info: GEN_FILE: TYPE_NORMAL de0_ram8k.inc FALSE
// Retrieval info: GEN_FILE: TYPE_NORMAL de0_ram8k.cmp FALSE
// Retrieval info: GEN_FILE: TYPE_NORMAL de0_ram8k.bsf FALSE
// Retrieval info: GEN_FILE: TYPE_NORMAL de0_ram8k_inst.v FALSE
// Retrieval info: GEN_FILE: TYPE_NORMAL de0_ram8k_bb.v TRUE
// Retrieval info: LIB_FILE: altera_mf

// synopsys translate_off
`timescale 1 ps / 1 ps
// synopsys translate_on
module de0_ram16k (
   address,
   byteena,
   clock,
   data,
   rden,
   wren,
   q);

   input [12:0]  address;
   input [1:0]  byteena;
   input   clock;
   input [15:0]  data;
   input   rden;
   input   wren;
   output   [15:0]  q;
`ifndef ALTERA_RESERVED_QIS
// synopsys translate_off
`endif
   tri1  [1:0]  byteena;
   tri1    clock;
   tri1    rden;
`ifndef ALTERA_RESERVED_QIS
// synopsys translate_on
`endif

   wire [15:0] sub_wire0;
   wire [15:0] q = sub_wire0[15:0];

   altsyncram  altsyncram_component (
            .address_a (address),
            .byteena_a (byteena),
            .clock0 (clock),
            .data_a (data),
            .wren_a (wren),
            .rden_a (rden),
            .q_a (sub_wire0),
            .aclr0 (1'b0),
            .aclr1 (1'b0),
            .address_b (1'b1),
            .addressstall_a (1'b0),
            .addressstall_b (1'b0),
            .byteena_b (1'b1),
            .clock1 (1'b1),
            .clocken0 (1'b1),
            .clocken1 (1'b1),
            .clocken2 (1'b1),
            .clocken3 (1'b1),
            .data_b (1'b1),
            .eccstatus (),
            .q_b (),
            .rden_b (1'b1),
            .wren_b (1'b0));
   defparam
      altsyncram_component.byte_size = 8,
      altsyncram_component.clock_enable_input_a = "BYPASS",
      altsyncram_component.clock_enable_output_a = "BYPASS",
      altsyncram_component.init_file = "./rtl/de0_test.mif",
      altsyncram_component.intended_device_family = "Cyclone III",
      altsyncram_component.lpm_hint = "ENABLE_RUNTIME_MOD=NO",
      altsyncram_component.lpm_type = "altsyncram",
      altsyncram_component.numwords_a = 8192,
      altsyncram_component.operation_mode = "SINGLE_PORT",
      altsyncram_component.outdata_aclr_a = "NONE",
      altsyncram_component.outdata_reg_a = "UNREGISTERED",
      altsyncram_component.power_up_uninitialized = "FALSE",
      altsyncram_component.ram_block_type = "M9K",
      altsyncram_component.read_during_write_mode_port_a = "NEW_DATA_NO_NBE_READ",
      altsyncram_component.widthad_a = 13,
      altsyncram_component.width_a = 16,
      altsyncram_component.width_byteena_a = 2;


endmodule

// ============================================================
// CNX file retrieval info
// ============================================================
// Retrieval info: PRIVATE: ADDRESSSTALL_A NUMERIC "0"
// Retrieval info: PRIVATE: AclrAddr NUMERIC "0"
// Retrieval info: PRIVATE: AclrByte NUMERIC "0"
// Retrieval info: PRIVATE: AclrData NUMERIC "0"
// Retrieval info: PRIVATE: AclrOutput NUMERIC "0"
// Retrieval info: PRIVATE: BYTE_ENABLE NUMERIC "1"
// Retrieval info: PRIVATE: BYTE_SIZE NUMERIC "8"
// Retrieval info: PRIVATE: BlankMemory NUMERIC "0"
// Retrieval info: PRIVATE: CLOCK_ENABLE_INPUT_A NUMERIC "0"
// Retrieval info: PRIVATE: CLOCK_ENABLE_OUTPUT_A NUMERIC "0"
// Retrieval info: PRIVATE: Clken NUMERIC "0"
// Retrieval info: PRIVATE: DataBusSeparated NUMERIC "1"
// Retrieval info: PRIVATE: IMPLEMENT_IN_LES NUMERIC "0"
// Retrieval info: PRIVATE: INIT_FILE_LAYOUT STRING "PORT_A"
// Retrieval info: PRIVATE: INIT_TO_SIM_X NUMERIC "0"
// Retrieval info: PRIVATE: INTENDED_DEVICE_FAMILY STRING "Cyclone III"
// Retrieval info: PRIVATE: JTAG_ENABLED NUMERIC "0"
// Retrieval info: PRIVATE: JTAG_ID STRING "NONE"
// Retrieval info: PRIVATE: MAXIMUM_DEPTH NUMERIC "0"
// Retrieval info: PRIVATE: MIFfilename STRING "../rtl/de0_test.mif"
// Retrieval info: PRIVATE: NUMWORDS_A NUMERIC "8192"
// Retrieval info: PRIVATE: RAM_BLOCK_TYPE NUMERIC "2"
// Retrieval info: PRIVATE: READ_DURING_WRITE_MODE_PORT_A NUMERIC "3"
// Retrieval info: PRIVATE: RegAddr NUMERIC "1"
// Retrieval info: PRIVATE: RegData NUMERIC "1"
// Retrieval info: PRIVATE: RegOutput NUMERIC "0"
// Retrieval info: PRIVATE: SYNTH_WRAPPER_GEN_POSTFIX STRING "0"
// Retrieval info: PRIVATE: SingleClock NUMERIC "1"
// Retrieval info: PRIVATE: UseDQRAM NUMERIC "1"
// Retrieval info: PRIVATE: WRCONTROL_ACLR_A NUMERIC "0"
// Retrieval info: PRIVATE: WidthAddr NUMERIC "13"
// Retrieval info: PRIVATE: WidthData NUMERIC "16"
// Retrieval info: PRIVATE: rden NUMERIC "1"
// Retrieval info: LIBRARY: altera_mf altera_mf.altera_mf_components.all
// Retrieval info: CONSTANT: BYTE_SIZE NUMERIC "8"
// Retrieval info: CONSTANT: CLOCK_ENABLE_INPUT_A STRING "BYPASS"
// Retrieval info: CONSTANT: CLOCK_ENABLE_OUTPUT_A STRING "BYPASS"
// Retrieval info: CONSTANT: INIT_FILE STRING "../rtl/de0_test.mif"
// Retrieval info: CONSTANT: INTENDED_DEVICE_FAMILY STRING "Cyclone III"
// Retrieval info: CONSTANT: LPM_HINT STRING "ENABLE_RUNTIME_MOD=NO"
// Retrieval info: CONSTANT: LPM_TYPE STRING "altsyncram"
// Retrieval info: CONSTANT: NUMWORDS_A NUMERIC "8192"
// Retrieval info: CONSTANT: OPERATION_MODE STRING "SINGLE_PORT"
// Retrieval info: CONSTANT: OUTDATA_ACLR_A STRING "NONE"
// Retrieval info: CONSTANT: OUTDATA_REG_A STRING "UNREGISTERED"
// Retrieval info: CONSTANT: POWER_UP_UNINITIALIZED STRING "FALSE"
// Retrieval info: CONSTANT: RAM_BLOCK_TYPE STRING "M9K"
// Retrieval info: CONSTANT: READ_DURING_WRITE_MODE_PORT_A STRING "NEW_DATA_NO_NBE_READ"
// Retrieval info: CONSTANT: WIDTHAD_A NUMERIC "13"
// Retrieval info: CONSTANT: WIDTH_A NUMERIC "16"
// Retrieval info: CONSTANT: WIDTH_BYTEENA_A NUMERIC "2"
// Retrieval info: USED_PORT: address 0 0 13 0 INPUT NODEFVAL "address[12..0]"
// Retrieval info: USED_PORT: byteena 0 0 2 0 INPUT VCC "byteena[1..0]"
// Retrieval info: USED_PORT: clock 0 0 0 0 INPUT VCC "clock"
// Retrieval info: USED_PORT: data 0 0 16 0 INPUT NODEFVAL "data[15..0]"
// Retrieval info: USED_PORT: q 0 0 16 0 OUTPUT NODEFVAL "q[15..0]"
// Retrieval info: USED_PORT: rden 0 0 0 0 INPUT VCC "rden"
// Retrieval info: USED_PORT: wren 0 0 0 0 INPUT NODEFVAL "wren"
// Retrieval info: CONNECT: @address_a 0 0 13 0 address 0 0 13 0
// Retrieval info: CONNECT: @byteena_a 0 0 2 0 byteena 0 0 2 0
// Retrieval info: CONNECT: @clock0 0 0 0 0 clock 0 0 0 0
// Retrieval info: CONNECT: @data_a 0 0 16 0 data 0 0 16 0
// Retrieval info: CONNECT: @rden_a 0 0 0 0 rden 0 0 0 0
// Retrieval info: CONNECT: @wren_a 0 0 0 0 wren 0 0 0 0
// Retrieval info: CONNECT: q 0 0 16 0 @q_a 0 0 16 0
// Retrieval info: GEN_FILE: TYPE_NORMAL de0_ram16k.v TRUE
// Retrieval info: GEN_FILE: TYPE_NORMAL de0_ram16k.inc FALSE
// Retrieval info: GEN_FILE: TYPE_NORMAL de0_ram16k.cmp FALSE
// Retrieval info: GEN_FILE: TYPE_NORMAL de0_ram16k.bsf FALSE
// Retrieval info: GEN_FILE: TYPE_NORMAL de0_ram16k_inst.v FALSE
// Retrieval info: GEN_FILE: TYPE_NORMAL de0_ram16k_bb.v TRUE
// Retrieval info: LIB_FILE: altera_mf
