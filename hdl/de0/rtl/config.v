//
// Copyright (c) 2014-2015 by 1801BM1@gmail.com
//
// Project configuration parameters
//______________________________________________________________________________
//
`timescale 1ns / 100ps
`define  _CONFIG_                      1
`define   _CONFIG_VM1_               _CONFIG_
//`define  _CONFIG_VM2_               _CONFIG_
//`define  _CONFIG_VM3_               _CONFIG_


//
// Simulation stops (breakpoint) after this time elapsed
//
`define  CONFIG_SIM_TIME_LIMIT         2000000
//
// External clock frequency
//
`define  CONFIG_SIM_CLOCK_HPERIOD      5
//
// Test software start address
//
`define  CONFIG_SIM_START_ADDRESS      16'o000000

//
// Generated processor clock phase durations
// In system clocks
//
`define  CONFIG_SIM_CLK_HIGH           1
`define  CONFIG_SIM_CLK_LOW            1

//`define  CONFIG_SIM_DEBUG_MC         _CONFIG_
//`define  CONFIG_SIM_DEBUG_IO         _CONFIG_
`define  CONFIG_SIM_DEBUG_TTY          _CONFIG_

//______________________________________________________________________________
//
// External oscillator clock, feeds the PLLs
//
`define  CONFIG_OSC_CLOCK              50000000
//
// Global system clock
//
`define  CONFIG_SYS_CLOCK              100000000

// 4.2MHz
// `define  VM1_CLOCK_HIGH             8'd11
// `define  VM1_CLOCK_LOW              8'd23

// 5.0MHz
`define  VM1_CLOCK_HIGH                8'd09
`define  VM1_CLOCK_LOW                 8'd19

// 7.2MHz
//`define  VM3_CLOCK_HIGH              8'd06
//`define  VM3_CLOCK_LOW               8'd13

// 6.25MHz
//`define  VM3_CLOCK_HIGH              8'd07
//`define  VM3_CLOCK_LOW               8'd15

// 5.0MHz
`define  VM3_CLOCK_HIGH                8'd09
`define  VM3_CLOCK_LOW                 8'd19

// 10.0MHz
`define  VM2_CLOCK_HIGH                8'd04
`define  VM2_CLOCK_LOW                 8'd09

`define  DE0_DCLO_WIDTH_CLK            200
`define  DE0_ACLO_DELAY_CLK            300

//______________________________________________________________________________
//
// Reset button debounce interval (in ms))
//
`define  CONFIG_RESET_BUTTON_DEBOUNCE_MS   5
//
// Internal reset pulse width (in system clocks)
//
`define  CONFIG_RESET_PULSE_WIDTH_CLK      7
