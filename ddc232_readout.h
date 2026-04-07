#pragma once

#include "config.h"

/* System CLK via LEDC */
void clk_start(uint8_t pin, uint32_t freq_hz);
void clk_set_freq(uint8_t pin, uint32_t freq_hz);
void clk_stop(uint8_t pin);

/* Configuration register */
void write_config_register();
void apply_config();
void write_and_readback_config();

/* DDC232 readout */
void ddc232_flush();
bool ddc232_prime();
bool ddc232_read(int32_t data[NUM_CHANNELS]);

/* Timing diagnostics (accumulated by ddc232_read, read/reset by commands) */
extern uint32_t dbg_wait_us, dbg_dvalid_us, dbg_spi_us, dbg_unpack_us;
extern int dbg_samples;

/* Hardware diagnostics commands */
void cmd_diag();
void cmd_scope();
