#pragma once

#include "config.h"

/* DDS frequency conversion */
uint32_t dac_freq_to_inc(uint32_t freq_hz);

/* Timer ISR (needed by hv2901.cpp for timer restart) */
void IRAM_ATTR dac_timer_isr();

/* DAC write (used by hv2901 init) */
void dac_write_cmd(uint8_t cs_pin, uint8_t cmd, uint16_t data);

/* DAC output task (FreeRTOS) */
void dac_output_task(void *param);

/* DAC lifecycle */
void dac_init();
void dac_start();
void dac_stop();

/* Voltage output */
uint16_t bias_v_to_code(float v_out);
uint16_t sw_v_to_code(float v_out);
void dac_set_voltage(uint8_t cs_pin, uint8_t cmd, uint16_t code);

/* DAC commands */
void cmd_dac(const char* arg1, const char* arg2);
void cmd_vbias1(const char* arg);
void cmd_vbias2(const char* arg);
void cmd_vsw1(const char* arg);
void cmd_vsw2(const char* arg);
