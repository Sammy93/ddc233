#pragma once

#include "config.h"

/* HV2901 switch state: one uint32_t per chip, bit N = switch N state */
extern uint32_t hv_reg[HV_NUM_CHIPS];

/* SPI bus sharing (pauses/resumes DAC when switching FSPI) */
void hv_spi_begin();
void hv_spi_end();

/* Switch control */
void hv_enforce_exclusivity();
void hv_write();
void hv_init();
bool hv_set_pair(int chip, int pair, int state);

/* Column select (modify hv_reg but do NOT call hv_write — caller must call hv_write_fast) */
void hv_all_neg();
void hv_set_column(int col);
void hv_set_column_debug(int col_idx);
void hv_set_column_debug12(int col_idx);

/* Fast SPI write (assumes FSPI already configured for HV pins) */
void hv_write_fast();
