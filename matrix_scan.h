#pragma once

#include "config.h"

bool matrix_scan(int32_t matrix[MATRIX_COLS][MATRIX_ROWS],
                 bool streaming, bool manage_spi = true);

bool matrix_scan_full(int32_t matrix[FULL_MATRIX_COLS][FULL_MATRIX_ROWS],
                      bool streaming, bool manage_spi = true);

bool matrix_scan_debug(int32_t matrix[DBG_MATRIX_COLS][DBG_MATRIX_ROWS],
                       int avg_count);

bool matrix_scan_debug12(int32_t matrix[DBG12_MATRIX_COLS][DBG12_MATRIX_ROWS],
                         int avg_count);
