#include "matrix_scan.h"
#include "hv2901.h"

/* ================================================================
 * 12x12 Matrix Scan
 * ================================================================
 *
 * Scans all 12 columns by switching the HV2901 column select and
 * reading 12 DDC232 rows per column. Uses the DDC232 dual-integrator
 * pipeline: each CONV toggle starts integration on one side while
 * making the other side's data available.
 *
 * The HV switch is set immediately after CONV toggles so the new
 * column integrates for nearly the full integration window.
 *
 * For streaming mode, after col 11 the switch is set to col 0 so
 * the next frame starts with col 0 already integrating (no re-prime).
 */

/**
 * matrix_scan — read one complete 12x12 frame.
 *
 * matrix[col][row] — column-major storage.
 * streaming: if true, after col 11 set HV to col 0 for seamless next frame.
 * manage_spi: if true, call hv_spi_begin/end (for single-shot scans).
 *             For streaming, caller manages the SPI bus.
 * Returns true on success.
 */
bool matrix_scan(int32_t matrix[MATRIX_COLS][MATRIX_ROWS],
                 bool streaming, bool manage_spi)
{
    /* Step 1: Configure FSPI for HV2901 */
    if (manage_spi) hv_spi_begin();

    /* Step 2: Prime pipeline — all columns Vneg during priming so no column
     * accumulates extra charge. Two CONV toggles fill the dual-integrator
     * pipeline; both readings are discarded. */
    if (!matrix_primed) {
        hv_all_neg();
        hv_write_fast();

        /* Toggle 1: start integration on side A (all Vneg — no signal) */
        conv_state = !conv_state;
        digitalWrite(PIN_CONV, conv_state ? HIGH : LOW);
        delayMicroseconds(integration_us);

        /* Toggle 2: side A data available (stale — discard), side B starts */
        conv_state = !conv_state;
        digitalWrite(PIN_CONV, conv_state ? HIGH : LOW);

        /* Wait DVALID, discard stale data */
        uint32_t start = micros();
        while (digitalRead(PIN_DVALID) != LOW) {
            if (micros() - start > 200000) {
                Serial.println("ERROR: Timeout priming matrix (DVALID)");
                if (manage_spi) hv_spi_end();
                return false;
            }
        }
        uint8_t discard[80], zeros[80];
        memset(zeros, 0, sizeof(zeros));
        hspi.beginTransaction(SPISettings(SPI_CLK_HZ, MSBFIRST, SPI_MODE0));
        hspi.transferBytes(zeros, discard, 80);
        hspi.endTransaction();

        /* Now set col 0 active for the first real integration */
        hv_set_column(0);
        hv_write_fast();

        conv_toggle_time = micros();
        matrix_primed = true;
    }

    /* Step 3: Column loop */
    for (int col = 0; col < MATRIX_COLS; col++) {
        /* Wait remaining integration time */
        uint32_t elapsed = micros() - conv_toggle_time;
        if (elapsed < integration_us) {
            delayMicroseconds(integration_us - elapsed);
        }

        /* Toggle CONV — previous side's data (= this col's data) available */
        conv_state = !conv_state;
        digitalWrite(PIN_CONV, conv_state ? HIGH : LOW);
        conv_toggle_time = micros();

        /* Dead time: all columns Vneg before activating next column */
        if (matrix_dead_us > 0) {
            hv_all_neg();
            hv_write_fast();
            delayMicroseconds(matrix_dead_us);
        }

        /* Set HV for next column (no wrap on last col) */
        int next_col = (col + 1 < MATRIX_COLS) ? col + 1 : col;
        hv_set_column(next_col);
        hv_write_fast();

        /* Wait for DVALID */
        uint32_t start = micros();
        while (digitalRead(PIN_DVALID) != LOW) {
            if (micros() - start > 200000) {
                Serial.printf("ERROR: Timeout on col %d DVALID\n", col);
                if (manage_spi) hv_spi_end();
                matrix_primed = false;
                return false;
            }
        }

        /* SPI readout: 640 bits = 80 bytes */
        uint8_t rx_buf[80], tx_buf[80];
        memset(tx_buf, 0, sizeof(tx_buf));
        hspi.beginTransaction(SPISettings(SPI_CLK_HZ, MSBFIRST, SPI_MODE0));
        hspi.transferBytes(tx_buf, rx_buf, 80);
        hspi.endTransaction();

        /* Unpack 20-bit words and store rows for this column */
        for (int row = 0; row < MATRIX_ROWS; row++) {
            int ch = matrix_row_channels[row];
            int bit_offset = ch * 20;
            int byte_idx   = bit_offset / 8;
            int bit_shift  = bit_offset % 8;

            uint32_t raw = ((uint32_t)rx_buf[byte_idx]     << 24) |
                           ((uint32_t)rx_buf[byte_idx + 1] << 16) |
                           ((uint32_t)rx_buf[byte_idx + 2] <<  8) |
                           ((uint32_t)(byte_idx + 3 < 80 ? rx_buf[byte_idx + 3] : 0));
            raw <<= bit_shift;
            raw >>= (32 - 20);

            matrix[col][row] = (int32_t)raw;
        }
    }

    /* Step 4: Restore FSPI to DAC (only for single-shot) */
    if (manage_spi) hv_spi_end();

    /* Always invalidate pipeline — each frame re-primes for equal column timing */
    pipeline_primed = false;
    matrix_primed = false;
    digitalWrite(PIN_CONV, LOW);

    return true;
}

/**
 * matrix_scan_full — read one 12x32 frame (all 32 DDC channels per column).
 *
 * Same column switching as matrix_scan, but extracts all 32 channels.
 * matrix[col][row] — column-major, row = DDC data index 0-31.
 * manage_spi: if true, call hv_spi_begin/end. For streaming, caller manages.
 */
bool matrix_scan_full(int32_t matrix[FULL_MATRIX_COLS][FULL_MATRIX_ROWS],
                      bool streaming, bool manage_spi)
{
    if (manage_spi) hv_spi_begin();

    if (!matrix_primed) {
        hv_all_neg();
        hv_write_fast();

        conv_state = !conv_state;
        digitalWrite(PIN_CONV, conv_state ? HIGH : LOW);
        delayMicroseconds(integration_us);

        conv_state = !conv_state;
        digitalWrite(PIN_CONV, conv_state ? HIGH : LOW);

        uint32_t start = micros();
        while (digitalRead(PIN_DVALID) != LOW) {
            if (micros() - start > 200000) {
                Serial.println("ERROR: Timeout priming full matrix (DVALID)");
                if (manage_spi) hv_spi_end();
                return false;
            }
        }
        uint8_t discard[80], zeros[80];
        memset(zeros, 0, sizeof(zeros));
        hspi.beginTransaction(SPISettings(SPI_CLK_HZ, MSBFIRST, SPI_MODE0));
        hspi.transferBytes(zeros, discard, 80);
        hspi.endTransaction();

        hv_set_column(0);
        hv_write_fast();

        conv_toggle_time = micros();
        matrix_primed = true;
    }

    for (int col = 0; col < FULL_MATRIX_COLS; col++) {
        uint32_t elapsed = micros() - conv_toggle_time;
        if (elapsed < integration_us) {
            delayMicroseconds(integration_us - elapsed);
        }

        conv_state = !conv_state;
        digitalWrite(PIN_CONV, conv_state ? HIGH : LOW);
        conv_toggle_time = micros();

        if (matrix_dead_us > 0) {
            hv_all_neg();
            hv_write_fast();
            delayMicroseconds(matrix_dead_us);
        }

        int next_col = (col + 1 < FULL_MATRIX_COLS) ? col + 1 : col;
        hv_set_column(next_col);
        hv_write_fast();

        uint32_t start = micros();
        while (digitalRead(PIN_DVALID) != LOW) {
            if (micros() - start > 200000) {
                Serial.printf("ERROR: Timeout on full col %d DVALID\n", col);
                if (manage_spi) hv_spi_end();
                matrix_primed = false;
                return false;
            }
        }

        uint8_t rx_buf[80], tx_buf[80];
        memset(tx_buf, 0, sizeof(tx_buf));
        hspi.beginTransaction(SPISettings(SPI_CLK_HZ, MSBFIRST, SPI_MODE0));
        hspi.transferBytes(tx_buf, rx_buf, 80);
        hspi.endTransaction();

        /* Unpack all 32 channels in physical order (phys 1-32) */
        for (int p = 0; p < NUM_CHANNELS; p++) {
            int ch = physical_to_ddc[p];
            int bit_offset = ch * 20;
            int byte_idx   = bit_offset / 8;
            int bit_shift  = bit_offset % 8;

            uint32_t raw = ((uint32_t)rx_buf[byte_idx]     << 24) |
                           ((uint32_t)rx_buf[byte_idx + 1] << 16) |
                           ((uint32_t)rx_buf[byte_idx + 2] <<  8) |
                           ((uint32_t)(byte_idx + 3 < 80 ? rx_buf[byte_idx + 3] : 0));
            raw <<= bit_shift;
            raw >>= (32 - 20);

            matrix[col][p] = (int32_t)raw;
        }
    }

    if (manage_spi) hv_spi_end();

    pipeline_primed = false;
    matrix_primed = false;
    digitalWrite(PIN_CONV, LOW);

    return true;
}

/* ================================================================
 * 8x12 Debug Matrix Scan
 * ================================================================ */

/**
 * Helper: prime the pipeline for a given debug column.
 */
static bool dbg_prime_column(int col_idx)
{
    hv_set_column_debug(col_idx);
    hv_write_fast();

    uint8_t discard[80], zeros[80];
    memset(zeros, 0, sizeof(zeros));

    /* Toggle 1: ends previous integration (stale data from old column). */
    uint32_t t_toggle = micros();
    conv_state = !conv_state;
    digitalWrite(PIN_CONV, conv_state ? HIGH : LOW);

    uint32_t start = micros();
    while (digitalRead(PIN_DVALID) != LOW) {
        if (micros() - start > 200000) return false;
    }
    hspi.beginTransaction(SPISettings(SPI_CLK_HZ, MSBFIRST, SPI_MODE0));
    hspi.transferBytes(zeros, discard, 80);
    hspi.endTransaction();

    /* Wait for remaining integration time */
    uint32_t elapsed = micros() - t_toggle;
    if (elapsed < integration_us) {
        delayMicroseconds(integration_us - elapsed);
    }

    /* Toggle 2: ends first integration on new column — discard */
    conv_state = !conv_state;
    digitalWrite(PIN_CONV, conv_state ? HIGH : LOW);

    start = micros();
    while (digitalRead(PIN_DVALID) != LOW) {
        if (micros() - start > 200000) return false;
    }
    hspi.beginTransaction(SPISettings(SPI_CLK_HZ, MSBFIRST, SPI_MODE0));
    hspi.transferBytes(zeros, discard, 80);
    hspi.endTransaction();

    conv_toggle_time = micros();
    return true;
}

/**
 * Helper: do one pipelined read for debug matrix.
 */
static bool dbg_read_one(int32_t vals[DBG_MATRIX_ROWS])
{
    uint32_t elapsed = micros() - conv_toggle_time;
    if (elapsed < integration_us) {
        delayMicroseconds(integration_us - elapsed);
    }

    conv_state = !conv_state;
    digitalWrite(PIN_CONV, conv_state ? HIGH : LOW);
    conv_toggle_time = micros();

    uint32_t start = micros();
    while (digitalRead(PIN_DVALID) != LOW) {
        if (micros() - start > 200000) return false;
    }

    uint8_t rx_buf[80], tx_buf[80];
    memset(tx_buf, 0, sizeof(tx_buf));
    hspi.beginTransaction(SPISettings(SPI_CLK_HZ, MSBFIRST, SPI_MODE0));
    hspi.transferBytes(tx_buf, rx_buf, 80);
    hspi.endTransaction();

    for (int row = 0; row < DBG_MATRIX_ROWS; row++) {
        int ch = dbg_row_channels[row];
        int bit_offset = ch * 20;
        int byte_idx   = bit_offset / 8;
        int bit_shift  = bit_offset % 8;

        uint32_t raw = ((uint32_t)rx_buf[byte_idx]     << 24) |
                       ((uint32_t)rx_buf[byte_idx + 1] << 16) |
                       ((uint32_t)rx_buf[byte_idx + 2] <<  8) |
                       ((uint32_t)(byte_idx + 3 < 80 ? rx_buf[byte_idx + 3] : 0));
        raw <<= bit_shift;
        raw >>= (32 - 20);
        vals[row] = (int32_t)raw;
    }
    return true;
}

/**
 * Debug matrix scan with per-column averaging.
 */
bool matrix_scan_debug(int32_t matrix[DBG_MATRIX_COLS][DBG_MATRIX_ROWS],
                       int avg_count)
{
    if (avg_count < 1) avg_count = 1;

    hv_spi_begin();

    for (int col = 0; col < DBG_MATRIX_COLS; col++) {
        /* Prime: switch to this column, flush stale charge */
        if (!dbg_prime_column(col)) {
            Serial.printf("ERROR: Timeout priming debug col %d\n", col);
            hv_spi_end();
            return false;
        }

        /* Accumulate avg_count reads */
        int64_t accum[DBG_MATRIX_ROWS];
        memset(accum, 0, sizeof(accum));
        int32_t vals[DBG_MATRIX_ROWS];

        for (int s = 0; s < avg_count; s++) {
            if (!dbg_read_one(vals)) {
                Serial.printf("ERROR: Timeout on debug col %d sample %d\n", col, s);
                hv_spi_end();
                return false;
            }
            for (int row = 0; row < DBG_MATRIX_ROWS; row++) {
                accum[row] += vals[row];
            }
        }

        /* Store average */
        for (int row = 0; row < DBG_MATRIX_ROWS; row++) {
            matrix[col][row] = (int32_t)(accum[row] / avg_count);
        }
    }

    hv_spi_end();
    pipeline_primed = false;
    digitalWrite(PIN_CONV, LOW);

    return true;
}

/* ================================================================
 * 12x12 Debug Matrix Scan (even pairs across 2 chips)
 * ================================================================ */

/**
 * Helper: prime the pipeline for a 12x12 debug column.
 */
static bool dbg12_prime_column(int col_idx)
{
    hv_set_column_debug12(col_idx);
    hv_write_fast();

    uint8_t discard[80], zeros[80];
    memset(zeros, 0, sizeof(zeros));

    /* Toggle 1: ends previous integration (stale data from old column). */
    uint32_t t_toggle = micros();
    conv_state = !conv_state;
    digitalWrite(PIN_CONV, conv_state ? HIGH : LOW);

    uint32_t start = micros();
    while (digitalRead(PIN_DVALID) != LOW) {
        if (micros() - start > 200000) return false;
    }
    hspi.beginTransaction(SPISettings(SPI_CLK_HZ, MSBFIRST, SPI_MODE0));
    hspi.transferBytes(zeros, discard, 80);
    hspi.endTransaction();

    /* Wait for remaining integration time */
    uint32_t elapsed = micros() - t_toggle;
    if (elapsed < integration_us) {
        delayMicroseconds(integration_us - elapsed);
    }

    /* Toggle 2: ends first integration on new column — discard */
    conv_state = !conv_state;
    digitalWrite(PIN_CONV, conv_state ? HIGH : LOW);

    start = micros();
    while (digitalRead(PIN_DVALID) != LOW) {
        if (micros() - start > 200000) return false;
    }
    hspi.beginTransaction(SPISettings(SPI_CLK_HZ, MSBFIRST, SPI_MODE0));
    hspi.transferBytes(zeros, discard, 80);
    hspi.endTransaction();

    conv_toggle_time = micros();
    return true;
}

/**
 * 12x12 debug matrix scan with per-column averaging.
 */
bool matrix_scan_debug12(int32_t matrix[DBG12_MATRIX_COLS][DBG12_MATRIX_ROWS],
                         int avg_count)
{
    if (avg_count < 1) avg_count = 1;

    hv_spi_begin();

    for (int col = 0; col < DBG12_MATRIX_COLS; col++) {
        if (!dbg12_prime_column(col)) {
            Serial.printf("ERROR: Timeout priming debug12 col %d\n", col);
            hv_spi_end();
            return false;
        }

        int64_t accum[DBG12_MATRIX_ROWS];
        memset(accum, 0, sizeof(accum));
        int32_t vals[DBG12_MATRIX_ROWS];

        for (int s = 0; s < avg_count; s++) {
            if (!dbg_read_one(vals)) {
                Serial.printf("ERROR: Timeout on debug12 col %d sample %d\n", col, s);
                hv_spi_end();
                return false;
            }
            for (int row = 0; row < DBG12_MATRIX_ROWS; row++) {
                accum[row] += vals[row];
            }
        }

        for (int row = 0; row < DBG12_MATRIX_ROWS; row++) {
            matrix[col][row] = (int32_t)(accum[row] / avg_count);
        }
    }

    hv_spi_end();
    pipeline_primed = false;
    digitalWrite(PIN_CONV, LOW);

    return true;
}
