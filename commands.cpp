#include "commands.h"
#include "ddc232_readout.h"
#include "dac8562.h"
#include "hv2901.h"
#include "matrix_scan.h"

/* ================================================================
 * Single-channel commands
 * ================================================================ */

void cmd_read()
{
    int32_t data[NUM_CHANNELS];
    if (!ddc232_read(data)) return;

    /* Stop the pipeline — pull CONV low so we're not left integrating */
    digitalWrite(PIN_CONV, LOW);
    pipeline_primed = false;

    for (int ch = 0; ch < NUM_CHANNELS; ch++) {
        Serial.printf("CH%02d: %7ld\n", ch, (long)data[ch]);
    }
}

void cmd_range(const char* arg)
{
    if (!arg || *arg == '\0') {
        Serial.println("Usage: range <0-7>");
        Serial.println("  0 = 12.5 pC   4 = 200 pC");
        Serial.println("  1 = 50 pC     5 = 250 pC");
        Serial.println("  2 = 100 pC    6 = 300 pC");
        Serial.println("  3 = 150 pC    7 = 350 pC");
        return;
    }
    int r = atoi(arg);
    if (r < 0 || r > 7) { Serial.println("Invalid range"); return; }
    current_range = r;
    apply_config();
    Serial.printf("Range set to %d (%s)\n", r, range_labels[r]);
}

void cmd_inttime(const char* arg)
{
    if (!arg || *arg == '\0') {
        Serial.println("Usage: inttime <microseconds>");
        return;
    }
    uint32_t us = (uint32_t)atol(arg);
    if (us == 0) { Serial.println("Invalid time"); return; }
    integration_us = us;
    Serial.printf("Integration time set to %lu us\n", integration_us);
}

void cmd_deadtime(const char* arg)
{
    if (!arg || *arg == '\0') {
        Serial.printf("Matrix dead time: %lu us\n", matrix_dead_us);
        return;
    }
    matrix_dead_us = (uint32_t)atol(arg);
    Serial.printf("Matrix dead time set to %lu us\n", matrix_dead_us);
}

void cmd_skipvneg(const char* arg1, const char* arg2)
{
    if (!arg1 || *arg1 == '\0') {
        Serial.printf("Skip-Vneg mask: 0x%04X\n", hv_skip_vneg);
        for (int p = 0; p < HV_PAIRS_PER_CHIP; p++) {
            if (hv_skip_vneg & (1 << p))
                Serial.printf("  Pair %d: Vneg skipped (uses OFF)\n", p);
        }
        return;
    }
    int pair = atoi(arg1);
    if (pair < 0 || pair >= HV_PAIRS_PER_CHIP) {
        Serial.printf("Invalid pair %d (0-%d)\n", pair, HV_PAIRS_PER_CHIP - 1);
        return;
    }
    if (arg2 && (strcmp(arg2, "off") == 0 || strcmp(arg2, "0") == 0)) {
        hv_skip_vneg &= ~(1 << pair);
        Serial.printf("Pair %d: Vneg restored (normal)\n", pair);
    } else {
        hv_skip_vneg |= (1 << pair);
        Serial.printf("Pair %d: Vneg skipped (uses OFF when inactive)\n", pair);
    }
}

void cmd_clk(const char* arg)
{
    if (!arg || *arg == '\0') {
        Serial.println("Usage: clk <frequency_hz>");
        return;
    }
    uint32_t hz = (uint32_t)atol(arg);
    if (hz == 0) { Serial.println("Invalid frequency"); return; }
    clk_set_freq(PIN_CLK, hz);
    clk_freq_hz = hz;
    Serial.printf("CLK set to %lu Hz\n", clk_freq_hz);
}

void cmd_test(const char* arg)
{
    if (!arg || *arg == '\0') {
        Serial.println("Usage: test <on|off>");
        Serial.println("  Enables/disables DDC232 internal test mode.");
        Serial.println("  When on, inputs are disconnected and the DDC232 measures");
        Serial.println("  a zero-input signal (no external connections needed).");
        return;
    }
    test_mode = (strcmp(arg, "on") == 0 || strcmp(arg, "1") == 0);
    apply_config();
    Serial.printf("Test mode %s\n", test_mode ? "ON" : "OFF");
}

void cmd_continuous(const char* arg1, const char* arg2)
{
    int count     = arg1 ? atoi(arg1) : 10;
    int period_ms = arg2 ? atoi(arg2) : 0;   // default 0 = max speed
    if (count <= 0) count = 10;
    if (period_ms < 0) period_ms = 0;

    Serial.printf("Reading %d samples, %d ms period. Press any key to stop.\n", count, period_ms);

    /* Flush any leftover serial input (e.g. the newline from this command) */
    delay(10);
    while (Serial.available()) Serial.read();

    int32_t data[NUM_CHANNELS];
    uint32_t next_ms = millis();
    uint32_t t_start = micros();
    int completed = 0;
    uint32_t total_read_us = 0;
    uint32_t total_print_us = 0;

    for (int i = 0; i < count; i++) {
        /* Check for keypress to abort (only every 64 samples to reduce overhead) */
        if ((i & 63) == 0 && i > 0 && Serial.available()) {
            while (Serial.available()) Serial.read();
            Serial.println("\nStopped.");
            break;
        }

        /* Wait until the next scheduled sample time (skipped if period_ms == 0) */
        if (period_ms > 0) {
            while ((int32_t)(next_ms - millis()) > 0) { /* spin */ }
        }

        uint32_t t_us = micros();
        if (!ddc232_read(data)) break;
        uint32_t t_read = micros();
        completed++;

        /* CSV: timestamp_us, sample_index, ch0, ch1, ..., ch31 */
        static char line_buf[512];
        int pos = sprintf(line_buf, "%lu,%d", t_us, i);
        for (int ch = 0; ch < NUM_CHANNELS; ch++) {
            pos += sprintf(line_buf + pos, ",%ld", (long)data[ch]);
        }
        line_buf[pos++] = '\n';
        Serial.write(line_buf, pos);
        uint32_t t_print = micros();

        /* Accumulate timing stats */
        total_read_us += (t_read - t_us);
        total_print_us += (t_print - t_read);

        next_ms += period_ms;
    }

    /* Stop pipeline */
    digitalWrite(PIN_CONV, LOW);
    pipeline_primed = false;

    /* Print summary */
    uint32_t elapsed_us = micros() - t_start;
    float fps = (completed > 0 && elapsed_us > 0) ? (float)completed / (elapsed_us * 1e-6f) : 0;
    Serial.printf("\nDone: %d samples in %lu us (%.1f samples/s)\n", completed, elapsed_us, fps);
    if (completed > 0) {
        Serial.printf("  Avg read: %lu us, avg print: %lu us, avg total: %lu us/sample\n",
                      total_read_us / completed, total_print_us / completed,
                      elapsed_us / completed);
        Serial.printf("  Read: %.0f%%, Print: %.0f%%, Other: %.0f%%\n",
                      100.0f * total_read_us / elapsed_us,
                      100.0f * total_print_us / elapsed_us,
                      100.0f * (elapsed_us - total_read_us - total_print_us) / elapsed_us);
        if (dbg_samples > 0) {
            Serial.printf("  Read breakdown — wait: %lu us, dvalid: %lu us, spi: %lu us, unpack: %lu us\n",
                          dbg_wait_us / dbg_samples, dbg_dvalid_us / dbg_samples,
                          dbg_spi_us / dbg_samples, dbg_unpack_us / dbg_samples);
        }
        dbg_wait_us = dbg_dvalid_us = dbg_spi_us = dbg_unpack_us = 0;
        dbg_samples = 0;
    }
}

/*
 * Binary streaming mode for maximum throughput.
 *
 * Frame format (136 bytes, little-endian):
 *   [0-3]   Sync marker: 0xAA 0x55 0xAA 0x55
 *   [4-7]   Timestamp (micros(), uint32_t)
 *   [8-135] 32 channels x 4 bytes (int32_t each)
 *
 * Stops when any byte is received on serial.
 */
void cmd_stream(const char* arg1)
{
    Serial.println("Binary stream: 136 bytes/frame. Send any byte to stop.");
    Serial.flush();

    delay(10);
    while (Serial.available()) Serial.read();

    uint8_t frame[136];
    frame[0] = SYNC_MARKER_0;
    frame[1] = SYNC_MARKER_1;
    frame[2] = SYNC_MARKER_0;
    frame[3] = SYNC_MARKER_1;

    int32_t data[NUM_CHANNELS];
    int32_t phys[NUM_CHANNELS];
    uint32_t t_start = micros();
    uint32_t completed = 0;

    /* Run indefinitely until a byte is received */
    while (true) {
        if ((completed & 63) == 0 && completed > 0 && Serial.available()) {
            while (Serial.available()) Serial.read();
            break;
        }

        uint32_t t_us = micros();
        if (!ddc232_read(data)) break;
        completed++;

        /* Reorder DDC index -> physical channel order (phys 1-32) */
        for (int p = 0; p < NUM_CHANNELS; p++) {
            phys[p] = data[physical_to_ddc[p]];
        }

        /* Pack frame */
        memcpy(frame + 4, &t_us, 4);
        memcpy(frame + 8, phys, NUM_CHANNELS * 4);
        Serial.write(frame, 136);
    }

    /* Stop pipeline */
    digitalWrite(PIN_CONV, LOW);
    pipeline_primed = false;

    /* Text summary (after binary data) */
    Serial.println();
    uint32_t elapsed_us = micros() - t_start;
    float fps = (completed > 0 && elapsed_us > 0) ? (float)completed / (elapsed_us * 1e-6f) : 0;
    Serial.printf("Done: %lu samples in %lu us (%.1f samples/s)\n", completed, elapsed_us, fps);
}

/*
 * PSRAM capture mode — acquire at full hardware speed with zero serial
 * overhead, then dump the buffer over USB afterward.
 */

struct capture_sample {
    uint32_t timestamp_us;
    int32_t  data[NUM_CHANNELS];
};

void cmd_capture(const char* arg1)
{
    /* Determine max samples from PSRAM */
    size_t psram_free = ESP.getFreePsram();
    int max_samples = psram_free / sizeof(capture_sample);
    if (max_samples < 1) {
        Serial.println("ERROR: No PSRAM available. Enable PSRAM in board config.");
        return;
    }

    int count = arg1 ? atoi(arg1) : max_samples;
    if (count <= 0) count = max_samples;
    if (count > max_samples) {
        Serial.printf("Clamped to %d samples (PSRAM: %u bytes free)\n", max_samples, psram_free);
        count = max_samples;
    }

    /* Allocate buffer in PSRAM */
    capture_sample* buf = (capture_sample*)ps_malloc(count * sizeof(capture_sample));
    if (!buf) {
        Serial.println("ERROR: PSRAM allocation failed");
        return;
    }

    Serial.printf("Capturing %d samples to PSRAM (%u bytes)...\n",
                  count, count * (uint32_t)sizeof(capture_sample));
    Serial.flush();

    /* === Acquire — tight loop, no serial, no checks === */
    uint32_t t_start = micros();

    for (int i = 0; i < count; i++) {
        buf[i].timestamp_us = micros();
        if (!ddc232_read(buf[i].data)) {
            count = i;  // truncate on error
            break;
        }
    }

    uint32_t t_end = micros();

    /* Stop pipeline */
    digitalWrite(PIN_CONV, LOW);
    pipeline_primed = false;

    /* === Report === */
    uint32_t elapsed_us = t_end - t_start;
    float fps = (count > 0 && elapsed_us > 0) ? (float)count / (elapsed_us * 1e-6f) : 0;
    Serial.printf("Captured %d samples in %lu us (%.1f samples/s)\n", count, elapsed_us, fps);

    if (count >= 2) {
        /* Show timing consistency */
        uint32_t dt_min = UINT32_MAX, dt_max = 0;
        uint64_t dt_sum = 0;
        for (int i = 1; i < count; i++) {
            uint32_t dt = buf[i].timestamp_us - buf[i - 1].timestamp_us;
            if (dt < dt_min) dt_min = dt;
            if (dt > dt_max) dt_max = dt;
            dt_sum += dt;
        }
        Serial.printf("  Sample interval: min=%lu us, max=%lu us, avg=%lu us\n",
                      dt_min, dt_max, (uint32_t)(dt_sum / (count - 1)));
    }

    /* === Dump as binary === */
    Serial.printf("Dumping %d frames (136 bytes each)...\n", count);
    Serial.flush();
    delay(100);

    uint8_t frame[136];
    frame[0] = SYNC_MARKER_0;
    frame[1] = SYNC_MARKER_1;
    frame[2] = SYNC_MARKER_0;
    frame[3] = SYNC_MARKER_1;

    int32_t phys[NUM_CHANNELS];
    for (int i = 0; i < count; i++) {
        /* Reorder DDC index -> physical channel order */
        for (int p = 0; p < NUM_CHANNELS; p++) {
            phys[p] = buf[i].data[physical_to_ddc[p]];
        }
        memcpy(frame + 4, &buf[i].timestamp_us, 4);
        memcpy(frame + 8, phys, NUM_CHANNELS * 4);
        Serial.write(frame, 136);
    }

    free(buf);
    Serial.printf("\nDump complete.\n");
}

/* ================================================================
 * HV switch command
 * ================================================================ */

void cmd_sw(const char* arg1, const char* arg2)
{
    if (!arg1 || *arg1 == '\0') {
        Serial.println("Usage:");
        Serial.println("  sw <pair> pos       - Connect SW_even (pair 0-15, chip 1)");
        Serial.println("  sw <pair> neg       - Connect SW_odd  (pair 0-15, chip 1)");
        Serial.println("  sw <pair> off       - Disconnect both");
        Serial.println("  sw status           - Show all switch states");
        Serial.println("  sw clear            - Turn all switches OFF");
        Serial.println();
        Serial.println("  Pair 0: SW0 (pos) / SW1 (neg) -> Y01");
        Serial.println("  Pair 1: SW2 (pos) / SW3 (neg) -> Y23");
        Serial.println("  ...");
        Serial.println("  Pair 15: SW30 (pos) / SW31 (neg) -> Y3031");
        return;
    }

    if (strcmp(arg1, "status") == 0) {
        for (int chip = 0; chip < HV_NUM_CHIPS; chip++) {
            Serial.printf("Chip %d (0x%08lX):\n", chip + 1, (unsigned long)hv_reg[chip]);
            for (int p = 0; p < HV_PAIRS_PER_CHIP; p++) {
                int sw_even = p * 2;
                int sw_odd  = p * 2 + 1;
                bool even_on = (hv_reg[chip] >> sw_even) & 1;
                bool odd_on  = (hv_reg[chip] >> sw_odd)  & 1;
                if (even_on || odd_on) {
                    Serial.printf("  Pair %2d (Y%02d%02d): %s\n", p, sw_even, sw_odd,
                                  even_on ? "POS (SW_even)" : "NEG (SW_odd)");
                }
            }
            if (hv_reg[chip] == 0) Serial.println("  (all OFF)");
        }
        return;
    }

    if (strcmp(arg1, "clear") == 0) {
        hv_reg[0] = 0;
        hv_reg[1] = 0;
        hv_write();
        Serial.println("All switches OFF");
        return;
    }

    if (strcmp(arg1, "allpos") == 0) {
        /* Set all 16 pairs on chip 1 to POS (SW_even ON, SW_odd OFF) */
        hv_reg[0] = 0x55555555UL;  // bits 0,2,4,...,30 ON
        hv_write();
        Serial.println("All pairs: POS");
        return;
    }

    if (strcmp(arg1, "allneg") == 0) {
        /* Set all 16 pairs on chip 1 to NEG (SW_even OFF, SW_odd ON) */
        hv_reg[0] = 0xAAAAAAAAUL;  // bits 1,3,5,...,31 ON
        hv_write();
        Serial.println("All pairs: NEG");
        return;
    }

    if (strcmp(arg1, "toggle") == 0) {
        /* sw toggle <pair> [hz] — toggle a pair POS/OFF at given rate. */
        int pair = arg2 ? atoi(arg2) : 0;
        /* Parse optional 3rd arg for Hz */
        char* arg3 = strtok(NULL, " \t");
        int hz = arg3 ? atoi(arg3) : 10;
        if (hz < 1) hz = 1;
        if (hz > 1000) hz = 1000;
        if (pair < 0 || pair >= HV_PAIRS_PER_CHIP) {
            Serial.println("Invalid pair (0-15)");
            return;
        }
        int sw_even = pair * 2;
        int sw_odd  = pair * 2 + 1;
        uint32_t half_period_ms = 500 / hz;
        if (half_period_ms < 1) half_period_ms = 1;

        Serial.printf("Toggling pair %d (SW%d) POS/OFF at %d Hz. Send any byte to stop.\n",
                      pair, sw_even, hz);
        Serial.flush();

        delay(10);
        while (Serial.available()) Serial.read();

        uint32_t saved_reg = hv_reg[0];
        bool on = false;

        while (true) {
            if (Serial.available()) {
                while (Serial.available()) Serial.read();
                break;
            }

            on = !on;
            /* Clear both switches in this pair */
            hv_reg[0] &= ~((1UL << sw_even) | (1UL << sw_odd));
            if (on) {
                hv_reg[0] |= (1UL << sw_even);  // POS on
            }
            hv_write();
            delay(half_period_ms);
        }

        /* Restore original state */
        hv_reg[0] = saved_reg;
        hv_write();
        Serial.println("Stopped.");
        return;
    }

    if (strcmp(arg1, "diag") == 0) {
        Serial.println("--- HV2901 SPI Shift Register Test ---");
        Serial.println("  Scope: CLK (GPIO16), MOSI (GPIO17), MISO/DOUT (GPIO9)");
        Serial.println("  Sending 0xAAAAAAAA x10 — expect alternating 1/0 on MOSI.");
        Serial.println("  DOUT readback is delayed by 32 clocks.");
        Serial.println("  Starting in 2 seconds...");
        delay(2000);
        {
            uint32_t saved_reg = hv_reg[0];
            digitalWrite(PIN_HV_LE, HIGH);  // opaque — don't affect switches

            hv_spi_begin();
            dac_spi.beginTransaction(SPISettings(HV_SPI_CLK_HZ, MSBFIRST, SPI_MODE0));

            uint32_t readback[10];
            for (int i = 0; i < 10; i++) {
                readback[i] = dac_spi.transfer32(0xAAAAAAAAUL);
            }
            uint32_t readback_last = dac_spi.transfer32(0x00000000);

            dac_spi.endTransaction();
            hv_spi_end();

            Serial.println("  Results (DOUT readback, 1-bit shifted due to CLK/sample alignment):");
            for (int i = 0; i < 10; i++) {
                Serial.printf("    [%2d] wrote 0xAAAAAAAA  read 0x%08lX  %s\n",
                              i, (unsigned long)readback[i],
                              (i == 0) ? "(old register contents)" :
                              (readback[i] == 0x55555555UL) ? "OK (1-bit shift)" : "UNEXPECTED");
            }
            Serial.printf("    [fl] wrote 0x00000000  read 0x%08lX  %s\n",
                          (unsigned long)readback_last,
                          (readback_last == 0x55555555UL) ? "OK (1-bit shift)" : "UNEXPECTED");

            /* Restore and latch */
            hv_reg[0] = saved_reg;
            hv_write();
        }

        /* Bitbang readback test */
        Serial.println("\n  Bitbang GPIO readback test...");
        Serial.printf("  Reading DOUT on GPIO%d (PIN_HV_MISO)\n", PIN_HV_MISO);
        {
            /* Reclaim pins as GPIO */
            pinMode(PIN_HV_CLK, OUTPUT);
            pinMode(PIN_HV_MOSI, OUTPUT);
            pinMode(PIN_HV_MISO, INPUT);
            digitalWrite(PIN_HV_CLK, LOW);
            digitalWrite(PIN_HV_LE, HIGH);  // opaque

            /* Shift in 0xAAAAAAAA via bitbang */
            uint32_t val = 0xAAAAAAAAUL;
            for (int bit = 31; bit >= 0; bit--) {
                digitalWrite(PIN_HV_MOSI, (val >> bit) & 1);
                delayMicroseconds(1);
                digitalWrite(PIN_HV_CLK, HIGH);
                delayMicroseconds(1);
                digitalWrite(PIN_HV_CLK, LOW);
                delayMicroseconds(1);
            }

            /* Now shift 32 zeros, reading DOUT before each rising edge */
            uint32_t bb_readback = 0;
            for (int bit = 31; bit >= 0; bit--) {
                digitalWrite(PIN_HV_MOSI, LOW);
                delayMicroseconds(1);
                int dout = digitalRead(PIN_HV_MISO);
                bb_readback |= ((uint32_t)dout << bit);
                digitalWrite(PIN_HV_CLK, HIGH);
                delayMicroseconds(1);
                digitalWrite(PIN_HV_CLK, LOW);
                delayMicroseconds(1);
            }
            Serial.printf("  Bitbang readback (GPIO%d): 0x%08lX\n",
                          PIN_HV_MISO, (unsigned long)bb_readback);

            /* Also try GPIO18 in case DOUT is wired there instead */
            Serial.printf("  Also trying GPIO18 (old code used this)...\n");
            pinMode(18, INPUT);

            /* Shift pattern in again */
            for (int bit = 31; bit >= 0; bit--) {
                digitalWrite(PIN_HV_MOSI, (val >> bit) & 1);
                delayMicroseconds(1);
                digitalWrite(PIN_HV_CLK, HIGH);
                delayMicroseconds(1);
                digitalWrite(PIN_HV_CLK, LOW);
                delayMicroseconds(1);
            }
            uint32_t bb_readback18 = 0;
            for (int bit = 31; bit >= 0; bit--) {
                digitalWrite(PIN_HV_MOSI, LOW);
                delayMicroseconds(1);
                int dout = digitalRead(18);
                bb_readback18 |= ((uint32_t)dout << bit);
                digitalWrite(PIN_HV_CLK, HIGH);
                delayMicroseconds(1);
                digitalWrite(PIN_HV_CLK, LOW);
                delayMicroseconds(1);
            }
            Serial.printf("  Bitbang readback (GPIO18): 0x%08lX\n",
                          (unsigned long)bb_readback18);

            /* Clean up — restore HV pins */
            hv_reg[0] = 0;
            hv_write();
        }

        Serial.println("--- End HV2901 Diagnostics ---");
        return;
    }

    /* sw <pair> pos|neg|off — operates on chip 1 (first HV2901) */
    int pair = atoi(arg1);
    if (pair < 0 || pair >= HV_PAIRS_PER_CHIP) {
        Serial.printf("Invalid pair %d (must be 0-15)\n", pair);
        return;
    }

    if (!arg2 || *arg2 == '\0') {
        Serial.println("Usage: sw <pair> pos|neg|off");
        return;
    }

    int state;
    if (strcmp(arg2, "pos") == 0)       state = 0;   // SW_even ON
    else if (strcmp(arg2, "neg") == 0)  state = 1;   // SW_odd ON
    else if (strcmp(arg2, "off") == 0)  state = -1;  // both OFF
    else {
        Serial.printf("Unknown state '%s' (use pos, neg, or off)\n", arg2);
        return;
    }

    hv_set_pair(0, pair, state);  // chip 0 = first HV2901

    int sw_even = pair * 2;
    int sw_odd  = pair * 2 + 1;
    Serial.printf("Pair %d (Y%02d%02d): %s\n", pair, sw_even, sw_odd,
                  state == 0 ? "POS (SW_even ON)" :
                  state == 1 ? "NEG (SW_odd ON)" : "OFF");
}

void cmd_sw2(const char* arg1, const char* arg2)
{
    if (!arg1 || *arg1 == '\0') {
        Serial.println("Usage: sw2 <pair> pos|neg|off  (chip 2, pair 0-15)");
        return;
    }

    if (strcmp(arg1, "clear") == 0) {
        hv_reg[1] = 0;
        hv_write();
        Serial.println("Chip 2: all switches OFF");
        return;
    }

    int pair = atoi(arg1);
    if (pair < 0 || pair >= HV_PAIRS_PER_CHIP) {
        Serial.printf("Invalid pair %d (must be 0-15)\n", pair);
        return;
    }

    if (!arg2 || *arg2 == '\0') {
        Serial.println("Usage: sw2 <pair> pos|neg|off");
        return;
    }

    int state;
    if (strcmp(arg2, "pos") == 0)       state = 0;
    else if (strcmp(arg2, "neg") == 0)  state = 1;
    else if (strcmp(arg2, "off") == 0)  state = -1;
    else {
        Serial.printf("Unknown state '%s' (use pos, neg, or off)\n", arg2);
        return;
    }

    hv_set_pair(1, pair, state);  // chip 1 = second HV2901

    int sw_even = pair * 2;
    int sw_odd  = pair * 2 + 1;
    Serial.printf("Chip2 Pair %d (Y%02d%02d): %s\n", pair, sw_even, sw_odd,
                  state == 0 ? "POS (SW_even ON)" :
                  state == 1 ? "NEG (SW_odd ON)" : "OFF");
}

/* ================================================================
 * Matrix commands (12x12 scan)
 * ================================================================ */

void cmd_mscan()
{
    int32_t matrix[MATRIX_COLS][MATRIX_ROWS];

    /* Invalidate single-channel pipeline since matrix_scan manages its own */
    pipeline_primed = false;

    uint32_t t_start = micros();
    if (!matrix_scan(matrix, false)) {
        Serial.println("ERROR: Matrix scan failed");
        return;
    }
    uint32_t elapsed = micros() - t_start;

    /* Print header */
    Serial.print("        ");
    for (int col = 0; col < MATRIX_COLS; col++) {
        Serial.printf("Col%-5d", col);
    }
    Serial.println();

    /* Print rows with physical channel labels */
    for (int row = 0; row < MATRIX_ROWS; row++) {
        Serial.printf("CH%-4d  ", matrix_row_physical[row]);
        for (int col = 0; col < MATRIX_COLS; col++) {
            Serial.printf("%-8ld", (long)matrix[col][row]);
        }
        Serial.println();
    }
    Serial.printf("Scan time: %lu us\n", elapsed);
}

/*
 * Matrix binary streaming mode.
 */
void cmd_mstream()
{
    Serial.printf("Matrix stream: %d bytes/frame (12x12). Send any byte to stop.\n",
                  MATRIX_FRAME_SIZE);
    Serial.flush();

    delay(10);
    while (Serial.available()) Serial.read();

    /* Invalidate single-channel pipeline */
    pipeline_primed = false;

    /* Stop DAC waveform if running — static bias holds without refresh */
    bool dac_was_running = dac_running;
    if (dac_was_running) dac_stop();

    uint8_t frame[MATRIX_FRAME_SIZE];
    frame[0] = MATRIX_SYNC_0;
    frame[1] = MATRIX_SYNC_1;
    frame[2] = MATRIX_SYNC_2;
    frame[3] = MATRIX_SYNC_3;

    int32_t matrix[MATRIX_COLS][MATRIX_ROWS];
    uint32_t t_start = micros();
    uint32_t seq = 0;

    /* Hold FSPI on HV pins for the entire stream — no per-frame bus switching */
    hv_spi_begin();

    while (true) {
        if ((seq & 15) == 0 && seq > 0 && Serial.available()) {
            while (Serial.available()) Serial.read();
            break;
        }

        if (!matrix_scan(matrix, true, false)) break;

        uint32_t t_us = micros();
        memcpy(frame + 4, &t_us, 4);
        memcpy(frame + 8, &seq, 4);
        memcpy(frame + 12, matrix, MATRIX_COLS * MATRIX_ROWS * 4);
        Serial.write(frame, MATRIX_FRAME_SIZE);
        seq++;
    }

    /* Stop pipeline and restore FSPI to DAC */
    pipeline_primed = false;
    matrix_primed = false;
    digitalWrite(PIN_CONV, LOW);
    hv_spi_end();

    /* Restart DAC waveform if it was running before */
    if (dac_was_running) dac_start();

    Serial.println();
    uint32_t elapsed_us = micros() - t_start;
    float fps = (seq > 0 && elapsed_us > 0) ? (float)seq / (elapsed_us * 1e-6f) : 0;
    Serial.printf("Done: %lu frames in %lu us (%.1f frames/s)\n",
                  (unsigned long)seq, elapsed_us, fps);
}

/*
 * 12x32 full matrix binary streaming.
 */
void cmd_mf32stream()
{
    Serial.printf("Full matrix stream: %d bytes/frame (12x32). Send any byte to stop.\n",
                  FULL_FRAME_SIZE);
    Serial.flush();

    delay(10);
    while (Serial.available()) Serial.read();

    pipeline_primed = false;

    /* Stop DAC waveform if running — static bias holds without refresh */
    bool dac_was_running = dac_running;
    if (dac_was_running) dac_stop();

    uint8_t frame[FULL_FRAME_SIZE];
    frame[0] = FULL_SYNC_0;
    frame[1] = FULL_SYNC_1;
    frame[2] = FULL_SYNC_2;
    frame[3] = FULL_SYNC_3;

    int32_t matrix[FULL_MATRIX_COLS][FULL_MATRIX_ROWS];
    uint32_t t_start = micros();
    uint32_t seq = 0;

    /* Hold FSPI on HV pins for the entire stream */
    hv_spi_begin();

    while (true) {
        if ((seq & 15) == 0 && seq > 0 && Serial.available()) {
            while (Serial.available()) Serial.read();
            break;
        }

        if (!matrix_scan_full(matrix, true, false)) break;

        uint32_t t_us = micros();
        memcpy(frame + 4, &t_us, 4);
        memcpy(frame + 8, &seq, 4);
        memcpy(frame + 12, matrix, FULL_MATRIX_COLS * FULL_MATRIX_ROWS * 4);
        Serial.write(frame, FULL_FRAME_SIZE);
        seq++;
    }

    pipeline_primed = false;
    matrix_primed = false;
    digitalWrite(PIN_CONV, LOW);
    hv_spi_end();

    if (dac_was_running) dac_start();

    Serial.println();
    uint32_t elapsed_us = micros() - t_start;
    float fps = (seq > 0 && elapsed_us > 0) ? (float)seq / (elapsed_us * 1e-6f) : 0;
    Serial.printf("Done: %lu frames in %lu us (%.1f frames/s)\n",
                  (unsigned long)seq, elapsed_us, fps);
}

/*
 * PSRAM matrix capture — acquire 12x12 frames at full speed, then dump.
 */
struct matrix_capture_frame {
    uint32_t timestamp_us;
    uint32_t seq;
    int32_t  data[MATRIX_COLS][MATRIX_ROWS];
};

void cmd_mcapture(const char* arg1)
{
    size_t psram_free = ESP.getFreePsram();
    int max_frames = psram_free / sizeof(matrix_capture_frame);
    if (max_frames < 1) {
        Serial.println("ERROR: No PSRAM available.");
        return;
    }

    int count = arg1 ? atoi(arg1) : max_frames;
    if (count <= 0) count = max_frames;
    if (count > max_frames) {
        Serial.printf("Clamped to %d frames (PSRAM: %u bytes free)\n", max_frames, psram_free);
        count = max_frames;
    }

    matrix_capture_frame* buf = (matrix_capture_frame*)ps_malloc(
        count * sizeof(matrix_capture_frame));
    if (!buf) {
        Serial.println("ERROR: PSRAM allocation failed");
        return;
    }

    Serial.printf("Capturing %d matrix frames to PSRAM (%u bytes)...\n",
                  count, count * (uint32_t)sizeof(matrix_capture_frame));
    Serial.flush();

    /* Invalidate single-channel pipeline */
    pipeline_primed = false;

    uint32_t t_start = micros();

    for (int i = 0; i < count; i++) {
        buf[i].timestamp_us = micros();
        buf[i].seq = i;
        if (!matrix_scan(buf[i].data, true)) {
            count = i;
            break;
        }
    }

    uint32_t t_end = micros();

    /* Stop pipeline */
    pipeline_primed = false;
    matrix_primed = false;
    digitalWrite(PIN_CONV, LOW);

    uint32_t elapsed_us = t_end - t_start;
    float fps = (count > 0 && elapsed_us > 0) ? (float)count / (elapsed_us * 1e-6f) : 0;
    Serial.printf("Captured %d frames in %lu us (%.1f frames/s)\n", count, elapsed_us, fps);

    if (count >= 2) {
        uint32_t dt_min = UINT32_MAX, dt_max = 0;
        uint64_t dt_sum = 0;
        for (int i = 1; i < count; i++) {
            uint32_t dt = buf[i].timestamp_us - buf[i - 1].timestamp_us;
            if (dt < dt_min) dt_min = dt;
            if (dt > dt_max) dt_max = dt;
            dt_sum += dt;
        }
        Serial.printf("  Frame interval: min=%lu us, max=%lu us, avg=%lu us\n",
                      dt_min, dt_max, (uint32_t)(dt_sum / (count - 1)));
    }

    /* Dump as binary MATX frames */
    Serial.printf("Dumping %d frames (%d bytes each)...\n", count, MATRIX_FRAME_SIZE);
    Serial.flush();
    delay(100);

    uint8_t frame[MATRIX_FRAME_SIZE];
    frame[0] = MATRIX_SYNC_0;
    frame[1] = MATRIX_SYNC_1;
    frame[2] = MATRIX_SYNC_2;
    frame[3] = MATRIX_SYNC_3;

    for (int i = 0; i < count; i++) {
        memcpy(frame + 4, &buf[i].timestamp_us, 4);
        memcpy(frame + 8, &buf[i].seq, 4);
        memcpy(frame + 12, buf[i].data, MATRIX_COLS * MATRIX_ROWS * 4);
        Serial.write(frame, MATRIX_FRAME_SIZE);
    }

    free(buf);
    Serial.printf("\nDump complete.\n");
}

/* ================================================================
 * Debug matrix commands (8x12: even pairs x odd channels)
 * ================================================================ */

void cmd_mdscan(const char* arg1)
{
    int avg = arg1 ? atoi(arg1) : 1;
    if (avg < 1) avg = 1;

    int32_t matrix[DBG_MATRIX_COLS][DBG_MATRIX_ROWS];
    pipeline_primed = false;

    Serial.printf("Debug matrix scan: avg=%d samples/col, dwell=%lu us/col\n",
                  avg, (unsigned long)(avg + 2) * integration_us);
    Serial.flush();

    uint32_t t_start = micros();
    if (!matrix_scan_debug(matrix, avg)) {
        Serial.println("ERROR: Debug matrix scan failed");
        return;
    }
    uint32_t elapsed = micros() - t_start;

    /* Header: show actual HV pair numbers */
    Serial.print("          ");
    for (int c = 0; c < DBG_MATRIX_COLS; c++) {
        Serial.printf("P%-7d", dbg_col_pairs[c]);
    }
    Serial.println();

    /* Rows: show physical channel numbers */
    for (int r = 0; r < DBG_MATRIX_ROWS; r++) {
        Serial.printf("CH%-4d    ", dbg_row_physical[r]);
        for (int c = 0; c < DBG_MATRIX_COLS; c++) {
            Serial.printf("%-8ld", (long)matrix[c][r]);
        }
        Serial.println();
    }
    Serial.printf("Scan time: %lu us\n", elapsed);
    Serial.flush();
}

/*
 * Debug matrix binary streaming.
 */
void cmd_mdstream(const char* arg1)
{
    int avg = arg1 ? atoi(arg1) : 1;
    if (avg < 1) avg = 1;

    Serial.printf("Debug matrix stream: %d bytes/frame, avg=%d. Send any byte to stop.\n",
                  DBG_FRAME_SIZE, avg);
    Serial.flush();

    delay(10);
    while (Serial.available()) Serial.read();

    pipeline_primed = false;

    bool dac_was_running = dac_running;
    if (dac_was_running) dac_stop();

    uint8_t frame[DBG_FRAME_SIZE];
    frame[0] = DBG_SYNC_0;
    frame[1] = DBG_SYNC_1;
    frame[2] = DBG_SYNC_2;
    frame[3] = DBG_SYNC_3;

    int32_t matrix[DBG_MATRIX_COLS][DBG_MATRIX_ROWS];
    uint32_t t_start = micros();
    uint32_t seq = 0;

    while (true) {
        if ((seq & 15) == 0 && seq > 0 && Serial.available()) {
            while (Serial.available()) Serial.read();
            break;
        }

        if (!matrix_scan_debug(matrix, avg)) break;

        uint32_t t_us = micros();
        memcpy(frame + 4, &t_us, 4);
        memcpy(frame + 8, &seq, 4);
        memcpy(frame + 12, matrix, DBG_MATRIX_COLS * DBG_MATRIX_ROWS * 4);
        Serial.write(frame, DBG_FRAME_SIZE);
        seq++;
    }

    pipeline_primed = false;
    digitalWrite(PIN_CONV, LOW);

    if (dac_was_running) dac_start();

    Serial.println();
    uint32_t elapsed_us = micros() - t_start;
    float fps = (seq > 0 && elapsed_us > 0) ? (float)seq / (elapsed_us * 1e-6f) : 0;
    Serial.printf("Done: %lu frames in %lu us (%.1f frames/s)\n",
                  (unsigned long)seq, elapsed_us, fps);
}

struct dbg_capture_frame {
    uint32_t timestamp_us;
    uint32_t seq;
    int32_t  data[DBG_MATRIX_COLS][DBG_MATRIX_ROWS];
};

void cmd_mdcapture(const char* arg1, const char* arg2)
{
    int avg = arg2 ? atoi(arg2) : 1;
    if (avg < 1) avg = 1;

    size_t psram_free = ESP.getFreePsram();
    int max_frames = psram_free / sizeof(dbg_capture_frame);
    if (max_frames < 1) {
        Serial.println("ERROR: No PSRAM available.");
        return;
    }

    int count = arg1 ? atoi(arg1) : max_frames;
    if (count <= 0) count = max_frames;
    if (count > max_frames) {
        Serial.printf("Clamped to %d frames (PSRAM: %u bytes free)\n", max_frames, psram_free);
        count = max_frames;
    }

    dbg_capture_frame* buf = (dbg_capture_frame*)ps_malloc(
        count * sizeof(dbg_capture_frame));
    if (!buf) {
        Serial.println("ERROR: PSRAM allocation failed");
        return;
    }

    Serial.printf("Capturing %d debug matrix frames to PSRAM (avg=%d)...\n", count, avg);
    Serial.flush();

    pipeline_primed = false;

    uint32_t t_start = micros();
    for (int i = 0; i < count; i++) {
        buf[i].timestamp_us = micros();
        buf[i].seq = i;
        if (!matrix_scan_debug(buf[i].data, avg)) {
            count = i;
            break;
        }
    }
    uint32_t t_end = micros();

    pipeline_primed = false;
    digitalWrite(PIN_CONV, LOW);

    uint32_t elapsed_us = t_end - t_start;
    float fps = (count > 0 && elapsed_us > 0) ? (float)count / (elapsed_us * 1e-6f) : 0;
    Serial.printf("Captured %d frames in %lu us (%.1f frames/s)\n", count, elapsed_us, fps);

    if (count >= 2) {
        uint32_t dt_min = UINT32_MAX, dt_max = 0;
        uint64_t dt_sum = 0;
        for (int i = 1; i < count; i++) {
            uint32_t dt = buf[i].timestamp_us - buf[i - 1].timestamp_us;
            if (dt < dt_min) dt_min = dt;
            if (dt > dt_max) dt_max = dt;
            dt_sum += dt;
        }
        Serial.printf("  Frame interval: min=%lu us, max=%lu us, avg=%lu us\n",
                      dt_min, dt_max, (uint32_t)(dt_sum / (count - 1)));
    }

    Serial.printf("Dumping %d frames (%d bytes each)...\n", count, DBG_FRAME_SIZE);
    Serial.flush();
    delay(100);

    uint8_t frame[DBG_FRAME_SIZE];
    frame[0] = DBG_SYNC_0;
    frame[1] = DBG_SYNC_1;
    frame[2] = DBG_SYNC_2;
    frame[3] = DBG_SYNC_3;

    for (int i = 0; i < count; i++) {
        memcpy(frame + 4, &buf[i].timestamp_us, 4);
        memcpy(frame + 8, &buf[i].seq, 4);
        memcpy(frame + 12, buf[i].data, DBG_MATRIX_COLS * DBG_MATRIX_ROWS * 4);
        Serial.write(frame, DBG_FRAME_SIZE);
    }

    free(buf);
    Serial.printf("\nDump complete.\n");
}

/* ================================================================
 * 12x12 Debug matrix commands (even pairs across 2 chips x odd channels)
 * ================================================================ */

void cmd_md12scan(const char* arg1)
{
    int avg = arg1 ? atoi(arg1) : 1;
    if (avg < 1) avg = 1;

    int32_t matrix[DBG12_MATRIX_COLS][DBG12_MATRIX_ROWS];
    pipeline_primed = false;

    Serial.printf("12x12 debug matrix scan: avg=%d samples/col, dwell=%lu us/col\n",
                  avg, (unsigned long)(avg + 2) * integration_us);
    Serial.flush();

    uint32_t t_start = micros();
    if (!matrix_scan_debug12(matrix, avg)) {
        Serial.println("ERROR: 12x12 debug matrix scan failed");
        return;
    }
    uint32_t elapsed = micros() - t_start;

    /* Header: chip:pair labels */
    Serial.print("          ");
    for (int c = 0; c < DBG12_MATRIX_COLS; c++) {
        Serial.printf("C%d:P%-4d", dbg12_col_chip[c], dbg12_col_pair[c]);
    }
    Serial.println();

    for (int r = 0; r < DBG12_MATRIX_ROWS; r++) {
        Serial.printf("CH%-4d    ", dbg_row_physical[r]);
        for (int c = 0; c < DBG12_MATRIX_COLS; c++) {
            Serial.printf("%-8ld", (long)matrix[c][r]);
        }
        Serial.println();
    }
    Serial.printf("Scan time: %lu us\n", elapsed);
    Serial.flush();
}

void cmd_md12stream(const char* arg1)
{
    int avg = arg1 ? atoi(arg1) : 1;
    if (avg < 1) avg = 1;

    Serial.printf("12x12 debug matrix stream: %d bytes/frame, avg=%d. Send any byte to stop.\n",
                  DBG12_FRAME_SIZE, avg);
    Serial.flush();

    delay(10);
    while (Serial.available()) Serial.read();

    pipeline_primed = false;

    bool dac_was_running = dac_running;
    if (dac_was_running) dac_stop();

    uint8_t frame[DBG12_FRAME_SIZE];
    frame[0] = DBG12_SYNC_0;
    frame[1] = DBG12_SYNC_1;
    frame[2] = DBG12_SYNC_2;
    frame[3] = DBG12_SYNC_3;

    int32_t matrix[DBG12_MATRIX_COLS][DBG12_MATRIX_ROWS];
    uint32_t t_start = micros();
    uint32_t seq = 0;

    while (true) {
        if ((seq & 15) == 0 && seq > 0 && Serial.available()) {
            while (Serial.available()) Serial.read();
            break;
        }

        if (!matrix_scan_debug12(matrix, avg)) break;

        uint32_t t_us = micros();
        memcpy(frame + 4, &t_us, 4);
        memcpy(frame + 8, &seq, 4);
        memcpy(frame + 12, matrix, DBG12_MATRIX_COLS * DBG12_MATRIX_ROWS * 4);
        Serial.write(frame, DBG12_FRAME_SIZE);
        seq++;
    }

    pipeline_primed = false;
    digitalWrite(PIN_CONV, LOW);

    if (dac_was_running) dac_start();

    Serial.println();
    uint32_t elapsed_us = micros() - t_start;
    float fps = (seq > 0 && elapsed_us > 0) ? (float)seq / (elapsed_us * 1e-6f) : 0;
    Serial.printf("Done: %lu frames in %lu us (%.1f frames/s)\n",
                  (unsigned long)seq, elapsed_us, fps);
}

struct dbg12_capture_frame {
    uint32_t timestamp_us;
    uint32_t seq;
    int32_t  data[DBG12_MATRIX_COLS][DBG12_MATRIX_ROWS];
};

void cmd_md12capture(const char* arg1, const char* arg2)
{
    int avg = arg2 ? atoi(arg2) : 1;
    if (avg < 1) avg = 1;

    size_t psram_free = ESP.getFreePsram();
    int max_frames = psram_free / sizeof(dbg12_capture_frame);
    if (max_frames < 1) {
        Serial.println("ERROR: No PSRAM available.");
        return;
    }

    int count = arg1 ? atoi(arg1) : max_frames;
    if (count <= 0) count = max_frames;
    if (count > max_frames) {
        Serial.printf("Clamped to %d frames (PSRAM: %u bytes free)\n", max_frames, psram_free);
        count = max_frames;
    }

    dbg12_capture_frame* buf = (dbg12_capture_frame*)ps_malloc(
        count * sizeof(dbg12_capture_frame));
    if (!buf) {
        Serial.println("ERROR: PSRAM allocation failed");
        return;
    }

    Serial.printf("Capturing %d 12x12 debug matrix frames to PSRAM (avg=%d)...\n", count, avg);
    Serial.flush();

    pipeline_primed = false;

    uint32_t t_start = micros();
    for (int i = 0; i < count; i++) {
        buf[i].timestamp_us = micros();
        buf[i].seq = i;
        if (!matrix_scan_debug12(buf[i].data, avg)) {
            count = i;
            break;
        }
    }
    uint32_t t_end = micros();

    pipeline_primed = false;
    digitalWrite(PIN_CONV, LOW);

    uint32_t elapsed_us = t_end - t_start;
    float fps = (count > 0 && elapsed_us > 0) ? (float)count / (elapsed_us * 1e-6f) : 0;
    Serial.printf("Captured %d frames in %lu us (%.1f frames/s)\n", count, elapsed_us, fps);

    if (count >= 2) {
        uint32_t dt_min = UINT32_MAX, dt_max = 0;
        uint64_t dt_sum = 0;
        for (int i = 1; i < count; i++) {
            uint32_t dt = buf[i].timestamp_us - buf[i - 1].timestamp_us;
            if (dt < dt_min) dt_min = dt;
            if (dt > dt_max) dt_max = dt;
            dt_sum += dt;
        }
        Serial.printf("  Frame interval: min=%lu us, max=%lu us, avg=%lu us\n",
                      dt_min, dt_max, (uint32_t)(dt_sum / (count - 1)));
    }

    Serial.printf("Dumping %d frames (%d bytes each)...\n", count, DBG12_FRAME_SIZE);
    Serial.flush();
    delay(100);

    uint8_t frame[DBG12_FRAME_SIZE];
    frame[0] = DBG12_SYNC_0;
    frame[1] = DBG12_SYNC_1;
    frame[2] = DBG12_SYNC_2;
    frame[3] = DBG12_SYNC_3;

    for (int i = 0; i < count; i++) {
        memcpy(frame + 4, &buf[i].timestamp_us, 4);
        memcpy(frame + 8, &buf[i].seq, 4);
        memcpy(frame + 12, buf[i].data, DBG12_MATRIX_COLS * DBG12_MATRIX_ROWS * 4);
        Serial.write(frame, DBG12_FRAME_SIZE);
    }

    free(buf);
    Serial.printf("\nDump complete.\n");
}

/* ================================================================
 * Settings and help
 * ================================================================ */

void cmd_settings()
{
    /* Machine-parseable key=value output for GUI logging header */
    Serial.println("SETTINGS_BEGIN");
    Serial.printf("range=%d\n", current_range);
    Serial.printf("range_pC=%.1f\n",
                  (current_range <= 7) ? range_pc[current_range] : 0.0f);
    Serial.printf("integration_us=%lu\n", (unsigned long)integration_us);
    Serial.printf("matrix_dead_us=%lu\n", (unsigned long)matrix_dead_us);
    Serial.printf("hv_skip_vneg=0x%04X\n", hv_skip_vneg);
    Serial.printf("clk_hz=%lu\n", (unsigned long)clk_freq_hz);
    Serial.printf("clk_4x=%d\n", clk_4x ? 1 : 0);
    Serial.printf("test_mode=%d\n", test_mode ? 1 : 0);
    Serial.printf("vbias1=%.4f\n", last_vbias1);
    Serial.printf("vbias2=%.4f\n", last_vbias2);
    Serial.printf("vsw1=%.4f\n", last_vsw1);
    Serial.printf("vsw2=%.4f\n", last_vsw2);
    /* HV switch register (hex) */
    Serial.printf("hv_reg0=0x%08lX\n", (unsigned long)hv_reg[0]);
    Serial.printf("hv_reg1=0x%08lX\n", (unsigned long)hv_reg[1]);
    Serial.println("SETTINGS_END");
}

void cmd_help()
{
    Serial.println("Commands:");
    Serial.println("  read              - Single readout of all 32 channels");
    Serial.println("  range <0-7>       - Set full-scale charge range");
    Serial.println("  inttime <us>      - Set integration time (microseconds)");
    Serial.println("  deadtime [us]     - Set/show matrix column dead time (0=off)");
    Serial.println("  skipvneg [pair] [off] - Skip Vneg for a pair (use OFF instead)");
    Serial.println("  clk <hz>          - Set system clock frequency");
    Serial.println("  test <on|off>     - Enable/disable internal test mode");
    Serial.println("  continuous [n] [ms] - Read n samples (CSV text)");
    Serial.println("  stream [n]        - Binary stream (max speed)");
    Serial.println("  capture [n]       - Capture to PSRAM, then dump (measures true HW speed)");
    Serial.println("  vbias1 <V>        - Set VBias1 (-5 to +5 V)");
    Serial.println("  vbias2 <V>        - Set VBias2 (-5 to +5 V)");
    Serial.println("  vsw1 <V>          - Set switch voltage 1 (0 to +30 V)");
    Serial.println("  vsw2 <V>          - Set switch voltage 2 (-30 to 0 V)");
    Serial.println("  dac on|off        - Start/stop DAC8562 sine wave output");
    Serial.println("  dac freq1|freq2 <hz> - Set DAC1/DAC2 sine frequency");
    Serial.println("  dac status        - Show DAC settings");
    Serial.println("  sw <pair> pos|neg|off - Set HV switch pair (0-15)");
    Serial.println("  sw allpos|allneg  - Set all pairs to POS or NEG");
    Serial.println("  sw status         - Show all switch states");
    Serial.println("  sw clear          - All switches OFF");
    Serial.println("  sw toggle <pair> [hz] - Toggle pair POS/OFF (default 10 Hz)");
    Serial.println("  sw diag           - Toggle HV2901 pins for scope/DMM check");
    Serial.println("  sw2 <pair> pos|neg|off - Set HV switch pair on chip 2");
    Serial.println("  mscan             - Single 12x12 matrix scan (text)");
    Serial.println("  mstream           - Matrix binary stream (12x12, 588 B/frame)");
    Serial.println("  mf32stream        - Full matrix stream (12x32, 1548 B/frame)");
    Serial.println("  mcapture [n]      - Capture matrix frames to PSRAM");
    Serial.println("  mdscan [avg]      - Debug 8x12 matrix scan (avg samples/col)");
    Serial.println("  mdstream [avg]    - Debug matrix stream (avg samples/col)");
    Serial.println("  mdcapture [n] [avg] - Capture debug matrix frames to PSRAM");
    Serial.println("  md12scan [avg]    - 12x12 debug scan (even pairs, 2 chips)");
    Serial.println("  md12stream [avg]  - 12x12 debug stream (588 B/frame)");
    Serial.println("  md12capture [n] [avg] - Capture 12x12 debug frames to PSRAM");
    Serial.println("  readcfg           - Write + read back config register");
    Serial.println("  settings          - Dump all settings (machine-parseable)");
    Serial.println("  scope             - Slow signal test for oscilloscope");
    Serial.println("  diag              - Run hardware diagnostics");
    Serial.println("  help              - Show this message");
}
