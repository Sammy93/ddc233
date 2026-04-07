#include "ddc232_readout.h"

/* ================================================================
 * Per-phase timing accumulators (for diagnostics)
 * ================================================================ */

uint32_t dbg_wait_us = 0, dbg_dvalid_us = 0, dbg_spi_us = 0, dbg_unpack_us = 0;
int dbg_samples = 0;

/* ================================================================
 * System CLK via LEDC
 * ================================================================ */

void clk_start(uint8_t pin, uint32_t freq_hz)
{
    /* Arduino ESP32 core 3.x API */
    ledcAttach(pin, freq_hz, 1);  // 1-bit resolution -> 50% duty
    ledcWrite(pin, 1);            // duty = 1 out of 2
}

void clk_set_freq(uint8_t pin, uint32_t freq_hz)
{
    ledcWriteTone(pin, freq_hz);
    /* ledcWriteTone sets 50% duty automatically */
}

void clk_stop(uint8_t pin)
{
    ledcDetach(pin);
    digitalWrite(pin, LOW);
}

/* ================================================================
 * Configuration register (12-bit, bit-bang via DIN_CFG / CLK_CFG)
 * ================================================================
 *
 *   Bits 11-9 : FSR (full-scale range, 0-7)
 *   Bit  8    : FORMAT (1 = 20-bit)
 *   Bit  7    : VERSION (0 = DDC232C, 1 = DDC232CK)
 *   Bit  6    : CLK_4x
 *   Bits 5-1  : Reserved (0)
 *   Bit  0    : TEST
 */

void write_config_register()
{
    uint16_t reg = 0;
    reg |= (uint16_t)(current_range & 0x07) << 9;  // bits 11-9: FSR
    reg |= (1 << 8);                                // bit 8: FORMAT = 20-bit
    reg |= (1 << 7);                                // bit 7: VERSION = 1 (DDC232CK)
    if (clk_4x)    reg |= (1 << 6);
    if (test_mode) reg |= (1 << 0);

    /* Per datasheet: hold CONV low, strobe RESET to begin config write.
     * NOTE: CLK must be running during this entire sequence.
     * tRST    >= 1 ms   (RESET pulse width)
     * tWTRST  >= 2 ms   (RESET high to first CLK_CFG rising edge)  */
    digitalWrite(PIN_CONV, LOW);
    delayMicroseconds(1);
    digitalWrite(PIN_RESET, LOW);
    delay(2);                  // tRST: hold reset >= 1 ms
    digitalWrite(PIN_RESET, HIGH);
    delay(3);                  // tWTRST: wait >= 2 ms before CLK_CFG

    digitalWrite(PIN_CLK_CFG, LOW);

    /* Clock out 12 bits MSB-first; DDC232 latches on falling edge of CLK_CFG */
    for (int i = CFG_REG_BITS - 1; i >= 0; i--) {
        digitalWrite(PIN_DIN_CFG, (reg >> i) & 1);
        delayMicroseconds(1);

        digitalWrite(PIN_CLK_CFG, HIGH);
        delayMicroseconds(1);

        digitalWrite(PIN_CLK_CFG, LOW);   // latch
        delayMicroseconds(1);
    }
    digitalWrite(PIN_DIN_CFG, LOW);

    Serial.printf("Config register written: 0x%03X (range=%d/%s, 20bit, test=%d, clk4x=%d)\n",
                  reg, current_range, range_labels[current_range],
                  test_mode, clk_4x);
}

/* ================================================================
 * Dummy conversion to flush stale integrator state
 * ================================================================ */

void ddc232_flush()
{
    pipeline_primed = false;
    digitalWrite(PIN_CONV, HIGH);
    delayMicroseconds(integration_us);
    digitalWrite(PIN_CONV, LOW);

    uint32_t start = micros();
    while (digitalRead(PIN_DVALID) != LOW) {
        if (micros() - start > 200000) {
            Serial.println("WARNING: Timeout on dummy conversion — check wiring and CLK");
            return;
        }
    }

    /* Discard the data */
    uint8_t discard[80];
    hspi.beginTransaction(SPISettings(SPI_CLK_HZ, MSBFIRST, SPI_MODE0));
    for (int i = 0; i < 80; i++) {
        discard[i] = hspi.transfer(0x00);
    }
    hspi.endTransaction();
}

/**
 * Write config register and strobe CONV to enter normal operation.
 * Use this for runtime config changes (range, test mode, etc.).
 * Do NOT use this when readback is needed — use write_and_readback_config() instead.
 */
void apply_config()
{
    write_config_register();

    /* Per datasheet: strobe CONV to begin normal operation after config write */
    digitalWrite(PIN_CONV, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_CONV, LOW);

    /* Flush stale integrator data from the previous config */
    ddc232_flush();
}

/* ================================================================
 * Read back configuration register via DCLK/DOUT (SPI)
 * ================================================================
 *
 * Per datasheet: after config write, wait tWTWR (>= 2 ms), then
 * clock DCLK and read DOUT on rising edges. Output is:
 *   - 12 bits: config register contents
 *   -  4 bits: revision ID
 *   - test pattern (304 bits for Format=1)
 * Total 640 bits for Format=1 (20-bit mode), repeated x2.
 */

/* Readback is only available immediately after a write sequence.
 * This function does a full write -> wait tWTWR -> read -> strobe CONV cycle. */
void write_and_readback_config()
{
    write_config_register();

    /* tWTWR: wait >= 2 ms from last CLK_CFG to first DCLK */
    delay(3);

    /* Read 2 bytes = 16 bits: 12 config + 4 revision ID */
    uint8_t rx[2] = {0, 0};
    hspi.beginTransaction(SPISettings(SPI_CLK_HZ, MSBFIRST, SPI_MODE0));
    rx[0] = hspi.transfer(0x00);
    rx[1] = hspi.transfer(0x00);
    hspi.endTransaction();

    uint16_t raw = ((uint16_t)rx[0] << 8) | rx[1];
    uint16_t config  = (raw >> 4) & 0x0FFF;  // top 12 bits
    uint8_t  rev_id  = raw & 0x0F;           // bottom 4 bits

    Serial.printf("Config readback: 0x%03X  Rev ID: 0x%X\n", config, rev_id);

    /* Decode fields */
    uint8_t fsr    = (config >> 9) & 0x07;
    uint8_t format = (config >> 8) & 0x01;
    uint8_t ver    = (config >> 7) & 0x01;
    uint8_t clk4x  = (config >> 6) & 0x01;
    uint8_t tst    = config & 0x01;

    Serial.printf("  FSR=%d (%s), FORMAT=%d, VER=%d, CLK4X=%d, TEST=%d\n",
                  fsr, (fsr <= 7) ? range_labels[fsr] : "?",
                  format, ver, clk4x, tst);

    /* Per datasheet: "Strobe CONV to begin normal operation" after readback */
    digitalWrite(PIN_CONV, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_CONV, LOW);
}

/* ================================================================
 * Hardware diagnostics
 * ================================================================ */

void cmd_diag()
{
    Serial.println("--- DDC232/233 Hardware Diagnostics ---");

    /* 1. Current pin states */
    int dvalid_state = digitalRead(PIN_DVALID);
    int reset_state  = digitalRead(PIN_RESET);
    Serial.printf("  RESET pin (GPIO%d):  %s\n", PIN_RESET,
                  reset_state ? "HIGH (released)" : "LOW (held in reset!)");
    Serial.printf("  DVALID pin (GPIO%d): %s\n", PIN_DVALID,
                  dvalid_state ? "HIGH (no data ready)" : "LOW (data ready)");
    Serial.printf("  CONV pin (GPIO%d):   %d\n", PIN_CONV,
                  digitalRead(PIN_CONV));

    /* 2. CLK status */
    Serial.printf("  CLK pin (GPIO%d):    configured for %lu Hz via LEDC (verify with scope)\n",
                  PIN_CLK, clk_freq_hz);

    /* 3. Probe DVALID response to a CONV pulse */
    Serial.println("  Testing CONV -> DVALID response...");
    digitalWrite(PIN_CONV, HIGH);
    delayMicroseconds(integration_us);
    digitalWrite(PIN_CONV, LOW);

    uint32_t t0 = micros();
    bool dvalid_seen = false;
    while (micros() - t0 < 50000) {  // 50 ms timeout
        if (digitalRead(PIN_DVALID) == LOW) {
            uint32_t latency = micros() - t0;
            Serial.printf("  DVALID asserted %lu us after CONV falling edge — chip is alive!\n",
                          latency);
            dvalid_seen = true;

            /* Clock out and discard the data so chip is ready for next cycle */
            hspi.beginTransaction(SPISettings(SPI_CLK_HZ, MSBFIRST, SPI_MODE0));
            for (int i = 0; i < 80; i++) hspi.transfer(0x00);
            hspi.endTransaction();
            break;
        }
    }
    if (!dvalid_seen) {
        Serial.println("  ** DVALID never asserted (50 ms timeout) **");
        Serial.println("  Possible causes:");
        Serial.println("    - CLK not reaching DDC232 (check wiring to CLK pin)");
        Serial.println("    - Chip still in reset (check RESET pin/wiring)");
        Serial.println("    - Power supply issue (check VDD, AVDD, DVDD)");
        Serial.println("    - CONV not reaching DDC232 (check wiring)");
        Serial.println("    - DVALID not connected or pulled wrong direction");
    }

    /* 4. Report current config (what we WROTE, can't read back) */
    uint16_t reg = 0;
    reg |= (uint16_t)(current_range & 0x07) << 9;
    reg |= (1 << 8);
    reg |= (1 << 7);  // VERSION = 1 (DDC232CK)
    if (clk_4x)    reg |= (1 << 6);
    if (test_mode) reg |= (1 << 0);
    Serial.printf("  Config written: 0x%03X (range=%d/%s, 20bit, test=%d, clk4x=%d)\n",
                  reg, current_range, range_labels[current_range],
                  test_mode, clk_4x);
    Serial.println("  (Note: DDC232/233 config register is write-only, no readback available)");
    Serial.println("--- End Diagnostics ---");
}

/* ================================================================
 * Scope test — slow config write + readback with serial prompts
 * ================================================================ */

void cmd_scope()
{
    Serial.println("=== SCOPE TEST ===");
    Serial.println("Probe these signals:");
    Serial.println("  CH1: CLK_CFG  (GPIO5)  - config clock");
    Serial.println("  CH2: DIN_CFG  (GPIO6)  - config data");
    Serial.println("  CH3: DCLK     (GPIO12) - SPI clock (readback)");
    Serial.println("  CH4: DOUT     (GPIO9)  - SPI data from DDC (readback)");
    Serial.println();

    /* ---- TEST 1: Verify individual GPIO control ---- */
    Serial.println("[Test 1] Toggling each GPIO 5 times at 10 Hz...");

    Serial.println("  Toggling CLK_CFG (GPIO5)...");
    for (int i = 0; i < 5; i++) {
        digitalWrite(PIN_CLK_CFG, HIGH);
        delay(50);
        digitalWrite(PIN_CLK_CFG, LOW);
        delay(50);
    }

    Serial.println("  Toggling DIN_CFG (GPIO6)...");
    for (int i = 0; i < 5; i++) {
        digitalWrite(PIN_DIN_CFG, HIGH);
        delay(50);
        digitalWrite(PIN_DIN_CFG, LOW);
        delay(50);
    }

    Serial.println("  Toggling RESET (GPIO15)...");
    for (int i = 0; i < 5; i++) {
        digitalWrite(PIN_RESET, LOW);
        delay(50);
        digitalWrite(PIN_RESET, HIGH);
        delay(50);
    }

    Serial.println("  Toggling CONV (GPIO8)...");
    for (int i = 0; i < 5; i++) {
        digitalWrite(PIN_CONV, HIGH);
        delay(50);
        digitalWrite(PIN_CONV, LOW);
        delay(50);
    }

    /* ---- TEST 2: Slow config write (visible on scope) ---- */
    Serial.println();
    Serial.println("[Test 2] Slow config write — 0x381 (10 ms per bit)");
    Serial.println("  Trigger on CLK_CFG rising edge. Expect 12 clock pulses.");
    Serial.println("  DIN_CFG pattern MSB-first: 0 0 1 1 1 0 0 0 0 0 0 1");
    Serial.println("  Starting in 2 seconds...");
    delay(2000);

    uint16_t reg = 0x381;  // range=1, format=1, version=1, test=1

    /* RESET strobe */
    digitalWrite(PIN_CONV, LOW);
    digitalWrite(PIN_RESET, LOW);
    delay(2);   // tRST >= 1 ms
    digitalWrite(PIN_RESET, HIGH);
    delay(3);   // tWTRST >= 2 ms

    /* Clock out 12 bits, 10 ms per bit so it's visible on scope */
    Serial.println("  Clocking 12 bits...");
    for (int i = CFG_REG_BITS - 1; i >= 0; i--) {
        int bit = (reg >> i) & 1;
        digitalWrite(PIN_DIN_CFG, bit);
        delay(5);

        digitalWrite(PIN_CLK_CFG, HIGH);
        delay(5);

        digitalWrite(PIN_CLK_CFG, LOW);
        delay(5);

        Serial.printf("    Bit %2d: DIN_CFG=%d\n", i, bit);
    }
    digitalWrite(PIN_DIN_CFG, LOW);
    Serial.println("  Write complete.");

    /* ---- TEST 3: SPI readback ---- */
    Serial.println();
    Serial.println("[Test 3] SPI readback — watch DCLK (GPIO12) and DOUT (GPIO9)");
    Serial.println("  tWTWR wait (3 ms)...");
    delay(3);

    Serial.println("  Clocking 16 bits on DCLK, reading DOUT...");
    uint8_t rx[2] = {0, 0};
    hspi.beginTransaction(SPISettings(100000, MSBFIRST, SPI_MODE0));  // 100 kHz — slow for scope
    rx[0] = hspi.transfer(0x00);
    rx[1] = hspi.transfer(0x00);
    hspi.endTransaction();

    uint16_t raw = ((uint16_t)rx[0] << 8) | rx[1];
    Serial.printf("  Raw bytes: 0x%02X 0x%02X (16-bit: 0x%04X)\n", rx[0], rx[1], raw);
    Serial.printf("  Config (top 12): 0x%03X  Rev ID (bottom 4): 0x%X\n",
                  (raw >> 4) & 0x0FFF, raw & 0x0F);

    /* CONV strobe to return to normal operation */
    digitalWrite(PIN_CONV, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_CONV, LOW);

    Serial.println();
    Serial.println("=== SCOPE TEST COMPLETE ===");
    Serial.println("If CLK_CFG and DIN_CFG show correct signals at the ESP32 pins");
    Serial.println("but not at the DDC232, check trace continuity.");
    Serial.println("If DOUT is always low during readback, the DDC may not be");
    Serial.println("receiving the config write (check DIN_CFG/CLK_CFG at the chip).");
}

/* DDC232 20-bit output:
 *   Code 0x00000 (0)      = zero input (confirmed by test mode)
 *   Code 0xFFFFF (1048575)= positive full scale
 * We output raw unsigned 20-bit values; conversion happens in GUI. */

/* ================================================================
 * DDC232 pipelined read cycle (dual integrator)
 * ================================================================ */

bool ddc232_prime()
{
    /* Prime the dual integrator pipeline:
     * Toggle 1: start integration on side A (discard stale data)
     * Toggle 2: start integration on side B, side A data available
     */

    /* Toggle 1: CONV HIGH — start side A integration */
    conv_state = true;
    digitalWrite(PIN_CONV, HIGH);
    delayMicroseconds(integration_us);

    /* Toggle 2: CONV LOW — end side A, start side B, side A data available */
    conv_state = false;
    digitalWrite(PIN_CONV, LOW);

    /* Wait for DVALID — discard stale side A data */
    uint32_t start = micros();
    while (digitalRead(PIN_DVALID) != LOW) {
        if (micros() - start > 200000) {
            Serial.println("ERROR: Timeout priming pipeline (DVALID)");
            return false;
        }
    }

    uint8_t discard[80], zeros[80];
    memset(zeros, 0, sizeof(zeros));
    hspi.beginTransaction(SPISettings(SPI_CLK_HZ, MSBFIRST, SPI_MODE0));
    hspi.transferBytes(zeros, discard, 80);
    hspi.endTransaction();

    /* Side B is now integrating. Record the toggle time so the first
     * ddc232_read() knows how much integration time remains. */
    conv_toggle_time = micros();
    pipeline_primed = true;
    return true;
}

/*
 * Pipelined read — maximizes overlap between integration and readout.
 */
bool ddc232_read(int32_t data[NUM_CHANNELS])
{
    if (!pipeline_primed) {
        if (!ddc232_prime()) return false;
    }

    /* Ensure minimum integration time has elapsed since last CONV toggle */
    uint32_t t0 = micros();
    uint32_t elapsed = t0 - conv_toggle_time;
    if (elapsed < integration_us) {
        delayMicroseconds(integration_us - elapsed);
    }

    /* Toggle CONV: this simultaneously:
     *  - Ends integration on current side (data becomes available after tCONV)
     *  - Starts integration on the other side (runs during our readout) */
    uint32_t t1 = micros();
    conv_state = !conv_state;
    digitalWrite(PIN_CONV, conv_state ? HIGH : LOW);
    conv_toggle_time = micros();
    /* >>> Other side is now integrating — clock is ticking <<< */

    /* Wait for DVALID low (completed side's data ready after tCONV) */
    uint32_t start = micros();
    while (digitalRead(PIN_DVALID) != LOW) {
        if (micros() - start > 200000) {
            Serial.println("ERROR: Timeout waiting for DVALID");
            pipeline_primed = false;
            return false;
        }
    }
    uint32_t t2 = micros();

    /* SPI readout: 640 bits = 80 bytes (bulk transfer)
     * Integration on the other side continues during this */
    uint8_t rx_buf[80];
    uint8_t tx_buf[80];
    memset(tx_buf, 0, sizeof(tx_buf));

    hspi.beginTransaction(SPISettings(SPI_CLK_HZ, MSBFIRST, SPI_MODE0));
    hspi.transferBytes(tx_buf, rx_buf, 80);
    hspi.endTransaction();
    uint32_t t3 = micros();

    /* Unpack 20-bit words from byte stream */
    for (int ch = 0; ch < NUM_CHANNELS; ch++) {
        int bit_offset = ch * 20;
        int byte_idx   = bit_offset / 8;
        int bit_shift  = bit_offset % 8;

        uint32_t raw = ((uint32_t)rx_buf[byte_idx]     << 24) |
                       ((uint32_t)rx_buf[byte_idx + 1] << 16) |
                       ((uint32_t)rx_buf[byte_idx + 2] <<  8) |
                       ((uint32_t)(byte_idx + 3 < 80 ? rx_buf[byte_idx + 3] : 0));

        raw <<= bit_shift;
        raw >>= (32 - 20);

        data[ch] = (int32_t)raw;  // raw offset binary, 0-1048575
    }
    uint32_t t4 = micros();

    /* Accumulate timing */
    dbg_wait_us   += (t1 - t0);
    dbg_dvalid_us += (t2 - t1);
    dbg_spi_us    += (t3 - t2);
    dbg_unpack_us += (t4 - t3);
    dbg_samples++;

    return true;
}
