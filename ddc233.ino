/*
 * DDC232/233 Charge-to-Digital Converter Readout
 *
 * Arduino sketch for ESP32-S3.
 * Reads 32 channels of 20-bit charge data via SPI, with serial
 * console commands for configuration and continuous sampling.
 *
 * Board: ESP32-S3 Dev Module (Arduino ESP32 core 3.x)
 * Upload Speed: 921600
 * USB CDC On Boot: Enabled
 */

#include <SPI.h>

/* ---- Pin assignment (adjust to your PCB) ---- */
#define PIN_CLK        14   // LEDC PWM -> DDC system clock
#define PIN_CONV       8    // GPIO out -> DDC CONV
#define PIN_DVALID     7    // GPIO in  <- DDC DVALID (active-low)
#define PIN_DCLK       12   // SPI CLK  -> DDC DCLK
#define PIN_DOUT       13   // SPI MISO <- DDC DOUT
#define PIN_DIN        11   // SPI MOSI -> DDC DIN
#define PIN_DIN_CFG    6    // GPIO out -> DDC DIN_CFG (config data)
#define PIN_CLK_CFG    5    // GPIO out -> DDC CLK_CFG (config clock)
#define PIN_RESET      15   // GPIO out -> DDC RESET (active-low)

/* ---- Operating parameters ---- */
#define DEFAULT_CLK_HZ          10000000  // 10 MHz system clock
#define DEFAULT_INTEGRATION_US  1000      // 1 ms
#define DEFAULT_RANGE           1         // 50 pC

#define NUM_CHANNELS  32
#define DATA_BITS     20
#define CFG_REG_BITS  12
#define SPI_CLK_HZ    20000000  // 20 MHz DCLK for data readout (DDC232 max)

/* ---- Range labels ---- */
static const char* range_labels[] = {
    "12.5 pC", "50 pC", "100 pC", "150 pC",
    "200 pC",  "250 pC", "300 pC", "350 pC"
};

/* ---- Device state ---- */
static uint32_t clk_freq_hz    = DEFAULT_CLK_HZ;
static uint32_t integration_us = DEFAULT_INTEGRATION_US;
static uint8_t  current_range  = DEFAULT_RANGE;
static bool     test_mode      = false;
static bool     clk_4x         = false;

/* SPI instance on HSPI (SPI2) */
static SPIClass hspi(HSPI);

/* Forward declarations */
static void ddc232_flush();

/* ================================================================
 * System CLK via LEDC
 * ================================================================ */

static void clk_start(uint8_t pin, uint32_t freq_hz)
{
    /* Arduino ESP32 core 3.x API */
    ledcAttach(pin, freq_hz, 1);  // 1-bit resolution -> 50% duty
    ledcWrite(pin, 1);            // duty = 1 out of 2
}

static void clk_set_freq(uint8_t pin, uint32_t freq_hz)
{
    ledcWriteTone(pin, freq_hz);
    /* ledcWriteTone sets 50% duty automatically */
}

static void clk_stop(uint8_t pin)
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

static void write_config_register()
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

/**
 * Write config register and strobe CONV to enter normal operation.
 * Use this for runtime config changes (range, test mode, etc.).
 * Do NOT use this when readback is needed — use write_and_readback_config() instead.
 */
static void apply_config()
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
static void write_and_readback_config()
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
 * ================================================================
 *
 * The DDC232/233 config register is write-only (no readback path).
 * Instead, we diagnose by checking the signals we CAN observe:
 *   - DVALID pin state
 *   - CLK output toggling
 *   - CONV response (does DVALID assert after integration?)
 */

static void cmd_diag()
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

    /* 2. CLK status (cannot sample — 10 MHz is too fast for digitalRead,
     *    and detaching LEDC to sample would stop it). Inferred from DVALID test below. */
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
 * ================================================================
 *
 * Probing guide:
 *   CH1: CLK_CFG  (GPIO5)  — config clock, should see 12 pulses during write
 *   CH2: DIN_CFG  (GPIO6)  — config data, bit pattern during write
 *   CH3: DCLK     (GPIO12) — SPI clock, should see pulses during readback
 *   CH4: DOUT     (GPIO9)  — SPI data from DDC, readback data here
 *
 * Also worth checking independently:
 *   RESET   (GPIO15) — strobe low then high before write
 *   CONV    (GPIO8)  — must be low during write, strobed after readback
 *   CLK     (GPIO14) — 10 MHz system clock, must be running throughout
 */

static void cmd_scope()
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

/* DDC232 uses offset binary coding (NOT two's complement):
 *   Code 0x00000 (0)      = negative full scale
 *   Code 0x10000 (65536)  = zero input
 *   Code 0xFFFFF (1048575)= positive full scale
 * We output raw unsigned 20-bit values; conversion happens in GUI. */

/* ================================================================
 * DDC232 pipelined read cycle (dual integrator)
 * ================================================================
 *
 * The DDC232 has a ping-pong dual integrator. When CONV toggles,
 * one side begins integrating while the OTHER side's completed data
 * becomes available (DVALID asserts). This means readout overlaps
 * with the next integration — they are NOT sequential.
 *
 * Pipeline timing:
 *
 *   CONV ──┐         ┌──────────────────────┐         ┌───
 *          │         │  Side B integrating   │         │
 *          └─────────┘                       └─────────┘
 *            Side A integrating
 *                    ↑ DVALID asserts        ↑ DVALID asserts
 *                    │ read Side A data      │ read Side B data
 *                    └── SPI readout ──┘     └── SPI readout ──┘
 *
 * On the FIRST call we need to prime the pipeline: toggle CONV once
 * to start the first integration, then toggle again to get data.
 * Subsequent calls just toggle CONV and read — integration happened
 * during the previous readout + processing time.
 *
 * If the caller's processing time (serial print etc.) exceeds
 * integration_us, we don't need to wait at all — data is already
 * ready by the time we come back.
 */

static bool pipeline_primed = false;
static uint32_t conv_toggle_time = 0;  // micros() when CONV last toggled
static bool conv_state = false;        // current CONV pin level

static bool ddc232_prime()
{
    /* Prime the dual integrator pipeline:
     * Toggle 1: start integration on side A (discard stale data)
     * Toggle 2: start integration on side B, side A data available
     *
     * After priming, the first real ddc232_read() will toggle again
     * and get side B's data while side A integrates.
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

/* Per-phase timing accumulators (for diagnostics) */
static uint32_t dbg_wait_us = 0, dbg_dvalid_us = 0, dbg_spi_us = 0, dbg_unpack_us = 0;
static int dbg_samples = 0;

/*
 * Pipelined read — maximizes overlap between integration and readout.
 *
 * DDC232 dual integrator: each CONV edge switches sides.
 * Side A integrates while side B's data is read out, and vice versa.
 *
 * Optimal pipeline (each CONV toggle = one edge):
 *
 *   CONV edge ──────────────────────────────────── next CONV edge
 *   │                                              │
 *   ├─ Side B starts integrating (160 us)          │
 *   ├─ Side A conversion (tCONV ~26 us @ 20 MHz)  │
 *   ├─ DVALID asserts                              │
 *   ├─ SPI readout side A (~56 us)                 │
 *   ├─ Unpack + caller processing                  │
 *   ├─ Wait for remaining integration time ────────┤
 *
 * The key: DVALID + SPI + unpack all happen WHILE the other side
 * integrates. So per-sample time = max(integration, readout_total).
 *
 * conv_state tracks which level CONV is at. Each call toggles it.
 */

static bool ddc232_read(int32_t data[NUM_CHANNELS])
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

    /* No second CONV toggle here — integration continues on the other
     * side while we unpack and the caller processes the data.
     * Next call to ddc232_read() will toggle CONV again. */

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

        data[ch] = (int32_t)raw;  // raw offset binary, 0–1048575
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

/* ================================================================
 * Dummy conversion to flush stale integrator state
 * ================================================================ */

static void ddc232_flush()
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

/* ================================================================
 * Serial command processing
 * ================================================================ */

static void cmd_read()
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

static void cmd_range(const char* arg)
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

static void cmd_inttime(const char* arg)
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

static void cmd_clk(const char* arg)
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

static void cmd_test(const char* arg)
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

static void cmd_continuous(const char* arg1, const char* arg2)
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

        /* CSV: timestamp_us, sample_index, ch0, ch1, ..., ch31
         * Build in buffer and send in one write to minimize serial overhead */
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
 *   [8-135] 32 channels × 4 bytes (int32_t each)
 *
 * Stops when any byte is received on serial.
 * Prints text summary after stopping.
 */
#define SYNC_MARKER_0 0xAA
#define SYNC_MARKER_1 0x55

static void cmd_stream(const char* arg1)
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

        /* Pack frame */
        memcpy(frame + 4, &t_us, 4);
        memcpy(frame + 8, data, NUM_CHANNELS * 4);
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
 *
 * Per-sample storage: 4 bytes timestamp + 32×4 bytes = 132 bytes
 * Uses ps_malloc() for PSRAM allocation.
 *
 * Usage: capture [count]
 *   Default count based on available PSRAM.
 */

struct capture_sample {
    uint32_t timestamp_us;
    int32_t  data[NUM_CHANNELS];
};

static void cmd_capture(const char* arg1)
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

    for (int i = 0; i < count; i++) {
        memcpy(frame + 4, &buf[i].timestamp_us, 4);
        memcpy(frame + 8, buf[i].data, NUM_CHANNELS * 4);
        Serial.write(frame, 136);
    }

    free(buf);
    Serial.printf("\nDump complete.\n");
}

static void cmd_help()
{
    Serial.println("Commands:");
    Serial.println("  read              - Single readout of all 32 channels");
    Serial.println("  range <0-7>       - Set full-scale charge range");
    Serial.println("  inttime <us>      - Set integration time (microseconds)");
    Serial.println("  clk <hz>          - Set system clock frequency");
    Serial.println("  test <on|off>     - Enable/disable internal test mode");
    Serial.println("  continuous [n] [ms] - Read n samples (CSV text)");
    Serial.println("  stream [n]        - Binary stream (max speed)");
    Serial.println("  capture [n]       - Capture to PSRAM, then dump (measures true HW speed)");
    Serial.println("  readcfg           - Write + read back config register");
    Serial.println("  scope             - Slow signal test for oscilloscope");
    Serial.println("  diag              - Run hardware diagnostics");
    Serial.println("  help              - Show this message");
}

/* ---- Command parser ---- */

static char cmd_buf[256];
static int  cmd_len = 0;

static void process_command(const char* line)
{
    /* Skip leading whitespace */
    while (*line == ' ' || *line == '\t') line++;
    if (*line == '\0') return;

    /* Tokenize: command + up to 2 arguments */
    char buf[256];
    strncpy(buf, line, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    char* cmd  = strtok(buf, " \t");
    char* arg1 = strtok(NULL, " \t");
    char* arg2 = strtok(NULL, " \t");

    if      (strcmp(cmd, "read") == 0)       cmd_read();
    else if (strcmp(cmd, "range") == 0)      cmd_range(arg1);
    else if (strcmp(cmd, "inttime") == 0)    cmd_inttime(arg1);
    else if (strcmp(cmd, "clk") == 0)        cmd_clk(arg1);
    else if (strcmp(cmd, "test") == 0)       cmd_test(arg1);
    else if (strcmp(cmd, "continuous") == 0) cmd_continuous(arg1, arg2);
    else if (strcmp(cmd, "stream") == 0)     cmd_stream(arg1);
    else if (strcmp(cmd, "capture") == 0)    cmd_capture(arg1);
    else if (strcmp(cmd, "diag") == 0)        cmd_diag();
    else if (strcmp(cmd, "readcfg") == 0)    write_and_readback_config();
    else if (strcmp(cmd, "scope") == 0)      cmd_scope();
    else if (strcmp(cmd, "help") == 0)       cmd_help();
    else    Serial.printf("Unknown command: '%s'. Type 'help' for list.\n", cmd);
}

/* ================================================================
 * Arduino setup & loop
 * ================================================================ */

void setup()
{
    Serial.begin(115200);
    while (!Serial) { delay(10); }  // wait for USB CDC

    Serial.println("\n=== DDC232/233 Readout (Arduino) ===");

    /* ---- GPIO setup ---- */
    pinMode(PIN_CONV,    OUTPUT);
    pinMode(PIN_RESET,   OUTPUT);
    pinMode(PIN_DIN_CFG, OUTPUT);
    pinMode(PIN_CLK_CFG, OUTPUT);
    pinMode(PIN_DVALID,  INPUT_PULLUP);

    digitalWrite(PIN_CONV,    LOW);
    digitalWrite(PIN_RESET,   LOW);   // hold in reset
    digitalWrite(PIN_DIN_CFG, LOW);
    digitalWrite(PIN_CLK_CFG, LOW);

    /* ---- Start system clock via LEDC ---- */
    clk_start(PIN_CLK, clk_freq_hz);
    delay(10);  // let CLK settle before config operations

    /* ---- Setup SPI for data readout ---- */
    hspi.begin(PIN_DCLK, PIN_DOUT, -1, -1);  // SCLK, MISO, MOSI=-1, SS=-1

    /* ---- Write config register, read back to verify, strobe CONV for normal op ---- */
    write_and_readback_config();

    /* ---- Dummy conversion to flush integrators ---- */
    Serial.println("Running dummy conversion...");
    ddc232_flush();

    Serial.printf("Ready: CLK=%lu Hz, integration=%lu us, range=%d (%s)\n",
                  clk_freq_hz, integration_us, current_range,
                  range_labels[current_range]);
    cmd_help();
    Serial.print("\nddc> ");
}

void loop()
{
    /* Read serial input line-by-line */
    while (Serial.available()) {
        char c = Serial.read();

        if (c == '\n' || c == '\r') {
            if (cmd_len > 0) {
                cmd_buf[cmd_len] = '\0';
                Serial.println();  // echo newline
                process_command(cmd_buf);
                cmd_len = 0;
                Serial.print("ddc> ");
            }
        } else if (c == '\b' || c == 127) {
            /* Backspace */
            if (cmd_len > 0) {
                cmd_len--;
                Serial.print("\b \b");
            }
        } else if (cmd_len < (int)(sizeof(cmd_buf) - 1)) {
            cmd_buf[cmd_len++] = c;
            Serial.print(c);  // echo
        }
    }
}
