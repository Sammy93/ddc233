#include "dac8562.h"

/* ================================================================
 * DAC8562 dual sine wave generator (DDS, timer-driven)
 * ================================================================
 *
 * Two DAC8562 on a shared SPI bus (FSPI/SPI2) with separate CS lines.
 * A hardware timer fires at DAC_SAMPLE_RATE (100 kHz) and notifies a
 * FreeRTOS task on core 0. The task advances two 32-bit DDS phase
 * accumulators and writes the corresponding sine LUT values to each
 * DAC via SPI. This runs entirely independently of the DDC232 readout
 * on core 1.
 *
 * DAC8562 SPI: 24-bit frame = 8-bit command + 16-bit data, Mode 1
 *   0x18 = Write DAC-A input register and update all
 *   0x38 = Internal reference setup (data 0x0001 = enable)
 */

/* Module-private DDS state */
static uint16_t dac_sine_lut[DAC_LUT_SIZE];
static volatile uint32_t dac1_phase_inc = 0;
static volatile uint32_t dac2_phase_inc = 0;
static uint32_t dac1_phase_acc = 0;
static uint32_t dac2_phase_acc = 0;

uint32_t dac_freq_to_inc(uint32_t freq_hz)
{
    return (uint32_t)((uint64_t)freq_hz * (1ULL << DAC_PHASE_BITS) / DAC_SAMPLE_RATE);
}

void IRAM_ATTR dac_timer_isr()
{
    BaseType_t wake = pdFALSE;
    vTaskNotifyGiveFromISR(dac_task_handle, &wake);
    if (wake) portYIELD_FROM_ISR();
}

void dac_write_cmd(uint8_t cs_pin, uint8_t cmd, uint16_t data)
{
    digitalWrite(cs_pin, LOW);
    dac_spi.transfer(cmd);
    dac_spi.transfer16(data);
    digitalWrite(cs_pin, HIGH);
}

/* Fast 24-bit DAC write using exactly 24 SPI clocks (8-bit cmd + 16-bit data).
 * SPI transaction is held open by the caller to avoid per-sample mutex overhead. */
static inline void dac_write_fast(uint8_t cs_pin, uint8_t cmd, uint16_t data)
{
    digitalWrite(cs_pin, LOW);
    dac_spi.transfer(cmd);
    dac_spi.transfer16(data);
    digitalWrite(cs_pin, HIGH);
}

void dac_output_task(void *param)
{
    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (!dac_running) continue;

        dac1_phase_acc += dac1_phase_inc;
        dac2_phase_acc += dac2_phase_inc;

        uint16_t val1 = dac_sine_lut[dac1_phase_acc >> (DAC_PHASE_BITS - 8)];
        uint16_t val2 = dac_sine_lut[dac2_phase_acc >> (DAC_PHASE_BITS - 8)];

        dac_spi.beginTransaction(SPISettings(DAC_SPI_CLK_HZ, MSBFIRST, SPI_MODE1));
        dac_write_fast(PIN_DAC_CS1, 0x18, val1);  // DAC1 ch A
        dac_write_fast(PIN_DAC_CS1, 0x19, val1);  // DAC1 ch B
        dac_write_fast(PIN_DAC_CS2, 0x18, val2);  // DAC2 ch A
        dac_write_fast(PIN_DAC_CS2, 0x19, val2);  // DAC2 ch B
        dac_spi.endTransaction();
    }
}

void dac_init()
{
    /* GPIO, SPI, reference, and mid-scale already done at top of setup().
     * Here we just build the LUT, create the task, and start output. */

    /* Build sine LUT (full-scale 0-65535) */
    for (int i = 0; i < DAC_LUT_SIZE; i++) {
        dac_sine_lut[i] = (uint16_t)(32767.5 + 32767.5 * sin(2.0 * M_PI * i / DAC_LUT_SIZE));
    }

    /* Set default DDS frequencies */
    dac1_phase_inc = dac_freq_to_inc(dac1_freq_hz);
    dac2_phase_inc = dac_freq_to_inc(dac2_freq_hz);

    /* Create DAC output task on core 0 (loop() runs on core 1) */
    xTaskCreatePinnedToCore(dac_output_task, "dac_out", 2048, NULL, 5, &dac_task_handle, 0);

    Serial.printf("DAC8562 x2 initialized (DAC1=%lu Hz, DAC2=%lu Hz)\n",
                  dac1_freq_hz, dac2_freq_hz);

}

void dac_start()
{
    if (dac_running) return;

    dac1_phase_acc = 0;
    dac2_phase_acc = 0;
    dac_running = true;

    dac_timer = timerBegin(1000000);  /* 1 MHz base counter */
    timerAttachInterrupt(dac_timer, &dac_timer_isr);
    timerAlarm(dac_timer, 20, true, 0);  /* alarm every 20 us = 50 kHz */

    Serial.printf("DAC started: DAC1=%lu Hz, DAC2=%lu Hz @ %d kHz sample rate\n",
                  dac1_freq_hz, dac2_freq_hz, DAC_SAMPLE_RATE / 1000);
}

void dac_stop()
{
    if (!dac_running) return;

    /* Stop timer first so no more notifications arrive */
    if (dac_timer) {
        timerEnd(dac_timer);
        dac_timer = NULL;
    }
    dac_running = false;
    delay(1);  /* let task finish current iteration */

    /* Restore boot values */
    dac_spi.beginTransaction(SPISettings(DAC_SPI_CLK_HZ, MSBFIRST, SPI_MODE1));
    dac_write_cmd(PIN_DAC_CS1, 0x18, 0x0000);  // DAC1 ch A: zero
    dac_write_cmd(PIN_DAC_CS1, 0x19, 0x0000);  // DAC1 ch B: zero
    dac_write_cmd(PIN_DAC_CS2, 0x18, 0x4000);  // DAC2 ch A: mid-voltage
    dac_write_cmd(PIN_DAC_CS2, 0x19, 0x4000);  // DAC2 ch B: mid-voltage
    dac_spi.endTransaction();

    Serial.println("DAC stopped (outputs at mid-scale)");
}

void cmd_dac(const char* arg1, const char* arg2)
{
    if (!arg1 || *arg1 == '\0') {
        Serial.println("Usage:");
        Serial.println("  dac on              - Start sine wave output");
        Serial.println("  dac off             - Stop output (mid-scale)");
        Serial.println("  dac freq1 <hz>      - Set DAC1 frequency");
        Serial.println("  dac freq2 <hz>      - Set DAC2 frequency");
        Serial.println("  dac status          - Show current settings");
        return;
    }

    if (strcmp(arg1, "on") == 0) {
        dac_start();
    } else if (strcmp(arg1, "off") == 0) {
        dac_stop();
    } else if (strcmp(arg1, "freq1") == 0) {
        if (!arg2 || *arg2 == '\0') { Serial.println("Usage: dac freq1 <hz>"); return; }
        dac1_freq_hz = (uint32_t)atol(arg2);
        dac1_phase_inc = dac_freq_to_inc(dac1_freq_hz);
        Serial.printf("DAC1 frequency set to %lu Hz\n", dac1_freq_hz);
    } else if (strcmp(arg1, "freq2") == 0) {
        if (!arg2 || *arg2 == '\0') { Serial.println("Usage: dac freq2 <hz>"); return; }
        dac2_freq_hz = (uint32_t)atol(arg2);
        dac2_phase_inc = dac_freq_to_inc(dac2_freq_hz);
        Serial.printf("DAC2 frequency set to %lu Hz\n", dac2_freq_hz);
    } else if (strcmp(arg1, "status") == 0) {
        Serial.printf("DAC status: %s\n", dac_running ? "RUNNING" : "STOPPED");
        Serial.printf("  DAC1 (CS=GPIO%d): %lu Hz\n", PIN_DAC_CS1, dac1_freq_hz);
        Serial.printf("  DAC2 (CS=GPIO%d): %lu Hz\n", PIN_DAC_CS2, dac2_freq_hz);
        Serial.printf("  Sample rate: %d kHz, LUT: %d entries\n",
                      DAC_SAMPLE_RATE / 1000, DAC_LUT_SIZE);
    } else {
        Serial.printf("Unknown dac subcommand: '%s'\n", arg1);
    }
}

/* ================================================================
 * DAC voltage commands (DC output mode)
 * ================================================================
 *
 * Transfer functions (hardware-specific):
 *   DAC2 (bias):  V_opamp = 4 x V_dac - 5        (+/-5 V range)
 *   DAC1 ch A:    V_out   = +7.8 x V_dac          (0 to +30 V)
 *   DAC1 ch B:    V_out   = -7.8 x V_dac          (0 to -30 V)
 *
 * DAC voltage: V_dac = code x 5 / 65536  (gain=2, Vref=2.5 V)
 */

uint16_t bias_v_to_code(float v_out)
{
    float v_dac = (v_out + DAC_BIAS_OFFSET) / DAC_BIAS_GAIN;
    float code_f = v_dac / DAC_V_PER_CODE;
    if (code_f < 0) code_f = 0;
    if (code_f > 65535) code_f = 65535;
    return (uint16_t)(code_f + 0.5f);
}

uint16_t sw_v_to_code(float v_out)
{
    float v_dac = fabsf(v_out) / DAC_SW_GAIN;
    float code_f = v_dac / DAC_V_PER_CODE;
    if (code_f < 0) code_f = 0;
    if (code_f > 65535) code_f = 65535;
    return (uint16_t)(code_f + 0.5f);
}

void dac_set_voltage(uint8_t cs_pin, uint8_t cmd, uint16_t code)
{
    /* Stop sine wave if running — DC and sine are mutually exclusive */
    if (dac_running) dac_stop();

    dac_spi.beginTransaction(SPISettings(DAC_SPI_CLK_HZ, MSBFIRST, SPI_MODE1));
    dac_write_cmd(cs_pin, cmd, code);
    dac_spi.endTransaction();
}

void cmd_vbias1(const char* arg)
{
    if (!arg || *arg == '\0') {
        Serial.println("Usage: vbias1 <voltage>  (-5.0 to +5.0 V)");
        return;
    }
    float v = atof(arg);
    if (v < -5.0f || v > 5.0f) { Serial.println("Out of range (-5 to +5)"); return; }
    uint16_t code = bias_v_to_code(v);
    dac_set_voltage(PIN_DAC_CS2, 0x18, code);
    last_vbias1 = v;
    Serial.printf("VBias1 = %.3f V (code 0x%04X)\n", v, code);
}

void cmd_vbias2(const char* arg)
{
    if (!arg || *arg == '\0') {
        Serial.println("Usage: vbias2 <voltage>  (-5.0 to +5.0 V)");
        return;
    }
    float v = atof(arg);
    if (v < -5.0f || v > 5.0f) { Serial.println("Out of range (-5 to +5)"); return; }
    uint16_t code = bias_v_to_code(v);
    dac_set_voltage(PIN_DAC_CS2, 0x19, code);
    last_vbias2 = v;
    Serial.printf("VBias2 = %.3f V (code 0x%04X)\n", v, code);
}

void cmd_vsw1(const char* arg)
{
    if (!arg || *arg == '\0') {
        Serial.println("Usage: vsw1 <voltage>  (0.0 to +30.0 V)");
        return;
    }
    float v = atof(arg);
    if (v < 0.0f || v > 30.0f) { Serial.println("Out of range (0 to 30)"); return; }
    uint16_t code = sw_v_to_code(v);
    dac_set_voltage(PIN_DAC_CS1, 0x18, code);
    last_vsw1 = v;
    Serial.printf("VSW1 = %.3f V (code 0x%04X)\n", v, code);
}

void cmd_vsw2(const char* arg)
{
    if (!arg || *arg == '\0') {
        Serial.println("Usage: vsw2 <voltage>  (-30.0 to 0.0 V)");
        return;
    }
    float v = atof(arg);
    if (v < -30.0f || v > 0.0f) { Serial.println("Out of range (-30 to 0)"); return; }
    uint16_t code = sw_v_to_code(v);
    dac_set_voltage(PIN_DAC_CS1, 0x19, code);
    last_vsw2 = v;
    Serial.printf("VSW2 = %.3f V (code 0x%04X)\n", v, code);
}
