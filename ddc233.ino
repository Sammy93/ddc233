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

#include "config.h"
#include "ddc232_readout.h"
#include "dac8562.h"
#include "hv2901.h"
#include "matrix_scan.h"
#include "commands.h"

/* ================================================================
 * Global variable definitions (extern-declared in config.h)
 * ================================================================ */

/* Device state */
uint32_t clk_freq_hz    = DEFAULT_CLK_HZ;
uint32_t integration_us = DEFAULT_INTEGRATION_US;
uint32_t matrix_dead_us = 0;  // dead time (all Vneg) between column switches
uint16_t hv_skip_vneg  = 0;  // bitmask: pairs that use OFF instead of Vneg when inactive
uint8_t  current_range  = DEFAULT_RANGE;
bool     test_mode      = false;
bool     clk_4x         = false;

/* Track last-set DAC voltages for settings query */
float    last_vbias1    = 0.0f;
float    last_vbias2    = 0.0f;
float    last_vsw1      = 0.0f;
float    last_vsw2      = 0.0f;

/* SPI instance on HSPI (SPI3 on ESP32-S3) for DDC232 readout */
SPIClass hspi(HSPI);

/* SPI instance on FSPI (SPI2 on ESP32-S3) for DAC8562 output */
SPIClass dac_spi(FSPI);

/* DAC8562 DDS state */
volatile bool dac_running = false;
TaskHandle_t dac_task_handle = NULL;
hw_timer_t *dac_timer = NULL;
uint32_t dac1_freq_hz = 1000;
uint32_t dac2_freq_hz = 2000;

/* Pipeline state */
bool pipeline_primed = false;
uint32_t conv_toggle_time = 0;  // micros() when CONV last toggled
bool conv_state = false;        // current CONV pin level
bool matrix_primed = false;

/* ================================================================
 * Command parser
 * ================================================================ */

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
    else if (strcmp(cmd, "deadtime") == 0)  cmd_deadtime(arg1);
    else if (strcmp(cmd, "skipvneg") == 0) cmd_skipvneg(arg1, arg2);
    else if (strcmp(cmd, "clk") == 0)        cmd_clk(arg1);
    else if (strcmp(cmd, "test") == 0)       cmd_test(arg1);
    else if (strcmp(cmd, "continuous") == 0) cmd_continuous(arg1, arg2);
    else if (strcmp(cmd, "stream") == 0)     cmd_stream(arg1);
    else if (strcmp(cmd, "capture") == 0)    cmd_capture(arg1);
    else if (strcmp(cmd, "dac") == 0)        cmd_dac(arg1, arg2);
    else if (strcmp(cmd, "vbias1") == 0)    cmd_vbias1(arg1);
    else if (strcmp(cmd, "vbias2") == 0)    cmd_vbias2(arg1);
    else if (strcmp(cmd, "vsw1") == 0)      cmd_vsw1(arg1);
    else if (strcmp(cmd, "vsw2") == 0)      cmd_vsw2(arg1);
    else if (strcmp(cmd, "sw") == 0)         cmd_sw(arg1, arg2);
    else if (strcmp(cmd, "sw2") == 0)        cmd_sw2(arg1, arg2);
    else if (strcmp(cmd, "mscan") == 0)     cmd_mscan();
    else if (strcmp(cmd, "mstream") == 0)   cmd_mstream();
    else if (strcmp(cmd, "mf32stream") == 0) cmd_mf32stream();
    else if (strcmp(cmd, "mcapture") == 0)  cmd_mcapture(arg1);
    else if (strcmp(cmd, "mdscan") == 0)    cmd_mdscan(arg1);
    else if (strcmp(cmd, "mdstream") == 0)  cmd_mdstream(arg1);
    else if (strcmp(cmd, "mdcapture") == 0) cmd_mdcapture(arg1, arg2);
    else if (strcmp(cmd, "md12scan") == 0)    cmd_md12scan(arg1);
    else if (strcmp(cmd, "md12stream") == 0)  cmd_md12stream(arg1);
    else if (strcmp(cmd, "md12capture") == 0) cmd_md12capture(arg1, arg2);
    else if (strcmp(cmd, "diag") == 0)       cmd_diag();
    else if (strcmp(cmd, "readcfg") == 0)    write_and_readback_config();
    else if (strcmp(cmd, "settings") == 0)   cmd_settings();
    else if (strcmp(cmd, "scope") == 0)      cmd_scope();
    else if (strcmp(cmd, "help") == 0)       cmd_help();
    else    Serial.printf("Unknown command: '%s'. Type 'help' for list.\n", cmd);
}

/* ================================================================
 * Arduino setup & loop
 * ================================================================ */

void setup()
{
    /* ---- DAC2 mid-voltage FIRST — absolute priority at boot ---- */
    pinMode(PIN_DAC_CS2, OUTPUT);
    pinMode(PIN_DAC_CLR, OUTPUT);
    digitalWrite(PIN_DAC_CS2, HIGH);
    digitalWrite(PIN_DAC_CLR, HIGH);  // ensure CLR inactive
    dac_spi.begin(PIN_DAC_SCK, -1, PIN_DAC_MOSI, -1);
    dac_spi.beginTransaction(SPISettings(DAC_SPI_CLK_HZ, MSBFIRST, SPI_MODE1));
    dac_write_cmd(PIN_DAC_CS2, 0x18, 0x4000);  // DAC2 ch A: load mid-voltage code
    dac_write_cmd(PIN_DAC_CS2, 0x19, 0x4000);  // DAC2 ch B: load mid-voltage code
    dac_write_cmd(PIN_DAC_CS2, 0x38, 0x0001);  // enable ref — output ramps smoothly to 1.25V

    /* ---- Now DAC1 ---- */
    pinMode(PIN_DAC_CS1, OUTPUT);
    digitalWrite(PIN_DAC_CS1, HIGH);
    dac_write_cmd(PIN_DAC_CS1, 0x18, 0x0000);  // DAC1 ch A: zero
    dac_write_cmd(PIN_DAC_CS1, 0x19, 0x0000);  // DAC1 ch B: zero
    dac_write_cmd(PIN_DAC_CS1, 0x38, 0x0001);  // enable ref
    dac_spi.endTransaction();

    /* ---- Now proceed with normal init ---- */
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

    /* ---- Setup SPI for DDC232 data readout ---- */
    hspi.begin(PIN_DCLK, PIN_DOUT, -1, -1);  // SCLK, MISO, MOSI=-1, SS=-1

    /* ---- Setup HV2901 x2 analog switch matrix ---- */
    hv_init();

    /* ---- Setup DAC8562 x2 sine wave generator ---- */
    dac_init();

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
