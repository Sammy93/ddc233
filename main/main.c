#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_console.h"
#include "esp_vfs_dev.h"
#include "driver/usb_serial_jtag.h"
#include "esp_vfs_usb_serial_jtag.h"
#include "linenoise/linenoise.h"
#include "ddc232.h"

static const char *TAG = "main";

/* ---- Default pin assignment (adjust to your PCB) ---- */
#define PIN_MCLK      GPIO_NUM_5
#define PIN_CONV       GPIO_NUM_6
#define PIN_DVALID     GPIO_NUM_7
#define PIN_DCLK       GPIO_NUM_12
#define PIN_DOUT       GPIO_NUM_13
#define PIN_DIN        GPIO_NUM_11
#define PIN_FORMAT     GPIO_NUM_15

/* ---- Default operating parameters ---- */
#define DEFAULT_MCLK_HZ        5000000   // 5 MHz
#define DEFAULT_INTEGRATION_US 1000      // 1 ms
#define DEFAULT_RANGE          DDC232_RANGE_50PC

static ddc232_handle_t adc;

/* ---------- Console commands ---------- */

static int cmd_read(int argc, char **argv)
{
    int32_t data[DDC232_NUM_CHANNELS];
    esp_err_t err = ddc232_read(adc, data);
    if (err != ESP_OK) {
        printf("Read failed: %s\n", esp_err_to_name(err));
        return 1;
    }
    for (int ch = 0; ch < DDC232_NUM_CHANNELS; ch++) {
        printf("CH%02d: %7ld\n", ch, (long)data[ch]);
    }
    return 0;
}

static int cmd_range(int argc, char **argv)
{
    if (argc < 2) {
        printf("Usage: range <0-7>\n"
               "  0 = 12.5 pC   4 = 200 pC\n"
               "  1 = 50 pC     5 = 250 pC\n"
               "  2 = 100 pC    6 = 300 pC\n"
               "  3 = 150 pC    7 = 350 pC\n");
        return 1;
    }
    int r = atoi(argv[1]);
    if (r < 0 || r > 7) { printf("Invalid range\n"); return 1; }
    esp_err_t err = ddc232_set_range(adc, (ddc232_range_t)r);
    if (err != ESP_OK) { printf("Error: %s\n", esp_err_to_name(err)); return 1; }
    printf("Range set to %d\n", r);
    return 0;
}

static int cmd_inttime(int argc, char **argv)
{
    if (argc < 2) {
        printf("Usage: inttime <microseconds>\n");
        return 1;
    }
    uint32_t us = (uint32_t)atoi(argv[1]);
    if (us == 0) { printf("Invalid time\n"); return 1; }
    esp_err_t err = ddc232_set_integration_time(adc, us);
    if (err != ESP_OK) { printf("Error: %s\n", esp_err_to_name(err)); return 1; }
    printf("Integration time set to %lu us\n", us);
    return 0;
}

static int cmd_mclk(int argc, char **argv)
{
    if (argc < 2) {
        printf("Usage: mclk <frequency_hz>\n");
        return 1;
    }
    uint32_t hz = (uint32_t)atoi(argv[1]);
    if (hz == 0) { printf("Invalid frequency\n"); return 1; }
    esp_err_t err = ddc232_set_mclk(adc, hz);
    if (err != ESP_OK) { printf("Error: %s\n", esp_err_to_name(err)); return 1; }
    printf("MCLK set to %lu Hz\n", hz);
    return 0;
}

static int cmd_test(int argc, char **argv)
{
    if (argc < 2) {
        printf("Usage: test <on|off>\n"
               "  Enables/disables DDC232 internal test mode.\n"
               "  When on, internal reference currents drive all channels\n"
               "  producing a known pattern (no external input needed).\n");
        return 1;
    }
    bool enable = (strcmp(argv[1], "on") == 0 || strcmp(argv[1], "1") == 0);
    esp_err_t err = ddc232_set_test_mode(adc, enable);
    if (err != ESP_OK) { printf("Error: %s\n", esp_err_to_name(err)); return 1; }
    printf("Test mode %s\n", enable ? "ON" : "OFF");
    return 0;
}

static int cmd_continuous(int argc, char **argv)
{
    int count = 10;
    int delay_ms = 100;
    if (argc >= 2) count = atoi(argv[1]);
    if (argc >= 3) delay_ms = atoi(argv[2]);

    printf("Reading %d samples, %d ms apart. Press Ctrl+C to abort.\n", count, delay_ms);

    int32_t data[DDC232_NUM_CHANNELS];
    for (int i = 0; i < count; i++) {
        esp_err_t err = ddc232_read(adc, data);
        if (err != ESP_OK) {
            printf("Read error: %s\n", esp_err_to_name(err));
            break;
        }
        /* Print as CSV: sample_index, ch0, ch1, ..., ch31 */
        printf("%d", i);
        for (int ch = 0; ch < DDC232_NUM_CHANNELS; ch++) {
            printf(",%ld", (long)data[ch]);
        }
        printf("\n");
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
    return 0;
}

static void register_commands(void)
{
    const esp_console_cmd_t cmds[] = {
        {
            .command = "read",
            .help    = "Read all 32 channels once",
            .func    = cmd_read,
        },
        {
            .command = "range",
            .help    = "Set input range: range <0-7>",
            .func    = cmd_range,
        },
        {
            .command = "inttime",
            .help    = "Set integration time: inttime <us>",
            .func    = cmd_inttime,
        },
        {
            .command = "mclk",
            .help    = "Set MCLK frequency: mclk <hz>",
            .func    = cmd_mclk,
        },
        {
            .command = "test",
            .help    = "Toggle test mode: test <on|off>",
            .func    = cmd_test,
        },
        {
            .command = "continuous",
            .help    = "Read N samples: continuous [count] [delay_ms]",
            .func    = cmd_continuous,
        },
    };

    for (int i = 0; i < sizeof(cmds) / sizeof(cmds[0]); i++) {
        ESP_ERROR_CHECK(esp_console_cmd_register(&cmds[i]));
    }

    esp_console_register_help_command();
}

static void console_task(void *arg)
{
    const char *prompt = "ddc> ";
    while (true) {
        char *line = linenoise(prompt);
        if (line == NULL) continue;

        /* Skip empty input */
        if (strlen(line) > 0) {
            linenoiseHistoryAdd(line);
            int ret;
            esp_err_t err = esp_console_run(line, &ret);
            if (err == ESP_ERR_NOT_FOUND) {
                printf("Unknown command. Type 'help' for list.\n");
            } else if (err == ESP_ERR_INVALID_ARG) {
                /* empty or bad command */
            }
        }
        linenoiseFree(line);
    }
}

void app_main(void)
{
    /* --- USB-Serial-JTAG console setup --- */
    usb_serial_jtag_driver_config_t usb_cfg = {
        .rx_buffer_size = 1024,
        .tx_buffer_size = 1024,
    };
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_cfg));
    esp_vfs_usb_serial_jtag_use_driver();

    esp_console_config_t console_cfg = {
        .max_cmdline_args   = 8,
        .max_cmdline_length = 256,
    };
    ESP_ERROR_CHECK(esp_console_init(&console_cfg));
    linenoiseSetMultiLine(1);

    /* --- DDC232 initialisation --- */
    ddc232_config_t cfg = {
        .pins = {
            .mclk   = PIN_MCLK,
            .conv   = PIN_CONV,
            .dvalid = PIN_DVALID,
            .dclk   = PIN_DCLK,
            .dout   = PIN_DOUT,
            .din    = PIN_DIN,
            .format = PIN_FORMAT,
        },
        .mclk_freq_hz   = DEFAULT_MCLK_HZ,
        .integration_us  = DEFAULT_INTEGRATION_US,
        .range           = DEFAULT_RANGE,
    };

    esp_err_t err = ddc232_init(&cfg, &adc);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "DDC232 init failed: %s", esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG, "DDC232 readout ready. Type 'help' for commands.");
    printf("\n"
           "=== DDC232/233 Readout ===\n"
           "Commands:\n"
           "  read              - Single readout of all 32 channels\n"
           "  range <0-7>       - Set full-scale charge range\n"
           "  inttime <us>      - Set integration time (microseconds)\n"
           "  mclk <hz>         - Set master clock frequency\n"
           "  test <on|off>       - Enable/disable internal test mode\n"
           "  continuous [n] [ms] - Read n samples with delay\n"
           "  help              - Show all commands\n\n");

    register_commands();

    /* Run console on a dedicated task with enough stack */
    xTaskCreate(console_task, "console", 4096, NULL, 5, NULL);
}
