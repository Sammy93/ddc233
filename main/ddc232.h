#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/gpio.h"

#define DDC232_NUM_CHANNELS 32
#define DDC232_DATA_BITS    20

/**
 * Input range (full-scale charge) selection.
 * DDC232/233 supports multiple ranges via the RANGE pins.
 * The two RANGE pins encode 4 settings; remaining ranges
 * require the serial configuration register.
 */
typedef enum {
    DDC232_RANGE_12_5PC = 0,  // 12.5 pC
    DDC232_RANGE_50PC   = 1,  // 50 pC
    DDC232_RANGE_100PC  = 2,  // 100 pC
    DDC232_RANGE_150PC  = 3,  // 150 pC
} ddc232_range_t;

/**
 * Pin configuration for wiring between ESP32-S3 and DDC232.
 *
 * Signal summary (active-high unless noted):
 *   MCLK   - Master clock output from ESP32 (LEDC PWM)
 *   CONV   - Conversion signal (rising edge starts integration on side A,
 *            falling edge switches to side B)
 *   DVALID - Data-valid input, active-low: low while data is being clocked out
 *   DCLK   - Data clock output from ESP32 (SPI CLK)
 *   DOUT   - Serial data input to ESP32 (SPI MISO)
 *   RANGE0/1 - Range select outputs
 */
typedef struct {
    gpio_num_t mclk;       // LEDC output -> DDC MCLK
    gpio_num_t conv;       // GPIO output  -> DDC CONV
    gpio_num_t dvalid;     // GPIO input   <- DDC DVALID (active-low)
    gpio_num_t dclk;       // SPI CLK      -> DDC DCLK
    gpio_num_t dout;       // SPI MISO     <- DDC DOUT
    gpio_num_t range0;     // GPIO output  -> DDC RANGE0
    gpio_num_t range1;     // GPIO output  -> DDC RANGE1
} ddc232_pins_t;

typedef struct {
    ddc232_pins_t pins;
    uint32_t      mclk_freq_hz;       // Master clock frequency in Hz (typ. 1-10 MHz)
    uint32_t      integration_us;     // Desired integration time in microseconds
    ddc232_range_t range;
} ddc232_config_t;

/** Opaque driver handle */
typedef struct ddc232_dev *ddc232_handle_t;

/**
 * Initialise the DDC232 driver.
 *  - Configures LEDC to output MCLK
 *  - Sets up GPIOs for CONV, DVALID, RANGE pins
 *  - Configures SPI for serial data readout (DCLK + DOUT)
 */
esp_err_t ddc232_init(const ddc232_config_t *cfg, ddc232_handle_t *out_handle);

/**
 * Set input range. Takes effect on the next conversion.
 */
esp_err_t ddc232_set_range(ddc232_handle_t h, ddc232_range_t range);

/**
 * Set integration time in microseconds. Takes effect on the next conversion.
 */
esp_err_t ddc232_set_integration_time(ddc232_handle_t h, uint32_t us);

/**
 * Set MCLK frequency. Reconfigures the LEDC timer.
 */
esp_err_t ddc232_set_mclk(ddc232_handle_t h, uint32_t freq_hz);

/**
 * Perform one conversion cycle and read all 32 channels.
 *
 * Drives CONV high for the configured integration time, waits for DVALID,
 * then clocks out 32 × 20-bit words via SPI.
 *
 * @param data Output array, must have room for DDC232_NUM_CHANNELS int32_t values.
 *             Values are sign-extended 20-bit (range -524288 .. +524287).
 */
esp_err_t ddc232_read(ddc232_handle_t h, int32_t data[DDC232_NUM_CHANNELS]);

/**
 * Stop MCLK and release resources.
 */
esp_err_t ddc232_deinit(ddc232_handle_t h);
