#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/gpio.h"

#define DDC232_NUM_CHANNELS 32
#define DDC232_DATA_BITS    20

/**
 * Input range (full-scale charge) selection.
 * 3-bit FSR field in the 12-bit configuration register (bits 11-9).
 */
typedef enum {
    DDC232_RANGE_12_5PC = 0,  // 12.5 pC
    DDC232_RANGE_50PC   = 1,  // 50 pC
    DDC232_RANGE_100PC  = 2,  // 100 pC
    DDC232_RANGE_150PC  = 3,  // 150 pC
    DDC232_RANGE_200PC  = 4,  // 200 pC
    DDC232_RANGE_250PC  = 5,  // 250 pC
    DDC232_RANGE_300PC  = 6,  // 300 pC
    DDC232_RANGE_350PC  = 7,  // 350 pC
} ddc232_range_t;

/**
 * Pin configuration for wiring between ESP32-S3 and DDC232.
 *
 * Signal summary (active-high unless noted):
 *   CLK     - System clock output (LEDC PWM, typ 10 MHz)
 *   CONV    - Conversion / integration control (toggling period = integration time)
 *   DVALID  - Data-valid input (active-low, asserted when data ready on DOUT)
 *   DCLK    - Data readout clock (SPI CLK)
 *   DOUT    - Serial data from DDC232 (SPI MISO)
 *   DIN_CFG - Configuration register data input (GPIO bit-bang)
 *   CLK_CFG - Configuration register clock (GPIO bit-bang)
 *   RESET   - Active-low asynchronous reset
 */
typedef struct {
    gpio_num_t clk;        // LEDC output  -> DDC CLK (system clock)
    gpio_num_t conv;       // GPIO output  -> DDC CONV
    gpio_num_t dvalid;     // GPIO input   <- DDC DVALID (active-low)
    gpio_num_t dclk;       // SPI CLK      -> DDC DCLK
    gpio_num_t dout;       // SPI MISO     <- DDC DOUT
    gpio_num_t din_cfg;    // GPIO output  -> DDC DIN_CFG (config data)
    gpio_num_t clk_cfg;    // GPIO output  -> DDC CLK_CFG (config clock)
    gpio_num_t reset;      // GPIO output  -> DDC RESET (active-low)
} ddc232_pins_t;

typedef struct {
    ddc232_pins_t  pins;
    uint32_t       clk_freq_hz;        // System clock frequency (typ. 10 MHz)
    uint32_t       integration_us;     // Integration time in microseconds
    ddc232_range_t range;
} ddc232_config_t;

/** Opaque driver handle */
typedef struct ddc232_dev *ddc232_handle_t;

/**
 * Initialise the DDC232 driver.
 *  - Configures LEDC to output CLK
 *  - Sets up GPIOs (CONV, DVALID, RESET, DIN_CFG, CLK_CFG)
 *  - Configures SPI for data readout (DCLK + DOUT)
 *  - Pulses RESET, writes config register, runs a dummy conversion
 */
esp_err_t ddc232_init(const ddc232_config_t *cfg, ddc232_handle_t *out_handle);

/**
 * Set input range via the 12-bit configuration register.
 */
esp_err_t ddc232_set_range(ddc232_handle_t h, ddc232_range_t range);

/**
 * Set integration time in microseconds.
 * Controls how long CONV is held per half-cycle.
 */
esp_err_t ddc232_set_integration_time(ddc232_handle_t h, uint32_t us);

/**
 * Set CLK frequency. Reconfigures the LEDC timer.
 */
esp_err_t ddc232_set_clk(ddc232_handle_t h, uint32_t freq_hz);

/**
 * Enable or disable test mode via the config register (bit 0).
 * In test mode the DDC232 disconnects analog inputs and measures
 * a zero-input signal, useful for verifying digital readout.
 */
esp_err_t ddc232_set_test_mode(ddc232_handle_t h, bool enable);

/**
 * Perform one conversion cycle and read all 32 channels.
 *
 * Toggles CONV for the configured integration time, waits for DVALID,
 * then clocks out 32 × 20-bit words via SPI.
 *
 * @param data Output array, must have room for DDC232_NUM_CHANNELS int32_t values.
 *             Values are sign-extended 20-bit (range -524288 .. +524287).
 */
esp_err_t ddc232_read(ddc232_handle_t h, int32_t data[DDC232_NUM_CHANNELS]);

/**
 * Stop CLK and release resources.
 */
esp_err_t ddc232_deinit(ddc232_handle_t h);
