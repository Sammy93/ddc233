#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/gpio.h"

#define DDC232_NUM_CHANNELS 32
#define DDC232_DATA_BITS    20

/**
 * Input range (full-scale charge) selection via serial config register.
 * 3-bit encoding per the DDC232 datasheet.
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
 * Signal summary:
 *   MCLK   - Master clock output from ESP32 (LEDC PWM)
 *   CONV   - Conversion trigger (rising edge starts integration)
 *   DVALID - Data-valid input, active-low
 *   DCLK   - Data clock (SPI CLK), shared for config write and data read
 *   DOUT   - Serial data from DDC232 (SPI MISO)
 *   DIN    - Serial data to DDC232 (SPI MOSI) for config register
 *   FORMAT - Tied high to enable serial config register mode
 */
typedef struct {
    gpio_num_t mclk;       // LEDC output -> DDC MCLK
    gpio_num_t conv;       // GPIO output  -> DDC CONV
    gpio_num_t dvalid;     // GPIO input   <- DDC DVALID (active-low)
    gpio_num_t dclk;       // SPI CLK      -> DDC DCLK
    gpio_num_t dout;       // SPI MISO     <- DDC DOUT
    gpio_num_t din;        // SPI MOSI     -> DDC DIN (config register write)
    gpio_num_t format;     // GPIO output  -> DDC FORMAT (high = serial config)
} ddc232_pins_t;

typedef struct {
    ddc232_pins_t  pins;
    uint32_t       mclk_freq_hz;       // Master clock frequency (typ. 1-10 MHz)
    uint32_t       integration_us;     // Desired integration time in microseconds
    ddc232_range_t range;
} ddc232_config_t;

/** Opaque driver handle */
typedef struct ddc232_dev *ddc232_handle_t;

/**
 * Initialise the DDC232 driver.
 *  - Configures LEDC to output MCLK
 *  - Sets up GPIOs (CONV, DVALID, FORMAT)
 *  - Configures SPI (DCLK, DOUT, DIN)
 *  - Writes initial range and integration time to the serial config register
 */
esp_err_t ddc232_init(const ddc232_config_t *cfg, ddc232_handle_t *out_handle);

/**
 * Set input range via the serial config register.
 */
esp_err_t ddc232_set_range(ddc232_handle_t h, ddc232_range_t range);

/**
 * Set integration time in microseconds via the serial config register.
 * Actual time depends on MCLK frequency (NINT = time × f_MCLK / 2).
 */
esp_err_t ddc232_set_integration_time(ddc232_handle_t h, uint32_t us);

/**
 * Set MCLK frequency. Reconfigures the LEDC timer and re-writes the
 * config register to maintain the requested integration time.
 */
esp_err_t ddc232_set_mclk(ddc232_handle_t h, uint32_t freq_hz);

/**
 * Perform one conversion cycle and read all 32 channels.
 *
 * Pulses CONV to trigger a conversion (integration time is controlled
 * by NINT in the serial config register), waits for DVALID, then
 * clocks out 32 × 20-bit words via SPI.
 *
 * @param data Output array, must have room for DDC232_NUM_CHANNELS int32_t values.
 *             Values are sign-extended 20-bit (range -524288 .. +524287).
 */
esp_err_t ddc232_read(ddc232_handle_t h, int32_t data[DDC232_NUM_CHANNELS]);

/**
 * Enable or disable test mode via the serial config register.
 * In test mode the DDC232 connects internal reference currents to the
 * integrators, producing a known output pattern on all channels.
 * Useful for verifying digital readout without analog inputs.
 */
esp_err_t ddc232_set_test_mode(ddc232_handle_t h, bool enable);

/**
 * Stop MCLK and release resources.
 */
esp_err_t ddc232_deinit(ddc232_handle_t h);
