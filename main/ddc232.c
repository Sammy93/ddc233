#include "ddc232.h"

#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "driver/ledc.h"
#include "driver/spi_master.h"
#include "rom/ets_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "ddc232";

/*
 * DDC232 serial configuration register (40 bits, MSB first):
 *
 *   Bit 39      : Reserved (0)
 *   Bit 38      : Test mode (0 = normal)
 *   Bit 37      : Reserved (0)
 *   Bits 36-34  : Range side A (3 bits)
 *   Bits 33-31  : Range side B (3 bits)
 *   Bits 30-11  : NINT (20 bits) — integration clocks (MCLK/2 half-periods)
 *   Bits 10-0   : Reserved (0)
 *
 * Integration time = NINT × 2 / f_MCLK
 * NINT must be >= 100 for valid operation.
 */
#define CFG_REG_BITS   40
#define NINT_MIN       100
#define NINT_MAX       0xFFFFF   // 20-bit max

/* Internal device struct */
struct ddc232_dev {
    ddc232_pins_t   pins;
    uint32_t        mclk_freq_hz;
    uint32_t        integration_us;
    ddc232_range_t  range;
    uint32_t        nint;           // current NINT value in config register
    spi_device_handle_t spi;
};

/* ---------- MCLK via LEDC ---------- */

static esp_err_t mclk_start(gpio_num_t pin, uint32_t freq_hz)
{
    ledc_timer_config_t timer = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .timer_num       = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_1_BIT,   // 50 % duty
        .freq_hz         = freq_hz,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    esp_err_t err = ledc_timer_config(&timer);
    if (err != ESP_OK) return err;

    ledc_channel_config_t ch = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = LEDC_CHANNEL_0,
        .timer_sel  = LEDC_TIMER_0,
        .gpio_num   = pin,
        .duty       = 1,   // 1 out of 2 -> 50 %
        .hpoint     = 0,
    };
    return ledc_channel_config(&ch);
}

static esp_err_t mclk_set_freq(uint32_t freq_hz)
{
    return ledc_set_freq(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0, freq_hz);
}

static esp_err_t mclk_stop(void)
{
    return ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
}

/* ---------- Compute NINT from integration time and MCLK ---------- */

static uint32_t us_to_nint(uint32_t integration_us, uint32_t mclk_freq_hz)
{
    /*
     * Integration time = NINT × 2 / f_MCLK
     * => NINT = integration_time × f_MCLK / 2
     *
     * Using uint64_t to avoid overflow for large values.
     */
    uint64_t nint = ((uint64_t)integration_us * mclk_freq_hz) / 2000000ULL;
    if (nint < NINT_MIN) nint = NINT_MIN;
    if (nint > NINT_MAX) nint = NINT_MAX;
    return (uint32_t)nint;
}

/* ---------- Serial config register write ---------- */

/**
 * Build and write the 40-bit serial configuration register via SPI.
 * The register is clocked into DIN on rising edges of DCLK, MSB first.
 * CONV must be low during the write.
 */
static esp_err_t write_config_register(struct ddc232_dev *dev)
{
    uint8_t tx_buf[5] = {0};  // 40 bits = 5 bytes

    uint8_t range_a = dev->range & 0x07;
    uint8_t range_b = dev->range & 0x07;  // same range for both sides
    uint32_t nint   = dev->nint;

    /*
     * Pack the 40-bit register into tx_buf[0..4], MSB first:
     *
     *   Byte 0, bit 7 = register bit 39 (reserved, 0)
     *   Byte 0, bit 6 = register bit 38 (test mode, 0)
     *   Byte 0, bit 5 = register bit 37 (reserved, 0)
     *   Byte 0, bits 4-2 = register bits 36-34 (range A)
     *   Byte 0, bits 1-0 + Byte 1 bit 7 = register bits 33-31 (range B)
     *   Byte 1 bits 6-0 + Byte 2-3 + Byte 4 bits 7-3 = register bits 30-11 (NINT)
     *   Byte 4 bits 2-0 + remaining = register bits 10-0 (reserved, 0)
     */
    tx_buf[0]  = (range_a << 2) | (range_b >> 1);
    tx_buf[1]  = (range_b << 7) | ((nint >> 14) & 0x7F);
    tx_buf[2]  = (nint >> 6) & 0xFF;
    tx_buf[3]  = (nint << 2) & 0xFC;
    tx_buf[4]  = 0;

    /* Ensure CONV is low during config write */
    gpio_set_level(dev->pins.conv, 0);

    spi_transaction_t txn = {
        .length    = CFG_REG_BITS,
        .tx_buffer = tx_buf,
    };
    esp_err_t err = spi_device_transmit(dev->spi, &txn);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Config register write failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "Config register written: range=%d, NINT=%lu", dev->range, dev->nint);
    return ESP_OK;
}

/* ---------- SPI setup ---------- */

#define SPI_HOST_USED  SPI2_HOST
#define SPI_CLK_HZ     (4 * 1000 * 1000)  // 4 MHz data clock

static esp_err_t spi_setup(const ddc232_pins_t *p, spi_device_handle_t *out)
{
    spi_bus_config_t bus = {
        .mosi_io_num   = p->din,
        .miso_io_num   = p->dout,
        .sclk_io_num   = p->dclk,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = DDC232_NUM_CHANNELS * 4,
    };
    esp_err_t err = spi_bus_initialize(SPI_HOST_USED, &bus, SPI_DMA_CH_AUTO);
    if (err != ESP_OK) return err;

    spi_device_interface_config_t dev = {
        .mode           = 0,            // CPOL=0, CPHA=0
        .clock_speed_hz = SPI_CLK_HZ,
        .spics_io_num   = -1,           // no CS — use DVALID for framing
        .queue_size     = 1,
    };
    return spi_bus_add_device(SPI_HOST_USED, &dev, out);
}

/* ---------- GPIO helpers ---------- */

static esp_err_t gpio_setup(const ddc232_pins_t *p)
{
    // Outputs: CONV, FORMAT
    uint64_t out_mask = (1ULL << p->conv) | (1ULL << p->format);
    gpio_config_t out_cfg = {
        .pin_bit_mask = out_mask,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    esp_err_t err = gpio_config(&out_cfg);
    if (err != ESP_OK) return err;

    gpio_set_level(p->conv, 0);
    gpio_set_level(p->format, 1);  // FORMAT=1: serial config register mode

    // Input: DVALID (active-low, use internal pull-up)
    gpio_config_t in_cfg = {
        .pin_bit_mask = (1ULL << p->dvalid),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    return gpio_config(&in_cfg);
}

/* ---------- Sign-extend 20-bit to int32 ---------- */

static inline int32_t sign_extend_20(uint32_t val)
{
    if (val & (1 << 19))
        return (int32_t)(val | 0xFFF00000);
    return (int32_t)val;
}

/* ---------- Public API ---------- */

esp_err_t ddc232_init(const ddc232_config_t *cfg, ddc232_handle_t *out_handle)
{
    if (!cfg || !out_handle) return ESP_ERR_INVALID_ARG;

    struct ddc232_dev *dev = calloc(1, sizeof(*dev));
    if (!dev) return ESP_ERR_NO_MEM;

    dev->pins           = cfg->pins;
    dev->mclk_freq_hz   = cfg->mclk_freq_hz;
    dev->integration_us = cfg->integration_us;
    dev->range          = cfg->range;
    dev->nint           = us_to_nint(cfg->integration_us, cfg->mclk_freq_hz);

    esp_err_t err;

    err = gpio_setup(&dev->pins);
    if (err != ESP_OK) { free(dev); return err; }

    err = mclk_start(dev->pins.mclk, dev->mclk_freq_hz);
    if (err != ESP_OK) { free(dev); return err; }

    err = spi_setup(&dev->pins, &dev->spi);
    if (err != ESP_OK) { mclk_stop(); free(dev); return err; }

    /* Write initial configuration to the DDC232 */
    err = write_config_register(dev);
    if (err != ESP_OK) {
        spi_bus_remove_device(dev->spi);
        spi_bus_free(SPI_HOST_USED);
        mclk_stop();
        free(dev);
        return err;
    }

    ESP_LOGI(TAG, "Initialised: MCLK=%lu Hz, integration=%lu us (NINT=%lu), range=%d",
             dev->mclk_freq_hz, dev->integration_us, dev->nint, dev->range);

    *out_handle = dev;
    return ESP_OK;
}

esp_err_t ddc232_set_range(ddc232_handle_t h, ddc232_range_t range)
{
    if (!h || range > DDC232_RANGE_350PC) return ESP_ERR_INVALID_ARG;
    h->range = range;
    return write_config_register(h);
}

esp_err_t ddc232_set_integration_time(ddc232_handle_t h, uint32_t us)
{
    if (!h || us == 0) return ESP_ERR_INVALID_ARG;
    h->integration_us = us;
    h->nint = us_to_nint(us, h->mclk_freq_hz);
    ESP_LOGI(TAG, "Integration time %lu us -> NINT=%lu", us, h->nint);
    return write_config_register(h);
}

esp_err_t ddc232_set_mclk(ddc232_handle_t h, uint32_t freq_hz)
{
    if (!h || freq_hz == 0) return ESP_ERR_INVALID_ARG;
    esp_err_t err = mclk_set_freq(freq_hz);
    if (err != ESP_OK) return err;

    h->mclk_freq_hz = freq_hz;
    /* Recalculate NINT to maintain the same integration time */
    h->nint = us_to_nint(h->integration_us, freq_hz);
    ESP_LOGI(TAG, "MCLK=%lu Hz, NINT recalculated to %lu", freq_hz, h->nint);
    return write_config_register(h);
}

esp_err_t ddc232_read(ddc232_handle_t h, int32_t data[DDC232_NUM_CHANNELS])
{
    if (!h || !data) return ESP_ERR_INVALID_ARG;

    /*
     * DDC232 conversion cycle (serial config register mode):
     *  1. Pulse CONV high to start integration.
     *     The DDC232 integrates for NINT half-clocks automatically.
     *  2. Wait for DVALID to go low (conversion complete, data ready).
     *  3. Clock out 32 × 20-bit = 640 bits via SPI.
     */

    /* 1. Pulse CONV — rising edge triggers integration */
    gpio_set_level(h->pins.conv, 1);
    ets_delay_us(1);
    gpio_set_level(h->pins.conv, 0);

    /*
     * 2. Wait for conversion to complete.
     *    Conversion takes approximately integration_us, then DVALID goes low.
     *    Wait for the expected integration time, then poll DVALID.
     */
    if (h->integration_us > 100) {
        ets_delay_us(h->integration_us - 100);
    }

    int timeout = 200000;  // 200 ms in 1-us ticks
    while (gpio_get_level(h->pins.dvalid) != 0) {
        ets_delay_us(1);
        if (--timeout <= 0) {
            ESP_LOGE(TAG, "Timeout waiting for DVALID");
            return ESP_ERR_TIMEOUT;
        }
    }

    /*
     * 3. Read 640 bits (80 bytes).  SPI clocks MSB-first.
     *    DDC232 outputs channel 0 first, MSB first, 20 bits per channel.
     */
    uint8_t rx_buf[80] = {0};
    spi_transaction_t txn = {
        .length    = 640,        // 32 channels × 20 bits
        .rxlength  = 640,
        .rx_buffer = rx_buf,
    };
    esp_err_t err = spi_device_transmit(h->spi, &txn);
    if (err != ESP_OK) return err;

    /* Unpack 20-bit words from the byte stream */
    for (int ch = 0; ch < DDC232_NUM_CHANNELS; ch++) {
        int bit_offset = ch * 20;
        int byte_idx   = bit_offset / 8;
        int bit_shift  = bit_offset % 8;

        uint32_t raw = ((uint32_t)rx_buf[byte_idx]     << 24) |
                       ((uint32_t)rx_buf[byte_idx + 1] << 16) |
                       ((uint32_t)rx_buf[byte_idx + 2] <<  8) |
                       ((uint32_t)(byte_idx + 3 < 80 ? rx_buf[byte_idx + 3] : 0));

        raw <<= bit_shift;          // align MSB of 20-bit field to bit 31
        raw >>= (32 - 20);          // shift down to low 20 bits

        data[ch] = sign_extend_20(raw);
    }

    return ESP_OK;
}

esp_err_t ddc232_deinit(ddc232_handle_t h)
{
    if (!h) return ESP_ERR_INVALID_ARG;
    mclk_stop();
    spi_bus_remove_device(h->spi);
    spi_bus_free(SPI_HOST_USED);
    free(h);
    ESP_LOGI(TAG, "De-initialised");
    return ESP_OK;
}
