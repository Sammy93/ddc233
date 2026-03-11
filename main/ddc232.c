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

/* Internal device struct */
struct ddc232_dev {
    ddc232_pins_t   pins;
    uint32_t        mclk_freq_hz;
    uint32_t        integration_us;
    ddc232_range_t  range;
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

/* ---------- Range pins ---------- */

static void set_range_pins(const ddc232_pins_t *p, ddc232_range_t range)
{
    gpio_set_level(p->range0, (range >> 0) & 1);
    gpio_set_level(p->range1, (range >> 1) & 1);
}

/* ---------- SPI readout setup ---------- */

#define SPI_HOST_USED  SPI2_HOST
#define SPI_CLK_HZ     (4 * 1000 * 1000)  // 4 MHz data clock

static esp_err_t spi_setup(const ddc232_pins_t *p, spi_device_handle_t *out)
{
    spi_bus_config_t bus = {
        .mosi_io_num   = -1,            // not used
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
    // Outputs: CONV, RANGE0, RANGE1
    uint64_t out_mask = (1ULL << p->conv) |
                        (1ULL << p->range0) |
                        (1ULL << p->range1);
    gpio_config_t out_cfg = {
        .pin_bit_mask = out_mask,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    esp_err_t err = gpio_config(&out_cfg);
    if (err != ESP_OK) return err;

    gpio_set_level(p->conv, 0);

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

    esp_err_t err;

    err = gpio_setup(&dev->pins);
    if (err != ESP_OK) { free(dev); return err; }

    set_range_pins(&dev->pins, dev->range);

    err = mclk_start(dev->pins.mclk, dev->mclk_freq_hz);
    if (err != ESP_OK) { free(dev); return err; }

    err = spi_setup(&dev->pins, &dev->spi);
    if (err != ESP_OK) { mclk_stop(); free(dev); return err; }

    ESP_LOGI(TAG, "Initialised: MCLK=%lu Hz, integration=%lu us, range=%d",
             dev->mclk_freq_hz, dev->integration_us, dev->range);

    *out_handle = dev;
    return ESP_OK;
}

esp_err_t ddc232_set_range(ddc232_handle_t h, ddc232_range_t range)
{
    if (!h || range > DDC232_RANGE_150PC) return ESP_ERR_INVALID_ARG;
    h->range = range;
    set_range_pins(&h->pins, range);
    ESP_LOGI(TAG, "Range set to %d", range);
    return ESP_OK;
}

esp_err_t ddc232_set_integration_time(ddc232_handle_t h, uint32_t us)
{
    if (!h || us == 0) return ESP_ERR_INVALID_ARG;
    h->integration_us = us;
    ESP_LOGI(TAG, "Integration time set to %lu us", us);
    return ESP_OK;
}

esp_err_t ddc232_set_mclk(ddc232_handle_t h, uint32_t freq_hz)
{
    if (!h || freq_hz == 0) return ESP_ERR_INVALID_ARG;
    esp_err_t err = mclk_set_freq(freq_hz);
    if (err == ESP_OK) {
        h->mclk_freq_hz = freq_hz;
        ESP_LOGI(TAG, "MCLK set to %lu Hz", freq_hz);
    }
    return err;
}

esp_err_t ddc232_read(ddc232_handle_t h, int32_t data[DDC232_NUM_CHANNELS])
{
    if (!h || !data) return ESP_ERR_INVALID_ARG;

    /*
     * DDC232 conversion cycle:
     *  1. Drive CONV high — starts integration on side A
     *  2. Hold CONV high for the desired integration time
     *  3. Drive CONV low  — side A results become available for readout,
     *     side B starts integrating
     *  4. Wait for DVALID to go low (data ready)
     *  5. Clock out 32 × 20-bit = 640 bits via SPI
     */

    /* 1-2. Assert CONV for the integration window */
    gpio_set_level(h->pins.conv, 1);
    ets_delay_us(h->integration_us);

    /* 3. De-assert CONV */
    gpio_set_level(h->pins.conv, 0);

    /* 4. Wait for DVALID low (timeout after 100 ms) */
    int timeout = 100000;  // 100 ms in 1-us ticks
    while (gpio_get_level(h->pins.dvalid) != 0) {
        ets_delay_us(1);
        if (--timeout <= 0) {
            ESP_LOGE(TAG, "Timeout waiting for DVALID");
            return ESP_ERR_TIMEOUT;
        }
    }

    /*
     * 5. Read 640 bits (80 bytes).  SPI clocks MSB-first.
     *    DDC232 outputs channel 1 first, MSB first, 20 bits per channel.
     */
    uint8_t rx_buf[80] = {0};
    spi_transaction_t txn = {
        .length    = 640,        // 32 channels × 20 bits
        .rxlength  = 640,
        .rx_buffer = rx_buf,
    };
    esp_err_t err = spi_device_transmit(h->spi, &txn);
    if (err != ESP_OK) return err;

    /* Unpack 20-bit words from the byte stream.
     * Bit layout (MSB first): channel 0 occupies bits 639..620,
     * channel 1 bits 619..600, etc.  In the rx_buf, byte 0 bit 7 is
     * the first bit clocked in (MSB of channel 0).
     *
     * Each 20-bit value starts at bit offset (ch * 20) from the MSB.
     */
    for (int ch = 0; ch < DDC232_NUM_CHANNELS; ch++) {
        int bit_offset = ch * 20;
        int byte_idx   = bit_offset / 8;
        int bit_shift  = bit_offset % 8;

        /* Grab 4 bytes starting at byte_idx and shift to align */
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
