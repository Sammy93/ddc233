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
 * DDC232 12-bit configuration register (clocked in via DIN_CFG / CLK_CFG):
 *
 *   Bits 11-9 : FSR — full-scale range (3 bits, 000=12.5pC .. 111=350pC)
 *   Bit  8    : FORMAT (0=16-bit data, 1=20-bit data)
 *   Bit  7    : Reserved / device version
 *   Bit  6    : CLK_4x (0=no division, 1=divide CLK by 4 internally)
 *   Bits 5-1  : Reserved (0)
 *   Bit  0    : TEST (0=normal, 1=test mode — inputs disconnected)
 *
 * Integration time is NOT in the register; it is determined by the
 * period of the CONV signal toggling.
 */
#define CFG_REG_BITS  12

/* Internal device struct */
struct ddc232_dev {
    ddc232_pins_t   pins;
    uint32_t        clk_freq_hz;
    uint32_t        integration_us;
    ddc232_range_t  range;
    bool            test_mode;
    bool            clk_4x;
    spi_device_handle_t spi;
};

/* ---------- System CLK via LEDC ---------- */

static esp_err_t clk_start(gpio_num_t pin, uint32_t freq_hz)
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

static esp_err_t clk_set_freq(uint32_t freq_hz)
{
    return ledc_set_freq(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0, freq_hz);
}

static esp_err_t clk_stop(void)
{
    return ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
}

/* ---------- Configuration register via DIN_CFG / CLK_CFG (bit-bang) ---------- */

/**
 * Build and clock out the 12-bit configuration register via DIN_CFG/CLK_CFG.
 * Data is clocked in on falling edges of CLK_CFG, MSB first.
 * CLK must be running. CONV must be low. DIN_CFG/CLK_CFG must not toggle
 * during data readout.
 */
static esp_err_t write_config_register(struct ddc232_dev *dev)
{
    uint16_t reg = 0;

    reg |= ((uint16_t)(dev->range & 0x07)) << 9;   // bits 11-9: FSR
    reg |= (1 << 8);                                 // bit 8: FORMAT = 1 (20-bit)
    /* bit 7: reserved, leave 0 */
    if (dev->clk_4x)
        reg |= (1 << 6);                             // bit 6: CLK_4x
    /* bits 5-1: reserved, 0 */
    if (dev->test_mode)
        reg |= (1 << 0);                             // bit 0: TEST

    /* Ensure CONV is low during config write */
    gpio_set_level(dev->pins.conv, 0);
    gpio_set_level(dev->pins.clk_cfg, 0);

    /* Clock out 12 bits, MSB first, on falling edge of CLK_CFG */
    for (int i = CFG_REG_BITS - 1; i >= 0; i--) {
        /* Set DIN_CFG to the current bit */
        gpio_set_level(dev->pins.din_cfg, (reg >> i) & 1);
        ets_delay_us(1);

        /* Rising edge */
        gpio_set_level(dev->pins.clk_cfg, 1);
        ets_delay_us(1);

        /* Falling edge — DDC232 latches the bit */
        gpio_set_level(dev->pins.clk_cfg, 0);
        ets_delay_us(1);
    }

    gpio_set_level(dev->pins.din_cfg, 0);

    ESP_LOGI(TAG, "Config register written: 0x%03X (range=%d, format=20bit, test=%d, clk4x=%d)",
             reg, dev->range, dev->test_mode, dev->clk_4x);
    return ESP_OK;
}

/* ---------- SPI readout setup ---------- */

#define SPI_HOST_USED  SPI2_HOST
#define SPI_CLK_HZ     (4 * 1000 * 1000)  // 4 MHz DCLK

static esp_err_t spi_setup(const ddc232_pins_t *p, spi_device_handle_t *out)
{
    spi_bus_config_t bus = {
        .mosi_io_num   = -1,            // not used for data readout
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
        .spics_io_num   = -1,           // no CS — framed by DVALID
        .queue_size     = 1,
    };
    return spi_bus_add_device(SPI_HOST_USED, &dev, out);
}

/* ---------- GPIO helpers ---------- */

static esp_err_t gpio_setup(const ddc232_pins_t *p)
{
    // Outputs: CONV, RESET, DIN_CFG, CLK_CFG
    uint64_t out_mask = (1ULL << p->conv) |
                        (1ULL << p->reset) |
                        (1ULL << p->din_cfg) |
                        (1ULL << p->clk_cfg);
    gpio_config_t out_cfg = {
        .pin_bit_mask = out_mask,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    esp_err_t err = gpio_config(&out_cfg);
    if (err != ESP_OK) return err;

    gpio_set_level(p->conv, 0);
    gpio_set_level(p->reset, 0);     // hold in reset initially
    gpio_set_level(p->din_cfg, 0);
    gpio_set_level(p->clk_cfg, 0);

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
    dev->clk_freq_hz    = cfg->clk_freq_hz;
    dev->integration_us = cfg->integration_us;
    dev->range          = cfg->range;
    dev->test_mode      = false;
    dev->clk_4x         = false;

    esp_err_t err;

    err = gpio_setup(&dev->pins);
    if (err != ESP_OK) { free(dev); return err; }

    /*
     * DDC232 power-on / reset sequence:
     *  1. Assert RESET low (already done in gpio_setup), hold CONV low
     *  2. Start CLK — DDC232 needs a running system clock
     *  3. Wait for CLK to stabilise
     *  4. Pulse RESET: low for >= 1 us, then release high
     *  5. Wait for the device to initialise
     *  6. Write the 12-bit config register via DIN_CFG/CLK_CFG
     *  7. Run a dummy CONV cycle to flush stale integrator state
     */

    /* 1-2. RESET is already low; start CLK */
    err = clk_start(dev->pins.clk, dev->clk_freq_hz);
    if (err != ESP_OK) { free(dev); return err; }

    /* 3. Let CLK settle */
    ets_delay_us(10);

    /* 4. Release RESET (was held low since gpio_setup) */
    gpio_set_level(dev->pins.reset, 1);

    /* 5. Wait for DDC232 to initialise after reset */
    ets_delay_us(1000);

    /* 6. Write initial configuration register */
    err = write_config_register(dev);
    if (err != ESP_OK) { clk_stop(); free(dev); return err; }

    err = spi_setup(&dev->pins, &dev->spi);
    if (err != ESP_OK) { clk_stop(); free(dev); return err; }

    /* 7. Dummy conversion to flush integrators */
    ESP_LOGI(TAG, "Running dummy conversion to flush integrators...");
    gpio_set_level(dev->pins.conv, 1);
    ets_delay_us(dev->integration_us);
    gpio_set_level(dev->pins.conv, 0);

    /* Wait for DVALID low (data ready), then discard */
    int timeout = 200000;
    while (gpio_get_level(dev->pins.dvalid) != 0 && --timeout > 0) {
        ets_delay_us(1);
    }
    if (timeout <= 0) {
        ESP_LOGW(TAG, "Timeout on dummy conversion (DVALID not asserted) — "
                      "check wiring and CLK");
    } else {
        uint8_t discard[80] = {0};
        spi_transaction_t txn = {
            .length    = 640,
            .rxlength  = 640,
            .rx_buffer = discard,
        };
        spi_device_transmit(dev->spi, &txn);
    }

    ESP_LOGI(TAG, "Initialised: CLK=%lu Hz, integration=%lu us, range=%d",
             dev->clk_freq_hz, dev->integration_us, dev->range);

    *out_handle = dev;
    return ESP_OK;
}

/**
 * Write config register then strobe CONV + flush to enter normal operation.
 */
static esp_err_t apply_config(struct ddc232_dev *dev)
{
    esp_err_t err = write_config_register(dev);
    if (err != ESP_OK) return err;

    /* Strobe CONV to begin normal operation after config write */
    gpio_set_level(dev->pins.conv, 1);
    ets_delay_us(10);
    gpio_set_level(dev->pins.conv, 0);

    /* Flush stale integrator data */
    gpio_set_level(dev->pins.conv, 1);
    ets_delay_us(dev->integration_us);
    gpio_set_level(dev->pins.conv, 0);

    int timeout = 200000;
    while (gpio_get_level(dev->pins.dvalid) != 0 && --timeout > 0) {
        ets_delay_us(1);
    }
    if (timeout > 0) {
        uint8_t discard[80] = {0};
        spi_transaction_t txn = {
            .length    = 640,
            .rxlength  = 640,
            .rx_buffer = discard,
        };
        spi_device_transmit(dev->spi, &txn);
    }

    return ESP_OK;
}

esp_err_t ddc232_set_range(ddc232_handle_t h, ddc232_range_t range)
{
    if (!h || range > DDC232_RANGE_350PC) return ESP_ERR_INVALID_ARG;
    h->range = range;
    return apply_config(h);
}

esp_err_t ddc232_set_integration_time(ddc232_handle_t h, uint32_t us)
{
    if (!h || us == 0) return ESP_ERR_INVALID_ARG;
    h->integration_us = us;
    ESP_LOGI(TAG, "Integration time set to %lu us", us);
    return ESP_OK;
}

esp_err_t ddc232_set_clk(ddc232_handle_t h, uint32_t freq_hz)
{
    if (!h || freq_hz == 0) return ESP_ERR_INVALID_ARG;
    esp_err_t err = clk_set_freq(freq_hz);
    if (err == ESP_OK) {
        h->clk_freq_hz = freq_hz;
        ESP_LOGI(TAG, "CLK set to %lu Hz", freq_hz);
    }
    return err;
}

esp_err_t ddc232_set_test_mode(ddc232_handle_t h, bool enable)
{
    if (!h) return ESP_ERR_INVALID_ARG;
    h->test_mode = enable;
    ESP_LOGI(TAG, "Test mode %s", enable ? "ON" : "OFF");
    return apply_config(h);
}

esp_err_t ddc232_read(ddc232_handle_t h, int32_t data[DDC232_NUM_CHANNELS])
{
    if (!h || !data) return ESP_ERR_INVALID_ARG;

    /*
     * DDC232 conversion cycle:
     *  1. Toggle CONV high — starts integration on one side of the
     *     dual integrator, while the other side's data becomes available.
     *  2. Hold CONV high for the integration time.
     *  3. Wait for DVALID to go low (data ready).
     *  4. Clock out 32 × 20-bit = 640 bits via SPI (DCLK/DOUT).
     *
     * The integration time is determined by how long CONV stays in each
     * state (high or low), not by anything in the config register.
     */

    /* 1-2. Assert CONV for the integration window */
    gpio_set_level(h->pins.conv, 1);
    ets_delay_us(h->integration_us);
    gpio_set_level(h->pins.conv, 0);

    /* 3. Wait for DVALID low (timeout after 200 ms) */
    int timeout = 200000;
    while (gpio_get_level(h->pins.dvalid) != 0) {
        ets_delay_us(1);
        if (--timeout <= 0) {
            ESP_LOGE(TAG, "Timeout waiting for DVALID");
            return ESP_ERR_TIMEOUT;
        }
    }

    /*
     * 4. Read 640 bits (80 bytes).  SPI clocks MSB-first.
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
    gpio_set_level(h->pins.reset, 0);  // hold DDC232 in reset
    clk_stop();
    spi_bus_remove_device(h->spi);
    spi_bus_free(SPI_HOST_USED);
    free(h);
    ESP_LOGI(TAG, "De-initialised");
    return ESP_OK;
}
