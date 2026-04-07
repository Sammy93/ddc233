#include "hv2901.h"
#include "dac8562.h"

/* ================================================================
 * HV2901 high-voltage analog switch control (2x daisy-chained)
 * ================================================================
 *
 * Each HV2901 has 32 switches organized as 16 pairs:
 *   SW0/SW1 share Y01, SW2/SW3 share Y23, ... SW30/SW31 share Y3031
 *
 * CRITICAL: Within each pair, only ONE switch may be ON at a time.
 * Both ON = short circuit between the two source signals.
 *
 * Daisy chain: data is shifted through chip 1 into chip 2.
 * First 32 bits shifted out end up in chip 2 (far end),
 * last 32 bits shifted out end up in chip 1 (near ESP32).
 *
 * Hardware SPI interface (shares FSPI with DAC8562, reconfigured per transfer):
 *   - CLK rising edge clocks data into shift register (SPI Mode 0)
 *   - LE low pulse latches shift register to switch outputs
 *   - CLR high forces all switches OFF (override)
 */

/* Switch state: one uint32_t per chip, bit N = switch N state */
uint32_t hv_reg[HV_NUM_CHIPS] = {0, 0};

/* Temporarily pause DAC DDS if running, reconfigure FSPI for HV2901 */
static bool hv_dac_was_running = false;

void hv_spi_begin()
{
    hv_dac_was_running = dac_running;
    if (hv_dac_was_running) {
        /* Stop timer first so no new ISR notifications arrive */
        if (dac_timer) {
            timerEnd(dac_timer);
            dac_timer = NULL;
        }
        dac_running = false;
        vTaskDelay(pdMS_TO_TICKS(1));  // let any in-flight DAC transfer finish
    }
    dac_spi.end();
    dac_spi.begin(PIN_HV_CLK, PIN_HV_MISO, PIN_HV_MOSI, -1);
}

/* Reconfigure FSPI back to DAC8562 pins, resume DAC if it was running */
void hv_spi_end()
{
    dac_spi.end();
    dac_spi.begin(PIN_DAC_SCK, -1, PIN_DAC_MOSI, -1);

    /* Drive HV CLK/MOSI low so they don't float and inject noise */
    pinMode(PIN_HV_CLK, OUTPUT);
    digitalWrite(PIN_HV_CLK, LOW);
    pinMode(PIN_HV_MOSI, OUTPUT);
    digitalWrite(PIN_HV_MOSI, LOW);

    if (hv_dac_was_running) {
        dac_running = true;
        /* Restart timer */
        dac_timer = timerBegin(1000000);
        timerAttachInterrupt(dac_timer, &dac_timer_isr);
        timerAlarm(dac_timer, 20, true, 0);
    }
}

/**
 * Enforce mutual exclusivity: for each switch pair, if both sw_even and
 * sw_odd are set, turn both OFF. This prevents shoot-through damage.
 * Called automatically before every SPI write to hardware.
 */
void hv_enforce_exclusivity()
{
    for (int chip = 0; chip < HV_NUM_CHIPS; chip++) {
        for (int p = 0; p < HV_PAIRS_PER_CHIP; p++) {
            uint32_t mask_even = 1UL << (p * 2);
            uint32_t mask_odd  = 1UL << (p * 2 + 1);
            if ((hv_reg[chip] & mask_even) && (hv_reg[chip] & mask_odd)) {
                /* Both switches ON — force both OFF and warn */
                hv_reg[chip] &= ~(mask_even | mask_odd);
                Serial.printf("WARNING: HV chip %d pair %d had both switches ON — forced OFF\n",
                              chip, p);
            }
        }
    }
}

void hv_write()
{
    hv_enforce_exclusivity();

    /* Hold LE high (opaque) while shifting to avoid glitches */
    digitalWrite(PIN_HV_LE, HIGH);

    /* Reconfigure FSPI for HV2901 pins (pauses DAC if running) */
    hv_spi_begin();
    dac_spi.beginTransaction(SPISettings(HV_SPI_CLK_HZ, MSBFIRST, SPI_MODE0));

    /* Shift out: far chip first, near chip last. */
    for (int chip = HV_ACTIVE_CHIPS - 1; chip >= 0; chip--) {
        dac_spi.transfer32(hv_reg[chip]);
    }

    dac_spi.endTransaction();

    /* Pulse LE low (transparent — data flows to switches),
     * then back high (rising edge captures and holds). */
    digitalWrite(PIN_HV_LE, LOW);
    delayMicroseconds(5);   /* t_WLE: 56 ns min at 3.3V, use 5 us for margin */
    digitalWrite(PIN_HV_LE, HIGH);
    delayMicroseconds(1);

    /* Restore FSPI to DAC8562 pins (resumes DAC if it was running) */
    hv_spi_end();
}

void hv_init()
{
    /* Step 1: Set control pins to known safe state immediately */
    pinMode(PIN_HV_CLR,  OUTPUT);
    digitalWrite(PIN_HV_CLR, HIGH);   // CLR active — all switches OFF, safe state

    pinMode(PIN_HV_LE,   OUTPUT);
    digitalWrite(PIN_HV_LE, HIGH);    // latch opaque

    /* Step 2: Hold CLR high for 20 ms */
    delay(20);

    /* Step 3: Release CLR, then wait for the chip to settle */
    digitalWrite(PIN_HV_CLR, LOW);
    delay(5);

    /* Step 4: Cycle CLR again — double-tap to be sure */
    digitalWrite(PIN_HV_CLR, HIGH);
    delay(20);
    digitalWrite(PIN_HV_CLR, LOW);
    delay(5);

    /* Step 5: Flush shift registers with zeros via SPI (all active chips). */
    hv_spi_begin();
    dac_spi.beginTransaction(SPISettings(HV_SPI_CLK_HZ, MSBFIRST, SPI_MODE0));
    for (int i = 0; i < HV_ACTIVE_CHIPS; i++) {
        dac_spi.transfer32(0x00000000);
    }
    dac_spi.endTransaction();
    hv_spi_end();

    /* Step 6: NOW safe to latch — shift register is all zeros */
    digitalWrite(PIN_HV_LE, LOW);
    delayMicroseconds(100);
    digitalWrite(PIN_HV_LE, HIGH);
    delayMicroseconds(10);

    /* Step 7: Normal write using hv_write() to confirm */
    hv_reg[0] = 0;
    hv_reg[1] = 0;
    hv_write();

    Serial.println("HV2901 initialized (all switches OFF)");
}

/**
 * Set a switch pair on a given chip.
 *   chip: 0 or 1
 *   pair: 0-15
 *   state: 0 = SW_even ON, 1 = SW_odd ON, -1 = both OFF
 */
bool hv_set_pair(int chip, int pair, int state)
{
    if (chip < 0 || chip >= HV_NUM_CHIPS) return false;
    if (pair < 0 || pair >= HV_PAIRS_PER_CHIP) return false;

    int sw_even = pair * 2;
    int sw_odd  = pair * 2 + 1;

    /* Clear both switches in the pair first (safety) */
    hv_reg[chip] &= ~((1UL << sw_even) | (1UL << sw_odd));

    /* Set the requested switch */
    if (state == 0) {
        hv_reg[chip] |= (1UL << sw_even);
    } else if (state == 1) {
        hv_reg[chip] |= (1UL << sw_odd);
    }
    /* state == -1: both stay OFF */

    hv_write();
    return true;
}

/**
 * Set all HV pairs to Vneg (FETs OFF), except skipped pairs get both OFF.
 * Does NOT call hv_write — caller is responsible for SPI transfer.
 */
void hv_all_neg()
{
    uint32_t reg = 0;
    for (int p = 0; p < HV_PAIRS_PER_CHIP; p++) {
        if (!(hv_skip_vneg & (1 << p))) {
            reg |= (1UL << (p * 2 + 1));
        }
    }
    hv_reg[0] = reg;
}

/**
 * Configure hv_reg[0] so that one column pair has sw_even ON (Vpos ->
 * FET ON), inactive pairs get Vneg (or OFF if in hv_skip_vneg mask),
 * and pairs 12-15 get Vneg/OFF likewise.
 * Does NOT call hv_write — caller is responsible for SPI transfer.
 */
void hv_set_column(int col)
{
    uint32_t reg = 0;
    for (int p = MATRIX_COL_START; p < MATRIX_COL_START + MATRIX_COLS; p++) {
        if (p != MATRIX_COL_START + col) {
            if (!(hv_skip_vneg & (1 << p))) {
                reg |= (1UL << (p * 2 + 1));  // inactive -> Vneg
            }
            /* else: both OFF (skipped pair) */
        } else {
            reg |= (1UL << (p * 2));           // active -> Vpos
        }
    }
    for (int p = MATRIX_COL_START + MATRIX_COLS; p < HV_PAIRS_PER_CHIP; p++) {
        if (!(hv_skip_vneg & (1 << p))) {
            reg |= (1UL << (p * 2 + 1));
        }
    }
    hv_reg[0] = reg;
}

/**
 * Debug matrix column select: uses even-numbered HV pairs (0,2,4,6,8,10,12,14).
 * Active column pair -> sw_even ON (Vpos -> FET ON),
 * other 7 debug pairs -> NEG (sw_odd ON -> Vneg, FET OFF),
 * all non-debug pairs -> NEG (FET OFF).
 * col_idx: 0-7 index into dbg_col_pairs[].
 */
void hv_set_column_debug(int col_idx)
{
    uint32_t reg = 0;
    for (int i = 0; i < DBG_MATRIX_COLS; i++) {
        int p = dbg_col_pairs[i];
        if (i != col_idx) {
            if (!(hv_skip_vneg & (1 << p))) {
                reg |= (1UL << (p * 2 + 1));  // inactive -> Vneg
            }
        } else {
            reg |= (1UL << (p * 2));           // active -> Vpos
        }
    }
    for (int p = 0; p < HV_PAIRS_PER_CHIP; p++) {
        bool is_debug = false;
        for (int i = 0; i < DBG_MATRIX_COLS; i++) {
            if (dbg_col_pairs[i] == p) { is_debug = true; break; }
        }
        if (!is_debug && !(hv_skip_vneg & (1 << p))) {
            reg |= (1UL << (p * 2 + 1));
        }
    }
    hv_reg[0] = reg;
}

/**
 * SPI write to HV2901 assuming FSPI is already configured for HV pins.
 * Skips hv_spi_begin()/hv_spi_end() bus reconfiguration.
 * Used inside matrix scan loop where FSPI stays on HV pins.
 */
void hv_write_fast()
{
    hv_enforce_exclusivity();

    digitalWrite(PIN_HV_LE, HIGH);

    dac_spi.beginTransaction(SPISettings(HV_SPI_CLK_HZ, MSBFIRST, SPI_MODE0));
    for (int chip = HV_ACTIVE_CHIPS - 1; chip >= 0; chip--) {
        dac_spi.transfer32(hv_reg[chip]);
    }
    dac_spi.endTransaction();

    digitalWrite(PIN_HV_LE, LOW);
    delayMicroseconds(5);
    digitalWrite(PIN_HV_LE, HIGH);
    delayMicroseconds(1);
}

/**
 * 12x12 debug column select: even HV pairs across 2 chips.
 * Chip 0: pairs 0,2,4,6,8,10,12,14  (cols 0-7)
 * Chip 1: pairs 0,2,4,6             (cols 8-11)
 * Active column -> Vpos, all others -> Vneg (or OFF if in skip mask).
 */
void hv_set_column_debug12(int col_idx)
{
    uint32_t reg0 = 0, reg1 = 0;

    for (int i = 0; i < DBG12_MATRIX_COLS; i++) {
        int chip = dbg12_col_chip[i];
        int p = dbg12_col_pair[i];
        uint32_t *reg = (chip == 0) ? &reg0 : &reg1;

        if (i == col_idx) {
            *reg |= (1UL << (p * 2));           // active -> Vpos
        } else {
            if (!(hv_skip_vneg & (1 << p))) {
                *reg |= (1UL << (p * 2 + 1));   // inactive -> Vneg
            }
        }
    }

    /* Non-debug pairs -> Vneg on both chips */
    for (int chip = 0; chip < HV_ACTIVE_CHIPS; chip++) {
        uint32_t *reg = (chip == 0) ? &reg0 : &reg1;
        for (int p = 0; p < HV_PAIRS_PER_CHIP; p++) {
            bool is_debug = false;
            for (int i = 0; i < DBG12_MATRIX_COLS; i++) {
                if (dbg12_col_chip[i] == chip && dbg12_col_pair[i] == p) {
                    is_debug = true;
                    break;
                }
            }
            if (!is_debug && !(hv_skip_vneg & (1 << p))) {
                *reg |= (1UL << (p * 2 + 1));
            }
        }
    }

    hv_reg[0] = reg0;
    hv_reg[1] = reg1;
}
