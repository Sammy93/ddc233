#pragma once

#include <Arduino.h>
#include <SPI.h>

/* ---- Pin assignment (adjust to your PCB) ---- */
#define PIN_CLK        14   // LEDC PWM -> DDC system clock
#define PIN_CONV       8    // GPIO out -> DDC CONV
#define PIN_DVALID     7    // GPIO in  <- DDC DVALID (active-low)
#define PIN_DCLK       12   // SPI CLK  -> DDC DCLK
#define PIN_DOUT       13   // SPI MISO <- DDC DOUT
#define PIN_DIN        11   // SPI MOSI -> DDC DIN
#define PIN_DIN_CFG    6    // GPIO out -> DDC DIN_CFG (config data)
#define PIN_CLK_CFG    5    // GPIO out -> DDC CLK_CFG (config clock)
#define PIN_RESET      15   // GPIO out -> DDC RESET (active-low)

/* ---- HV2901 Pin assignment (2x daisy-chained, shares FSPI/DAC bus) ---- */
#define PIN_HV_MOSI    17   // DIN  -> HV2901 serial data in
#define PIN_HV_MISO    18   // DOUT <- HV2901 serial data out
#define PIN_HV_CLK     16   // CLK  -> HV2901 shift register clock
#define PIN_HV_CLR     47   // CLR  -> HV2901 clear (active-high, all switches OFF)
#define PIN_HV_LE      48   // LE   -> HV2901 latch enable (active-low)
#define HV_SPI_CLK_HZ  1000000  // 8 MHz SPI clock for HV2901 (max at Vdd=3V)

#define HV_NUM_CHIPS       2
#define HV_ACTIVE_CHIPS    2    // both chips populated
#define HV_BITS_PER_CHIP   32
#define HV_PAIRS_PER_CHIP  16

/* ---- DAC8562 Pin assignment (2x DAC on SPI2/FSPI) ---- */
#define PIN_DAC_SCK    1    // SPI CLK  -> DAC SCLK
#define PIN_DAC_MOSI   2    // SPI MOSI -> DAC DIN
#define PIN_DAC_CS1    45   // SPI CS   -> DAC1 SYNC (active-low)
#define PIN_DAC_CS2    3    // SPI CS   -> DAC2 SYNC (active-low)
#define PIN_DAC_CLR    21   // GPIO out -> DAC CLR (active-low)

/* ---- Operating parameters ---- */
#define DEFAULT_CLK_HZ          10000000  // 10 MHz system clock
#define DEFAULT_INTEGRATION_US  1000      // 1 ms
#define DEFAULT_RANGE           1         // 50 pC

#define NUM_CHANNELS  32
#define DATA_BITS     20
#define CFG_REG_BITS  12
#define SPI_CLK_HZ    20000000  // 20 MHz DCLK for data readout (DDC232 max)

/* ---- DAC8562 parameters ---- */
#define DAC_SPI_CLK_HZ   20000000  // 20 MHz SPI clock (DAC8562 max 50 MHz)
#define DAC_SAMPLE_RATE   50000    // 50 kHz DDS sample rate
#define DAC_LUT_SIZE      256      // sine lookup table entries
#define DAC_PHASE_BITS    32       // DDS phase accumulator width

/* ---- Range labels ---- */
static const char* range_labels[] = {
    "12.5 pC", "50 pC", "100 pC", "150 pC",
    "200 pC",  "250 pC", "300 pC", "350 pC"
};
static const float range_pc[] = {
    12.5f, 50.0f, 100.0f, 150.0f, 200.0f, 250.0f, 300.0f, 350.0f
};

/* ---- Matrix scan config ---- */
#define MATRIX_ROWS      12
#define MATRIX_COLS      12
#define MATRIX_COL_START 0    // first HV pair = col 0

// Physical channels 1-12 -> data[] indices (0-based):
// DDC232 DOUT outputs IN32 first (Fig.17), so data[i] = IN(32-i).
// Physical CHn -> INx (PCB routing) -> data index (32-x).
//   Phys 1->IN15->idx17, Phys 2->IN16->idx16, Phys 3->IN31->idx1,  Phys 4->IN32->idx0,
//   Phys 5->IN14->idx18, Phys 6->IN12->idx20, Phys 7->IN13->idx19, Phys 8->IN11->idx21,
//   Phys 9->IN28->idx4,  Phys10->IN30->idx2,  Phys11->IN27->idx5,  Phys12->IN10->idx22
static const int matrix_row_channels[MATRIX_ROWS] = {
    17, 16, 1, 0, 18, 20, 19, 21, 4, 2, 5, 22
};
// Physical channel labels for text output (matches row order above)
static const int matrix_row_physical[MATRIX_ROWS] = {
    1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12
};

// Full 32-channel mapping: physical channel P (1-based) -> DDC data index.
// physical_to_ddc[0] = DDC index for physical channel 1, etc.
// First 12 entries match matrix_row_channels[].
static const int physical_to_ddc[NUM_CHANNELS] = {
    17, 16,  1,  0, 18, 20, 19, 21,   // phys 1-8
     4,  2,  5, 22,  6,  3,  7, 23,   // phys 9-16
     8, 24,  9, 25, 10, 13, 11, 12,   // phys 17-24
    28, 26, 27, 29, 15, 14, 31, 30,   // phys 25-32
};

// Matrix binary frame sync: "MATX"
#define MATRIX_SYNC_0  0x4D
#define MATRIX_SYNC_1  0x41
#define MATRIX_SYNC_2  0x54
#define MATRIX_SYNC_3  0x58

/* ---- 12x32 full matrix: 12 HV columns x all 32 DDC channels ---- */
#define FULL_MATRIX_COLS  MATRIX_COLS   // 12 HV pairs
#define FULL_MATRIX_ROWS  NUM_CHANNELS  // 32 DDC channels
// Full matrix binary frame sync: "MF32"
#define FULL_SYNC_0  0x4D
#define FULL_SYNC_1  0x46
#define FULL_SYNC_2  0x33
#define FULL_SYNC_3  0x32

/* ---- Debug matrix: 8 cols (even HV pairs) x 12 rows (odd physical channels) ---- */
#define DBG_MATRIX_COLS  8
#define DBG_MATRIX_ROWS  12
// Column HV pairs: 0, 2, 4, 6, 8, 10, 12, 14 (even-numbered, 0-indexed)
static const int dbg_col_pairs[DBG_MATRIX_COLS] = {
    0, 2, 4, 6, 8, 10, 12, 14
};
// Physical odd channels 1,3,...,23 -> data[] indices (0-based):
static const int dbg_row_channels[DBG_MATRIX_ROWS] = {
    17, 1, 18, 19, 4, 5, 6, 7, 8, 9, 10, 11
};
// Physical channel labels for text output (matches row order above)
static const int dbg_row_physical[DBG_MATRIX_ROWS] = {
    1, 3, 5, 7, 9, 11, 13, 15, 17, 19, 21, 23
};

// Debug matrix binary frame sync: "MDBG"
#define DBG_SYNC_0  0x4D
#define DBG_SYNC_1  0x44
#define DBG_SYNC_2  0x42
#define DBG_SYNC_3  0x47

/* ---- 12x12 Debug matrix: even HV pairs across 2 chips x 12 odd channels ---- */
#define DBG12_MATRIX_COLS  12
#define DBG12_MATRIX_ROWS  12
// Column mapping: chip 0 even pairs (0,2,...,14) then chip 1 even pairs (0,2,4,6)
static const int dbg12_col_chip[DBG12_MATRIX_COLS] = {
    0, 0, 0, 0, 0, 0, 0, 0,  1, 1, 1, 1
};
static const int dbg12_col_pair[DBG12_MATRIX_COLS] = {
    0, 2, 4, 6, 8, 10, 12, 14,  0, 2, 4, 6
};
// Rows: same 12 odd physical channels as 8x12 debug (reuse dbg_row_channels/physical)

// 12x12 debug matrix binary frame sync: "MD12"
#define DBG12_SYNC_0  0x4D
#define DBG12_SYNC_1  0x44
#define DBG12_SYNC_2  0x31
#define DBG12_SYNC_3  0x32

/* ---- Stream sync markers ---- */
#define SYNC_MARKER_0 0xAA
#define SYNC_MARKER_1 0x55

/* ---- Frame sizes ---- */
#define MATRIX_FRAME_SIZE 588
#define FULL_FRAME_SIZE (12 + FULL_MATRIX_COLS * FULL_MATRIX_ROWS * 4)    // 1548
#define DBG_FRAME_SIZE (12 + DBG_MATRIX_COLS * DBG_MATRIX_ROWS * 4)      // 396
#define DBG12_FRAME_SIZE (12 + DBG12_MATRIX_COLS * DBG12_MATRIX_ROWS * 4) // 588

/* ---- DAC voltage transfer functions ---- */
#define DAC_BIAS_GAIN     4.0f
#define DAC_BIAS_OFFSET   5.0f     // V_opamp = GAIN * V_dac - OFFSET
#define DAC_SW_GAIN       7.8f
#define DAC_V_PER_CODE    (5.0f / 65536.0f)

/* ================================================================
 * Extern declarations for shared globals (defined in ddc233.ino)
 * ================================================================ */

/* System state */
extern uint32_t clk_freq_hz;
extern uint32_t integration_us;
extern uint32_t matrix_dead_us;
extern uint16_t hv_skip_vneg;
extern uint8_t  current_range;
extern bool     test_mode;
extern bool     clk_4x;

/* Voltage tracking */
extern float    last_vbias1;
extern float    last_vbias2;
extern float    last_vsw1;
extern float    last_vsw2;

/* SPI instances */
extern SPIClass hspi;
extern SPIClass dac_spi;

/* DAC state */
extern volatile bool dac_running;
extern TaskHandle_t  dac_task_handle;
extern hw_timer_t   *dac_timer;
extern uint32_t      dac1_freq_hz;
extern uint32_t      dac2_freq_hz;

/* Pipeline state */
extern bool     pipeline_primed;
extern uint32_t conv_toggle_time;
extern bool     conv_state;
extern bool     matrix_primed;
