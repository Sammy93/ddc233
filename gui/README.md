# DDC232/233 Readout Software

## Overview

ESP32-S3 firmware reads 32 channels of 20-bit charge data from a DDC232/233 via SPI. A Python GUI streams, plots, and logs the data in real time. Two daisy-chained HV2901 high-voltage switch chips provide a 12-column matrix scan, and two DAC8562 provide bias/switch voltage control.

## Quick Start

```
pip install pyserial matplotlib numpy
python ddc233_gui.py
```

Select the ESP32 serial port, click **Connect**, choose a mode, click **Start**.

## GUI Controls

### Row 1 — Connection & Mode

| Control | Description |
|---------|-------------|
| **Port** | Serial port dropdown. Click **Refresh** to rescan. |
| **Connect / Disconnect** | Open or close the serial connection. |
| **Mode** | Selects what data is streamed and how the heatmap is displayed (see Modes below). |

### Row 2 — Streaming & Display

| Control | Description |
|---------|-------------|
| **Delay (ms)** | Minimum interval between frames (0 = max speed). |
| **Start / Stop** | Begin or end continuous binary streaming. |
| **Single Read** | One-shot readout of all 32 channels (text mode). |
| **Matrix Scan** | One-shot 12x12 matrix scan (text mode, not streamed). |
| **Debug Scan** | One-shot 8x12 or 12x12 debug scan (text mode). |
| **Avg/col** | Number of integration cycles averaged per column in debug modes. Higher = slower but cleaner. |
| **Units** | Display units: **Bits** (raw 20-bit ADC), **nA** (current), **pC** (charge). nA and pC require correct range and integration time settings. |
| **SMA** | Simple Moving Average window for the line plot (1 = off). |
| **Notch** | Notch filter on line plot: Off, 50 Hz, or 60 Hz. |
| **Scale** | Check to manually set heatmap color limits (min-max). Unchecked = auto-scale. |
| **Calibrate Zero** | Captures the current average reading as a zero offset. Subtracts it from all subsequent data. |
| **Clear Cal** | Removes the zero calibration. |
| **Start Log / Stop Log** | Saves streamed data to a timestamped CSV file with a header containing all device settings. |

### Modes

| Mode | Firmware command | Data shape | Description |
|------|-----------------|------------|-------------|
| **1x32 All** | `stream` | 32 channels | All 32 DDC channels, no HV switching. |
| **1x12 Manual** | `stream` | Channels 1-12 | Same stream, displays only physical channels 1-12. |
| **1x12 Odd** | `stream` | Odd channels | Same stream, displays odd physical channels (1,3,...,23). |
| **12x12 Matrix** | `mstream` | 12 cols x 12 rows | Sequential HV pairs 0-11 as columns, physical channels 1-12 as rows. |
| **12x32 Full** | `mf32stream` | 12 cols x 32 rows | Same column switching, all 32 DDC channels per column. |
| **8x12 Debug** | `mdstream` | 8 cols x 12 rows | Even HV pairs on chip 1 (P0,P2,...,P14), odd physical channels as rows. Per-column priming eliminates crosstalk. |
| **12x12 Debug** | `md12stream` | 12 cols x 12 rows | Even HV pairs across both chips (chip 1: P0-P14, chip 2: P0-P6), odd physical channels. Same per-column priming. |

### Plots

- **Top: Heatmap** — Color-coded latest reading. In matrix modes, shows the full grid. When calibrated with a bias voltage set, resistance values are overlaid on each cell.
- **Bottom: Line plot** — Scrolling time series. In single/manual/odd modes, select individual channels via checkboxes. In matrix modes, use the **Show** (row/column/average) and **Index** selectors to choose which traces to display.

### DDC232 Controls

| Control | Description |
|---------|-------------|
| **Range** | Full-scale charge range (0-7): 12.5 pC to 350 pC. Writes the DDC232 config register. |
| **Int. time (us)** | Integration time in microseconds (default 1000). Longer = more charge accumulated per sample. |
| **CLK (Hz)** | DDC232 system clock frequency (default 10 MHz). |
| **Test mode** | Disconnects DDC inputs internally — measures zero-input baseline. |
| **Verify Config** | Reads back the DDC232 config register to confirm settings. |

### DAC Voltage Controls

| Control | Range | Description |
|---------|-------|-------------|
| **VBias1** | -5 to +5 V | Bias voltage channel 1 (DAC2 ch A). |
| **VBias2** | -5 to +5 V | Bias voltage channel 2 (DAC2 ch B). |
| **VSW1** | 0 to +30 V | Positive switch voltage (DAC1 ch A). |
| **VSW2** | -30 to 0 V | Negative switch voltage (DAC1 ch B). |

Setting any voltage stops the DAC sine wave generator if it was running.

### HV2901 Switch Matrix (Chip 1 & Chip 2)

Each chip has 16 switch pairs (0-15). Each pair has two switches sharing a common output:

- **POS** — SW_even ON (connects to positive voltage rail)
- **NEG** — SW_odd ON (connects to negative voltage rail)
- **OFF** — Both switches disconnected

Only one switch per pair can be on at a time (hardware-enforced).

- **All OFF / All POS / All NEG** — Bulk control for chip 1.
- **Skip Vneg** checkboxes — When checked, inactive pairs use OFF instead of NEG during matrix scans (reduces leakage for unused columns).
- **Chip 2** has its own panel with the same POS/NEG/OFF controls and an All OFF button.

### Resistance Overlay

When **Calibrate Zero** is active and **VBias1** is set to a non-zero voltage, the heatmap overlays calculated resistance values (R = V / I) on each cell. Values are formatted as ohms, k-ohms, M-ohms, or G-ohms. "OL" means open loop (near-zero current).

## Firmware Serial Commands

All commands are available via the serial console at 115200 baud. Type `help` for the full list. The GUI sends these commands automatically.
