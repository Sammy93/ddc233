#!/usr/bin/env python3
"""
DDC232/233 Real-Time Readout GUI

Python GUI for streaming and plotting 32-channel charge data from the
DDC232/233 via serial. Supports 1x32 single-channel mode and 12x12
matrix scan mode with heatmap and time-series visualization.

Requirements:
    pip install pyserial matplotlib numpy

Usage:
    python ddc233_gui.py
"""

import gc
import os
import csv
import datetime
import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import struct
import threading
import queue
import time
import numpy as np
import serial
import serial.tools.list_ports
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import matplotlib.patheffects as pe

NUM_CHANNELS = 32
HISTORY_LEN = 500  # rolling buffer size for line plots
DEFAULT_DELAY_MS = 0
# DDC232 coding:
#   Code 0       = zero input (confirmed by test mode reading ~4000)
#   Code 1048575 = positive full scale (0xFFFFF, 20-bit max)
ZERO_CODE = 0
MAX_CODE  = 1048575        # 0xFFFFF — positive full scale code
FS_RANGE_CODES = MAX_CODE  # full code range from zero to full scale

# Full-scale charge in pC for each range setting (index 0-7)
RANGE_PC = [12.5, 50.0, 100.0, 150.0, 200.0, 250.0, 300.0, 350.0]

# Matrix mode constants
SYNC_MATRIX = b'MATX'
FRAME_SIZE_MATRIX = 588  # 4 sync + 4 timestamp + 4 seq + 12*12*4 data
MATRIX_ROWS = 12
MATRIX_COLS = 12

# Firmware sends data in physical channel order (1-32).
# Frame index i = physical channel (i+1).
MATRIX_ROW_PHYSICAL = list(range(1, 13))  # [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12]

# Manual 1x12 mode: first 12 frame indices = physical channels 1-12
MANUAL_PHYS_CHANNELS = MATRIX_ROW_PHYSICAL
MANUAL_INDICES = list(range(12))
NUM_MANUAL_CHANNELS = 12

# Odd physical channels (1, 3, 5, ..., 23) → frame indices 0, 2, 4, ..., 22
ODD_PHYS_CHANNELS = list(range(1, 24, 2))  # [1, 3, 5, 7, 9, 11, 13, 15, 17, 19, 21, 23]
ODD_INDICES = list(range(0, 24, 2))  # frame indices for phys 1, 3, 5, ..., 23
NUM_ODD_CHANNELS = 12

# 12x32 full matrix mode constants
SYNC_FULL_MATRIX = b'MF32'
FULL_MATRIX_COLS = 12
FULL_MATRIX_ROWS = NUM_CHANNELS  # 32
FRAME_SIZE_FULL = 12 + FULL_MATRIX_COLS * FULL_MATRIX_ROWS * 4  # 1548

# Debug matrix mode constants (even HV pairs × odd physical channels)
SYNC_DBG_MATRIX = b'MDBG'
DBG_MATRIX_COLS = 8
DBG_MATRIX_ROWS = 12
FRAME_SIZE_DBG = 12 + DBG_MATRIX_COLS * DBG_MATRIX_ROWS * 4  # 396
DBG_COL_PAIRS = [0, 2, 4, 6, 8, 10, 12, 14]
# Physical labels for debug rows (what the user sees)
DBG_ROW_PHYSICAL = [1, 3, 5, 7, 9, 11, 13, 15, 17, 19, 21, 23]

# 12x12 Debug matrix mode (even HV pairs across 2 chips × odd channels)
SYNC_DBG12_MATRIX = b'MD12'
DBG12_MATRIX_COLS = 12
DBG12_MATRIX_ROWS = 12
FRAME_SIZE_DBG12 = 12 + DBG12_MATRIX_COLS * DBG12_MATRIX_ROWS * 4  # 588
# Column labels: chip:pair
DBG12_COL_CHIP = [0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1]
DBG12_COL_PAIR = [0, 2, 4, 6, 8, 10, 12, 14, 0, 2, 4, 6]
DBG12_COL_LABELS = [f"C{c}:P{p}" for c, p in zip(DBG12_COL_CHIP, DBG12_COL_PAIR)]
DBG12_ROW_PHYSICAL = DBG_ROW_PHYSICAL  # same 12 odd channels


class SerialReader:
    """Manages serial connection and background reading thread."""

    def __init__(self, data_queue: queue.Queue):
        self.port = None
        self.ser = None
        self.data_queue = data_queue
        self._stop_event = threading.Event()
        self._thread = None
        self.streaming = False
        self.stream_mode = "single"  # "single" or "matrix"
        self.dbg_avg = 1

    def connect(self, port: str, baudrate: int = 115200) -> str:
        try:
            self.ser = serial.Serial(port, baudrate, timeout=1)
            self.port = port
            # flush startup text
            time.sleep(0.5)
            self.ser.reset_input_buffer()
            return None
        except Exception as e:
            return str(e)

    def disconnect(self):
        self.stop_streaming()
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.ser = None
        self.port = None

    def send_command(self, cmd: str):
        if self.ser and self.ser.is_open:
            self.ser.write((cmd.strip() + "\n").encode())

    def start_streaming(self, mode="single", dbg_avg=1):
        if not self.ser or not self.ser.is_open:
            return
        self.stream_mode = mode
        self.dbg_avg = max(1, dbg_avg)
        self._stop_event.clear()
        self.streaming = True
        if mode == "matrix":
            target = self._read_loop_matrix
        elif mode == "full":
            target = self._read_loop_full_matrix
        elif mode == "debug":
            target = self._read_loop_dbg_matrix
        elif mode == "debug12":
            target = self._read_loop_dbg12_matrix
        else:
            target = self._read_loop_binary
        self._thread = threading.Thread(target=target, daemon=True)
        self._thread.start()

    def stop_streaming(self):
        self._stop_event.set()
        self.streaming = False
        # Send a byte to tell the ESP32 to stop
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(b'\x00')
            except Exception:
                pass
        if self._thread:
            self._thread.join(timeout=2)
            self._thread = None
        # Wait for ESP32 to finish last frame + print text summary, then flush
        if self.ser and self.ser.is_open:
            time.sleep(0.5)
            self.ser.reset_input_buffer()
            # Drain any stragglers that arrived during the flush
            time.sleep(0.1)
            self.ser.reset_input_buffer()

    SYNC = b'\xAA\x55\xAA\x55'
    FRAME_SIZE = 136  # 4 sync + 4 timestamp + 32*4 data

    def _read_loop_binary(self):
        """Send stream command and parse binary frames."""
        self.ser.reset_input_buffer()
        self.send_command("stream")

        # Discard text header lines until we see the sentinel
        time.sleep(0.1)
        while self.ser.in_waiting:
            line = self.ser.readline().decode(errors="replace").strip()
            if "Send any byte to stop" in line:
                break
            time.sleep(0.02)

        buf = bytearray()

        while not self._stop_event.is_set():
            try:
                # Read available bytes
                waiting = self.ser.in_waiting
                if waiting > 0:
                    buf.extend(self.ser.read(waiting))
                else:
                    buf.extend(self.ser.read(1))  # block for 1 byte

                # Scan for complete frames
                while len(buf) >= self.FRAME_SIZE:
                    # Find sync marker
                    idx = buf.find(self.SYNC)
                    if idx < 0:
                        # No sync found — keep last 3 bytes (partial sync)
                        del buf[:-3]
                        break
                    if idx > 0:
                        # Discard bytes before sync
                        del buf[:idx]
                    if len(buf) < self.FRAME_SIZE:
                        break

                    # Parse frame: skip sync(4) + timestamp(4), then 32 × int32
                    values = list(struct.unpack_from('<32i', buf, 8))
                    del buf[:self.FRAME_SIZE]

                    # Sanity check: offset binary 20-bit values (0 to 1048575)
                    if all(0 <= v <= 1048575 for v in values):
                        try:
                            self.data_queue.put_nowait(
                                {"type": "single", "data": values})
                        except queue.Full:
                            pass  # drop frame if GUI can't keep up

            except Exception:
                if self._stop_event.is_set():
                    break

    def _read_loop_matrix(self):
        """Send mstream command and parse 12x12 matrix frames."""
        self.ser.reset_input_buffer()
        self.send_command("mstream")

        # Discard text header lines until we see the sentinel
        time.sleep(0.1)
        while self.ser.in_waiting:
            line = self.ser.readline().decode(errors="replace").strip()
            if "Send any byte to stop" in line:
                break
            time.sleep(0.02)

        buf = bytearray()

        while not self._stop_event.is_set():
            try:
                waiting = self.ser.in_waiting
                if waiting > 0:
                    buf.extend(self.ser.read(waiting))
                else:
                    buf.extend(self.ser.read(1))

                while len(buf) >= FRAME_SIZE_MATRIX:
                    idx = buf.find(SYNC_MATRIX)
                    if idx < 0:
                        del buf[:-3]
                        break
                    if idx > 0:
                        del buf[:idx]
                    if len(buf) < FRAME_SIZE_MATRIX:
                        break

                    # Parse: sync(4) + timestamp(4) + seq(4) + 12*12 int32
                    values = struct.unpack_from(f'<{MATRIX_COLS * MATRIX_ROWS}i', buf, 12)
                    del buf[:FRAME_SIZE_MATRIX]
                    # Reshape to (12, 12) column-major: data[col][row] in firmware
                    matrix = np.array(values, dtype=np.float64).reshape(
                        (MATRIX_COLS, MATRIX_ROWS))

                    try:
                        self.data_queue.put_nowait(
                            {"type": "matrix", "data": matrix})
                    except queue.Full:
                        pass

            except Exception:
                if self._stop_event.is_set():
                    break

    def _read_loop_full_matrix(self):
        """Send mf32stream command and parse 12x32 full matrix frames."""
        self.ser.reset_input_buffer()
        self.send_command("mf32stream")

        # Discard text header lines until we see the sentinel
        time.sleep(0.1)
        while self.ser.in_waiting:
            line = self.ser.readline().decode(errors="replace").strip()
            if "Send any byte to stop" in line:
                break
            time.sleep(0.02)

        buf = bytearray()

        while not self._stop_event.is_set():
            try:
                waiting = self.ser.in_waiting
                if waiting > 0:
                    buf.extend(self.ser.read(waiting))
                else:
                    buf.extend(self.ser.read(1))

                while len(buf) >= FRAME_SIZE_FULL:
                    idx = buf.find(SYNC_FULL_MATRIX)
                    if idx < 0:
                        del buf[:-3]
                        break
                    if idx > 0:
                        del buf[:idx]
                    if len(buf) < FRAME_SIZE_FULL:
                        break

                    values = struct.unpack_from(
                        f'<{FULL_MATRIX_COLS * FULL_MATRIX_ROWS}i', buf, 12)
                    del buf[:FRAME_SIZE_FULL]
                    matrix = np.array(values, dtype=np.float64).reshape(
                        (FULL_MATRIX_COLS, FULL_MATRIX_ROWS))

                    try:
                        self.data_queue.put_nowait(
                            {"type": "full", "data": matrix})
                    except queue.Full:
                        pass

            except Exception:
                if self._stop_event.is_set():
                    break

    def _read_loop_dbg_matrix(self):
        """Send mdstream command and parse 8x12 debug matrix frames."""
        self.ser.reset_input_buffer()
        self.send_command(f"mdstream {self.dbg_avg}")

        # Discard all text header lines before binary data starts
        time.sleep(0.1)
        while self.ser.in_waiting:
            line = self.ser.readline().decode(errors="replace").strip()
            if "Send any byte to stop" in line:
                break
            time.sleep(0.02)

        buf = bytearray()
        frame_count = 0

        while not self._stop_event.is_set():
            try:
                waiting = self.ser.in_waiting
                if waiting > 0:
                    buf.extend(self.ser.read(waiting))
                else:
                    buf.extend(self.ser.read(1))

                while len(buf) >= FRAME_SIZE_DBG:
                    idx = buf.find(SYNC_DBG_MATRIX)
                    if idx < 0:
                        del buf[:-3]
                        break
                    if idx > 0:
                        del buf[:idx]
                    if len(buf) < FRAME_SIZE_DBG:
                        break

                    values = struct.unpack_from(
                        f'<{DBG_MATRIX_COLS * DBG_MATRIX_ROWS}i', buf, 12)
                    del buf[:FRAME_SIZE_DBG]
                    matrix = np.array(values, dtype=np.float64).reshape(
                        (DBG_MATRIX_COLS, DBG_MATRIX_ROWS))

                    try:
                        self.data_queue.put_nowait(
                            {"type": "debug", "data": matrix})
                    except queue.Full:
                        pass
                    frame_count += 1
                    if frame_count == 1:
                        print(f"[dbg_stream] first frame received, "
                              f"range [{matrix.min():.0f}, {matrix.max():.0f}]")

            except Exception as e:
                if self._stop_event.is_set():
                    break
                print(f"[dbg_stream] exception: {e}")

    def _read_loop_dbg12_matrix(self):
        """Send md12stream command and parse 12x12 debug matrix frames."""
        self.ser.reset_input_buffer()
        self.send_command(f"md12stream {self.dbg_avg}")

        time.sleep(0.1)
        while self.ser.in_waiting:
            line = self.ser.readline().decode(errors="replace").strip()
            if "Send any byte to stop" in line:
                break
            time.sleep(0.02)

        buf = bytearray()
        frame_count = 0

        while not self._stop_event.is_set():
            try:
                waiting = self.ser.in_waiting
                if waiting > 0:
                    buf.extend(self.ser.read(waiting))
                else:
                    buf.extend(self.ser.read(1))

                while len(buf) >= FRAME_SIZE_DBG12:
                    idx = buf.find(SYNC_DBG12_MATRIX)
                    if idx < 0:
                        del buf[:-3]
                        break
                    if idx > 0:
                        del buf[:idx]
                    if len(buf) < FRAME_SIZE_DBG12:
                        break

                    values = struct.unpack_from(
                        f'<{DBG12_MATRIX_COLS * DBG12_MATRIX_ROWS}i', buf, 12)
                    del buf[:FRAME_SIZE_DBG12]
                    matrix = np.array(values, dtype=np.float64).reshape(
                        (DBG12_MATRIX_COLS, DBG12_MATRIX_ROWS))

                    try:
                        self.data_queue.put_nowait(
                            {"type": "debug12", "data": matrix})
                    except queue.Full:
                        pass
                    frame_count += 1
                    if frame_count == 1:
                        print(f"[dbg12_stream] first frame received, "
                              f"range [{matrix.min():.0f}, {matrix.max():.0f}]")

            except Exception as e:
                if self._stop_event.is_set():
                    break
                print(f"[dbg12_stream] exception: {e}")


class DDC233Gui:
    """Main GUI application."""

    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("DDC232/233 Readout")
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        self.data_queue = queue.Queue(maxsize=200)
        self.reader = SerialReader(self.data_queue)

        # Mode: "single", "odd", "matrix", "debug", "debug12"
        self.display_mode = "single"

        # Data buffers — single mode (also used by odd mode, which filters from it)
        self.history = np.zeros((HISTORY_LEN, NUM_CHANNELS), dtype=np.float64)
        self.history_idx = 0
        self.history_filled = False
        self.latest = np.zeros(NUM_CHANNELS, dtype=np.float64)

        # Data buffers — matrix mode (12x12)
        self.matrix_latest = np.zeros((MATRIX_COLS, MATRIX_ROWS), dtype=np.float64)
        self.matrix_history = np.zeros((HISTORY_LEN, MATRIX_COLS, MATRIX_ROWS),
                                       dtype=np.float64)
        self.matrix_history_idx = 0
        self.matrix_history_filled = False

        # Data buffers — full matrix mode (12x32)
        self.full_latest = np.zeros((FULL_MATRIX_COLS, FULL_MATRIX_ROWS), dtype=np.float64)
        self.full_history = np.zeros((HISTORY_LEN, FULL_MATRIX_COLS, FULL_MATRIX_ROWS),
                                     dtype=np.float64)
        self.full_history_idx = 0
        self.full_history_filled = False

        # Data buffers — debug matrix mode (8x12)
        self.dbg_latest = np.zeros((DBG_MATRIX_COLS, DBG_MATRIX_ROWS), dtype=np.float64)
        self.dbg_history = np.zeros((HISTORY_LEN, DBG_MATRIX_COLS, DBG_MATRIX_ROWS),
                                    dtype=np.float64)
        self.dbg_history_idx = 0
        self.dbg_history_filled = False

        # Data buffers — 12x12 debug matrix mode
        self.dbg12_latest = np.zeros((DBG12_MATRIX_COLS, DBG12_MATRIX_ROWS), dtype=np.float64)
        self.dbg12_history = np.zeros((HISTORY_LEN, DBG12_MATRIX_COLS, DBG12_MATRIX_ROWS),
                                      dtype=np.float64)
        self.dbg12_history_idx = 0
        self.dbg12_history_filled = False

        # Calibration offsets (raw ADC units, None = not calibrated)
        self.dbg12_calibration = None
        self.dbg_calibration = None
        self.full_calibration = None
        self.matrix_calibration = None
        self.single_calibration = None
        self._resistance_texts = []

        # Selected channels for line plot (start with first 4)
        self.selected_channels = set(range(4))

        # Matrix line plot selection
        self.matrix_trace_mode = "row"  # "row" or "column"
        self.matrix_trace_idx = 0       # which row or column to show

        # FPS tracking
        self._sample_count = 0
        self._fps_time = time.time()
        self._measured_fps = 0.0
        self._last_plot_time = 0
        self._last_gc_time = 0

        # Pre-allocated work buffer for time-series reordering
        # (avoids creating new numpy arrays every plot update — prevents memory leak)
        self._work_buf = np.empty((HISTORY_LEN, NUM_CHANNELS), dtype=np.float64)

        # Logging state
        self._log_file = None
        self._log_writer = None
        self._log_sample_idx = 0

        # HV2901 switch state: "off", "pos", or "neg" for each pair
        self.sw_pairs = 16  # pairs per chip (chip 1 only for now)
        self.sw_vars = []   # StringVars, populated in _build_switch_controls

        # Build bottom panels FIRST (pack side=BOTTOM) so they're always visible,
        # then the plot fills the remaining space.
        self._build_config_status()
        self._build_switch_controls()
        self._build_dac_controls()
        self._build_controls()
        self._build_channel_selector()
        self._build_toolbar()
        self._build_plots()

        # Start GUI update loop
        self._poll_data()

    # ----------------------------------------------------------------
    # GUI construction
    # ----------------------------------------------------------------

    def _build_toolbar(self):
        # --- Row 1: Connection and mode ---
        row1 = ttk.Frame(self.root)
        row1.pack(side=tk.TOP, fill=tk.X, padx=5, pady=(3, 0))

        # Serial port
        ttk.Label(row1, text="Port:").pack(side=tk.LEFT)
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(
            row1, textvariable=self.port_var, width=20, state="readonly"
        )
        self.port_combo.pack(side=tk.LEFT, padx=(2, 5))
        self._refresh_ports()

        ttk.Button(row1, text="Refresh", command=self._refresh_ports).pack(
            side=tk.LEFT, padx=2
        )

        self.connect_btn = ttk.Button(
            row1, text="Connect", command=self._toggle_connect
        )
        self.connect_btn.pack(side=tk.LEFT, padx=5)

        ttk.Separator(row1, orient=tk.VERTICAL).pack(
            side=tk.LEFT, fill=tk.Y, padx=5
        )

        # Mode selector
        ttk.Label(row1, text="Mode:").pack(side=tk.LEFT)
        self.mode_var = tk.StringVar(value="1x32 All")
        self._mode_map = {
            "1x32 All": "single",
            "1x12 Manual": "manual",
            "1x12 Odd": "odd",
            "12x12 Matrix": "matrix",
            "12x32 Full": "full",
            "8x12 Debug": "debug",
            "12x12 Debug": "debug12",
        }
        mode_combo = ttk.Combobox(
            row1, textvariable=self.mode_var,
            values=list(self._mode_map.keys()),
            width=12, state="readonly",
        )
        mode_combo.pack(side=tk.LEFT, padx=2)
        mode_combo.bind("<<ComboboxSelected>>", lambda e: self._on_mode_change())

        # Status (right-aligned on row 1)
        self.status_var = tk.StringVar(value="Disconnected")
        ttk.Label(row1, textvariable=self.status_var, foreground="gray").pack(
            side=tk.RIGHT, padx=5
        )

        # --- Row 2: Streaming, scan actions, units, calibration ---
        row2 = ttk.Frame(self.root)
        row2.pack(side=tk.TOP, fill=tk.X, padx=5, pady=(1, 3))

        # Streaming controls
        ttk.Label(row2, text="Delay (ms):").pack(side=tk.LEFT)
        self.delay_var = tk.StringVar(value=str(DEFAULT_DELAY_MS))
        ttk.Entry(row2, textvariable=self.delay_var, width=6).pack(
            side=tk.LEFT, padx=(2, 5)
        )

        self.stream_btn = ttk.Button(
            row2, text="Start", command=self._toggle_stream, state=tk.DISABLED
        )
        self.stream_btn.pack(side=tk.LEFT, padx=5)

        self.single_btn = ttk.Button(
            row2, text="Single Read", command=self._single_read, state=tk.DISABLED
        )
        self.single_btn.pack(side=tk.LEFT, padx=2)

        self.mscan_btn = ttk.Button(
            row2, text="Matrix Scan", command=self._matrix_scan, state=tk.DISABLED
        )
        self.mscan_btn.pack(side=tk.LEFT, padx=2)

        self.mdscan_btn = ttk.Button(
            row2, text="Debug Scan", command=self._dbg_matrix_scan, state=tk.DISABLED
        )
        self.mdscan_btn.pack(side=tk.LEFT, padx=2)

        ttk.Label(row2, text="Avg/col:").pack(side=tk.LEFT, padx=(5, 0))
        self.dbg_avg_var = tk.StringVar(value="1")
        ttk.Entry(row2, textvariable=self.dbg_avg_var, width=4).pack(
            side=tk.LEFT, padx=(2, 5)
        )

        ttk.Separator(row2, orient=tk.VERTICAL).pack(
            side=tk.LEFT, fill=tk.Y, padx=5
        )

        # Units selector
        ttk.Label(row2, text="Units:").pack(side=tk.LEFT)
        self.units_var = tk.StringVar(value="Bits")
        units_combo = ttk.Combobox(
            row2, textvariable=self.units_var,
            values=["Bits", "nA", "pC"],
            width=5, state="readonly",
        )
        units_combo.pack(side=tk.LEFT, padx=(2, 5))

        ttk.Separator(row2, orient=tk.VERTICAL).pack(
            side=tk.LEFT, fill=tk.Y, padx=5
        )

        # SMA filter
        ttk.Label(row2, text="SMA:").pack(side=tk.LEFT)
        self.sma_var = tk.StringVar(value="30")
        sma_combo = ttk.Combobox(
            row2, textvariable=self.sma_var,
            values=["1", "5", "10", "20", "30", "50", "100", "200"],
            width=4, state="readonly",
        )
        sma_combo.pack(side=tk.LEFT, padx=(2, 5))

        # Notch filter
        ttk.Label(row2, text="Notch:").pack(side=tk.LEFT, padx=(5, 0))
        self.notch_var = tk.StringVar(value="Off")
        notch_combo = ttk.Combobox(
            row2, textvariable=self.notch_var,
            values=["Off", "50 Hz", "60 Hz"],
            width=5, state="readonly",
        )
        notch_combo.pack(side=tk.LEFT, padx=(2, 5))
        self.notch_info_var = tk.StringVar(value="")
        ttk.Label(row2, textvariable=self.notch_info_var,
                  foreground="gray").pack(side=tk.LEFT, padx=(0, 5))

        # Manual color scale
        ttk.Separator(row2, orient=tk.VERTICAL).pack(
            side=tk.LEFT, fill=tk.Y, padx=5
        )
        self.manual_clim_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(row2, text="Scale:", variable=self.manual_clim_var).pack(
            side=tk.LEFT
        )
        self.clim_min_var = tk.StringVar(value="0")
        ttk.Entry(row2, textvariable=self.clim_min_var, width=7).pack(
            side=tk.LEFT, padx=1
        )
        ttk.Label(row2, text="-").pack(side=tk.LEFT)
        self.clim_max_var = tk.StringVar(value="1000")
        ttk.Entry(row2, textvariable=self.clim_max_var, width=7).pack(
            side=tk.LEFT, padx=1
        )

        ttk.Separator(row2, orient=tk.VERTICAL).pack(
            side=tk.LEFT, fill=tk.Y, padx=5
        )

        # Calibration
        self.cal_btn = ttk.Button(
            row2, text="Calibrate Zero", command=self._calibrate,
            state=tk.DISABLED
        )
        self.cal_btn.pack(side=tk.LEFT, padx=2)
        self.cal_clear_btn = ttk.Button(
            row2, text="Clear Cal", command=self._clear_calibration,
            state=tk.DISABLED
        )
        self.cal_clear_btn.pack(side=tk.LEFT, padx=2)

        ttk.Separator(row2, orient=tk.VERTICAL).pack(
            side=tk.LEFT, fill=tk.Y, padx=5
        )

        # Logging
        self.log_btn = ttk.Button(
            row2, text="Start Log", command=self._toggle_log,
            state=tk.DISABLED
        )
        self.log_btn.pack(side=tk.LEFT, padx=2)
        self.log_status_var = tk.StringVar(value="")
        ttk.Label(row2, textvariable=self.log_status_var,
                  foreground="#27ae60").pack(side=tk.LEFT, padx=2)

    def _build_plots(self):
        self.fig = Figure(tight_layout=True)

        # Heatmap: 1×32 (will switch to 12×12 in matrix mode)
        self.ax_heat = self.fig.add_subplot(2, 1, 1)
        self.heatmap_data = np.zeros((1, NUM_CHANNELS))
        self.im = self.ax_heat.imshow(
            self.heatmap_data,
            aspect="auto",
            cmap="viridis",
            interpolation="nearest",
        )
        self.ax_heat.set_title("Channel Heatmap (latest reading)")
        self.ax_heat.set_xlabel("Channel")
        self.ax_heat.set_yticks([])
        self.ax_heat.set_xticks(range(0, NUM_CHANNELS, 2))
        self.ax_heat.set_xticklabels(
            list(range(1, NUM_CHANNELS + 1, 2)),
            fontsize=7)
        self.cbar = self.fig.colorbar(self.im, ax=self.ax_heat,
                                       orientation="vertical", pad=0.02)

        # Line plot: scrolling time-series
        self.ax_line = self.fig.add_subplot(2, 1, 2)
        self.ax_line.set_title("Channel Time Series")
        self.ax_line.set_xlabel("Sample")
        self.ax_line.set_ylabel("Value (20-bit)")
        self.ax_line.grid(True, alpha=0.3)
        self.lines = {}

        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)

    def _build_channel_selector(self):
        self.ch_frame = ttk.LabelFrame(self.root, text="Channels (line plot)")
        self.ch_frame.pack(side=tk.BOTTOM, fill=tk.X, padx=5, pady=3)

        # Data is already in physical channel order (1-32) from firmware
        self.ch_vars = [None] * NUM_CHANNELS
        self.ch_inner = ttk.Frame(self.ch_frame)
        self.ch_inner.pack(fill=tk.X, padx=3, pady=2)

        for ch in range(NUM_CHANNELS):
            phys = ch + 1
            var = tk.BooleanVar(value=(ch in self.selected_channels))
            cb = ttk.Checkbutton(
                self.ch_inner,
                text=str(phys),
                variable=var,
                command=lambda c=ch, v=var: self._toggle_channel(c, v),
            )
            cb.grid(row=ch // 16, column=ch % 16, padx=2, pady=1)
            self.ch_vars[ch] = var

        self.ch_btn_frame = ttk.Frame(self.ch_frame)
        self.ch_btn_frame.pack(fill=tk.X, padx=3, pady=2)
        ttk.Button(self.ch_btn_frame, text="All", command=self._select_all_ch).pack(
            side=tk.LEFT, padx=2
        )
        ttk.Button(self.ch_btn_frame, text="None", command=self._select_no_ch).pack(
            side=tk.LEFT, padx=2
        )

        # Matrix trace selector (hidden initially)
        self.matrix_trace_frame = ttk.Frame(self.ch_frame)
        ttk.Label(self.matrix_trace_frame, text="Show:").pack(side=tk.LEFT, padx=2)
        self.trace_mode_var = tk.StringVar(value="row")
        ttk.Radiobutton(self.matrix_trace_frame, text="Row traces",
                        variable=self.trace_mode_var, value="row",
                        command=self._on_trace_mode_change).pack(side=tk.LEFT, padx=2)
        ttk.Radiobutton(self.matrix_trace_frame, text="Column traces",
                        variable=self.trace_mode_var, value="column",
                        command=self._on_trace_mode_change).pack(side=tk.LEFT, padx=2)
        ttk.Radiobutton(self.matrix_trace_frame, text="Avg all",
                        variable=self.trace_mode_var, value="avg",
                        command=self._on_trace_mode_change).pack(side=tk.LEFT, padx=2)
        ttk.Label(self.matrix_trace_frame, text="Index:").pack(side=tk.LEFT, padx=(10, 2))
        self.trace_idx_var = tk.StringVar(value="0")
        self.trace_idx_combo = ttk.Combobox(
            self.matrix_trace_frame, textvariable=self.trace_idx_var,
            values=[str(i) for i in range(12)], width=3, state="readonly"
        )
        self.trace_idx_combo.pack(side=tk.LEFT, padx=2)
        self.trace_idx_combo.bind("<<ComboboxSelected>>", self._on_trace_idx_change)
        # Overlay mean line on row/column traces
        self.show_mean_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(self.matrix_trace_frame, text="+ Mean",
                        variable=self.show_mean_var).pack(side=tk.LEFT, padx=(10, 2))

    def _build_dac_controls(self):
        frame = ttk.LabelFrame(self.root, text="DAC Voltage Controls")
        frame.pack(side=tk.BOTTOM, fill=tk.X, padx=5, pady=3)

        inner = ttk.Frame(frame)
        inner.pack(fill=tk.X, padx=3, pady=2)

        # VBias1: -5 to +5 V
        ttk.Label(inner, text="VBias1 (V):").grid(row=0, column=0, padx=2)
        self.vbias1_var = tk.StringVar(value="0.0")
        ttk.Entry(inner, textvariable=self.vbias1_var, width=8).grid(
            row=0, column=1, padx=2
        )
        ttk.Label(inner, text="(-5 to +5)").grid(row=0, column=2, padx=(0, 5))
        ttk.Button(inner, text="Set", command=self._set_vbias1).grid(
            row=0, column=3, padx=5
        )

        ttk.Separator(inner, orient=tk.VERTICAL).grid(
            row=0, column=4, sticky="ns", padx=5
        )

        # VBias2: -5 to +5 V
        ttk.Label(inner, text="VBias2 (V):").grid(row=0, column=5, padx=2)
        self.vbias2_var = tk.StringVar(value="0.0")
        ttk.Entry(inner, textvariable=self.vbias2_var, width=8).grid(
            row=0, column=6, padx=2
        )
        ttk.Label(inner, text="(-5 to +5)").grid(row=0, column=7, padx=(0, 5))
        ttk.Button(inner, text="Set", command=self._set_vbias2).grid(
            row=0, column=8, padx=5
        )

        ttk.Separator(inner, orient=tk.VERTICAL).grid(
            row=0, column=9, sticky="ns", padx=5
        )

        # VSW1: 0 to +30 V
        ttk.Label(inner, text="VSW1 (V):").grid(row=0, column=10, padx=2)
        self.vsw1_var = tk.StringVar(value="0.0")
        ttk.Entry(inner, textvariable=self.vsw1_var, width=8).grid(
            row=0, column=11, padx=2
        )
        ttk.Label(inner, text="(0 to +30)").grid(row=0, column=12, padx=(0, 5))
        ttk.Button(inner, text="Set", command=self._set_vsw1).grid(
            row=0, column=13, padx=5
        )

        ttk.Separator(inner, orient=tk.VERTICAL).grid(
            row=0, column=14, sticky="ns", padx=5
        )

        # VSW2: -30 to 0 V
        ttk.Label(inner, text="VSW2 (V):").grid(row=0, column=15, padx=2)
        self.vsw2_var = tk.StringVar(value="0.0")
        ttk.Entry(inner, textvariable=self.vsw2_var, width=8).grid(
            row=0, column=16, padx=2
        )
        ttk.Label(inner, text="(-30 to 0)").grid(row=0, column=17, padx=(0, 5))
        ttk.Button(inner, text="Set", command=self._set_vsw2).grid(
            row=0, column=18, padx=5
        )

    def _build_controls(self):
        frame = ttk.LabelFrame(self.root, text="DDC232 Controls")
        frame.pack(side=tk.BOTTOM, fill=tk.X, padx=5, pady=3)

        inner = ttk.Frame(frame)
        inner.pack(fill=tk.X, padx=3, pady=2)

        # Range
        ttk.Label(inner, text="Range:").grid(row=0, column=0, padx=2)
        self.range_var = tk.StringVar(value="1")
        range_combo = ttk.Combobox(
            inner,
            textvariable=self.range_var,
            values=[
                "0 - 12.5 pC", "1 - 50 pC", "2 - 100 pC", "3 - 150 pC",
                "4 - 200 pC", "5 - 250 pC", "6 - 300 pC", "7 - 350 pC",
            ],
            width=12,
            state="readonly",
        )
        range_combo.current(1)
        range_combo.grid(row=0, column=1, padx=2)
        ttk.Button(inner, text="Set", command=self._set_range).grid(
            row=0, column=2, padx=5
        )

        # Integration time
        ttk.Label(inner, text="Int. time (us):").grid(row=0, column=3, padx=2)
        self.inttime_var = tk.StringVar(value="1000")
        ttk.Entry(inner, textvariable=self.inttime_var, width=8).grid(
            row=0, column=4, padx=2
        )
        ttk.Button(inner, text="Set", command=self._set_inttime).grid(
            row=0, column=5, padx=5
        )

        # Clock
        ttk.Label(inner, text="CLK (Hz):").grid(row=0, column=6, padx=2)
        self.clk_var = tk.StringVar(value="10000000")
        ttk.Entry(inner, textvariable=self.clk_var, width=10).grid(
            row=0, column=7, padx=2
        )
        ttk.Button(inner, text="Set", command=self._set_clk).grid(
            row=0, column=8, padx=5
        )

        # Test mode
        self.test_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(
            inner, text="Test Mode", variable=self.test_var, command=self._set_test
        ).grid(row=0, column=9, padx=10)

        # Verify config
        ttk.Button(inner, text="Verify Config", command=self._verify_config).grid(
            row=0, column=10, padx=10
        )

    def _build_config_status(self):
        frame = ttk.LabelFrame(self.root, text="Config Status (from device)")
        frame.pack(side=tk.BOTTOM, fill=tk.X, padx=5, pady=3)

        self.config_status_var = tk.StringVar(value="Not connected")
        # Use a tk.Label (not ttk) so we can set background color directly
        self.config_status_label = tk.Label(
            frame, textvariable=self.config_status_var,
            font=("Courier", 13, "bold"), foreground="white",
            background="#2c3e50", anchor="w", justify=tk.LEFT,
            padx=10, pady=6, relief=tk.SUNKEN,
            height=2,
        )
        self.config_status_label.pack(fill=tk.X, padx=5, pady=3)

    def _build_switch_controls(self):
        frame = ttk.LabelFrame(self.root, text="HV2901 Switch Matrix (Chip 1)")
        frame.pack(side=tk.BOTTOM, fill=tk.X, padx=5, pady=3)

        inner = ttk.Frame(frame)
        inner.pack(fill=tk.X, padx=3, pady=2)

        # Header row
        ttk.Label(inner, text="Pair:", font=("", 9, "bold")).grid(
            row=0, column=0, padx=2, sticky="w"
        )
        for p in range(self.sw_pairs):
            sw_even = p * 2
            ttk.Label(inner, text=f"{p}\nY{sw_even:02d}{sw_even+1:02d}",
                      font=("", 8), justify=tk.CENTER).grid(
                row=0, column=p + 1, padx=1
            )

        # Radio buttons: POS / NEG / OFF for each pair
        labels = [("POS", "pos"), ("NEG", "neg"), ("OFF", "off")]
        colors = {"pos": "#27ae60", "neg": "#c0392b", "off": "#7f8c8d"}

        for row_idx, (label, value) in enumerate(labels, start=1):
            ttk.Label(inner, text=label, font=("", 9, "bold"),
                      foreground=colors[value]).grid(
                row=row_idx, column=0, padx=2, sticky="w"
            )

        self.sw_vars = []
        for p in range(self.sw_pairs):
            var = tk.StringVar(value="off")
            self.sw_vars.append(var)
            for row_idx, (label, value) in enumerate(labels, start=1):
                rb = ttk.Radiobutton(
                    inner, variable=var, value=value,
                    command=lambda pair=p: self._set_sw_pair(pair),
                )
                rb.grid(row=row_idx, column=p + 1, padx=1)

        # Skip-Vneg checkboxes: when checked, pair uses OFF instead of Vneg
        ttk.Label(inner, text="Skip\nVneg", font=("", 8, "bold"),
                  foreground="#8e44ad").grid(
            row=len(labels) + 1, column=0, padx=2, sticky="w"
        )
        self.skip_vneg_vars = []
        for p in range(self.sw_pairs):
            var = tk.BooleanVar(value=False)
            self.skip_vneg_vars.append(var)
            cb = ttk.Checkbutton(
                inner, variable=var,
                command=lambda pair=p: self._toggle_skip_vneg(pair),
            )
            cb.grid(row=len(labels) + 1, column=p + 1, padx=1)

        # Clear All button
        btn_frame = ttk.Frame(frame)
        btn_frame.pack(fill=tk.X, padx=3, pady=2)
        ttk.Button(btn_frame, text="All OFF", command=self._sw_clear_all).pack(
            side=tk.LEFT, padx=2
        )
        ttk.Button(btn_frame, text="All POS", command=self._sw_all_pos).pack(
            side=tk.LEFT, padx=2
        )
        ttk.Button(btn_frame, text="All NEG", command=self._sw_all_neg).pack(
            side=tk.LEFT, padx=2
        )

        # ---- Chip 2 ----
        frame2 = ttk.LabelFrame(self.root, text="HV2901 Switch Matrix (Chip 2)")
        frame2.pack(side=tk.BOTTOM, fill=tk.X, padx=5, pady=3)

        inner2 = ttk.Frame(frame2)
        inner2.pack(fill=tk.X, padx=3, pady=2)

        ttk.Label(inner2, text="Pair:", font=("", 9, "bold")).grid(
            row=0, column=0, padx=2, sticky="w"
        )
        for p in range(self.sw_pairs):
            sw_even = p * 2
            ttk.Label(inner2, text=f"{p}\nY{sw_even:02d}{sw_even+1:02d}",
                      font=("", 8), justify=tk.CENTER).grid(
                row=0, column=p + 1, padx=1
            )

        for row_idx, (label, value) in enumerate(labels, start=1):
            ttk.Label(inner2, text=label, font=("", 9, "bold"),
                      foreground=colors[value]).grid(
                row=row_idx, column=0, padx=2, sticky="w"
            )

        self.sw2_vars = []
        for p in range(self.sw_pairs):
            var = tk.StringVar(value="off")
            self.sw2_vars.append(var)
            for row_idx, (label, value) in enumerate(labels, start=1):
                rb = ttk.Radiobutton(
                    inner2, variable=var, value=value,
                    command=lambda pair=p: self._set_sw2_pair(pair),
                )
                rb.grid(row=row_idx, column=p + 1, padx=1)

        btn_frame2 = ttk.Frame(frame2)
        btn_frame2.pack(fill=tk.X, padx=3, pady=2)
        ttk.Button(btn_frame2, text="All OFF", command=self._sw2_clear_all).pack(
            side=tk.LEFT, padx=2
        )

    # ----------------------------------------------------------------
    # Unit conversion
    # ----------------------------------------------------------------

    def _get_scale_and_label(self):
        """Return (conversion_func, y_label) based on current units selection.

        DDC232 offset binary: Code 0x10000 = zero, 0xFFFFF = full scale.
        Q = ((Code - 65536) / 982911) * FSR_pC
        I(nA) = Q(pC) * 1000 / int_us
        """
        units = self.units_var.get()
        if units in ("nA", "pC"):
            try:
                range_idx = int(self.range_var.get().split(" ")[0])
            except (ValueError, IndexError):
                range_idx = 1
            try:
                int_us = float(self.inttime_var.get())
            except ValueError:
                int_us = 1000.0
            if int_us <= 0:
                int_us = 1000.0
            range_pc = RANGE_PC[range_idx] if 0 <= range_idx <= 7 else 50.0
            # scale converts (code - ZERO_CODE) to pC
            pc_per_code = range_pc / FS_RANGE_CODES
            if units == "pC":
                return pc_per_code, "Charge (pC)"
            else:  # nA
                # I(nA) = Q(pC) * 1000 / int_us
                return pc_per_code * 1000.0 / int_us, "Current (nA)"
        else:
            return 1.0, "Value (bits)"

    def _apply_clim(self, data):
        """Set heatmap color limits — manual if checkbox is on, else auto from data."""
        if self.manual_clim_var.get():
            try:
                vmin = float(self.clim_min_var.get())
                vmax = float(self.clim_max_var.get())
                if vmin >= vmax:
                    vmax = vmin + 1
                self.im.set_clim(vmin, vmax)
                return
            except ValueError:
                pass
        # Auto scale
        vmin = data.min()
        vmax = data.max()
        scale, _ = self._get_scale_and_label()
        if vmin == vmax:
            fallback = max(abs(vmin) * 0.01, 1e-6) if scale != 1.0 else 1
            vmax = vmin + fallback
        self.im.set_clim(vmin, vmax)

    # ----------------------------------------------------------------
    # Calibration & resistance
    # ----------------------------------------------------------------

    def _get_nA_scale(self):
        """Return scale factor converting raw ADC code to nA."""
        try:
            range_idx = int(self.range_var.get().split(" ")[0])
        except (ValueError, IndexError):
            range_idx = 1
        try:
            int_us = float(self.inttime_var.get())
        except ValueError:
            int_us = 1000.0
        if int_us <= 0:
            int_us = 1000.0
        range_pc = RANGE_PC[range_idx] if 0 <= range_idx <= 7 else 50.0
        return range_pc / FS_RANGE_CODES * 1000.0 / int_us

    @staticmethod
    def _format_resistance(r_ohms):
        """Format resistance value for heatmap overlay (compact)."""
        if not np.isfinite(r_ohms):
            return "OL"
        neg = r_ohms < 0
        r = abs(r_ohms)
        if r < 1e3:
            s = f"{r:.0f}"
        elif r < 10e3:
            s = f"{r/1e3:.1f}k"
        elif r < 1e6:
            s = f"{r/1e3:.0f}k"
        elif r < 10e6:
            s = f"{r/1e6:.1f}M"
        elif r < 1e9:
            s = f"{r/1e6:.0f}M"
        else:
            s = f"{r/1e9:.0f}G"
        return f"-{s}" if neg else s

    def _clear_resistance_texts(self):
        for t in self._resistance_texts:
            t.remove()
        self._resistance_texts = []

    def _calibrate(self):
        """Capture current readings as zero offset for the active mode."""
        mode = self.display_mode
        if mode == "debug":
            n = min(self.dbg_history_idx, HISTORY_LEN)
            if n > 0:
                idx = self.dbg_history_idx % HISTORY_LEN
                if self.dbg_history_filled:
                    hist = np.concatenate([self.dbg_history[idx:],
                                           self.dbg_history[:idx]])
                else:
                    hist = self.dbg_history[:n]
                avg_n = min(n, 50)
                self.dbg_calibration = hist[-avg_n:].mean(axis=0)
            else:
                self.dbg_calibration = self.dbg_latest.copy()
            avg_n = min(n, 50) if n > 0 else 1
            self.status_var.set(f"Calibrated (debug, {avg_n} frames averaged)")
        elif mode == "debug12":
            n = min(self.dbg12_history_idx, HISTORY_LEN)
            if n > 0:
                idx = self.dbg12_history_idx % HISTORY_LEN
                if self.dbg12_history_filled:
                    hist = np.concatenate([self.dbg12_history[idx:],
                                           self.dbg12_history[:idx]])
                else:
                    hist = self.dbg12_history[:n]
                avg_n = min(n, 50)
                self.dbg12_calibration = hist[-avg_n:].mean(axis=0)
            else:
                self.dbg12_calibration = self.dbg12_latest.copy()
            avg_n = min(n, 50) if n > 0 else 1
            self.status_var.set(f"Calibrated (debug12, {avg_n} frames averaged)")
        elif mode == "matrix":
            n = min(self.matrix_history_idx, HISTORY_LEN)
            if n > 0:
                idx = self.matrix_history_idx % HISTORY_LEN
                if self.matrix_history_filled:
                    hist = np.concatenate([self.matrix_history[idx:],
                                           self.matrix_history[:idx]])
                else:
                    hist = self.matrix_history[:n]
                avg_n = min(n, 50)
                self.matrix_calibration = hist[-avg_n:].mean(axis=0)
            else:
                self.matrix_calibration = self.matrix_latest.copy()
            avg_n = min(n, 50) if n > 0 else 1
            self.status_var.set(f"Calibrated (matrix, {avg_n} frames averaged)")
        else:
            n = min(self.history_idx, HISTORY_LEN)
            if n > 0:
                idx = self.history_idx % HISTORY_LEN
                if self.history_filled:
                    hist = np.concatenate([self.history[idx:],
                                           self.history[:idx]])
                else:
                    hist = self.history[:n]
                avg_n = min(n, 50)
                self.single_calibration = hist[-avg_n:].mean(axis=0)
            else:
                self.single_calibration = self.latest.copy()
            avg_n = min(n, 50) if n > 0 else 1
            self.status_var.set(f"Calibrated (single, {avg_n} frames averaged)")

    def _clear_calibration(self):
        mode = self.display_mode
        if mode == "debug":
            self.dbg_calibration = None
        elif mode == "debug12":
            self.dbg12_calibration = None
        elif mode == "matrix":
            self.matrix_calibration = None
        else:
            self.single_calibration = None
        self._clear_resistance_texts()
        self.status_var.set("Calibration cleared")

    def _hide_resistance_texts(self):
        for t in self._resistance_texts:
            t.set_visible(False)

    def _overlay_resistance(self, ax, cal_data, n_cols, n_rows,
                            flip_rows=False):
        """Update resistance text annotations on the heatmap, reusing objects.

        cal_data: calibrated raw values, shape (n_cols, n_rows) — column-major.
        flip_rows: if True, row 0 data maps to the bottom of the display.
        """
        try:
            vbias = float(self.vbias1_var.get())
        except ValueError:
            self._hide_resistance_texts()
            return
        if abs(vbias) < 1e-6:
            self._hide_resistance_texts()
            return

        needed = n_cols * n_rows
        # Scale font size to grid: fewer cells → larger text
        if needed <= 96:       # 8x12 debug
            fs = 7
        elif needed <= 144:    # 12x12 matrix
            fs = 6
        else:                  # 12x32 full
            fs = 5
        # Create text objects only once per mode (lazy init)
        if len(self._resistance_texts) != needed:
            self._clear_resistance_texts()
            stroke = pe.withStroke(linewidth=1, foreground='black')
            for row in range(n_rows):
                for col in range(n_cols):
                    t = ax.text(
                        col, row, '',
                        ha='center', va='center', fontsize=fs,
                        color='white', fontweight='bold',
                        path_effects=[stroke],
                    )
                    self._resistance_texts.append(t)

        nA_scale = self._get_nA_scale()
        idx = 0
        for row in range(n_rows):
            display_row = (n_rows - 1 - row) if flip_rows else row
            for col in range(n_cols):
                i_nA = cal_data[col, row] * nA_scale
                if abs(i_nA) > 0.01:
                    r = vbias / (i_nA * 1e-9)
                    text = self._format_resistance(r)
                else:
                    text = "OL"
                self._resistance_texts[idx].set_text(text)
                self._resistance_texts[idx].set_position((col, display_row))
                self._resistance_texts[idx].set_visible(True)
                idx += 1

    # ----------------------------------------------------------------
    # Logging
    # ----------------------------------------------------------------

    def _poll_settings(self) -> dict:
        """Send 'settings' command to ESP32 and parse key=value response."""
        lines = self._send_and_read_response("settings", wait=0.5)
        settings = {}
        in_block = False
        for line in lines:
            if line.strip() == "SETTINGS_BEGIN":
                in_block = True
                continue
            if line.strip() == "SETTINGS_END":
                break
            if in_block and "=" in line:
                key, _, val = line.strip().partition("=")
                settings[key] = val
        return settings

    def _toggle_log(self):
        if self._log_file:
            self._stop_log()
        else:
            self._start_log()

    def _start_log(self):
        """Open a CSV log file, write settings header, begin logging."""
        # Choose file
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        mode = self.display_mode
        default_name = f"ddc233_log_{mode}_{timestamp}.csv"
        path = filedialog.asksaveasfilename(
            defaultextension=".csv",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
            initialfile=default_name,
        )
        if not path:
            return

        # Poll settings from ESP32
        settings = self._poll_settings()

        # Open file and write header
        self._log_file = open(path, "w", newline="")
        self._log_writer = csv.writer(self._log_file)
        self._log_sample_idx = 0

        # Comment block with settings
        self._log_file.write(f"# DDC233 Log — {datetime.datetime.now().isoformat()}\n")
        self._log_file.write(f"# Mode: {mode}\n")
        self._log_file.write(f"# GUI units: {self.units_var.get()}\n")
        self._log_file.write(f"# GUI SMA: {self.sma_var.get()}\n")
        self._log_file.write(f"# GUI notch: {self.notch_var.get()}\n")
        for k, v in settings.items():
            self._log_file.write(f"# {k}: {v}\n")
        self._log_file.write("#\n")

        # CSV column header
        if mode in ("matrix", "full", "debug", "debug12"):
            if mode == "matrix":
                n_cols, n_rows = MATRIX_COLS, MATRIX_ROWS
            elif mode == "full":
                n_cols, n_rows = FULL_MATRIX_COLS, FULL_MATRIX_ROWS
            elif mode == "debug12":
                n_cols, n_rows = DBG12_MATRIX_COLS, DBG12_MATRIX_ROWS
            else:
                n_cols, n_rows = DBG_MATRIX_COLS, DBG_MATRIX_ROWS
            header = ["sample", "timestamp"]
            for c in range(n_cols):
                for r in range(n_rows):
                    header.append(f"c{c}_r{r}")
            self._log_writer.writerow(header)
        else:
            # single / manual / odd — log all 32 raw channels
            header = ["sample", "timestamp"]
            for i in range(NUM_CHANNELS):
                header.append(f"ch{i + 1}")
            self._log_writer.writerow(header)

        self.log_btn.config(text="Stop Log")
        self.log_status_var.set(os.path.basename(path))
        self.status_var.set(f"Logging to {os.path.basename(path)}")

    def _stop_log(self):
        """Close the log file."""
        if self._log_file:
            self._log_file.close()
            self._log_file = None
            self._log_writer = None
        self.log_btn.config(text="Start Log")
        n = self._log_sample_idx
        self.log_status_var.set("")
        self.status_var.set(f"Log stopped ({n} samples saved)")

    def _log_sample(self, item_type, data):
        """Write one sample row to the CSV log."""
        if not self._log_writer:
            return
        ts = time.time()
        self._log_sample_idx += 1
        if item_type in ("matrix", "full", "debug", "debug12"):
            # data is numpy (cols, rows) — flatten column-major
            row = [self._log_sample_idx, f"{ts:.6f}"]
            flat = data.flatten()  # column-major (C order of (cols,rows))
            row.extend(f"{v:.0f}" for v in flat)
        else:
            # data is list or 1D array of 32 values
            row = [self._log_sample_idx, f"{ts:.6f}"]
            row.extend(str(int(v)) for v in data)
        self._log_writer.writerow(row)
        # Flush periodically so data isn't lost on crash
        if self._log_sample_idx % 100 == 0:
            self._log_file.flush()

    # ----------------------------------------------------------------
    # Helpers
    # ----------------------------------------------------------------

    def _send_and_read_response(self, cmd: str, wait: float = 0.5,
                                resume: bool = True) -> list:
        """Send a command and collect response lines. Pauses streaming if active.

        Args:
            resume: If False, don't restart streaming afterwards (caller will
                    handle it).  Use this when sending multiple commands in
                    sequence to avoid a stop/start race between each one.
        """
        if not self.reader.ser or not self.reader.ser.is_open:
            return []

        was_streaming = self.reader.streaming
        if was_streaming:
            self.reader.stop_streaming()

        self.reader.ser.reset_input_buffer()
        self.reader.send_command(cmd)
        time.sleep(wait)
        lines = []
        # Read all available lines, retrying briefly for stragglers
        retries = 3
        while retries > 0:
            while self.reader.ser.in_waiting:
                line = self.reader.ser.readline().decode(errors="replace").strip()
                if line:
                    lines.append(line)
                retries = 3  # reset retries when we get data
            retries -= 1
            if retries > 0:
                time.sleep(0.1)

        if was_streaming and resume:
            mode = self._get_mode()
            stream_mode = "single" if mode in ("odd", "manual") else mode
            self.reader.start_streaming(mode=stream_mode,
                                        dbg_avg=self._get_dbg_avg())

        return lines

    def _get_mode(self):
        """Translate combobox label to internal mode string."""
        return self._mode_map.get(self.mode_var.get(), "single")

    def _get_dbg_avg(self):
        try:
            return max(1, int(self.dbg_avg_var.get()))
        except ValueError:
            return 1

    def _update_config_status(self):
        """Send readcfg and update the config status label."""
        lines = self._send_and_read_response("readcfg", wait=0.8)
        if lines:
            display = []
            for line in lines:
                if any(k in line for k in ["readback", "Config readback", "FSR=", "Rev ID"]):
                    display.append(line[:120])
            text = "\n".join(display) if display else "\n".join(
                l[:120] for l in lines[:3])
            self.config_status_var.set(text)
        else:
            self.config_status_var.set("No response from device")

    def _switch_plot_mode(self, mode):
        """Rebuild heatmap axes for single or matrix mode."""
        self.display_mode = mode

        # Clear resistance overlay and existing heatmap
        self._resistance_texts = []  # axes being cleared removes them
        self.cbar.remove()
        self.ax_heat.clear()

        if mode in ("matrix", "full", "debug", "debug12"):
            if mode == "matrix":
                n_cols, n_rows = MATRIX_COLS, MATRIX_ROWS
                col_labels = list(range(MATRIX_COLS))
                row_labels = MATRIX_ROW_PHYSICAL
                title = "12x12 Matrix Heatmap"
            elif mode == "full":
                n_cols, n_rows = FULL_MATRIX_COLS, FULL_MATRIX_ROWS
                col_labels = list(range(FULL_MATRIX_COLS))
                row_labels = list(range(1, FULL_MATRIX_ROWS + 1))
                title = "12x32 Full Matrix Heatmap"
            elif mode == "debug12":
                n_cols, n_rows = DBG12_MATRIX_COLS, DBG12_MATRIX_ROWS
                col_labels = DBG12_COL_LABELS
                row_labels = DBG12_ROW_PHYSICAL[::-1]
                title = "12x12 Debug Matrix Heatmap"
            else:
                n_cols, n_rows = DBG_MATRIX_COLS, DBG_MATRIX_ROWS
                col_labels = DBG_COL_PAIRS
                row_labels = DBG_ROW_PHYSICAL[::-1]
                title = "8x12 Debug Matrix Heatmap"

            init_data = np.zeros((n_rows, n_cols))
            asp = "auto" if mode in ("debug", "debug12") else "equal"
            self.im = self.ax_heat.imshow(
                init_data,
                aspect=asp,
                cmap="viridis",
                interpolation="nearest",
                origin="upper",
            )
            self.ax_heat.set_title(title)
            self.ax_heat.set_xlabel("Column")
            self.ax_heat.set_ylabel("Row")
            self.ax_heat.set_xticks(range(n_cols))
            self.ax_heat.set_xticklabels(col_labels, fontsize=7)
            self.ax_heat.set_yticks(range(n_rows))
            self.ax_heat.set_yticklabels(row_labels, fontsize=7)

            # Update trace index combo for correct number of cols/rows
            # "Row traces" → each trace is a row, index selects column
            # "Column traces" → each trace is a column, index selects row
            self.trace_idx_combo["values"] = [str(i) for i in range(
                n_cols if self.trace_mode_var.get() == "row" else n_rows)]

            # Show matrix trace selector, hide channel checkboxes
            self.ch_inner.pack_forget()
            self.ch_btn_frame.pack_forget()
            self.matrix_trace_frame.pack(fill=tk.X, padx=3, pady=2)
            frame_label = {"matrix": "Matrix Traces (line plot)",
                           "full": "Full Matrix Traces (line plot)",
                           "debug": "Debug Matrix Traces (line plot)",
                           "debug12": "12x12 Debug Traces (line plot)"}.get(mode, "Traces")
            self.ch_frame.config(text=frame_label)
        elif mode == "manual":
            self.im = self.ax_heat.imshow(
                np.zeros((1, NUM_MANUAL_CHANNELS)),
                aspect="auto",
                cmap="viridis",
                interpolation="nearest",
            )
            self.ax_heat.set_title("Channels 1-12")
            self.ax_heat.set_xlabel("Channel")
            self.ax_heat.set_yticks([])
            self.ax_heat.set_xticks(range(NUM_MANUAL_CHANNELS))
            self.ax_heat.set_xticklabels(MANUAL_PHYS_CHANNELS, fontsize=7)

            self.matrix_trace_frame.pack_forget()
            self.ch_inner.pack_forget()
            self.ch_btn_frame.pack_forget()
            self.ch_frame.config(text="Channels 1-12 (all shown)")
        elif mode == "odd":
            self.heatmap_data_odd = np.zeros((1, NUM_ODD_CHANNELS))
            self.im = self.ax_heat.imshow(
                self.heatmap_data_odd,
                aspect="auto",
                cmap="viridis",
                interpolation="nearest",
            )
            self.ax_heat.set_title("Odd Channels (1-23)")
            self.ax_heat.set_xlabel("Channel")
            self.ax_heat.set_yticks([])
            self.ax_heat.set_xticks(range(NUM_ODD_CHANNELS))
            self.ax_heat.set_xticklabels(ODD_PHYS_CHANNELS, fontsize=7)

            # Hide both selectors — all 12 odd channels shown automatically
            self.matrix_trace_frame.pack_forget()
            self.ch_inner.pack_forget()
            self.ch_btn_frame.pack_forget()
            self.ch_frame.config(text="Odd Channels 1-23 (all shown)")
        else:
            self.heatmap_data = np.zeros((1, NUM_CHANNELS))
            self.im = self.ax_heat.imshow(
                self.heatmap_data,
                aspect="auto",
                cmap="viridis",
                interpolation="nearest",
            )
            self.ax_heat.set_title("Channel Heatmap (latest reading)")
            self.ax_heat.set_xlabel("Channel")
            self.ax_heat.set_yticks([])
            self.ax_heat.set_xticks(range(0, NUM_CHANNELS, 2))
            self.ax_heat.set_xticklabels(
                list(range(1, NUM_CHANNELS + 1, 2)),
                fontsize=7)

            # Show channel checkboxes, hide matrix trace selector
            self.matrix_trace_frame.pack_forget()
            self.ch_inner.pack(fill=tk.X, padx=3, pady=2)
            self.ch_btn_frame.pack(fill=tk.X, padx=3, pady=2)
            self.ch_frame.config(text="Channels (line plot)")

        self.cbar = self.fig.colorbar(self.im, ax=self.ax_heat,
                                       orientation="vertical", pad=0.02)

        # Clear line plot
        self.ax_line.clear()
        self.ax_line.set_title("Time Series")
        self.ax_line.set_xlabel("Sample")
        self.ax_line.grid(True, alpha=0.3)
        self.lines = {}

        self.canvas.draw_idle()

    # ----------------------------------------------------------------
    # Actions
    # ----------------------------------------------------------------

    def _refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo["values"] = ports
        if ports:
            self.port_combo.current(0)

    def _toggle_connect(self):
        if self.reader.ser and self.reader.ser.is_open:
            self.reader.disconnect()
            self.connect_btn.config(text="Connect")
            self.stream_btn.config(state=tk.DISABLED)
            self.single_btn.config(state=tk.DISABLED)
            self.mscan_btn.config(state=tk.DISABLED)
            self.mdscan_btn.config(state=tk.DISABLED)
            self.cal_btn.config(state=tk.DISABLED)
            self.cal_clear_btn.config(state=tk.DISABLED)
            if self._log_file:
                self._stop_log()
            self.log_btn.config(state=tk.DISABLED)
            self.status_var.set("Disconnected")
            self.config_status_var.set("Not connected")
        else:
            port = self.port_var.get()
            if not port:
                self.status_var.set("No port selected")
                return
            err = self.reader.connect(port)
            if err:
                self.status_var.set(f"Error: {err}")
                return
            self.connect_btn.config(text="Disconnect")
            self.stream_btn.config(state=tk.NORMAL)
            self.single_btn.config(state=tk.NORMAL)
            self.mscan_btn.config(state=tk.NORMAL)
            self.mdscan_btn.config(state=tk.NORMAL)
            self.cal_btn.config(state=tk.NORMAL)
            self.cal_clear_btn.config(state=tk.NORMAL)
            self.log_btn.config(state=tk.NORMAL)
            self.status_var.set(f"Connected: {port}")
            self._update_config_status()

    def _on_mode_change(self):
        """Handle mode combobox change."""
        new_mode = self._get_mode()
        was_streaming = self.reader.streaming
        if was_streaming:
            self.reader.stop_streaming()

        # Drain stale frames from the old mode
        while not self.data_queue.empty():
            try:
                self.data_queue.get_nowait()
            except queue.Empty:
                break

        self._switch_plot_mode(new_mode)

        # Reset history for the new mode
        if new_mode == "matrix":
            self.matrix_history_idx = 0
            self.matrix_history_filled = False
        elif new_mode == "full":
            self.full_history_idx = 0
            self.full_history_filled = False
        elif new_mode == "debug":
            self.dbg_history_idx = 0
            self.dbg_history_filled = False
        elif new_mode == "debug12":
            self.dbg12_history_idx = 0
            self.dbg12_history_filled = False
        else:
            self.history_idx = 0
            self.history_filled = False

        if was_streaming:
            stream_mode = "single" if new_mode in ("odd", "manual") else new_mode
            self.reader.start_streaming(mode=stream_mode,
                                        dbg_avg=self._get_dbg_avg())
            self.stream_btn.config(text="Stop")
            mode_labels = {"matrix": "matrix", "full": "full matrix",
                           "debug": "debug matrix", "debug12": "12x12 debug",
                           "single": "binary",
                           "odd": "odd channels", "manual": "channels 1-12"}
            self.status_var.set(f"Streaming ({mode_labels.get(new_mode, 'binary')})...")

    def _toggle_stream(self):
        if self.reader.streaming:
            self.reader.stop_streaming()
            self.stream_btn.config(text="Start")
            self.status_var.set("Stopped")
            self._measured_fps = 0.0
        else:
            mode = self._get_mode()
            stream_mode = "single" if mode in ("odd", "manual") else mode
            self._measured_fps = 0.0
            self.reader.start_streaming(mode=stream_mode,
                                        dbg_avg=self._get_dbg_avg())
            self.stream_btn.config(text="Stop")
            mode_labels = {"matrix": "matrix", "full": "full matrix",
                           "debug": "debug matrix", "debug12": "12x12 debug",
                           "single": "binary",
                           "odd": "odd channels", "manual": "channels 1-12"}
            self.status_var.set(f"Streaming ({mode_labels.get(mode, 'binary')})...")

    def _single_read(self):
        if self.reader.ser and self.reader.ser.is_open:
            self.reader.ser.reset_input_buffer()
            self.reader.send_command("read")
            # Read response lines
            time.sleep(0.3)
            values = []
            while self.reader.ser.in_waiting:
                line = self.reader.ser.readline().decode(errors="replace").strip()
                if line.startswith("CH") and ":" in line:
                    try:
                        val = int(line.split(":")[1].strip())
                        values.append(val)
                    except ValueError:
                        pass
            if len(values) == NUM_CHANNELS:
                self.data_queue.put({"type": "single", "data": values})

    def _matrix_scan(self):
        """Send mscan command, parse text response, update heatmap."""
        lines = self._send_and_read_response("mscan", wait=2.0)
        if not lines:
            return

        matrix = np.zeros((MATRIX_COLS, MATRIX_ROWS), dtype=np.float64)
        for line in lines:
            if line.startswith("Row"):
                parts = line.split()
                if len(parts) < 2:
                    continue
                try:
                    row_idx = int(parts[0].replace("Row", ""))
                except ValueError:
                    continue
                vals = []
                for p in parts[1:]:
                    try:
                        vals.append(int(p))
                    except ValueError:
                        pass
                if len(vals) == MATRIX_COLS and 0 <= row_idx < MATRIX_ROWS:
                    for col in range(MATRIX_COLS):
                        matrix[col][row_idx] = vals[col]

        self.data_queue.put({"type": "matrix", "data": matrix})

    def _dbg_matrix_scan(self):
        """Send mdscan command, parse text response, update heatmap."""
        avg = self._get_dbg_avg()
        try:
            int_us = float(self.inttime_var.get())
        except ValueError:
            int_us = 1000.0
        # Wait = scan time + margin: 8 cols × (avg+2) integrations + serial output
        wait = max(2.0, 8 * (avg + 2) * int_us / 1e6 + 1.0)
        print(f"[dbg_scan] sending 'mdscan {avg}', wait={wait:.1f}s")
        lines = self._send_and_read_response(f"mdscan {avg}", wait=wait)
        if not lines:
            print("[dbg_scan] no response lines received")
            return

        print(f"[dbg_scan] got {len(lines)} response lines")
        for line in lines:
            print(f"[dbg_scan]   {line}")

        matrix = np.zeros((DBG_MATRIX_COLS, DBG_MATRIX_ROWS), dtype=np.float64)
        rows_parsed = 0
        for line in lines:
            if line.startswith("CH"):
                parts = line.split()
                if len(parts) < 2:
                    continue
                try:
                    ch_num = int(parts[0].replace("CH", ""))
                except ValueError:
                    continue
                # Firmware prints physical channel numbers
                if ch_num in DBG_ROW_PHYSICAL:
                    row_idx = DBG_ROW_PHYSICAL.index(ch_num)
                else:
                    continue
                vals = []
                for p in parts[1:]:
                    try:
                        vals.append(int(p))
                    except ValueError:
                        pass
                if len(vals) == DBG_MATRIX_COLS:
                    for col in range(DBG_MATRIX_COLS):
                        matrix[col][row_idx] = vals[col]
                    rows_parsed += 1

        print(f"[dbg_scan] parsed {rows_parsed}/{DBG_MATRIX_ROWS} rows, "
              f"range [{matrix.min():.0f}, {matrix.max():.0f}]")
        self.data_queue.put({"type": "debug", "data": matrix})

    def _on_trace_mode_change(self):
        self.matrix_trace_mode = self.trace_mode_var.get()
        # Disable index combo when in avg mode
        self.trace_idx_combo.config(
            state="disabled" if self.matrix_trace_mode == "avg" else "readonly")
        self.lines = {}
        self.ax_line.clear()
        self.ax_line.set_title("Time Series")
        self.ax_line.set_xlabel("Sample")
        self.ax_line.grid(True, alpha=0.3)

    def _on_trace_idx_change(self, event=None):
        try:
            self.matrix_trace_idx = int(self.trace_idx_var.get())
        except ValueError:
            self.matrix_trace_idx = 0
        self.lines = {}
        self.ax_line.clear()
        self.ax_line.set_title("Time Series")
        self.ax_line.set_xlabel("Sample")
        self.ax_line.grid(True, alpha=0.3)

    def _verify_config(self):
        self._update_config_status()

    def _set_range(self):
        idx = self.range_var.get().split(" ")[0]
        self._send_and_read_response(f"range {idx}", wait=0.8, resume=False)
        self._update_config_status()

    def _set_inttime(self):
        val = self.inttime_var.get().strip()
        if not val.isdigit() or int(val) == 0:
            self.status_var.set("Invalid integration time")
            return
        self._send_and_read_response(f"inttime {val}", resume=False)
        self._update_config_status()

    def _set_clk(self):
        val = self.clk_var.get().strip()
        if not val.isdigit() or int(val) == 0:
            self.status_var.set("Invalid clock frequency")
            return
        self._send_and_read_response(f"clk {val}", resume=False)
        self._update_config_status()

    def _set_test(self):
        state = "on" if self.test_var.get() else "off"
        self._send_and_read_response(f"test {state}", wait=0.8, resume=False)
        self._update_config_status()

    def _set_vbias1(self):
        try:
            v = float(self.vbias1_var.get())
        except ValueError:
            messagebox.showerror("Invalid input", "Enter a number for VBias1")
            return
        if v < -5.0 or v > 5.0:
            messagebox.showerror("Out of range", "VBias1 must be -5 to +5 V")
            return
        self._send_and_read_response(f"vbias1 {v:.4f}")

    def _set_vbias2(self):
        try:
            v = float(self.vbias2_var.get())
        except ValueError:
            messagebox.showerror("Invalid input", "Enter a number for VBias2")
            return
        if v < -5.0 or v > 5.0:
            messagebox.showerror("Out of range", "VBias2 must be -5 to +5 V")
            return
        self._send_and_read_response(f"vbias2 {v:.4f}")

    def _set_vsw1(self):
        try:
            v = float(self.vsw1_var.get())
        except ValueError:
            messagebox.showerror("Invalid input", "Enter a number for VSW1")
            return
        if v < 0.0 or v > 30.0:
            messagebox.showerror("Out of range", "VSW1 must be 0 to +30 V")
            return
        self._send_and_read_response(f"vsw1 {v:.4f}")

    def _set_vsw2(self):
        try:
            v = float(self.vsw2_var.get())
        except ValueError:
            messagebox.showerror("Invalid input", "Enter a number for VSW2")
            return
        if v < -30.0 or v > 0.0:
            messagebox.showerror("Out of range", "VSW2 must be -30 to 0 V")
            return
        self._send_and_read_response(f"vsw2 {v:.4f}")

    def _set_sw_pair(self, pair):
        state = self.sw_vars[pair].get()  # "pos", "neg", or "off"
        self._send_and_read_response(f"sw {pair} {state}", wait=0.2)

    def _sw_clear_all(self):
        for var in self.sw_vars:
            var.set("off")
        for var in self.sw2_vars:
            var.set("off")
        self._send_and_read_response("sw clear", wait=0.2)

    def _sw_all_pos(self):
        for var in self.sw_vars:
            var.set("pos")
        self._send_and_read_response("sw allpos", wait=0.2)

    def _sw_all_neg(self):
        for var in self.sw_vars:
            var.set("neg")
        self._send_and_read_response("sw allneg", wait=0.2)

    def _toggle_skip_vneg(self, pair):
        on = self.skip_vneg_vars[pair].get()
        if on:
            self._send_and_read_response(f"skipvneg {pair}", wait=0.2)
        else:
            self._send_and_read_response(f"skipvneg {pair} off", wait=0.2)

    def _set_sw2_pair(self, pair):
        state = self.sw2_vars[pair].get()
        self._send_and_read_response(f"sw2 {pair} {state}", wait=0.2)

    def _sw2_clear_all(self):
        for var in self.sw2_vars:
            var.set("off")
        self._send_and_read_response("sw2 clear", wait=0.2)

    def _toggle_channel(self, ch, var):
        if var.get():
            self.selected_channels.add(ch)
        else:
            self.selected_channels.discard(ch)

    def _select_all_ch(self):
        for i, v in enumerate(self.ch_vars):
            v.set(True)
            self.selected_channels.add(i)

    def _select_no_ch(self):
        for i, v in enumerate(self.ch_vars):
            v.set(False)
        self.selected_channels.clear()

    # ----------------------------------------------------------------
    # Data polling and plot update
    # ----------------------------------------------------------------

    def _poll_data(self):
        """Called periodically from the main thread to drain the queue and update plots."""
        batch = 0
        while not self.data_queue.empty() and batch < 5000:
            try:
                item = self.data_queue.get_nowait()
                if isinstance(item, dict):
                    item_type = item.get("type", "single")
                    data = item["data"]
                else:
                    # Legacy: plain list
                    item_type = "single"
                    data = item

                if item_type == "matrix":
                    self.matrix_latest = data  # (12, 12) numpy array
                    self.matrix_history[self.matrix_history_idx % HISTORY_LEN] = data
                    self.matrix_history_idx += 1
                    if self.matrix_history_idx >= HISTORY_LEN:
                        self.matrix_history_filled = True
                    self._log_sample(item_type, data)
                elif item_type == "full":
                    self.full_latest = data  # (12, 32) numpy array
                    self.full_history[self.full_history_idx % HISTORY_LEN] = data
                    self.full_history_idx += 1
                    if self.full_history_idx >= HISTORY_LEN:
                        self.full_history_filled = True
                    self._log_sample(item_type, data)
                elif item_type == "debug":
                    self.dbg_latest = data  # (8, 12) numpy array
                    self.dbg_history[self.dbg_history_idx % HISTORY_LEN] = data
                    self.dbg_history_idx += 1
                    if self.dbg_history_idx >= HISTORY_LEN:
                        self.dbg_history_filled = True
                    self._log_sample(item_type, data)
                elif item_type == "debug12":
                    self.dbg12_latest = data  # (12, 12) numpy array
                    self.dbg12_history[self.dbg12_history_idx % HISTORY_LEN] = data
                    self.dbg12_history_idx += 1
                    if self.dbg12_history_idx >= HISTORY_LEN:
                        self.dbg12_history_filled = True
                    self._log_sample(item_type, data)
                else:
                    arr = np.array(data, dtype=np.float64)
                    self.latest = arr
                    self.history[self.history_idx % HISTORY_LEN] = arr
                    self.history_idx += 1
                    if self.history_idx >= HISTORY_LEN:
                        self.history_filled = True
                    self._log_sample(item_type, data)
                batch += 1
            except queue.Empty:
                break

        if batch > 0:
            self._sample_count += batch
            now = time.time()
            elapsed = now - self._fps_time
            if elapsed >= 1.0:
                fps = self._sample_count / elapsed
                self._measured_fps = fps
                self.status_var.set(f"{'Streaming' if self.reader.streaming else 'Connected'}  |  {fps:.1f} samples/s")
                self._sample_count = 0
                self._fps_time = now

            # Update notch info label
            self._update_notch_info()

            # Throttle plot updates — redraw at most every 200 ms (5 fps)
            if now - self._last_plot_time >= 0.2:
                if self.display_mode == "matrix":
                    self._update_plots_matrix()
                elif self.display_mode == "full":
                    self._update_plots_full_matrix()
                elif self.display_mode == "debug":
                    self._update_plots_dbg_matrix()
                elif self.display_mode == "debug12":
                    self._update_plots_dbg12_matrix()
                elif self.display_mode == "manual":
                    self._update_plots_manual()
                elif self.display_mode == "odd":
                    self._update_plots_odd()
                else:
                    self._update_plots()
                self._last_plot_time = now

        # Periodic garbage collection to reclaim matplotlib internals
        now = time.time()
        if now - self._last_gc_time >= 2.0:
            gc.collect()
            self._last_gc_time = now

        self.root.after(100, self._poll_data)  # drain queue every 100 ms

    def _get_sma_window(self):
        """Return the current SMA window size (1 = no filtering)."""
        try:
            return max(1, int(self.sma_var.get()))
        except ValueError:
            return 1

    @staticmethod
    def _apply_sma(data, window):
        """Apply simple moving average along axis 0. Returns filtered copy."""
        if window <= 1 or data.shape[0] < window:
            return data
        kernel = np.ones(window) / window
        if data.ndim == 1:
            return np.convolve(data, kernel, mode='valid')
        out = np.empty((data.shape[0] - window + 1, data.shape[1]),
                       dtype=data.dtype)
        for i in range(data.shape[1]):
            out[:, i] = np.convolve(data[:, i], kernel, mode='valid')
        return out

    def _avg_from_history(self, history, hist_idx, hist_filled):
        """Return the average of the last SMA-window samples from a circular buffer."""
        window = self._get_sma_window()
        n = min(window, hist_idx, HISTORY_LEN)
        if n <= 1:
            return None  # caller should fall back to latest
        idx = hist_idx % HISTORY_LEN
        if n <= idx:
            return history[idx - n:idx].mean(axis=0)
        elif hist_filled:
            n_end = n - idx
            total = history[HISTORY_LEN - n_end:].sum(axis=0)
            if idx > 0:
                total += history[:idx].sum(axis=0)
            return total / n
        else:
            return history[:hist_idx].mean(axis=0)

    def _get_notch_freq(self):
        """Return notch frequency in Hz, or 0 if disabled."""
        val = self.notch_var.get()
        if val == "50 Hz":
            return 50.0
        elif val == "60 Hz":
            return 60.0
        return 0.0

    def _update_notch_info(self):
        """Update the notch info label with LS filter parameters."""
        f0 = self._get_notch_freq()
        if f0 <= 0:
            self.notch_info_var.set("")
            return
        fs = self._get_sample_rate()
        self.notch_info_var.set(
            f"LS K=4 win=30  (fs={fs:.0f})"
        )

    def _get_sample_rate(self):
        """Return the effective sample (frame) rate in Hz.

        Uses the measured FPS when available.  Falls back to an estimate
        based on the integration time and the number of columns scanned
        per frame in matrix modes.
        """
        if self._measured_fps > 0:
            return self._measured_fps

        try:
            int_us = float(self.inttime_var.get())
        except ValueError:
            int_us = 1000.0
        if int_us <= 0:
            int_us = 1000.0

        # For matrix modes the frame period is n_cols × integration time
        mode = self.display_mode
        if mode == "matrix":
            return 1e6 / (MATRIX_COLS * int_us)
        elif mode == "full":
            return 1e6 / (FULL_MATRIX_COLS * int_us)
        elif mode == "debug":
            return 1e6 / (DBG_MATRIX_COLS * int_us)
        elif mode == "debug12":
            return 1e6 / (DBG12_MATRIX_COLS * int_us)
        return 1e6 / int_us

    @staticmethod
    def _apply_notch(data, fs, f0, _Q=3.0):
        """Remove mains interference using windowed LS sinusoidal subtraction.

        Fits and subtracts f0 plus K-1 harmonics in a sliding window
        using synthesized uniform timestamps.  Immune to aliasing.

        Vectorized: precomputes the projection matrix for the common
        (full-size) window and applies it to all interior samples in
        one batched matrix multiply.  Edge samples use per-sample fits.
        """
        K_max = 4   # try up to 4 harmonics: f0, 2*f0, 3*f0, 4*f0
        win = 30    # sliding window size (samples)

        if f0 <= 0 or fs <= 0 or data.shape[0] < 3:
            return data

        nyq = fs / 2

        # ── Select harmonics that don't alias near DC or Nyquist ──
        valid_harmonics = []
        used_aliases = []
        for k in range(1, K_max + 1):
            f_alias = (f0 * k) % fs
            if f_alias > nyq:
                f_alias = fs - f_alias
            # Skip near-DC or near-Nyquist (same guard as old biquad)
            if f_alias < 2.0 or f_alias > nyq - 1.0:
                continue
            # Skip if too close to an already-included alias
            if any(abs(f_alias - fa) < 1.0 for fa in used_aliases):
                continue
            valid_harmonics.append(k)
            used_aliases.append(f_alias)

        K = len(valid_harmonics)
        if K == 0:
            return data

        N = data.shape[0]
        half = win // 2
        ensure_2d = data.ndim == 1
        if ensure_2d:
            data = data[:, np.newaxis]

        out = np.empty_like(data, dtype=float)

        # ── Build design matrix for a full-size window (win samples) ──
        t_full = np.arange(win) / fs
        X_full = np.empty((win, 2 * K))
        for idx, k in enumerate(valid_harmonics):
            phase = 2.0 * np.pi * k * f0 * t_full
            X_full[:, 2 * idx] = np.cos(phase)
            X_full[:, 2 * idx + 1] = np.sin(phase)

        # Projection matrix P = X (X^T X)^{-1} X^T   (win × win)
        # The residual for a window is  (I - P) @ data_window
        # We only need row `half` of (I - P) for each center sample.
        XtX_inv = np.linalg.pinv(X_full.T @ X_full)
        # hat_row = X_full[half] @ XtX_inv @ X_full.T   → (win,)
        hat_row = X_full[half] @ XtX_inv @ X_full.T
        # residual weights: e_half - hat_row  (identity row minus hat row)
        res_weights = -hat_row.copy()
        res_weights[half] += 1.0  # (win,) weights to get residual at center

        # ── Interior samples: batched dot product ──
        # For sample i in [half, N-half), the window is data[i-half : i-half+win]
        # and the filtered value is res_weights @ data[i-half : i-half+win]
        n_interior = N - win + 1
        if n_interior > 0:
            # Build a (n_interior, win, C) view using stride tricks
            C = data.shape[1]
            from numpy.lib.stride_tricks import as_strided
            item = data.strides[0]
            col_stride = data.strides[1]
            windows = as_strided(
                data, shape=(n_interior, win, C),
                strides=(item, item, col_stride),
            )
            # (n_interior, win, C) contracted with (win,) → (n_interior, C)
            out[half:half + n_interior] = np.einsum(
                'j,ijc->ic', res_weights, windows
            )

        # ── Edge samples: small per-sample LS fits ──
        ts = np.arange(N) / fs
        for i in list(range(min(half, N))) + \
                 list(range(max(half + n_interior, 0), N)):
            lo = max(0, i - half)
            hi = min(N, i + half + 1)
            t_w = ts[lo:hi] - ts[lo]
            w = hi - lo
            if w < 2 * K + 1:
                out[i] = data[i]
                continue
            X_e = np.empty((w, 2 * K))
            for idx_k, k in enumerate(valid_harmonics):
                phase = 2.0 * np.pi * k * f0 * t_w
                X_e[:, 2 * idx_k] = np.cos(phase)
                X_e[:, 2 * idx_k + 1] = np.sin(phase)
            beta, _, _, _ = np.linalg.lstsq(X_e, data[lo:hi], rcond=None)
            out[i] = data[i] - X_e[i - lo] @ beta

        if ensure_2d:
            out = out[:, 0]
        return out

    def _update_plots(self):
        scale, y_label = self._get_scale_and_label()

        # --- Apply calibration ---
        cal_latest = self.latest
        if self.single_calibration is not None:
            cal_latest = cal_latest - self.single_calibration

        # --- Heatmap ---
        # For nA/pC modes, subtract the zero-code offset before scaling
        offset = ZERO_CODE if self.units_var.get() in ("nA", "pC") else 0
        self.heatmap_data[0, :] = (cal_latest - offset) * scale
        self.im.set_data(self.heatmap_data)
        scaled_latest = (cal_latest - offset) * scale
        self._apply_clim(scaled_latest)

        # --- Line plot ---
        self.ax_line.set_ylabel(y_label)

        length = min(self.history_idx, HISTORY_LEN)
        if length == 0:
            self.canvas.draw_idle()
            return

        # Reorder history into pre-allocated buffer (avoids heap allocation)
        idx = self.history_idx % HISTORY_LEN
        if self.history_filled:
            n_tail = HISTORY_LEN - idx
            self._work_buf[:n_tail] = self.history[idx:]
            self._work_buf[n_tail:HISTORY_LEN] = self.history[:idx]
        else:
            self._work_buf[:length] = self.history[:length]
        data = self._work_buf[:length]
        # Apply calibration then offset binary correction then scale (in-place)
        if self.single_calibration is not None:
            np.subtract(data, self.single_calibration, out=data)
        if offset:
            np.subtract(data, offset, out=data)
        if scale != 1.0:
            np.multiply(data, scale, out=data)

        # Apply notch filter (50/60 Hz mains rejection)
        notch_f = self._get_notch_freq()
        if notch_f > 0:
            data = self._apply_notch(data, self._get_sample_rate(), notch_f)

        # Apply SMA filter
        sma_w = self._get_sma_window()
        data = self._apply_sma(data, sma_w)
        length = data.shape[0]

        # Decimate for display — no point plotting more than ~1000 points
        MAX_PLOT_PTS = 1000
        if length > MAX_PLOT_PTS:
            step = length // MAX_PLOT_PTS
            data_dec = data[::step]
            x = np.arange(0, length, step)
        else:
            data_dec = data
            x = np.arange(length)

        # Remove lines for deselected channels
        for ch in list(self.lines.keys()):
            if ch not in self.selected_channels:
                self.lines[ch].remove()
                del self.lines[ch]

        # Update or create lines for selected channels
        cmap = matplotlib.colormaps["tab20"]
        legend_dirty = False
        for ch in self.selected_channels:
            y = data_dec[:, ch]
            if ch in self.lines:
                self.lines[ch].set_data(x, y)
            else:
                color = cmap(ch / NUM_CHANNELS)
                (line,) = self.ax_line.plot(x, y, color=color, linewidth=1, label=f"Ch {ch + 1}")
                self.lines[ch] = line
                legend_dirty = True

        self.ax_line.set_xlim(0, length - 1)
        visible_chs = list(self.selected_channels)
        if visible_chs:
            visible = data[:, visible_chs]
            ymin, ymax = visible.min(), visible.max()
            span = abs(ymax - ymin)
            if span == 0:
                margin = max(abs(ymin) * 0.01, 1e-6) if scale != 1.0 else 1
            else:
                margin = span * 0.05
            self.ax_line.set_ylim(ymin - margin, ymax + margin)

        # Only rebuild legend when channels change
        if legend_dirty or not hasattr(self, '_legend_channels') or self._legend_channels != self.selected_channels:
            old_legend = self.ax_line.get_legend()
            if old_legend:
                old_legend.remove()
            if self.selected_channels:
                self.ax_line.legend(loc="upper left", fontsize=7, ncol=4, framealpha=0.7)
            self._legend_channels = set(self.selected_channels)

        self.canvas.draw_idle()

    def _update_plots_manual(self):
        """Update plots showing physical channels 1-12."""
        scale, y_label = self._get_scale_and_label()
        offset = ZERO_CODE if self.units_var.get() in ("nA", "pC") else 0

        cal_latest = self.latest
        if self.single_calibration is not None:
            cal_latest = cal_latest - self.single_calibration
        manual_latest = cal_latest[MANUAL_INDICES]
        scaled = (manual_latest - offset) * scale

        # --- Heatmap: 1×12 ---
        self.im.set_data(scaled.reshape(1, -1))
        self._apply_clim(scaled)

        # --- Line plot: all 12 channels ---
        self.ax_line.set_ylabel(y_label)

        length = min(self.history_idx, HISTORY_LEN)
        if length == 0:
            self.canvas.draw_idle()
            return

        idx = self.history_idx % HISTORY_LEN
        if self.history_filled:
            n_tail = HISTORY_LEN - idx
            self._work_buf[:n_tail] = self.history[idx:]
            self._work_buf[n_tail:HISTORY_LEN] = self.history[:idx]
        else:
            self._work_buf[:length] = self.history[:length]
        data = self._work_buf[:length]

        if self.single_calibration is not None:
            np.subtract(data, self.single_calibration, out=data)
        data_man = data[:, MANUAL_INDICES]
        if offset:
            np.subtract(data_man, offset, out=data_man)
        if scale != 1.0:
            np.multiply(data_man, scale, out=data_man)

        notch_f = self._get_notch_freq()
        if notch_f > 0:
            data_man = self._apply_notch(data_man, self._get_sample_rate(), notch_f)

        sma_w = self._get_sma_window()
        data_man = self._apply_sma(data_man, sma_w)
        length = data_man.shape[0]

        MAX_PLOT_PTS = 1000
        if length > MAX_PLOT_PTS:
            step = length // MAX_PLOT_PTS
            data_dec = data_man[::step]
            x = np.arange(0, length, step)
        else:
            data_dec = data_man
            x = np.arange(length)

        for key in list(self.lines.keys()):
            if key >= NUM_MANUAL_CHANNELS:
                self.lines[key].remove()
                del self.lines[key]

        cmap = matplotlib.colormaps["tab20"]
        legend_dirty = False
        for i in range(NUM_MANUAL_CHANNELS):
            y = data_dec[:, i]
            if i in self.lines:
                self.lines[i].set_data(x, y)
            else:
                color = cmap(i / NUM_MANUAL_CHANNELS)
                label = str(MANUAL_PHYS_CHANNELS[i])
                (line,) = self.ax_line.plot(x, y, color=color, linewidth=1,
                                            label=label)
                self.lines[i] = line
                legend_dirty = True

        self.ax_line.set_xlim(0, max(length - 1, 1))
        ymin, ymax = data_man.min(), data_man.max()
        span = abs(ymax - ymin)
        if span == 0:
            margin = max(abs(ymin) * 0.01, 1e-6) if scale != 1.0 else 1
        else:
            margin = span * 0.05
        self.ax_line.set_ylim(ymin - margin, ymax + margin)

        self.ax_line.set_title("Time Series — Channels 1-12")
        if legend_dirty:
            old_legend = self.ax_line.get_legend()
            if old_legend:
                old_legend.remove()
            self.ax_line.legend(loc="upper left", fontsize=7, ncol=4, framealpha=0.7)

        self.canvas.draw_idle()

    def _update_plots_odd(self):
        """Update plots showing only the 12 odd physical channels (1-23)."""
        scale, y_label = self._get_scale_and_label()
        offset = ZERO_CODE if self.units_var.get() in ("nA", "pC") else 0

        # Apply calibration then extract odd channels
        cal_latest = self.latest
        if self.single_calibration is not None:
            cal_latest = cal_latest - self.single_calibration
        odd_latest = cal_latest[ODD_INDICES]
        scaled = (odd_latest - offset) * scale

        # --- Heatmap: 1×12 ---
        self.im.set_data(scaled.reshape(1, -1))
        self._apply_clim(scaled)

        # --- Line plot: all 12 odd channels ---
        self.ax_line.set_ylabel(y_label)

        length = min(self.history_idx, HISTORY_LEN)
        if length == 0:
            self.canvas.draw_idle()
            return

        # Reorder history into pre-allocated buffer (avoids heap allocation)
        idx = self.history_idx % HISTORY_LEN
        if self.history_filled:
            n_tail = HISTORY_LEN - idx
            self._work_buf[:n_tail] = self.history[idx:]
            self._work_buf[n_tail:HISTORY_LEN] = self.history[:idx]
        else:
            self._work_buf[:length] = self.history[:length]
        data = self._work_buf[:length]

        # Apply calibration in-place, then extract odd channels
        if self.single_calibration is not None:
            np.subtract(data, self.single_calibration, out=data)
        data_odd = data[:, ODD_INDICES]  # fancy indexing (small copy: 12 cols)
        if offset:
            np.subtract(data_odd, offset, out=data_odd)
        if scale != 1.0:
            np.multiply(data_odd, scale, out=data_odd)

        # Apply notch filter
        notch_f = self._get_notch_freq()
        if notch_f > 0:
            data_odd = self._apply_notch(data_odd, self._get_sample_rate(), notch_f)

        # Apply SMA filter
        sma_w = self._get_sma_window()
        data_odd = self._apply_sma(data_odd, sma_w)
        length = data_odd.shape[0]

        MAX_PLOT_PTS = 1000
        if length > MAX_PLOT_PTS:
            step = length // MAX_PLOT_PTS
            data_dec = data_odd[::step]
            x = np.arange(0, length, step)
        else:
            data_dec = data_odd
            x = np.arange(length)

        # Remove stale lines
        for key in list(self.lines.keys()):
            if key >= NUM_ODD_CHANNELS:
                self.lines[key].remove()
                del self.lines[key]

        cmap = matplotlib.colormaps["tab20"]
        legend_dirty = False
        for i in range(NUM_ODD_CHANNELS):
            y = data_dec[:, i]
            if i in self.lines:
                self.lines[i].set_data(x, y)
            else:
                color = cmap(i / NUM_ODD_CHANNELS)
                label = str(ODD_PHYS_CHANNELS[i])
                (line,) = self.ax_line.plot(x, y, color=color, linewidth=1,
                                            label=label)
                self.lines[i] = line
                legend_dirty = True

        self.ax_line.set_xlim(0, max(length - 1, 1))
        ymin, ymax = data_odd.min(), data_odd.max()
        span = abs(ymax - ymin)
        if span == 0:
            margin = max(abs(ymin) * 0.01, 1e-6) if scale != 1.0 else 1
        else:
            margin = span * 0.05
        self.ax_line.set_ylim(ymin - margin, ymax + margin)

        if legend_dirty or not hasattr(self, '_legend_odd'):
            old_legend = self.ax_line.get_legend()
            if old_legend:
                old_legend.remove()
            self.ax_line.legend(loc="upper left", fontsize=7, ncol=4, framealpha=0.7)
            self._legend_odd = True

        self.canvas.draw_idle()

    def _update_plots_matrix(self):
        scale, y_label = self._get_scale_and_label()
        offset = ZERO_CODE if self.units_var.get() in ("nA", "pC") else 0

        # --- Heatmap: 12x12 (SMA-averaged) ---
        avg = self._avg_from_history(self.matrix_history,
                                     self.matrix_history_idx,
                                     self.matrix_history_filled)
        cal_data = avg if avg is not None else self.matrix_latest
        if self.matrix_calibration is not None:
            cal_data = cal_data - self.matrix_calibration

        # cal_data is (cols, rows) — transpose to (rows, cols) for display
        display = (cal_data.T - offset) * scale
        self.im.set_data(display)
        self._apply_clim(display)

        # --- Resistance overlay ---
        if self.matrix_calibration is not None:
            avg_n = max(1, min(int(self._get_sample_rate()),
                               self.matrix_history_idx, HISTORY_LEN))
            idx_h = self.matrix_history_idx % HISTORY_LEN
            if avg_n <= idx_h:
                avg_data = self.matrix_history[idx_h - avg_n:idx_h].mean(axis=0)
            else:
                if self.matrix_history_filled:
                    n_end = avg_n - idx_h
                    avg_data = self.matrix_history[HISTORY_LEN - n_end:].sum(axis=0)
                    if idx_h > 0:
                        avg_data += self.matrix_history[:idx_h].sum(axis=0)
                    avg_data /= avg_n
                else:
                    avg_data = self.matrix_history[:self.matrix_history_idx].mean(axis=0)
            avg_cal = avg_data - self.matrix_calibration
            self._overlay_resistance(self.ax_heat, avg_cal,
                                     MATRIX_COLS, MATRIX_ROWS)
        else:
            self._hide_resistance_texts()

        # --- Line plot: traces for selected row, column, or average ---
        self.ax_line.set_ylabel(y_label)

        length = min(self.matrix_history_idx, HISTORY_LEN)
        if length == 0:
            self.canvas.draw_idle()
            return

        idx = self.matrix_history_idx % HISTORY_LEN
        trace_idx = self.matrix_trace_idx

        # Helper: reorder circular buffer into contiguous array
        def reorder(src):
            """src shape: (HISTORY_LEN, ...) → contiguous (length, ...)"""
            if self.matrix_history_filled:
                n_tail = HISTORY_LEN - idx
                return np.concatenate([src[idx:], src[:idx]], axis=0)[:length]
            return src[:length]

        if self.matrix_trace_mode == "avg":
            # Mean of all pixels per frame → single time series
            history_ordered = reorder(self.matrix_history)  # (length, cols, rows)
            if self.matrix_calibration is not None:
                history_ordered = history_ordered - self.matrix_calibration
            mean_trace = ((history_ordered - offset) * scale).mean(axis=(1, 2))
            traces = mean_trace.reshape(-1, 1)
            trace_labels = ["Mean (all pixels)"]
            title_detail = "Mean of all pixels"
        elif self.matrix_trace_mode == "row":
            # "Row traces": each trace is a row; index selects column
            if trace_idx >= MATRIX_COLS:
                trace_idx = 0
            n_cols = MATRIX_ROWS
            if self.matrix_history_filled:
                n_tail = HISTORY_LEN - idx
                self._work_buf[:n_tail, :n_cols] = self.matrix_history[idx:, trace_idx, :]
                self._work_buf[n_tail:HISTORY_LEN, :n_cols] = self.matrix_history[:idx, trace_idx, :]
            else:
                self._work_buf[:length, :n_cols] = self.matrix_history[:length, trace_idx, :]
            traces = self._work_buf[:length, :n_cols].copy()
            if self.matrix_calibration is not None:
                np.subtract(traces, self.matrix_calibration[trace_idx, :], out=traces)
            if offset:
                np.subtract(traces, offset, out=traces)
            if scale != 1.0:
                np.multiply(traces, scale, out=traces)
            trace_labels = [f"Ch {MATRIX_ROW_PHYSICAL[r]}" for r in range(MATRIX_ROWS)]
            title_detail = f"Col {trace_idx}"
        else:  # column
            # "Column traces": each trace is a column; index selects row
            if trace_idx >= MATRIX_ROWS:
                trace_idx = 0
            n_cols = MATRIX_COLS
            if self.matrix_history_filled:
                n_tail = HISTORY_LEN - idx
                self._work_buf[:n_tail, :n_cols] = self.matrix_history[idx:, :, trace_idx]
                self._work_buf[n_tail:HISTORY_LEN, :n_cols] = self.matrix_history[:idx, :, trace_idx]
            else:
                self._work_buf[:length, :n_cols] = self.matrix_history[:length, :, trace_idx]
            traces = self._work_buf[:length, :n_cols].copy()
            if self.matrix_calibration is not None:
                np.subtract(traces, self.matrix_calibration[:, trace_idx], out=traces)
            if offset:
                np.subtract(traces, offset, out=traces)
            if scale != 1.0:
                np.multiply(traces, scale, out=traces)
            trace_labels = [f"Col {c}" for c in range(MATRIX_COLS)]
            phys_ch = MATRIX_ROW_PHYSICAL[trace_idx]
            title_detail = f"Row {phys_ch}"

        # Append mean overlay if checkbox is on and not already in avg mode
        show_mean = self.show_mean_var.get() and self.matrix_trace_mode != "avg"
        if show_mean:
            history_ordered = reorder(self.matrix_history)
            if self.matrix_calibration is not None:
                history_ordered = history_ordered - self.matrix_calibration
            mean_trace = ((history_ordered - offset) * scale).mean(axis=(1, 2))
            traces = np.column_stack([traces, mean_trace])
            trace_labels.append("Mean")

        # Apply notch filter
        notch_f = self._get_notch_freq()
        if notch_f > 0:
            traces = self._apply_notch(traces, self._get_sample_rate(), notch_f)

        # Apply SMA filter
        sma_w = self._get_sma_window()
        traces = self._apply_sma(traces, sma_w)
        length = traces.shape[0]
        n_traces = traces.shape[1]

        # Decimate
        MAX_PLOT_PTS = 1000
        if length > MAX_PLOT_PTS:
            step = length // MAX_PLOT_PTS
            traces_dec = traces[::step]
            x = np.arange(0, length, step)
        else:
            traces_dec = traces
            x = np.arange(length)

        # Remove stale lines
        for key in list(self.lines.keys()):
            if key >= n_traces:
                self.lines[key].remove()
                del self.lines[key]

        cmap = matplotlib.colormaps["tab20"]
        legend_dirty = False
        for i in range(n_traces):
            y = traces_dec[:, i]
            if i in self.lines:
                self.lines[i].set_data(x, y)
            else:
                # Mean line: thick black dashed
                if trace_labels[i] in ("Mean", "Mean (all pixels)"):
                    color, lw, ls = "black", 2, "--"
                else:
                    color, lw, ls = cmap(i / max(n_traces - (1 if show_mean else 0), 1)), 1, "-"
                (line,) = self.ax_line.plot(x, y, color=color, linewidth=lw,
                                            linestyle=ls, label=trace_labels[i])
                self.lines[i] = line
                legend_dirty = True

        self.ax_line.set_xlim(0, max(length - 1, 1))
        ymin, ymax = traces.min(), traces.max()
        span = abs(ymax - ymin)
        if span == 0:
            margin = max(abs(ymin) * 0.01, 1e-6) if scale != 1.0 else 1
        else:
            margin = span * 0.05
        self.ax_line.set_ylim(ymin - margin, ymax + margin)

        self.ax_line.set_title(f"Time Series — {title_detail}")

        if legend_dirty or not hasattr(self, '_legend_matrix_key') or self._legend_matrix_key != (self.matrix_trace_mode, trace_idx, show_mean):
            old_legend = self.ax_line.get_legend()
            if old_legend:
                old_legend.remove()
            self.ax_line.legend(loc="upper left", fontsize=7, ncol=4, framealpha=0.7)
            self._legend_matrix_key = (self.matrix_trace_mode, trace_idx, show_mean)

        self.canvas.draw_idle()

    def _update_plots_full_matrix(self):
        scale, y_label = self._get_scale_and_label()
        offset = ZERO_CODE if self.units_var.get() in ("nA", "pC") else 0

        # --- Heatmap: 12x32 (SMA-averaged) ---
        avg = self._avg_from_history(self.full_history,
                                     self.full_history_idx,
                                     self.full_history_filled)
        cal_data = avg if avg is not None else self.full_latest
        if self.full_calibration is not None:
            cal_data = cal_data - self.full_calibration

        display = (cal_data.T - offset) * scale
        self.im.set_data(display)
        self._apply_clim(display)

        if self.full_calibration is not None:
            self._overlay_resistance(self.ax_heat, cal_data - self.full_calibration
                                     if avg is None else cal_data,
                                     FULL_MATRIX_COLS, FULL_MATRIX_ROWS)
        else:
            self._hide_resistance_texts()

        # --- Line plot: traces for selected row or column ---
        self.ax_line.set_ylabel(y_label)

        length = min(self.full_history_idx, HISTORY_LEN)
        if length == 0:
            self.canvas.draw_idle()
            return

        idx = self.full_history_idx % HISTORY_LEN
        trace_idx = self.matrix_trace_idx

        if self.matrix_trace_mode == "row":
            # "Row traces": each trace is a row; index selects column
            if trace_idx >= FULL_MATRIX_COLS:
                trace_idx = 0
            n_cols = FULL_MATRIX_ROWS
            if self.full_history_filled:
                n_tail = HISTORY_LEN - idx
                self._work_buf[:n_tail, :n_cols] = self.full_history[idx:, trace_idx, :]
                self._work_buf[n_tail:HISTORY_LEN, :n_cols] = self.full_history[:idx, trace_idx, :]
            else:
                self._work_buf[:length, :n_cols] = self.full_history[:length, trace_idx, :]
            traces = self._work_buf[:length, :n_cols]
            if self.full_calibration is not None:
                np.subtract(traces, self.full_calibration[trace_idx, :], out=traces)
            if offset:
                np.subtract(traces, offset, out=traces)
            if scale != 1.0:
                np.multiply(traces, scale, out=traces)
            trace_labels = [f"Ch {r + 1}" for r in range(FULL_MATRIX_ROWS)]
            title_detail = f"Col {trace_idx}"
        else:
            # "Column traces": each trace is a column; index selects row
            if trace_idx >= FULL_MATRIX_ROWS:
                trace_idx = 0
            n_cols = FULL_MATRIX_COLS
            if self.full_history_filled:
                n_tail = HISTORY_LEN - idx
                self._work_buf[:n_tail, :n_cols] = self.full_history[idx:, :, trace_idx]
                self._work_buf[n_tail:HISTORY_LEN, :n_cols] = self.full_history[:idx, :, trace_idx]
            else:
                self._work_buf[:length, :n_cols] = self.full_history[:length, :, trace_idx]
            traces = self._work_buf[:length, :n_cols]
            if self.full_calibration is not None:
                np.subtract(traces, self.full_calibration[:, trace_idx], out=traces)
            if offset:
                np.subtract(traces, offset, out=traces)
            if scale != 1.0:
                np.multiply(traces, scale, out=traces)
            trace_labels = [f"Col {c}" for c in range(FULL_MATRIX_COLS)]
            phys_ch = trace_idx + 1
            title_detail = f"Row {phys_ch}"

        notch_f = self._get_notch_freq()
        if notch_f > 0:
            traces = self._apply_notch(traces, self._get_sample_rate(), notch_f)

        sma_w = self._get_sma_window()
        traces = self._apply_sma(traces, sma_w)
        length = traces.shape[0]

        MAX_PLOT_PTS = 1000
        n_traces = traces.shape[1]
        if length > MAX_PLOT_PTS:
            step = length // MAX_PLOT_PTS
            traces_dec = traces[::step]
            x = np.arange(0, length, step)
        else:
            traces_dec = traces
            x = np.arange(length)

        for key in list(self.lines.keys()):
            if key >= n_traces:
                self.lines[key].remove()
                del self.lines[key]

        cmap = matplotlib.colormaps["tab20"]
        legend_dirty = False
        for i in range(n_traces):
            y = traces_dec[:, i]
            if i in self.lines:
                self.lines[i].set_data(x, y)
            else:
                color = cmap(i / max(n_traces, 1))
                (line,) = self.ax_line.plot(x, y, color=color, linewidth=1,
                                            label=trace_labels[i])
                self.lines[i] = line
                legend_dirty = True

        self.ax_line.set_xlim(0, max(length - 1, 1))
        ymin, ymax = traces.min(), traces.max()
        span = abs(ymax - ymin)
        if span == 0:
            margin = max(abs(ymin) * 0.01, 1e-6) if scale != 1.0 else 1
        else:
            margin = span * 0.05
        self.ax_line.set_ylim(ymin - margin, ymax + margin)

        self.ax_line.set_title(f"Time Series — {title_detail}")
        if legend_dirty:
            old_legend = self.ax_line.get_legend()
            if old_legend:
                old_legend.remove()
            self.ax_line.legend(loc="upper left", fontsize=6, ncol=6, framealpha=0.7)

        self.canvas.draw_idle()

    def _update_plots_dbg_matrix(self):
        scale, y_label = self._get_scale_and_label()
        offset = ZERO_CODE if self.units_var.get() in ("nA", "pC") else 0

        # --- Heatmap: 8x12 (SMA-averaged) ---
        avg = self._avg_from_history(self.dbg_history,
                                     self.dbg_history_idx,
                                     self.dbg_history_filled)
        cal_data = avg if avg is not None else self.dbg_latest
        if self.dbg_calibration is not None:
            cal_data = cal_data - self.dbg_calibration

        # cal_data is (cols, rows) — transpose to (rows, cols), flip rows
        # so row 12 is at the top
        display = (cal_data.T - offset) * scale
        self.im.set_data(display[::-1])
        self._apply_clim(display)

        # --- Resistance overlay ---
        if self.dbg_calibration is not None:
            avg_n = max(1, min(int(self._get_sample_rate()),
                               self.dbg_history_idx, HISTORY_LEN))
            idx_h = self.dbg_history_idx % HISTORY_LEN
            if avg_n <= idx_h:
                avg_data = self.dbg_history[idx_h - avg_n:idx_h].mean(axis=0)
            else:
                if self.dbg_history_filled:
                    n_end = avg_n - idx_h
                    avg_data = self.dbg_history[HISTORY_LEN - n_end:].sum(axis=0)
                    if idx_h > 0:
                        avg_data += self.dbg_history[:idx_h].sum(axis=0)
                    avg_data /= avg_n
                else:
                    avg_data = self.dbg_history[:self.dbg_history_idx].mean(axis=0)
            avg_cal = avg_data - self.dbg_calibration
            self._overlay_resistance(self.ax_heat, avg_cal,
                                     DBG_MATRIX_COLS, DBG_MATRIX_ROWS,
                                     flip_rows=True)
        else:
            self._hide_resistance_texts()

        # --- Line plot ---
        self.ax_line.set_ylabel(y_label)

        length = min(self.dbg_history_idx, HISTORY_LEN)
        if length == 0:
            self.canvas.draw_idle()
            return

        idx = self.dbg_history_idx % HISTORY_LEN
        trace_idx = self.matrix_trace_idx

        # Reorder traces into pre-allocated buffer (avoids np.concatenate)
        if self.matrix_trace_mode == "row":
            # "Row traces": each trace is a row; index selects column
            if trace_idx >= DBG_MATRIX_COLS:
                trace_idx = 0
            n_cols = DBG_MATRIX_ROWS
            if self.dbg_history_filled:
                n_tail = HISTORY_LEN - idx
                self._work_buf[:n_tail, :n_cols] = self.dbg_history[idx:, trace_idx, :]
                self._work_buf[n_tail:HISTORY_LEN, :n_cols] = self.dbg_history[:idx, trace_idx, :]
            else:
                self._work_buf[:length, :n_cols] = self.dbg_history[:length, trace_idx, :]
            traces = self._work_buf[:length, :n_cols]
            if self.dbg_calibration is not None:
                np.subtract(traces, self.dbg_calibration[trace_idx, :], out=traces)
            if offset:
                np.subtract(traces, offset, out=traces)
            if scale != 1.0:
                np.multiply(traces, scale, out=traces)
            trace_labels = [f"Ch {DBG_ROW_PHYSICAL[r]}" for r in range(DBG_MATRIX_ROWS)]
            title_detail = f"P{DBG_COL_PAIRS[trace_idx]}"
        else:
            # "Column traces": each trace is a column; index selects row
            if trace_idx >= DBG_MATRIX_ROWS:
                trace_idx = 0
            n_cols = DBG_MATRIX_COLS
            if self.dbg_history_filled:
                n_tail = HISTORY_LEN - idx
                self._work_buf[:n_tail, :n_cols] = self.dbg_history[idx:, :, trace_idx]
                self._work_buf[n_tail:HISTORY_LEN, :n_cols] = self.dbg_history[:idx, :, trace_idx]
            else:
                self._work_buf[:length, :n_cols] = self.dbg_history[:length, :, trace_idx]
            traces = self._work_buf[:length, :n_cols]
            if self.dbg_calibration is not None:
                np.subtract(traces, self.dbg_calibration[:, trace_idx], out=traces)
            if offset:
                np.subtract(traces, offset, out=traces)
            if scale != 1.0:
                np.multiply(traces, scale, out=traces)
            trace_labels = [f"P{DBG_COL_PAIRS[i]}" for i in range(DBG_MATRIX_COLS)]
            title_detail = f"Ch {DBG_ROW_PHYSICAL[trace_idx]}"

        # Apply notch filter
        notch_f = self._get_notch_freq()
        if notch_f > 0:
            traces = self._apply_notch(traces, self._get_sample_rate(), notch_f)

        # Apply SMA filter
        sma_w = self._get_sma_window()
        traces = self._apply_sma(traces, sma_w)
        length = traces.shape[0]
        n_traces = traces.shape[1]

        MAX_PLOT_PTS = 1000
        if length > MAX_PLOT_PTS:
            step = length // MAX_PLOT_PTS
            traces_dec = traces[::step]
            x = np.arange(0, length, step)
        else:
            traces_dec = traces
            x = np.arange(length)

        for key in list(self.lines.keys()):
            if key >= n_traces:
                self.lines[key].remove()
                del self.lines[key]

        cmap = matplotlib.colormaps["tab20"]
        legend_dirty = False
        for i in range(n_traces):
            y = traces_dec[:, i]
            if i in self.lines:
                self.lines[i].set_data(x, y)
            else:
                color = cmap(i / n_traces)
                (line,) = self.ax_line.plot(x, y, color=color, linewidth=1,
                                            label=trace_labels[i])
                self.lines[i] = line
                legend_dirty = True

        self.ax_line.set_xlim(0, max(length - 1, 1))
        ymin, ymax = traces.min(), traces.max()
        span = abs(ymax - ymin)
        if span == 0:
            margin = max(abs(ymin) * 0.01, 1e-6) if scale != 1.0 else 1
        else:
            margin = span * 0.05
        self.ax_line.set_ylim(ymin - margin, ymax + margin)

        self.ax_line.set_title(f"Time Series — {title_detail}")

        if legend_dirty or not hasattr(self, '_legend_dbg_key') or self._legend_dbg_key != (self.matrix_trace_mode, trace_idx):
            old_legend = self.ax_line.get_legend()
            if old_legend:
                old_legend.remove()
            self.ax_line.legend(loc="upper left", fontsize=7, ncol=4, framealpha=0.7)
            self._legend_dbg_key = (self.matrix_trace_mode, trace_idx)

        self.canvas.draw_idle()

    def _update_plots_dbg12_matrix(self):
        scale, y_label = self._get_scale_and_label()
        offset = ZERO_CODE if self.units_var.get() in ("nA", "pC") else 0

        # --- Heatmap: 12x12 (SMA-averaged) ---
        avg = self._avg_from_history(self.dbg12_history,
                                     self.dbg12_history_idx,
                                     self.dbg12_history_filled)
        cal_data = avg if avg is not None else self.dbg12_latest
        if self.dbg12_calibration is not None:
            cal_data = cal_data - self.dbg12_calibration

        display = (cal_data.T - offset) * scale
        self.im.set_data(display[::-1])
        self._apply_clim(display)

        # --- Resistance overlay ---
        if self.dbg12_calibration is not None:
            avg_n = max(1, min(int(self._get_sample_rate()),
                               self.dbg12_history_idx, HISTORY_LEN))
            idx_h = self.dbg12_history_idx % HISTORY_LEN
            if avg_n <= idx_h:
                avg_data = self.dbg12_history[idx_h - avg_n:idx_h].mean(axis=0)
            else:
                if self.dbg12_history_filled:
                    n_end = avg_n - idx_h
                    avg_data = self.dbg12_history[HISTORY_LEN - n_end:].sum(axis=0)
                    if idx_h > 0:
                        avg_data += self.dbg12_history[:idx_h].sum(axis=0)
                    avg_data /= avg_n
                else:
                    avg_data = self.dbg12_history[:self.dbg12_history_idx].mean(axis=0)
            avg_cal = avg_data - self.dbg12_calibration
            self._overlay_resistance(self.ax_heat, avg_cal,
                                     DBG12_MATRIX_COLS, DBG12_MATRIX_ROWS,
                                     flip_rows=True)
        else:
            self._hide_resistance_texts()

        # --- Line plot ---
        self.ax_line.set_ylabel(y_label)

        length = min(self.dbg12_history_idx, HISTORY_LEN)
        if length == 0:
            self.canvas.draw_idle()
            return

        idx = self.dbg12_history_idx % HISTORY_LEN
        trace_idx = self.matrix_trace_idx

        if self.matrix_trace_mode == "row":
            # "Row traces": each trace is a row; index selects column
            if trace_idx >= DBG12_MATRIX_COLS:
                trace_idx = 0
            n_cols = DBG12_MATRIX_ROWS
            if self.dbg12_history_filled:
                n_tail = HISTORY_LEN - idx
                self._work_buf[:n_tail, :n_cols] = self.dbg12_history[idx:, trace_idx, :]
                self._work_buf[n_tail:HISTORY_LEN, :n_cols] = self.dbg12_history[:idx, trace_idx, :]
            else:
                self._work_buf[:length, :n_cols] = self.dbg12_history[:length, trace_idx, :]
            traces = self._work_buf[:length, :n_cols]
            if self.dbg12_calibration is not None:
                np.subtract(traces, self.dbg12_calibration[trace_idx, :], out=traces)
            if offset:
                np.subtract(traces, offset, out=traces)
            if scale != 1.0:
                np.multiply(traces, scale, out=traces)
            trace_labels = [f"Ch {DBG12_ROW_PHYSICAL[r]}" for r in range(DBG12_MATRIX_ROWS)]
            title_detail = DBG12_COL_LABELS[trace_idx]
        else:
            # "Column traces": each trace is a column; index selects row
            if trace_idx >= DBG12_MATRIX_ROWS:
                trace_idx = 0
            n_cols = DBG12_MATRIX_COLS
            if self.dbg12_history_filled:
                n_tail = HISTORY_LEN - idx
                self._work_buf[:n_tail, :n_cols] = self.dbg12_history[idx:, :, trace_idx]
                self._work_buf[n_tail:HISTORY_LEN, :n_cols] = self.dbg12_history[:idx, :, trace_idx]
            else:
                self._work_buf[:length, :n_cols] = self.dbg12_history[:length, :, trace_idx]
            traces = self._work_buf[:length, :n_cols]
            if self.dbg12_calibration is not None:
                np.subtract(traces, self.dbg12_calibration[:, trace_idx], out=traces)
            if offset:
                np.subtract(traces, offset, out=traces)
            if scale != 1.0:
                np.multiply(traces, scale, out=traces)
            trace_labels = DBG12_COL_LABELS
            title_detail = f"Ch {DBG12_ROW_PHYSICAL[trace_idx]}"

        notch_f = self._get_notch_freq()
        if notch_f > 0:
            traces = self._apply_notch(traces, self._get_sample_rate(), notch_f)

        sma_w = self._get_sma_window()
        traces = self._apply_sma(traces, sma_w)
        length = traces.shape[0]
        n_traces = traces.shape[1]

        MAX_PLOT_PTS = 1000
        if length > MAX_PLOT_PTS:
            step = length // MAX_PLOT_PTS
            traces_dec = traces[::step]
            x = np.arange(0, length, step)
        else:
            traces_dec = traces
            x = np.arange(length)

        for key in list(self.lines.keys()):
            if key >= n_traces:
                self.lines[key].remove()
                del self.lines[key]

        cmap = matplotlib.colormaps["tab20"]
        legend_dirty = False
        for i in range(n_traces):
            y = traces_dec[:, i]
            if i in self.lines:
                self.lines[i].set_data(x, y)
            else:
                color = cmap(i / n_traces)
                (line,) = self.ax_line.plot(x, y, color=color, linewidth=1,
                                            label=trace_labels[i])
                self.lines[i] = line
                legend_dirty = True

        self.ax_line.set_xlim(0, max(length - 1, 1))
        ymin, ymax = traces.min(), traces.max()
        span = abs(ymax - ymin)
        if span == 0:
            margin = max(abs(ymin) * 0.01, 1e-6) if scale != 1.0 else 1
        else:
            margin = span * 0.05
        self.ax_line.set_ylim(ymin - margin, ymax + margin)

        self.ax_line.set_title(f"Time Series — {title_detail}")

        if legend_dirty or not hasattr(self, '_legend_dbg12_key') or self._legend_dbg12_key != (self.matrix_trace_mode, trace_idx):
            old_legend = self.ax_line.get_legend()
            if old_legend:
                old_legend.remove()
            self.ax_line.legend(loc="upper left", fontsize=7, ncol=4, framealpha=0.7)
            self._legend_dbg12_key = (self.matrix_trace_mode, trace_idx)

        self.canvas.draw_idle()

    def _on_close(self):
        if self._log_file:
            self._stop_log()
        self.reader.disconnect()
        self.root.destroy()


def main():
    root = tk.Tk()
    # Size to 90% of screen
    sw = root.winfo_screenwidth()
    sh = root.winfo_screenheight()
    w = int(sw * 0.9)
    h = int(sh * 0.9)
    x = (sw - w) // 2
    y = (sh - h) // 2
    root.geometry(f"{w}x{h}+{x}+{y}")
    DDC233Gui(root)
    root.mainloop()


if __name__ == "__main__":
    main()
