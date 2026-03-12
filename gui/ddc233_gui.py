#!/usr/bin/env python3
"""
DDC232/233 Real-Time Readout GUI

Python GUI for streaming and plotting 32-channel charge data from the
DDC232/233 via serial. Displays a 1×32 heatmap (future: 32×32 matrix)
and scrolling time-series line plots with channel selection.

Requirements:
    pip install pyserial matplotlib numpy

Usage:
    python ddc233_gui.py
"""

import tkinter as tk
from tkinter import ttk, messagebox
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

NUM_CHANNELS = 32
HISTORY_LEN = 500  # rolling buffer size for line plots
DEFAULT_DELAY_MS = 0
# DDC232 offset binary coding:
#   Code 0x00000 (0)       = negative full scale
#   Code 0x10000 (65536)   = zero input
#   Code 0xFFFFF (1048575) = positive full scale
ZERO_CODE = 65536          # 0x10000 — zero input code
MAX_CODE  = 1048575        # 0xFFFFF — positive full scale code
FS_RANGE_CODES = MAX_CODE - ZERO_CODE  # 982911 codes from zero to full scale

# Full-scale charge in pC for each range setting (index 0-7)
RANGE_PC = [12.5, 50.0, 100.0, 150.0, 200.0, 250.0, 300.0, 350.0]


class SerialReader:
    """Manages serial connection and background reading thread."""

    def __init__(self, data_queue: queue.Queue):
        self.port = None
        self.ser = None
        self.data_queue = data_queue
        self._stop_event = threading.Event()
        self._thread = None
        self.streaming = False

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

    def start_streaming(self):
        if not self.ser or not self.ser.is_open:
            return
        self._stop_event.clear()
        self.streaming = True
        self._thread = threading.Thread(
            target=self._read_loop_binary, daemon=True
        )
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
        # Flush any remaining binary data and the text summary
        if self.ser and self.ser.is_open:
            time.sleep(0.2)
            self.ser.reset_input_buffer()

    SYNC = b'\xAA\x55\xAA\x55'
    FRAME_SIZE = 136  # 4 sync + 4 timestamp + 32*4 data

    def _read_loop_binary(self):
        """Send stream command and parse binary frames."""
        self.ser.reset_input_buffer()
        self.send_command("stream")

        # Wait for and discard the text header line before binary data starts
        time.sleep(0.05)
        self.ser.readline()  # read and discard "Binary stream: ..." line

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
                        buf = buf[-3:]
                        break
                    if idx > 0:
                        # Discard bytes before sync
                        buf = buf[idx:]
                    if len(buf) < self.FRAME_SIZE:
                        break

                    # Parse frame: skip sync(4) + timestamp(4), then 32 × int32
                    frame = bytes(buf[:self.FRAME_SIZE])
                    buf = buf[self.FRAME_SIZE:]

                    values = list(struct.unpack_from('<32i', frame, 8))

                    # Sanity check: offset binary 20-bit values (0 to 1048575)
                    if all(0 <= v <= 1048575 for v in values):
                        self.data_queue.put(values)
                    # else: corrupted frame, silently drop

            except Exception:
                if self._stop_event.is_set():
                    break


class DDC233Gui:
    """Main GUI application."""

    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("DDC232/233 Readout")
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        self.data_queue = queue.Queue(maxsize=10000)
        self.reader = SerialReader(self.data_queue)

        # Data buffers
        self.history = np.zeros((HISTORY_LEN, NUM_CHANNELS), dtype=np.float64)
        self.history_idx = 0
        self.history_filled = False
        self.latest = np.zeros(NUM_CHANNELS, dtype=np.float64)

        # Selected channels for line plot (start with first 4)
        self.selected_channels = set(range(4))

        # FPS tracking
        self._sample_count = 0
        self._fps_time = time.time()
        self._last_plot_time = 0

        # Build bottom panels FIRST (pack side=BOTTOM) so they're always visible,
        # then the plot fills the remaining space.
        self._build_config_status()
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
        toolbar = ttk.Frame(self.root)
        toolbar.pack(side=tk.TOP, fill=tk.X, padx=5, pady=3)

        # Serial port
        ttk.Label(toolbar, text="Port:").pack(side=tk.LEFT)
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(
            toolbar, textvariable=self.port_var, width=20, state="readonly"
        )
        self.port_combo.pack(side=tk.LEFT, padx=(2, 5))
        self._refresh_ports()

        ttk.Button(toolbar, text="Refresh", command=self._refresh_ports).pack(
            side=tk.LEFT, padx=2
        )

        self.connect_btn = ttk.Button(
            toolbar, text="Connect", command=self._toggle_connect
        )
        self.connect_btn.pack(side=tk.LEFT, padx=5)

        ttk.Separator(toolbar, orient=tk.VERTICAL).pack(
            side=tk.LEFT, fill=tk.Y, padx=5
        )

        # Streaming controls
        ttk.Label(toolbar, text="Delay (ms):").pack(side=tk.LEFT)
        self.delay_var = tk.StringVar(value=str(DEFAULT_DELAY_MS))
        ttk.Entry(toolbar, textvariable=self.delay_var, width=6).pack(
            side=tk.LEFT, padx=(2, 5)
        )

        self.stream_btn = ttk.Button(
            toolbar, text="Start", command=self._toggle_stream, state=tk.DISABLED
        )
        self.stream_btn.pack(side=tk.LEFT, padx=5)

        ttk.Button(
            toolbar, text="Single Read", command=self._single_read, state=tk.DISABLED
        ).pack(side=tk.LEFT, padx=2)
        # keep reference to enable/disable
        self.single_btn = toolbar.winfo_children()[-1]

        ttk.Separator(toolbar, orient=tk.VERTICAL).pack(
            side=tk.LEFT, fill=tk.Y, padx=5
        )

        # Units selector
        ttk.Label(toolbar, text="Units:").pack(side=tk.LEFT)
        self.units_var = tk.StringVar(value="Bits")
        units_combo = ttk.Combobox(
            toolbar, textvariable=self.units_var,
            values=["Bits", "nA", "pC"],
            width=5, state="readonly",
        )
        units_combo.pack(side=tk.LEFT, padx=(2, 5))

        # Status
        self.status_var = tk.StringVar(value="Disconnected")
        ttk.Label(toolbar, textvariable=self.status_var, foreground="gray").pack(
            side=tk.RIGHT, padx=5
        )

    def _build_plots(self):
        self.fig = Figure(tight_layout=True)

        # Heatmap: 1×32 (will become 32×32)
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
        self.fig.colorbar(self.im, ax=self.ax_heat, orientation="vertical", pad=0.02)

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
        frame = ttk.LabelFrame(self.root, text="Channels (line plot)")
        frame.pack(side=tk.BOTTOM, fill=tk.X, padx=5, pady=3)

        self.ch_vars = []
        inner = ttk.Frame(frame)
        inner.pack(fill=tk.X, padx=3, pady=2)

        for ch in range(NUM_CHANNELS):
            var = tk.BooleanVar(value=(ch in self.selected_channels))
            cb = ttk.Checkbutton(
                inner,
                text=str(ch),
                variable=var,
                command=lambda c=ch, v=var: self._toggle_channel(c, v),
            )
            cb.grid(row=ch // 16, column=ch % 16, padx=2, pady=1)
            self.ch_vars.append(var)

        btn_frame = ttk.Frame(frame)
        btn_frame.pack(fill=tk.X, padx=3, pady=2)
        ttk.Button(btn_frame, text="All", command=self._select_all_ch).pack(
            side=tk.LEFT, padx=2
        )
        ttk.Button(btn_frame, text="None", command=self._select_no_ch).pack(
            side=tk.LEFT, padx=2
        )

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
        )
        self.config_status_label.pack(fill=tk.X, padx=5, pady=3)

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

    # ----------------------------------------------------------------
    # Helpers
    # ----------------------------------------------------------------

    def _send_and_read_response(self, cmd: str, wait: float = 0.5) -> list:
        """Send a command and collect response lines. Pauses streaming if active."""
        if not self.reader.ser or not self.reader.ser.is_open:
            return []

        was_streaming = self.reader.streaming
        if was_streaming:
            self.reader.stop_streaming()

        self.reader.ser.reset_input_buffer()
        self.reader.send_command(cmd)
        time.sleep(wait)
        lines = []
        while self.reader.ser.in_waiting:
            line = self.reader.ser.readline().decode(errors="replace").strip()
            if line:
                lines.append(line)

        if was_streaming:
            self.reader.start_streaming()

        return lines

    def _update_config_status(self):
        """Send readcfg and update the config status label."""
        lines = self._send_and_read_response("readcfg", wait=0.8)
        if lines:
            display = []
            for line in lines:
                if any(k in line for k in ["readback", "Config readback", "FSR=", "Rev ID"]):
                    display.append(line)
            self.config_status_var.set("\n".join(display) if display else "\n".join(lines))
        else:
            self.config_status_var.set("No response from device")

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
            self.status_var.set(f"Connected: {port}")
            self._update_config_status()

    def _toggle_stream(self):
        if self.reader.streaming:
            self.reader.stop_streaming()
            self.stream_btn.config(text="Start")
            self.status_var.set("Stopped")
        else:
            self.reader.start_streaming()
            self.stream_btn.config(text="Stop")
            self.status_var.set("Streaming (binary)...")

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
                self.data_queue.put(values)

    def _verify_config(self):
        self._update_config_status()

    def _set_range(self):
        idx = self.range_var.get().split(" ")[0]
        self._send_and_read_response(f"range {idx}", wait=0.8)
        self._update_config_status()

    def _set_inttime(self):
        self._send_and_read_response(f"inttime {self.inttime_var.get()}")
        self._update_config_status()

    def _set_clk(self):
        self._send_and_read_response(f"clk {self.clk_var.get()}")
        self._update_config_status()

    def _set_test(self):
        state = "on" if self.test_var.get() else "off"
        self._send_and_read_response(f"test {state}", wait=0.8)
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
                values = self.data_queue.get_nowait()
                arr = np.array(values, dtype=np.float64)
                self.latest = arr
                self.history[self.history_idx % HISTORY_LEN] = arr
                self.history_idx += 1
                if self.history_idx >= HISTORY_LEN:
                    self.history_filled = True
                batch += 1
            except queue.Empty:
                break

        if batch > 0:
            self._sample_count += batch
            now = time.time()
            elapsed = now - self._fps_time
            if elapsed >= 1.0:
                fps = self._sample_count / elapsed
                self.status_var.set(f"{'Streaming' if self.reader.streaming else 'Connected'}  |  {fps:.1f} samples/s")
                self._sample_count = 0
                self._fps_time = now

            # Throttle plot updates — redraw at most every 200 ms (5 fps)
            if now - self._last_plot_time >= 0.2:
                self._update_plots()
                self._last_plot_time = now

        self.root.after(100, self._poll_data)  # drain queue every 100 ms

    def _update_plots(self):
        scale, y_label = self._get_scale_and_label()

        # --- Heatmap ---
        # For nA/pC modes, subtract the zero-code offset before scaling
        offset = ZERO_CODE if self.units_var.get() in ("nA", "pC") else 0
        self.heatmap_data[0, :] = (self.latest - offset) * scale
        self.im.set_data(self.heatmap_data)
        scaled_latest = (self.latest - offset) * scale
        vmin = scaled_latest.min()
        vmax = scaled_latest.max()
        if vmin == vmax:
            fallback = max(abs(vmin) * 0.01, 1e-6) if scale != 1.0 else 1
            vmax = vmin + fallback
        self.im.set_clim(vmin, vmax)

        # --- Line plot ---
        self.ax_line.set_ylabel(y_label)

        length = min(self.history_idx, HISTORY_LEN)
        if length == 0:
            self.canvas.draw_idle()
            return

        # Build ordered data without np.roll — use two slices
        idx = self.history_idx % HISTORY_LEN
        if self.history_filled:
            data = np.concatenate([self.history[idx:], self.history[:idx]], axis=0)
        else:
            data = self.history[:length].copy()
        # Apply offset binary correction then scale
        data = (data - offset) * scale

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
                (line,) = self.ax_line.plot(x, y, color=color, linewidth=1, label=f"CH{ch}")
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

    def _on_close(self):
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
