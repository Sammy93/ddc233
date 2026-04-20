#!/usr/bin/env python3
"""Analyze notch filter effectiveness on DDC233 matrix logs.

Loads the notch-off recording and applies the old (buggy) vs. fixed
notch filter, showing time-domain traces and FFT side by side.
"""

import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from pathlib import Path

# ── Load data ────────────────────────────────────────────────────────
here = Path(__file__).parent
f_off = here / "ddc233_log_matrix_20260420_141445_notchoff.csv"

df = pd.read_csv(f_off, comment="#")
data_cols = [c for c in df.columns if c.startswith("c")]
mean_all = df[data_cols].mean(axis=1).values
ts = df["timestamp"].values

fs = (len(ts) - 1) / (ts[-1] - ts[0])
gui_assumed_fs = 1e6 / 1000.0

# Alias calculation
f_alias = 50.0 % fs
if f_alias > fs / 2:
    f_alias = fs - f_alias

w0_gui = 2 * np.pi * 50.0 / gui_assumed_fs
f_actual_notch = w0_gui / (2 * np.pi) * fs

print(f"Samples: {len(mean_all)},  fs = {fs:.1f} Hz,  Nyquist = {fs/2:.1f} Hz")
print(f"50 Hz aliases to {f_alias:.1f} Hz")
print(f"Old GUI notch targeted {f_actual_notch:.1f} Hz (bug)")


# ── Notch filters ───────────────────────────────────────────────────
def _notch_core(data, w0, Q=30.0):
    cos_w0 = np.cos(w0)
    alpha = np.sin(w0) / (2.0 * Q)
    inv_a0 = 1.0 / (1.0 + alpha)
    b0 = inv_a0
    b1 = -2.0 * cos_w0 * inv_a0
    b2 = inv_a0
    a1 = b1
    a2 = (1.0 - alpha) * inv_a0
    n = len(data)
    y = np.empty(n)
    y[0] = b0 * data[0]
    y[1] = b0 * data[1] + b1 * data[0] - a1 * y[0]
    for i in range(2, n):
        y[i] = (b0 * data[i] + b1 * data[i-1] + b2 * data[i-2]
                - a1 * y[i-1] - a2 * y[i-2])
    return y


def apply_notch_old(data, f0=50.0, Q=30.0):
    """OLD bug: uses gui_assumed_fs=1000, no alias handling."""
    fs_fake = gui_assumed_fs
    if f0 <= 0 or fs_fake <= 2.0 * f0:
        return data.copy()
    return _notch_core(data, 2.0 * np.pi * f0 / fs_fake, Q)


def apply_notch_fixed(data, f_center, Q=3.0):
    """FIXED: notch at measured peak frequency."""
    if f_center <= 0 or f_center >= fs / 2:
        return data.copy()
    return _notch_core(data, 2.0 * np.pi * f_center / fs, Q)


# Find actual FFT peak near the expected alias
raw = mean_all
sig_ac = raw - np.mean(raw)
_Y = np.abs(np.fft.rfft(sig_ac * np.hanning(len(sig_ac))))
_freqs = np.fft.rfftfreq(len(sig_ac), d=1.0 / fs)
_mask = (_freqs > 12) & (_freqs < 22)
f_peak = _freqs[_mask][np.argmax(_Y[_mask])]
print(f"Actual FFT peak: {f_peak:.2f} Hz  (alias formula gives {f_alias:.2f} Hz)")

old = apply_notch_old(raw)
fixed = apply_notch_fixed(raw, f_peak)

# SMA-10 after notch
def sma(data, w):
    kernel = np.ones(w) / w
    return np.convolve(data, kernel, mode='same')

fixed_sma = sma(fixed, 10)

# Convert to nA for display (range 7 = 350 pC, int 1000 us)
# Match GUI formula: pc_per_code * 1000 / int_us
range_pC = 350.0
int_us = 1000.0
pc_per_code = range_pC / 1048575
scale = pc_per_code * 1000.0 / int_us  # nA per code
raw_nA = raw * scale
old_nA = old * scale
fixed_nA = fixed * scale
fixed_sma_nA = fixed_sma * scale


# ── FFT helper ───────────────────────────────────────────────────────
def compute_fft(signal):
    sig = signal - np.mean(signal)
    N = len(sig)
    win = np.hanning(N)
    Y = np.fft.rfft(sig * win)
    freqs = np.fft.rfftfreq(N, d=1.0 / fs)
    mag = np.abs(Y) * 2.0 / (N * np.mean(win))
    mag_dB = 20 * np.log10(mag + 1e-12)
    return freqs, mag_dB


# ── Plot ─────────────────────────────────────────────────────────────
fig, axes = plt.subplots(3, 2, figsize=(14, 13))
fig.suptitle(
    f"Notch Filter Analysis  —  notch-off log, fs={fs:.1f} Hz, "
    f"peak={f_peak:.2f} Hz",
    fontsize=13, fontweight="bold")

t = np.arange(len(raw)) / fs  # time in seconds

# ── Row 0: time domain ──
ax = axes[0, 0]
ax.plot(t, raw_nA, linewidth=0.4, alpha=0.6, label="Raw (no filter)")
ax.plot(t, old_nA, linewidth=0.7, label="Old notch 50 Hz")
ax.set_title("OLD filter  (fs=1000, targets 3.3 Hz)")
ax.set_xlabel("Time (s)")
ax.set_ylabel("Current (nA)")
ax.legend(fontsize=8, loc="upper right")
ax.grid(True, alpha=0.3)

ax = axes[0, 1]
ax.plot(t, raw_nA, linewidth=0.4, alpha=0.6, label="Raw (no filter)")
ax.plot(t, fixed_nA, linewidth=0.7, color="tab:green",
        label=f"Fixed notch at {f_peak:.1f} Hz, Q=3")
ax.set_title(f"FIXED notch  (peak={f_peak:.1f} Hz, Q=3)")
ax.set_xlabel("Time (s)")
ax.set_ylabel("Current (nA)")
ax.legend(fontsize=8, loc="upper right")
ax.grid(True, alpha=0.3)

# match y-axes for row 0
ymin = min(axes[0, 0].get_ylim()[0], axes[0, 1].get_ylim()[0])
ymax = max(axes[0, 0].get_ylim()[1], axes[0, 1].get_ylim()[1])
axes[0, 0].set_ylim(ymin, ymax)
axes[0, 1].set_ylim(ymin, ymax)

# ── Row 1: FFT ──
freqs_raw, mag_raw = compute_fft(raw_nA)
freqs_old, mag_old = compute_fft(old_nA)
freqs_fix, mag_fix = compute_fft(fixed_nA)
freqs_sma, mag_sma = compute_fft(fixed_sma_nA)

ax = axes[1, 0]
ax.plot(freqs_raw, mag_raw, linewidth=0.7, alpha=0.5, label="Raw")
ax.plot(freqs_old, mag_old, linewidth=0.8, label="Old notch")
ax.axvline(f_peak, color="red", ls="--", alpha=0.7,
           label=f"Actual peak = {f_peak:.1f} Hz")
ax.axvline(f_actual_notch, color="orange", ls=":", alpha=0.7,
           label=f"Old target = {f_actual_notch:.1f} Hz")
ax.set_title("FFT — OLD filter")
ax.set_xlabel("Frequency (Hz)")
ax.set_ylabel("Magnitude (dB)")
ax.set_xlim(0, fs / 2)
ax.legend(fontsize=8)
ax.grid(True, alpha=0.3)

ax = axes[1, 1]
ax.plot(freqs_raw, mag_raw, linewidth=0.7, alpha=0.5, label="Raw")
ax.plot(freqs_fix, mag_fix, linewidth=0.8, color="tab:green",
        label=f"Fixed notch at {f_peak:.1f} Hz")
ax.axvline(f_peak, color="red", ls="--", alpha=0.7,
           label=f"Actual peak = {f_peak:.1f} Hz")
ax.set_title("FFT — FIXED notch")
ax.set_xlabel("Frequency (Hz)")
ax.set_ylabel("Magnitude (dB)")
ax.set_xlim(0, fs / 2)
ax.legend(fontsize=8)
ax.grid(True, alpha=0.3)

# ── Row 2: notch + SMA ──
ax = axes[2, 0]
ax.plot(t, raw_nA, linewidth=0.4, alpha=0.4, label="Raw")
ax.plot(t, fixed_nA, linewidth=0.5, alpha=0.5, color="tab:green", label="Notch only")
ax.plot(t, fixed_sma_nA, linewidth=1.0, color="tab:red",
        label="Notch + SMA(10)")
ax.set_title("Time domain — Notch + SMA(10)")
ax.set_xlabel("Time (s)")
ax.set_ylabel("Current (nA)")
ax.legend(fontsize=8, loc="upper right")
ax.grid(True, alpha=0.3)

ax = axes[2, 1]
ax.plot(freqs_raw, mag_raw, linewidth=0.7, alpha=0.4, label="Raw")
ax.plot(freqs_fix, mag_fix, linewidth=0.7, alpha=0.5, color="tab:green",
        label="Notch only")
ax.plot(freqs_sma, mag_sma, linewidth=0.8, color="tab:red",
        label="Notch + SMA(10)")
ax.axvline(f_peak, color="red", ls="--", alpha=0.5,
           label=f"Peak = {f_peak:.1f} Hz")
ax.set_title("FFT — Notch + SMA(10)")
ax.set_xlabel("Frequency (Hz)")
ax.set_ylabel("Magnitude (dB)")
ax.set_xlim(0, fs / 2)
ax.legend(fontsize=8)
ax.grid(True, alpha=0.3)

plt.tight_layout()
out = here / "notch_analysis.png"
plt.savefig(out, dpi=150)
print(f"\nSaved: {out}")
plt.show()
