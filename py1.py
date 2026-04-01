import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.widgets import Button
from collections import deque

# ── Config ───────────────────────────────────────────────────────────────────
PORT = '/dev/cu.usbmodem11403'
BAUD = 115200
WINDOW = 500

# ── Colours ──────────────────────────────────────────────────────────────────
BG      = "#080d12"
BG2     = "#0d1620"
GRID    = "#0e2030"
TRACE   = "#00e5ff"
WARN    = "#ff4d6d"
TEXT    = "#c8e6f5"
DIMTEXT = "#3a6070"
ACCENT  = "#00e5ff"
GREEN   = "#39ff14"

# ── Serial ───────────────────────────────────────────────────────────────────
ser = serial.Serial(PORT, BAUD, timeout=0.01)
ser.reset_input_buffer()
ser.reset_output_buffer()

# ── Data ─────────────────────────────────────────────────────────────────────
data = deque([0.0] * WINDOW, maxlen=WINDOW)

# ── Figure ───────────────────────────────────────────────────────────────────
plt.rcParams.update({
    "font.family": "monospace",
    "text.color": TEXT,
    "axes.labelcolor": TEXT,
    "xtick.color": DIMTEXT,
    "ytick.color": DIMTEXT,
})

fig, ax = plt.subplots(figsize=(12, 5), facecolor=BG)
fig.subplots_adjust(left=0.07, right=0.97, top=0.88, bottom=0.12)
ax.set_facecolor(BG2)

for sp in ax.spines.values():
    sp.set_color(GRID)

ax.set_ylim(-20, 20)
ax.set_xlim(0, WINDOW)
ax.set_title("RESP·SCOPE", color=ACCENT, fontsize=14, fontweight="bold", loc="left", pad=10)
ax.set_ylabel("Displacement (mm)", fontsize=9)
ax.set_xlabel("Samples", fontsize=9)
ax.tick_params(labelsize=8)
ax.grid(True, color=GRID, linewidth=0.6, linestyle="--")
ax.axhline(0, color=DIMTEXT, linewidth=0.6, linestyle=":")

fig.text(0.97, 0.93, f"● LIVE  {PORT}", ha="right", fontsize=8, color=GREEN, fontweight="bold")
fig.text(0.97, 0.89, f"{BAUD} BAUD", ha="right", fontsize=8, color=DIMTEXT)

# Main trace
line_glow, = ax.plot(range(WINDOW), list(data), color=TRACE, linewidth=6, alpha=0.06)
line,      = ax.plot(range(WINDOW), list(data), color=TRACE, linewidth=1.6)

# Live readout
readout = ax.text(
    0.99, 0.96, "",
    transform=ax.transAxes,
    ha="right", va="top",
    fontsize=9, color=TEXT, fontweight="bold"
)

# ── Buttons ──────────────────────────────────────────────────────────────────
ax_btn_reset = fig.add_axes([0.01, 0.01, 0.08, 0.06])
btn_reset = Button(ax_btn_reset, 'RESET', color=BG2, hovercolor=GRID)
btn_reset.label.set_color(ACCENT)
btn_reset.label.set_fontsize(9)
btn_reset.label.set_fontfamily("monospace")
for sp in ax_btn_reset.spines.values():
    sp.set_color(ACCENT)

def on_reset(event):
    ser.write(b'r')
    ser.reset_input_buffer()

btn_reset.on_clicked(on_reset)

ax_btn_cal = fig.add_axes([0.10, 0.01, 0.10, 0.06])
btn_baseline = Button(ax_btn_cal, 'CALIBRATE', color=BG2, hovercolor=GRID)
btn_baseline.label.set_color(GREEN)
btn_baseline.label.set_fontsize(9)
btn_baseline.label.set_fontfamily("monospace")
for sp in ax_btn_cal.spines.values():
    sp.set_color(GREEN)

def on_baseline(event):
    ser.write(b'b')
    ser.reset_input_buffer()

btn_baseline.on_clicked(on_baseline)

# ── Update ───────────────────────────────────────────────────────────────────
def update(frame):
    latest = None

    try:
        # Drain buffer so display stays live and does not replay stale samples
        while ser.in_waiting:
            latest = ser.readline().decode(errors='ignore').strip()

        if not latest:
            return line, line_glow, readout

        parts = latest.split(",")
        if len(parts) < 3:
            return line, line_glow, readout

        value = float(parts[0])
        peak_max = float(parts[1])
        peak_min = float(parts[2])

        data.append(value)

    except (ValueError, IndexError, serial.SerialException):
        return line, line_glow, readout

    d = list(data)
    line.set_ydata(d)
    line_glow.set_ydata(d)

    readout.set_text(f"now {value:+.2f}  ▲ {peak_max:.2f}  ▼ {peak_min:.2f}  mm")
    readout.set_color(WARN if abs(value) > 15 else TEXT)

    return line, line_glow, readout

# ── Cleanup ──────────────────────────────────────────────────────────────────
def on_close(event):
    try:
        if ser.is_open:
            ser.close()
    except Exception:
        pass

fig.canvas.mpl_connect('close_event', on_close)

ani = animation.FuncAnimation(
    fig,
    update,
    interval=20,   # 50 FPS UI refresh, good for breathing data
    cache_frame_data=False,
    blit=False
)

plt.show()