import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
import numpy as np

PORT = "/dev/tty.usbmodem1301"
BAUD = 115200

WINDOW_SIZE = 1200
USE_CHANNEL = 1

ser = serial.Serial(PORT, BAUD, timeout=1)

xdata = deque(maxlen=WINDOW_SIZE)
rawdata = deque(maxlen=WINDOW_SIZE)
filtdata = deque(maxlen=WINDOW_SIZE)

fig, ax = plt.subplots()
line, = ax.plot([], [], linewidth=1)
ax.set_title("Live ECG Waveform")
ax.set_xlabel("Sample")
ax.set_ylabel("ADC Counts (centered)")

sample_index = 0

# Simple smoothing
prev_lp = 0.0
def lowpass(x, alpha=0.12):
    global prev_lp
    prev_lp = alpha * x + (1 - alpha) * prev_lp
    return prev_lp

def update(frame):
    global sample_index

    while ser.in_waiting:
        line_in = ser.readline().decode("utf-8", errors="replace").strip()

        if not line_in or line_in.startswith("#"):
            continue

        parts = line_in.split(",")
        if len(parts) != 6:
            continue

        try:
            ch1 = int(parts[4])
            ch2 = int(parts[5])
        except ValueError:
            continue

        y = ch1 if USE_CHANNEL == 1 else ch2

        xdata.append(sample_index)
        rawdata.append(y)
        filtdata.append(lowpass(y))
        sample_index += 1

    if len(filtdata) > 20:
        y = np.array(filtdata, dtype=float)

        # Remove DC offset / baseline
        y_centered = y - np.median(y)

        line.set_data(list(xdata), y_centered)

        ax.set_xlim(xdata[0], xdata[-1])

        # Robust scaling: ignore extreme spikes
        lo = np.percentile(y_centered, 5)
        hi = np.percentile(y_centered, 95)

        pad = 0.25 * max(hi - lo, 1)
        ax.set_ylim(lo - pad, hi + pad)

    return line,

ani = FuncAnimation(fig, update, interval=30, blit=True)

try:
    plt.show()
finally:
    ser.close()