import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt, find_peaks

# Load your recorded data
df = pd.read_csv("ads1292r_capture.csv")

# Extract columns
t = df["t_us"].to_numpy() * 1e-6
x = df["ch1"].to_numpy().astype(float)

# Estimate sampling rate
fs = 1.0 / np.median(np.diff(t))
print("Sample rate:", fs)

# Bandpass filter
def bandpass(data, fs, low=0.5, high=25.0, order=3):
    nyq = 0.5 * fs
    b, a = butter(order, [low/nyq, high/nyq], btype="band")
    return filtfilt(b, a, data)

xf = bandpass(x, fs)

# Peak detection
threshold = np.mean(xf) + 0.8 * np.std(xf)
min_distance = int(0.25 * fs)

peaks, _ = find_peaks(xf, height=threshold, distance=min_distance)

# Heart rate calculation
peak_times = t[peaks]
rr = np.diff(peak_times)

if len(rr) > 0:
    bpm = 60.0 / rr
    print("Average BPM:", np.mean(bpm))
else:
    print("No peaks detected")

# Plot
plt.figure(figsize=(10,4))
plt.plot(t, xf)
plt.plot(t[peaks], xf[peaks], "ro")
plt.title("ECG + Detected Beats")
plt.show()