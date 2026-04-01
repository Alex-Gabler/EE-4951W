import csv
import serial
from datetime import datetime

PORT = "/dev/tty.usbmodem1301"   # change this
BAUD = 115200
OUTFILE = "ads1292r_capture.csv"

ser = serial.Serial(PORT, BAUD, timeout=1)

with open(OUTFILE, "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["pc_time_iso", "t_us", "status1", "status2", "status3", "ch1", "ch2"])

    print("Logging... press Ctrl+C to stop")
    try:
        while True:
            line = ser.readline().decode("utf-8", errors="replace").strip()
            if not line:
                continue
            if line.startswith("#"):
                print(line)
                continue

            parts = line.split(",")
            if len(parts) != 6:
                continue

            pc_time = datetime.now().isoformat(timespec="milliseconds")
            writer.writerow([pc_time] + parts)
            print(parts)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        ser.close()