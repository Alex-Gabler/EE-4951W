import serial

ser = serial.Serial("/dev/cu.usbmodem11403", 115200)

while True:
    line = ser.readline().decode(errors="ignore").strip()
    if line:
        print(line)