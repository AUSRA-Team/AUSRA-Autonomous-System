import serial
import sys

port   = '/dev/ttyACM0'
ns     = sys.argv[1]

s = serial.Serial(port, 115200, timeout=30)
print(f"Waiting for ESP32 ready signal...")

while True:
    line = s.readline().decode(errors='ignore').strip()
    print(f"ESP32: {line}")
    if line == "READY":
        s.write(f"{ns}\n".encode())
        print(f"Sent namespace: {ns}")
        break

s.close()