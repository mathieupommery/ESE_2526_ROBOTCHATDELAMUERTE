import serial
import serial.tools.list_ports
import matplotlib.pyplot as plt
import numpy as np
import sys

def find_stm32_port():
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if port.vid == 0x0483 and port.pid == 0x5740:
            return port.device
    if ports:
        print("No STM32 port auto-detected. Available ports:")
        for p in ports:
            vid = f"0x{p.vid:04X}" if p.vid else "None"
            pid = f"0x{p.pid:04X}" if p.pid else "None"
            print(f"{p.device}: {p.description} (VID:{vid}, PID:{pid})")
    else:
        print("No serial ports found at all. Check connections and drivers.")
        sys.exit(1)
    return input("Enter the serial port name (e.g., COM6 or /dev/ttyUSB0): ")

# Connect to serial port
port = find_stm32_port()
print(f"Connecting to {port} at 115200 baud...")
ser = serial.Serial(port, 115200, timeout=5)

# Set up real-time polar plot
plt.ion()
fig = plt.figure()
ax = fig.add_subplot(111, polar=True)

index_buffer = []
distance_buffer = []

while True:
    try:
        line = ser.readline().decode('utf-8', errors='ignore').strip()

        if line.startswith("Index"):
            index = int(line.split()[1])
            index_buffer.append(index)

        elif line.isdigit():
            distance = int(line)
            distance_buffer.append(distance)

        # When both buffers are filled and equal in length
        if len(index_buffer) > 0 and len(index_buffer) == len(distance_buffer):
            # Filter out zero distances and corresponding indexes
            filtered = [(i, d) for i, d in zip(index_buffer, distance_buffer) if d != 0]
            if filtered:
                indexes, distances = zip(*filtered)
                angles = [2 * np.pi * (i % 360) / 360.0 for i in indexes]

                ax.clear()
                ax.plot(angles, distances, 'b.')
                ax.set_ylim(0, max(distances) * 1.1)
                ax.set_title("LIDAR Distance Plot (Zero Values Ignored)")
                fig.canvas.draw()
                fig.canvas.flush_events()

            # Clear buffers for next cluster
            index_buffer.clear()
            distance_buffer.clear()

    except Exception as e:
        print(f"Serial read error: {e}")
        continue