import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import serial.tools.list_ports

# --- Configuration ---
BAUD_RATE = 115200
EXPECTED_VALUES = 90  # 1° résolution
PORT = None

# --- Auto-detect serial port ---
def find_serial_port():
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        print(f"Found port: {p.device}")
    return ports[0].device if ports else None

PORT = find_serial_port()
if PORT is None:
    print("No serial port found.")
    exit()

print(f"Using port: {PORT}")
ser = serial.Serial(PORT, BAUD_RATE, timeout=1)

# --- Setup plot ---
fig = plt.figure()
ax = fig.add_subplot(111, polar=True)
sc = ax.scatter([], [], s=5)
ax.set_ylim(0, 6000)  # max distance in mm

# --- Update function ---
def update(frame):
    line = ser.readline().decode(errors='ignore').strip()
    if not line:
        return sc,

    try:
        values = list(map(int, line.split(',')))
        if len(values) != EXPECTED_VALUES:
            print(f"Ignored line with {len(values)} values (expected {EXPECTED_VALUES})")
            return sc,

        angles = np.linspace(0, 2 * np.pi, EXPECTED_VALUES, endpoint=False)
        distances = np.array(values)

        sc.set_offsets(np.c_[angles, distances])
    except Exception as e:
        print("Error parsing line:", e)

    return sc,

ani = animation.FuncAnimation(fig, update, interval=100, blit=True)
plt.show()
