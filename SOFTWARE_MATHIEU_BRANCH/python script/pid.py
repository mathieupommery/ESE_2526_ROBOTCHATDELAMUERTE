import serial
from serial import SerialException
import time
import collections
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# -----------------------------
# Paramètres
# -----------------------------
PORT = "COM15"
BAUDRATE = 115200

SAMPLE_PERIOD_S = 0.005   # 5 ms entre échantillons
WINDOW_SECONDS = 10       # fenêtre d'affichage : 10 s
MAX_POINTS = int(WINDOW_SECONDS / SAMPLE_PERIOD_S)  # ~2000 points

# -----------------------------
# Initialisation série
# -----------------------------
ser = serial.Serial(
    PORT,
    BAUDRATE,
    timeout=0.1
)

running = True   # flag pour savoir si on doit encore lire

# -----------------------------
# FIFO pour les données
# -----------------------------
times = collections.deque(maxlen=MAX_POINTS)
vals1 = collections.deque(maxlen=MAX_POINTS)
vals2 = collections.deque(maxlen=MAX_POINTS)

t0 = time.time()

# -----------------------------
# Graphique
# -----------------------------
plt.ion()
fig, ax = plt.subplots()
line1, = ax.plot([], [], label="Signal 1")
line2, = ax.plot([], [], label="Signal 2")

ax.set_xlabel("Temps (s)")
ax.set_ylabel("Valeur")
ax.set_title("Lecture temps réel COM15 (2 flottants)")
ax.legend()
ax.grid(True)


def on_close(event):
    """Callback quand on ferme la fenêtre matplotlib."""
    global running
    running = False
    try:
        if ser.is_open:
            ser.close()
            print("Port série fermé.")
    except Exception:
        pass


fig.canvas.mpl_connect('close_event', on_close)


def read_serial_data():
    """Lit toutes les lignes disponibles sur le port série."""
    global running

    if not running:
        return

    # Si le port est déjà fermé, on ne fait rien
    if not ser.is_open:
        running = False
        return

    try:
        while ser.in_waiting > 0:
            raw = ser.readline()  # lit jusqu'à \n
            line = raw.decode('ascii', errors='ignore').strip()
            if not line:
                continue

            parts = line.split(',')
            if len(parts) != 2:
                continue

            try:
                v1 = float(parts[0])
                v2 = float(parts[1])
            except ValueError:
                continue

            t = time.time() - t0

            times.append(t)
            vals1.append(v1)
            vals2.append(v2)

    except SerialException as e:
        # Port non valide / débranché / fermé -> on arrête proprement
        print("Erreur série, arrêt de la lecture :", e)
        running = False
        try:
            if ser.is_open:
                ser.close()
        except Exception:
            pass


def update(frame):
    """Fonction appelée régulièrement par FuncAnimation."""
    if not running:
        return line1, line2

    read_serial_data()

    if len(times) == 0:
        return line1, line2

    line1.set_data(times, vals1)
    line2.set_data(times, vals2)

    # Fenêtre glissante de 10 s
    t_last = times[-1]
    t_first = max(0.0, t_last - WINDOW_SECONDS)
    ax.set_xlim(t_first, t_last)
    ax.set_ylim(-500, 500)

    return line1, line2


ani = animation.FuncAnimation(
    fig,
    update,
    interval=50,
    blit=False,
    cache_frame_data=False  # supprime le warning
)

plt.show()

# Pas de ser.close() ici, il est géré dans on_close()
