import math
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox

import serial
from serial.tools import list_ports


BAUD_PADRAO = 115200
ALPHA = 0.98  # filtro complementar: 98% giroscópio + 2% acelerômetro


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class SerialReader:
    def __init__(self):
        self.ser = None
        self.thread = None
        self.running = False
        self.lock = threading.Lock()
        self.latest = None
        self.error = None

    def connect(self, port, baud):
        self.disconnect()
        self.ser = serial.Serial(port, baud, timeout=0.2)
        time.sleep(1.2)  # ESP32 pode reiniciar ao abrir a porta serial
        self.ser.reset_input_buffer()
        self.running = True
        self.error = None
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def disconnect(self):
        self.running = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=0.5)
        self.thread = None
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None

    def _loop(self):
        while self.running and self.ser:
            try:
                line = self.ser.readline().decode("utf-8", errors="ignore").strip()
                if not line:
                    continue

                parts = line.split(",")
                if len(parts) != 6:
                    continue

                values = tuple(float(x) for x in parts)
                with self.lock:
                    self.latest = (time.perf_counter(), values)
            except Exception as e:
                self.error = str(e)
                self.running = False
                break

    def get_latest(self):
        with self.lock:
            return self.latest


class AttitudeEstimator:
    def __init__(self):
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.last_t = None
        self.initialized = False

        self.zero_roll = 0.0
        self.zero_pitch = 0.0
        self.zero_yaw = 0.0

        self.gx_bias = 0.0
        self.gy_bias = 0.0
        self.gz_bias = 0.0

        self.calibrating = False
        self.calib_samples = []

    def start_gyro_calibration(self):
        self.calibrating = True
        self.calib_samples = []

    def zero(self):
        self.zero_roll = self.roll
        self.zero_pitch = self.pitch
        self.zero_yaw = self.yaw

    def update(self, t, ax, ay, az, gx, gy, gz):
        # Calibração de bias do giroscópio.
        # Mantenha o sensor parado durante esta etapa.
        if self.calibrating:
            self.calib_samples.append((gx, gy, gz))
            if len(self.calib_samples) >= 120:
                n = len(self.calib_samples)
                self.gx_bias = sum(v[0] for v in self.calib_samples) / n
                self.gy_bias = sum(v[1] for v in self.calib_samples) / n
                self.gz_bias = sum(v[2] for v in self.calib_samples) / n
                self.calibrating = False

        gx -= self.gx_bias
        gy -= self.gy_bias
        gz -= self.gz_bias

        # Inclinação estimada pelo acelerômetro
        roll_acc = math.degrees(math.atan2(ay, az))
        pitch_acc = math.degrees(
            math.atan2(-ax, math.sqrt(ay * ay + az * az))
        )

        if not self.initialized:
            self.roll = roll_acc
            self.pitch = pitch_acc
            self.yaw = 0.0
            self.last_t = t
            self.initialized = True
            return

        dt = t - self.last_t
        self.last_t = t

        if dt <= 0 or dt > 0.25:
            return

        # Integração do giroscópio + correção pelo acelerômetro
        self.roll = ALPHA * (self.roll + gx * dt) + (1 - ALPHA) * roll_acc
        self.pitch = ALPHA * (self.pitch + gy * dt) + (1 - ALPHA) * pitch_acc

        # MPU6050 não tem magnetômetro: yaw é apenas a integral de gz e deriva com o tempo
        self.yaw += gz * dt

        # Mantém yaw em -180...180 só para facilitar a visualização
        self.yaw = (self.yaw + 180.0) % 360.0 - 180.0

    def display_angles(self):
        return (
            self.roll - self.zero_roll,
            self.pitch - self.zero_pitch,
            self.yaw - self.zero_yaw,
        )


def rot_x(p, a):
    x, y, z = p
    c, s = math.cos(a), math.sin(a)
    return (x, y * c - z * s, y * s + z * c)


def rot_y(p, a):
    x, y, z = p
    c, s = math.cos(a), math.sin(a)
    return (x * c + z * s, y, -x * s + z * c)


def rot_z(p, a):
    x, y, z = p
    c, s = math.cos(a), math.sin(a)
    return (x * c - y * s, x * s + y * c, z)


def rotate_body(p, roll_deg, pitch_deg, yaw_deg):
    # Eixos do modelo:
    # X = frente do avião
    # Y = asa direita
    # Z = para cima
    p = rot_x(p, math.radians(roll_deg))
    p = rot_y(p, math.radians(pitch_deg))
    p = rot_z(p, math.radians(yaw_deg))
    return p


def camera_transform(p):
    # Vista isométrica fixa para enxergar roll, pitch e yaw
    p = rot_z(p, math.radians(-35))
    p = rot_y(p, math.radians(-25))
    return p


class PlaneViewer(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Visualizador 3D - MPU6050 + ESP32")
        self.geometry("980x720")
        self.minsize(820, 620)

        self.reader = SerialReader()
        self.estimator = AttitudeEstimator()
        self.last_packet_id = None

        self._build_ui()
        self.refresh_ports()
        self.protocol("WM_DELETE_WINDOW", self.on_close)
        self.after(16, self.update_ui)

    def _build_ui(self):
        top = ttk.Frame(self, padding=8)
        top.pack(fill="x")

        ttk.Label(top, text="Porta:").pack(side="left")
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(
            top, width=18, textvariable=self.port_var, state="normal")
        self.port_combo.pack(side="left", padx=(4, 8))

        ttk.Button(top, text="Atualizar",
                   command=self.refresh_ports).pack(side="left")

        ttk.Label(top, text="Baud:").pack(side="left", padx=(14, 0))
        self.baud_var = tk.StringVar(value=str(BAUD_PADRAO))
        ttk.Entry(top, width=9, textvariable=self.baud_var).pack(
            side="left", padx=4)

        self.connect_btn = ttk.Button(
            top, text="Conectar", command=self.toggle_connection)
        self.connect_btn.pack(side="left", padx=8)

        ttk.Button(top, text="Calibrar gyro",
                   command=self.calibrate_gyro).pack(side="left", padx=4)
        ttk.Button(top, text="Zerar atitude",
                   command=self.zero_attitude).pack(side="left", padx=4)

        self.status_var = tk.StringVar(value="Desconectado")
        ttk.Label(top, textvariable=self.status_var).pack(side="right")

        info = ttk.Frame(self, padding=(8, 0, 8, 8))
        info.pack(fill="x")
        self.angles_var = tk.StringVar(
            value="Roll:   0.0°    Pitch:   0.0°    Yaw:   0.0°")
        ttk.Label(info, textvariable=self.angles_var, font=(
            "Segoe UI", 11, "bold")).pack(side="left")

        self.canvas = tk.Canvas(self, bg="#101820", highlightthickness=0)
        self.canvas.pack(fill="both", expand=True, padx=8, pady=(0, 8))

        self.help_var = tk.StringVar(
            value="Formato esperado da Serial: ax,ay,az,gx,gy,gz   |   acelerômetro em g, giroscópio em °/s"
        )
        ttk.Label(self, textvariable=self.help_var,
                  padding=(8, 0, 8, 8)).pack(fill="x")

    def refresh_ports(self):
        ports = [p.device for p in list_ports.comports()]
        self.port_combo["values"] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])

    def toggle_connection(self):
        if self.reader.ser and self.reader.running:
            self.reader.disconnect()
            self.connect_btn.config(text="Conectar")
            self.status_var.set("Desconectado")
            return

        port = self.port_var.get().strip()
        if not port:
            messagebox.showwarning(
                "Porta serial", "Selecione ou digite a porta serial.")
            return

        try:
            baud = int(self.baud_var.get())
            self.reader.connect(port, baud)
            self.estimator = AttitudeEstimator()
            self.connect_btn.config(text="Desconectar")
            self.status_var.set(f"Conectado em {port}")
        except Exception as e:
            messagebox.showerror("Erro ao conectar", str(e))

    def calibrate_gyro(self):
        self.estimator.start_gyro_calibration()
        self.status_var.set("Calibrando gyro: deixe o sensor parado...")

    def zero_attitude(self):
        self.estimator.zero()

    def on_close(self):
        self.reader.disconnect()
        self.destroy()

    def update_ui(self):
        packet = self.reader.get_latest()

        if packet and packet[0] != self.last_packet_id:
            self.last_packet_id = packet[0]
            t, (ax, ay, az, gx, gy, gz) = packet

            # Caso o sensor esteja fisicamente montado com outros eixos,
            # ajuste aqui a ordem e/ou o sinal das variáveis.
            self.estimator.update(t, ax, ay, az, gx, gy, gz)

            if self.estimator.calibrating:
                self.status_var.set(
                    f"Calibrando gyro... {len(self.estimator.calib_samples)}/120"
                )
            elif self.reader.ser and self.reader.running:
                self.status_var.set(f"Conectado em {self.port_var.get()}")

        if self.reader.error:
            self.status_var.set("Erro serial: " + self.reader.error)
            self.connect_btn.config(text="Conectar")

        roll, pitch, yaw = self.estimator.display_angles()
        self.angles_var.set(
            f"Roll: {roll:7.1f}°    Pitch: {pitch:7.1f}°    Yaw: {yaw:7.1f}°"
        )

        self.draw_plane(roll, pitch, yaw)
        self.after(16, self.update_ui)

    def draw_plane(self, roll, pitch, yaw):
        c = self.canvas
        c.delete("all")

        w = max(c.winfo_width(), 1)
        h = max(c.winfo_height(), 1)
        cx, cy = w / 2, h / 2 + 25
        scale = min(w, h) * 0.085

        # Modelo simples em coordenadas 3D
        # Cada item = (lista de pontos, cor)
        fuselage = [
            # lados do corpo (prisma alongado)
            ([(3.2, 0, 0), (-2.5, -0.28, -0.20), (-2.5, -0.28, 0.20)], "#d7dde5"),
            ([(3.2, 0, 0), (-2.5, 0.28, 0.20), (-2.5, 0.28, -0.20)], "#b7c1cc"),
            ([(3.2, 0, 0), (-2.5, -0.28, 0.20), (-2.5, 0.28, 0.20)], "#eef2f6"),
            ([(3.2, 0, 0), (-2.5, 0.28, -0.20), (-2.5, -0.28, -0.20)], "#95a3b3"),
        ]

        wing = [
            (
                [(0.7, 0.0, 0.0), (0.0, 3.15, 0.0), (-0.75, 3.15, 0.0),
                 (-0.45, 0.0, 0.0), (-0.75, -3.15, 0.0), (0.0, -3.15, 0.0)],
                "#3aa6ff",
            )
        ]

        tailplane = [
            (
                [(-1.65, 0.0, 0.05), (-2.1, 1.35, 0.05), (-2.55, 1.35, 0.05),
                 (-2.25, 0.0, 0.05), (-2.55, -1.35, 0.05), (-2.1, -1.35, 0.05)],
                "#6fc0ff",
            )
        ]

        fin = [
            ([(-1.85, 0.0, 0.15), (-2.55, 0.0, 1.15), (-2.55, 0.0, 0.15)], "#ffcf48")
        ]

        shapes = wing + tailplane + fin + fuselage

        projected_shapes = []

        for pts, color in shapes:
            transformed = []
            depths = []
            for p in pts:
                q = rotate_body(p, roll, pitch, yaw)
                q = camera_transform(q)
                transformed.append((cx + q[0] * scale, cy - q[2] * scale))
                depths.append(q[1])

            projected_shapes.append(
                (sum(depths) / len(depths), transformed, color))

        # Desenha do mais distante para o mais próximo
        projected_shapes.sort(key=lambda item: item[0], reverse=True)

        for _, pts2d, color in projected_shapes:
            flat = [coord for p in pts2d for coord in p]
            c.create_polygon(flat, fill=color, outline="#081018", width=2)

        # Eixos de referência na parte inferior esquerda
        ox, oy = 85, h - 85
        c.create_text(ox, oy - 52, text="referência",
                      fill="#cfd8e3", font=("Segoe UI", 9))
        c.create_line(ox, oy, ox + 50, oy, fill="#ff6b6b",
                      width=3, arrow=tk.LAST)
        c.create_text(ox + 62, oy, text="X", fill="#ff6b6b")
        c.create_line(ox, oy, ox, oy - 50, fill="#70e000",
                      width=3, arrow=tk.LAST)
        c.create_text(ox, oy - 62, text="Z", fill="#70e000")

        c.create_text(
            w / 2, 30,
            text="MPU6050 → atitude do avião",
            fill="#f1f5f9",
            font=("Segoe UI", 17, "bold")
        )


if __name__ == "__main__":
    app = PlaneViewer()
    app.mainloop()
