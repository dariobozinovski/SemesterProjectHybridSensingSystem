import sys
import serial
import numpy as np
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtWidgets import (
    QApplication, QWidget, QGridLayout, QLabel, QVBoxLayout,
    QHBoxLayout, QSlider
)
from PyQt5.QtGui import QColor

# ==========================================================
# Color utilities
# ==========================================================
def distance_to_color(dist_mm):
    if dist_mm is None or dist_mm == -1:
        return QColor(70, 70, 70)
    if dist_mm >= 3000:
        return QColor(0, 170, 0)
    if dist_mm <= 50:
        return QColor(200, 0, 0)

    ratio = (dist_mm - 50) / (3000 - 50)
    ratio = np.clip(ratio, 0, 1)

    if ratio > 0.5:
        t = (ratio - 0.5) * 2
        r = int(200 * (1 - t))
        g = 200
    else:
        t = ratio * 2
        r = 200
        g = int(200 * t)

    return QColor(r, g, 0)


def validity_to_color(val):
    if val == 5:
        return QColor(0, 200, 0)
    elif val == 255:
        return QColor(200, 0, 0)
    elif 6 <= val <= 9:
        return QColor(180, 180, 0)
    elif val == 0:
        return QColor(0, 100, 0)
    else:
        return QColor(200, 200, 0)


def valid_count_to_color(count, total):
    if total == 0:
        return QColor(70, 70, 70)
    ratio = count / total
    if ratio <= 0.5:
        t = ratio * 2
        r = 200
        g = int(200 * t)
    else:
        t = (ratio - 0.5) * 2
        r = int(200 * (1 - t))
        g = 200
    return QColor(r, g, 0)


def stddev_to_color(std_val, max_val):
    if np.isnan(std_val) or max_val == 0:
        return QColor(70, 70, 70)

    ratio = min(std_val / max_val, 1.0)
    if ratio <= 0.5:
        t = ratio * 2
        r = int(200 * t)
        g = 200
    else:
        t = (ratio - 0.5) * 2
        r = 200
        g = int(200 * (1 - t))
    return QColor(r, g, 0)


# ==========================================================
# Cell widget
# ==========================================================
class Cell(QLabel):
    def __init__(self):
        super().__init__()
        self.setAlignment(Qt.AlignCenter)
        self.setFixedSize(38, 38)
        self.setStyleSheet("""
            QLabel {
                border-radius: 6px;
                border: 1px solid #222;
                color: white;
                background-color: #202020;
                font-weight: bold;
                font-size: 14px;
            }
        """)

    def set_color_and_text(self, color: QColor, text: str):
        r, g, b = color.red(), color.green(), color.blue()
        self.setStyleSheet(f"""
            QLabel {{
                border-radius: 6px;
                border: 1px solid #111;
                color: white;
                background-color: rgb({r},{g},{b});
                font-weight: bold;
                font-size: 14px;
            }}
        """)
        self.setText(text)


# ==========================================================
# Main GUI
# ==========================================================
class SensorGUI(QWidget):
    def __init__(self, port="COM7", baud=115200):
        super().__init__()

        self.ser = serial.Serial(port, baud, timeout=0.1)

        self.buffer_size = 10
        self.recent_readings = []    # TOF history
        self.us_recent = []          # US history

        # Grids
        self.tof_cells = []
        self.validity_cells = []
        self.raw_cells = []

        self.mean_cells = []
        self.std_cells = []
        self.valid_count_cells = []

        # US arrays
        self.us_cells = []
        self.us_mean_cells = []
        self.us_std_cells = []

        self.frequency_label = QLabel("Freq: 0 Hz")
        self.frequency_label.setAlignment(Qt.AlignCenter)
        self.frequency_label.setFixedSize(80, 40)

        self.init_ui()

        self.timer = QTimer()
        self.timer.timeout.connect(self.read_serial_line)
        self.timer.start(30)

    # ------------------------------------------------------
    def init_ui(self):
        main = QVBoxLayout()

        # ---------------- TOP: TOF + Validity + Raw ----------------
        top = QHBoxLayout()
        top.addLayout(self.make_8x8_grid("Filtered TOF", self.tof_cells))
        top.addLayout(self.make_8x8_grid("Validity", self.validity_cells))
        top.addLayout(self.make_8x8_grid("Raw TOF", self.raw_cells))
        main.addLayout(top)

        # ---------------- MIDDLE: Mean + Std + Valid Count ---------
        mid = QHBoxLayout()
        mid.addLayout(self.make_8x8_grid("Mean", self.mean_cells))
        mid.addLayout(self.make_8x8_grid("Std Dev", self.std_cells))
        mid.addLayout(self.make_8x8_grid("Valid Count", self.valid_count_cells))
        main.addLayout(mid)

        # ---------------- BOTTOM: US + US Mean + US Std + Slider ---
        bottom = QHBoxLayout()

        # ---- US raw ----
        us_raw = QGridLayout()
        us_raw.addWidget(QLabel("US Raw"), 0, 0, 1, 8)
        for i in range(8):
            c = Cell()
            c.setFixedSize(40, 30)
            us_raw.addWidget(c, 1, i)
            self.us_cells.append(c)
        bottom.addLayout(us_raw)

        # ---- US mean ----
        us_mean = QGridLayout()
        us_mean.addWidget(QLabel("US Mean"), 0, 0, 1, 8)
        for i in range(8):
            c = Cell()
            c.setFixedSize(40, 30)
            us_mean.addWidget(c, 1, i)
            self.us_mean_cells.append(c)
        bottom.addLayout(us_mean)

        # ---- US std ----
        us_std = QGridLayout()
        us_std.addWidget(QLabel("US Std Dev"), 0, 0, 1, 8)
        for i in range(8):
            c = Cell()
            c.setFixedSize(40, 30)
            us_std.addWidget(c, 1, i)
            self.us_std_cells.append(c)
        bottom.addLayout(us_std)

        # ---- Right column: freq + slider ----
        right = QVBoxLayout()

        right.addWidget(self.frequency_label)

        label = QLabel("Num readings")
        label.setAlignment(Qt.AlignCenter)
        right.addWidget(label)

        self.slider = QSlider(Qt.Horizontal)
        self.slider.setMinimum(1)
        self.slider.setMaximum(100)
        self.slider.setValue(self.buffer_size)
        self.slider.valueChanged.connect(self.update_buffer_size)
        right.addWidget(self.slider)

        self.slider_label_value = QLabel(str(self.buffer_size))
        self.slider_label_value.setAlignment(Qt.AlignCenter)
        right.addWidget(self.slider_label_value)

        bottom.addLayout(right)

        main.addLayout(bottom)

        self.setLayout(main)
        self.setWindowTitle("TOF + Ultrasonic Visualization")
        self.show()

    # ------------------------------------------------------
    def make_8x8_grid(self, title, container):
        layout = QVBoxLayout()
        layout.addWidget(QLabel(title))
        grid = QGridLayout()
        for r in range(8):
            row = []
            for c in range(8):
                cell = Cell()
                grid.addWidget(cell, r, c)
                row.append(cell)
            container.append(row)
        layout.addLayout(grid)
        return layout

    # ------------------------------------------------------
    def update_buffer_size(self, val):
        self.buffer_size = val
        self.slider_label_value.setText(str(val))

        if len(self.recent_readings) > val:
            self.recent_readings = self.recent_readings[-val:]
        if len(self.us_recent) > val:
            self.us_recent = self.us_recent[-val:]

    # ------------------------------------------------------
    def read_serial_line(self):
        try:
            line = self.ser.readline().decode().strip()
            if not line:
                return

            parts = line.split(",")
            if len(parts) < 64 + 8 + 1 + 64 + 64:
                return

            # -------- TOF filtered --------
            tof_filtered = [int(parts[i]) if parts[i] != "X" else -1 for i in range(64)]
            self.recent_readings.append(tof_filtered)
            if len(self.recent_readings) > self.buffer_size:
                self.recent_readings.pop(0)

            # -------- Ultrasonic --------
            us_vals = []
            for i in range(64, 72):
                p = parts[i]
                if p == "X":
                    us_vals.append(np.nan)
                else:
                    v = int(p)
                    us_vals.append(3000 if v == 0 else v)

            self.us_recent.append(us_vals)
            if len(self.us_recent) > self.buffer_size:
                self.us_recent.pop(0)

            # -------- Frequency --------
            freq = parts[72]
            self.frequency_label.setText(f"Freq: {freq} Hz")

            # -------- Validity --------
            validity_vals = [int(v) if v != "X" else 0 for v in parts[73:137]]

            # -------- Raw TOF --------
            raw_vals = [int(v) if v != "X" else -1 for v in parts[137:201]]

            self.update_display(tof_filtered, validity_vals, raw_vals, us_vals)
            self.update_stats()

        except Exception as e:
            print("Error:", e)

    # ------------------------------------------------------
    def update_display(self, tof_vals, validity_vals, raw_vals, us_vals):
        # TOF filtered
        for i in range(64):
            r, c = divmod(i, 8)
            v = tof_vals[i]
            col = distance_to_color(v) if v != -1 else QColor(70, 70, 70)
            self.tof_cells[r][c].set_color_and_text(col, "X" if v == -1 else str(v))

        # Validity
        for i in range(64):
            r, c = divmod(i, 8)
            v = validity_vals[i]
            col = validity_to_color(v)
            self.validity_cells[r][c].set_color_and_text(col, str(v))

        # Raw TOF
        for i in range(64):
            r, c = divmod(i, 8)
            v = raw_vals[i]
            col = distance_to_color(v) if v != -1 else QColor(0, 150, 0)
            self.raw_cells[r][c].set_color_and_text(col, ">3m" if v == -1 else str(v))

        # Ultrasonic
        for i in range(8):
            v = us_vals[i]
            if np.isnan(v):
                self.us_cells[i].set_color_and_text(QColor(100, 100, 100), "NC")
            else:
                txt = ">3m" if v >= 3000 else str(int(v))
                col = distance_to_color(v)
                self.us_cells[i].set_color_and_text(col, txt)

    # ------------------------------------------------------
    def update_stats(self):
        if len(self.recent_readings) == 0:
            return

        # ======================= TOF stats ==========================
        arr = np.array(self.recent_readings)
        mask = np.where(arr == -1, np.nan, arr)

        mean_vals = np.nanmean(mask, axis=0)
        std_vals = np.nanstd(mask, axis=0)
        max_std = np.nanmax(std_vals)
        valid_count = np.sum(~np.isnan(mask), axis=0)

        for i in range(64):
            r, c = divmod(i, 8)

            # Mean
            mv = mean_vals[i]
            col = distance_to_color(mv) if not np.isnan(mv) else QColor(70, 70, 70)
            self.mean_cells[r][c].set_color_and_text(col, "X" if np.isnan(mv) else str(int(mv)))

            # Std
            sv = std_vals[i]
            col = stddev_to_color(sv, max_std)
            self.std_cells[r][c].set_color_and_text(col, "X" if np.isnan(sv) else str(int(sv)))

            # Valid count
            col = valid_count_to_color(valid_count[i], self.buffer_size)
            self.valid_count_cells[r][c].set_color_and_text(col, str(int(valid_count[i])))

        # ======================= US stats ==========================
        if len(self.us_recent) > 0:
            us_arr = np.array(self.us_recent, dtype=float)

            us_mean = np.nanmean(us_arr, axis=0)
            us_std = np.nanstd(us_arr, axis=0)
            max_us_std = np.nanmax(us_std)

            for i in range(8):
                mv = us_mean[i]
                sv = us_std[i]

                # Mean
                if np.isnan(mv):
                    self.us_mean_cells[i].set_color_and_text(QColor(70, 70, 70), "X")
                else:
                    col = distance_to_color(mv)
                    self.us_mean_cells[i].set_color_and_text(col, str(int(mv)))

                # Std
                if np.isnan(sv):
                    self.us_std_cells[i].set_color_and_text(QColor(70, 70, 70), "X")
                else:
                    col = stddev_to_color(sv, max_us_std)
                    self.us_std_cells[i].set_color_and_text(col, str(int(sv)))


# ==========================================================
if __name__ == "__main__":
    app = QApplication(sys.argv)
    gui = SensorGUI(port="COM7", baud=115200)
    sys.exit(app.exec_())
