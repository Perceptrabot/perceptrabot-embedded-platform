import serial
import re
from dataclasses import dataclass

@dataclass
class IMUData:
    gx: float; gy: float; gz: float  # Gyro (raw → deg/s)
    ax: float; ay: float; az: float  # Accel (raw → g)
    mx: float; my: float; mz: float  # Mag (raw → gauss)

# ── Calibration constants ──────────────────────────────────────────────────────
# L3GD20 default sensitivity (250 dps range) → 8.75 mdps/LSB
GYRO_SCALE  = 8.75e-3          # raw → deg/s

# LSM303DLHC accel default (±2g, high-res) → 1 mg/LSB
ACCEL_SCALE = 1e-3 / 16   # was 1e-3, should be 6.25e-5

# LSM303DLHC mag (±1.3 gauss default): XY=1100 LSB/gauss, Z=980 LSB/gauss
MAG_SCALE_XY = 1 / 1100.0
MAG_SCALE_Z  = 1 / 980.0

# ── Parser ─────────────────────────────────────────────────────────────────────
LINE_RE = re.compile(
    r"G\s+(-?\d+)\s+(-?\d+)\s+(-?\d+)"
    r"\s*\|\s*A\s+(-?\d+)\s+(-?\d+)\s+(-?\d+)"
    r"\s*\|\s*M\s+(-?\d+)\s+(-?\d+)\s+(-?\d+)"
)

def parse_line(line: str) -> IMUData | None:
    m = LINE_RE.search(line)
    if not m:
        return None
    gx, gy, gz, ax, ay, az, mx, my, mz = map(int, m.groups())
    return IMUData(
        gx=gx * GYRO_SCALE,  gy=gy * GYRO_SCALE,  gz=gz * GYRO_SCALE,
        ax=ax * ACCEL_SCALE, ay=ay * ACCEL_SCALE, az=az * ACCEL_SCALE,
        mx=mx * MAG_SCALE_XY, my=my * MAG_SCALE_XY, mz=mz * MAG_SCALE_Z,
    )

# ── Main read loop ─────────────────────────────────────────────────────────────
def main():
    PORT   = "/dev/ttyUSB0"   # ← change to your port (Windows: "COM3" etc.)
    BAUD   = 115200

    with serial.Serial(PORT, BAUD, timeout=1) as ser:
        print(f"Listening on {PORT} @ {BAUD} baud…\n")
        while True:
            raw = ser.readline().decode("utf-8", errors="ignore").strip()
            if not raw:
                continue

            data = parse_line(raw)
            if data is None:
                print(f"[skip] {raw}")
                continue

            print(
                f"Gyro  (°/s) : {data.gx:+8.3f}  {data.gy:+8.3f}  {data.gz:+8.3f}\n"
                f"Accel (g)   : {data.ax:+8.4f}  {data.ay:+8.4f}  {data.az:+8.4f}\n"
                f"Mag   (Gs)  : {data.mx:+8.4f}  {data.my:+8.4f}  {data.mz:+8.4f}\n"
            )

if __name__ == "__main__":
    main()