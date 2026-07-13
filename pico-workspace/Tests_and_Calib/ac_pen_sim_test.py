from __future__ import annotations

import sys
import time
from pathlib import Path
from datetime import datetime

import pandas as pd

ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from Drivers.cnc_api import CNC
from Drivers.balance_api import Balance
from Drivers.formulator_api import Actuator, Dispenser

# ================= User-tunable settings =================
CNC_ID = "cnc1"
CNC_PORT = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
CNC_BAUD = 115200

BALANCE_ID = "balance1"
BALANCE_PORT = "/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller_DBBMb147613-if00-port0"
BALANCE_BAUD = 9600
BALANCE_TIMEOUT = 2.0

DISPENSER_PORT = "/dev/serial/by-id/usb-MicroPython_Board_in_FS_mode_e66368254f52132d-if00"
DISPENSER_BAUD = 115200

Z_DOWN = -20.0  # absolute Z position to move down to
Z_UP = 0.0      # absolute Z position to move up to
REPEATS = 2

OUTPUT_DIR = ROOT / "logs" / "test"  # fixed path; change here if needed
# =========================================================


def main() -> int:
    # Dummy positions/config path are used to satisfy CNC constructor requirements for this test only.
    dummy_positions = pd.DataFrame([{"positionID": "SAFE_Y", "x": 0.0, "y": 0.0, "z": 0.0}])
    cnc = CNC(config_path="__dummy_config__.yaml", cnc_type=CNC_ID, positions=dummy_positions, virtual=False)
    cnc.SERIAL_PORT = CNC_PORT
    cnc.BAUD_RATE = CNC_BAUD

    dispenser = Dispenser(DISPENSER_PORT, baud=DISPENSER_BAUD, debug=True)
    balance = Balance(BALANCE_PORT, balance_id=BALANCE_ID, baud_rate=BALANCE_BAUD, timeout=BALANCE_TIMEOUT)

    balance.open()
    dispenser.open()

    results = []
    try:
        for i in range(1, REPEATS + 1):
            time.sleep(1.0)
            cnc.move_to_point(x=None, y=None, z=Z_DOWN, speed=cnc.SLOW_SPEED)
            time.sleep(1.0)
            dispenser.step()
            time.sleep(1.0)
            cnc.move_to_point(x=None, y=None, z=Z_UP, speed=cnc.SLOW_SPEED)
            time.sleep(1.0)

            balance.tare()
            dispenser.step()
            weight = balance.read_weight()

            results.append({
                "index": i,
                "weight_g": weight,
                "timestamp": datetime.now().isoformat(timespec="seconds"),
            })
            print(f"[RESULT] {i}: {weight:.6f} g")
            time.sleep(0.2)
    finally:
        dispenser.close()
        balance.close()

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    out_path = OUTPUT_DIR / f"ac_pen_sim_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    pd.DataFrame(results).to_csv(out_path, index=False)
    print(f"[OUTPUT] {out_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
