# Dispenser Project

This repository contains the Raspberry Pi 5 host-side control code and supporting firmware for the dispenser system. The primary runtime lives in `pico-workspace/`.

The code is intended to run on a Raspberry Pi 5 under Linux. It coordinates:

- CNC motion control over USB serial
- Balance / scale reads over USB serial
- Pico W formulator control using MicroPython firmware uploaded with `mpremote`
- Excel-based dispense result logging

## Main Components

- `pico-workspace/main_dispense_system.py` is the main orchestrator. It coordinates CNC motion, balance readings, Pico formulator commands, job queuing, and dispense result logging.
- `pico-workspace/Drivers/cnc_api.py` wraps the CNC controller. It loads CNC limits and serial settings from `config.yaml`, handles homing and safe motion, and supports dispenser-specific position offsets.
- `pico-workspace/Drivers/balance_api.py` talks to the serial-connected balance. It supports opening the device, reading weight, taring, zeroing, and closing the connection.
- `pico-workspace/Drivers/formulator_driver.py` is the host-side serial wrapper for the Pico W formulator firmware. It sends valve, mode, pump, and stepped-motion commands and waits for responses.
- `pico-workspace/Drivers/tool_changer_api.py` controls the Arduino-based tool changer. It can lock and unlock the tool by driving the motor until the tool-switch state changes.
- `pico-workspace/tool_changer.py` is the standalone test script for the tool changer. Use it to verify the lock/unlock behavior before running the main dispensing flow.
- `pico-workspace/Tests_and_Calib/` contains the quick hardware tests and calibration helpers used during setup and tuning:
	- `test_cnc_simple.py` for direct GRBL connectivity and CNC movement checks
	- `test_formulator.py` for Pico formulator communication tests
	- `ac_pen_sim_test.py` for simulated dispensing / integration checks
	- `valve_calibration.py` for valve and motion calibration work

The Pico-side firmware `pico-workspace/Drivers/pico_api_step.py` lives on the microcontroller and is uploaded from the Pi using `mpremote`. The host side expects the firmware command set used by the formulator driver.

## Repository Layout

- `pico-workspace/` - main Pi 5 runtime and drivers
- `pico-workspace_pretubprim/` - variant workspace for pretub priming calibration
- `pico-workspace_relief/` - variant workspace for relief calibration
- `logs/` - runtime logs and test output

## Requirements

Install the Python dependencies from the repository root:

```bash
python3 -m venv .venv
. .venv/bin/activate
pip install -r requirements.txt
```

The top-level `requirements.txt` includes the Pi 5 dependency set from `pico-workspace/requirements.pi5.txt` and adds `openpyxl` for Excel output support.

## Pi 5 Setup

1. Connect the CNC, balance, and Pico W to the Pi 5 over USB.
2. Update the serial device paths in the relevant scripts or `config.yaml` if your USB IDs change.
3. Upload the Pico firmware:

```bash
mpremote cp pico-workspace/Drivers/pico_api_step.py :main.py
```

4. Run the main system from the main workspace:

```bash
python pico-workspace/main_dispense_system.py
```

## Useful Notes

- The project uses Linux `/dev/serial/by-id/...` device paths rather than Windows COM ports.
- Generated files such as `dispense_results.xlsx` and logs are intentionally ignored by Git.
- The tool changer script may still need a local serial port override depending on the connected hardware.
