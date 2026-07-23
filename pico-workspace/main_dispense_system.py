"""====================================================
Integrated Formulator/CNC/Balance Dispensing System - Pi5 Version
====================================================

Main orchestration file combining:
- CNC motion control (Genmitsu 4040 Pro via GRBL)
- Dispensing Formulator (Pico W via WiFi/TCP)
- Balance/scale readings (serial)

Queue-based workflow with async processing.

FILES USED:
============
Python Drivers (PC-side):
  • Drivers/cnc_api.py          - CNC motion control (G-code generation, GRBL communication)
  • Drivers/balance_api.py       - Balance/scale serial interface (read weight, tare, zero)
  • Drivers/formulator_driver.py - Pico formulator WiFi/TCP wrapper (send commands, receive status)

Pico Firmware (upload to Pico W, once):
  • Drivers/pico_api_step.py - MicroPython firmware for Pico W (hardware control: valve, actuator,
    WiFiCommandHandler). Fixed hardware config lives here; per-fluid step-limit profiles and the
    currently-selected fluid are pushed at runtime from this file instead (see FLUID_PROFILES /
    sync_all_fluid_profiles below) so adding/tuning a fluid never requires reflashing.

Main Orchestration:
  • main_dispense_system.py      - This file (DispenseJob, IntegratedDispenser, queue processing)

Configuration:
  • config.yaml                  - CNC configuration (ports, limits, etc)
  • positions.csv                - Named CNC positions (VIAL_1, WB, etc)

SETUP/PREREQUISITES:
====================
1. Hardware connections:
    - CNC: Connect via USB serial (Windows COMx, Linux /dev/ttyUSBx or /dev/serial/by-id/...)
    - Balance: Connect via its serial device path
    - Dispensing Formulator (Pico W): WiFi only -- no USB cable needed at runtime

2. Lock the tool (Formulator Pen) in place using the tool_changer.py script before running this file.

3. Before running main_dispense_system.py:
   Step A: Upload Pico firmware (one-time, or after a firmware change)
   ─────────────────────────────────────────────────────────────────
   - Connect Pico W to PC via USB
   - Edit Drivers/pico_api_step.py's WIFI_SSID/WIFI_PASSWORD/FORMULATOR_ID (and STATIC_IP if used)
     for this specific physical unit before uploading
   - Use Thonny IDE or mpremote to upload pico_api_step.py to the Pico
   - Command example: mpremote cp Drivers/pico_api_step.py :main.py
   - Restart Pico (or it will auto-start main.py) and it will connect to WiFi automatically
   - To reset it later without USB, use formulator.reset() (sends RESET over the WiFi link)

   Step B: Create/update config files
    ──────────────────────────────────
    - Update config.yaml with CNC settings and serial device path
    - Create positions.csv with named locations (VIAL_1, VIAL_2, WB, etc)
    - Update FORMULATOR_HOST below to match the Pico's IP address

   Step C: Verify connections
   ───────────────────────────
   - Test CNC: Open terminal, verify GRBL responds to '?'
   - Test Balance: Run balance_api.py separately, verify tare/read works
   - Test Pico: Watch its USB serial console (for setup/debug only) for "[WIFI] Connected. IP=..."
     and "WiFi command handler ready", then confirm FORMULATOR_HOST below matches that IP

4. Run this file:
   ───────────────
   python main_dispense_system.py
"""

import asyncio
import time
import pandas as pd
from pathlib import Path
import sys

# Add paths for imports
ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from Drivers.cnc_api import CNC
from Drivers.balance_api import Balance
from Drivers.formulator_driver import FormulatorDriver


# =====================================================
# CONFIGURATION
# =====================================================

# CNC Configuration
CNC_CONFIG_PATH = "config.yaml"
CNC_TYPE = "cnc1"
CNC_VIRTUAL = False  # Set to False for real hardware

# Hardware Serial Devices
# These paths match the current Pi 5 hardware.
CNC_PORT = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
CNC_BAUD = 115200

BALANCE_PORT = "/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller_DBBMb147613-if00-port0"
BALANCE_BAUD = 9600
BALANCE_TIMEOUT = 2.0

# Formulator now talks over WiFi (see Drivers/pico_api_step.py WiFiCommandHandler)
# instead of USB serial. Set FORMULATOR_HOST to the Pico's IP -- either a static
# IP configured in the firmware's STATIC_IP constant, or a DHCP reservation on
# your router/Pi5 AP keyed to the Pico's MAC address so it stays stable.
FORMULATOR_HOST = "192.168.10.177"
FORMULATOR_TCP_PORT = 8888
FORMULATOR_ID = "formulator1"

# Formulator Calibration (reference profile used by firmware)
# Formula: target_percent = (volume_ml + CALIBRATION_OFFSET) / CALIBRATION_SLOPE
# Calibrated from: 15% ≈ 1mL, 90% ≈ 7mL → 75% movement over 6mL range → 12.5%/mL slope
# Slope = 1/12.5 = 0.08; Offset = 15% × 0.08 = 1.2
CALIBRATION_OFFSET = 1.2
CALIBRATION_SLOPE = 0.08
DEFAULT_HOME_POSITION = 15.0
# CALIBRATION_OFFSET = 0.5939
# CALIBRATION_SLOPE = 0.0771
# DEFAULT_HOME_POSITION = 10.0

# Formulator Motion
FORMULATOR_PWM_PERCENT = 30  # Actuator speed (0-100)
FORMULATOR_FLUID_PROFILE = "BLUESILV12"  # Must match Pico profile names (e.g., WATER, GLYCERIN, BLUESIL)

# Formulator operation mode (now per-job via enqueue(operation_mode=...))
# - NORMAL: Existing dispense behavior (volume-based PUMP IN/OUT)
# - PRIMING: Explicit stepped target behavior in 0-40% firmware window
PRIMING_IN_TARGET_PERCENT = 25.0
PRIMING_OUT_TARGET_PERCENT = 2.0

# Fluid profiles configured from main file.
# - formulator_profile: token sent to Pico firmware (viscosity profile)
# - calibration_*: fluid-specific dispensed-vs-commanded fit coefficients
#     dispensed = (calibration_slope * commanded) + calibration_offset
#   main inverts this fit to get commanded from desired:
#     commanded = (desired - calibration_offset) / calibration_slope
# - pwm_in_percent / pwm_out_percent: per-direction actuator speed
# - in_/out_ step-limit fields: this is the "step limit profile" data that used to
#   live hardcoded in the Pico firmware (VISCOSITY_IN/OUT_STEP_SIZE_LIMITS). It now
#   lives here and gets pushed to the Pico at runtime via sync_all_fluid_profiles()
#   below, so adding/tuning a fluid no longer requires reflashing firmware.
#
# Keep the reference profile at offset=0, slope=1 (no correction).
FLUID_PROFILES = {
    "WATER": {
        "formulator_profile": "WATER",
        "calibration_offset": 0.0981,
        "calibration_slope": 0.9627,
        "pwm_in_percent": 30,
        "pwm_out_percent": 30,
        "relief_enabled": False,
        "in_enabled": False, "in_min_step": 0.1, "in_max_step": 3.0, "in_pause_ms": 0,
        "out_enabled": False, "out_min_step": 1.0, "out_max_step": 3.0, "out_pause_ms": 0,
    },
    "GLYCERIN": {
        "formulator_profile": "GLYCERIN",
        "calibration_offset": -0.0206,
        "calibration_slope": 1.2165,
        "pwm_in_percent": 30,
        "pwm_out_percent": 30,
        "relief_enabled": False,
        "in_enabled": True, "in_min_step": 1.0, "in_max_step": 3.0, "in_pause_ms": 2000,
        "out_enabled": False, "out_min_step": 1.0, "out_max_step": 3.0, "out_pause_ms": 0,
    },
    "BLUESIL": {
        "formulator_profile": "BLUESIL",
        "calibration_offset": -0.2001,
        "calibration_slope": 0.9446,
        "pwm_in_percent": 25,
        "pwm_out_percent": 30,
        "relief_enabled": True,
        "in_enabled": True, "in_min_step": 0.6, "in_max_step": 2.4, "in_pause_ms": 4000,
        "out_enabled": True, "out_min_step": 0.6, "out_max_step": 42.0, "out_pause_ms": 4000,
    },
    "BLUESILV12": {
        "formulator_profile": "BLUESILV12",
        "calibration_offset": 0.1569,
        "calibration_slope": 0.9326,
        "pwm_in_percent": 25,
        "pwm_out_percent": 25,
        "relief_enabled": True,
        "in_enabled": True, "in_min_step": 0.3, "in_max_step": 3.6, "in_pause_ms": 3000,
        "out_enabled": True, "out_min_step": 0.3, "out_max_step": 10.0, "out_pause_ms": 5000,
    },
    "BLUESILV30": {
        "formulator_profile": "BLUESILV30",
        "calibration_offset": 0,
        "calibration_slope": 1,
        "pwm_in_percent": 25,
        "pwm_out_percent": 25,
        "relief_enabled": True,
        "in_enabled": True, "in_min_step": 0.6, "in_max_step": 1.2, "in_pause_ms": 5000,
        "out_enabled": True, "out_min_step": 0.6, "out_max_step": 8.0, "out_pause_ms": 5000,
    },
    "SILTECH60": {
        "formulator_profile": "SILTECH60",
        "calibration_offset": -0.0292,
        "calibration_slope": 0.9427,
        "pwm_in_percent": 25,
        "pwm_out_percent": 25,
        "relief_enabled": True,
        "in_enabled": True, "in_min_step": 0.4, "in_max_step": 1.2, "in_pause_ms": 1500,
        "out_enabled": True, "out_min_step": 0.6, "out_max_step": 8.0, "out_pause_ms": 5000,
    },
     "BLUESILV60": {
        "formulator_profile": "BLUESILV60",
        "calibration_offset": -0.0724,
        "calibration_slope": 0.9307,
        "pwm_in_percent": 25,
        "pwm_out_percent": 25,
        "relief_enabled": True,
        "in_enabled": True, "in_min_step": 0.4, "in_max_step": 1.2, "in_pause_ms": 5000,
        "out_enabled": True, "out_min_step": 0.6, "out_max_step": 8.0, "out_pause_ms": 7000,
    },
}


def sync_all_fluid_profiles(formulator):
    """Push every FLUID_PROFILES entry's step-limit config down to the Pico.

    Call once after connecting -- the Pico then has all fluids' step-limit
    profiles cached in RAM, and per-job code only needs to select which one
    is active (via set_default_fluid / the profile token passed per command).
    """
    print("[INIT] Syncing fluid step-limit profiles to formulator...")
    for key, profile in FLUID_PROFILES.items():
        token = str(profile.get("formulator_profile", key)).strip().upper()
        ok = formulator.sync_fluid_profile(
            token,
            in_min_step=profile["in_min_step"], in_max_step=profile["in_max_step"],
            in_pause_ms=profile["in_pause_ms"], in_enabled=profile["in_enabled"],
            out_min_step=profile["out_min_step"], out_max_step=profile["out_max_step"],
            out_pause_ms=profile["out_pause_ms"], out_enabled=profile["out_enabled"],
        )
        if not ok:
            print(f"[INIT] WARNING: Failed to sync profile {token}")

# Z-Only Motion (No XY moves for now)
# Update these Z positions for your setup.
Z_LOAD = -20       # Z position for loading/drawing fluid
Z_DISPENSE = 0  # Z position for dispense
Z_MOVE_SPEED = 300  # Z move speed (mm/min)

# Results Logging
RESULTS_XLSX = Path(__file__).parent / "dispense_results.xlsx"


# =====================================================
# DISPENSER QUEUE CLASS
# =====================================================

class DispenseJob:
    """Represents a single dispense job."""

    VALID_ACTIONS = ("FILL", "DISPENSE", "BOTH")

    def __init__(self, volume_ml=None, container_id=None, location=None, fluid_profile=None, operation_mode=None, action=None, cycles=None):
        self.location = location
        self.volume_ml = volume_ml
        self.container_id = container_id or f"VIAL_{time.time()}"
        self.fluid_profile = fluid_profile or FORMULATOR_FLUID_PROFILE
        self.operation_mode = operation_mode or "NORMAL"  # NORMAL or PRIMING
        if self.operation_mode == "NORMAL":
            self.action = (action or "BOTH").strip().upper()
            if self.action not in DispenseJob.VALID_ACTIONS:
                raise ValueError(f"Unknown action '{action}'. Must be one of {DispenseJob.VALID_ACTIONS}")
        else:
            self.action = "N/A"
        self.cycles = int(cycles) if cycles is not None else 1  # PRIMING mode only: number of IN/OUT purge cycles
        if self.cycles < 1:
            raise ValueError(f"cycles must be >= 1, got {self.cycles}")
        self.dispense_status = "OK"  # OK or INSUFFICIENT_VOLUME (set when a DISPENSE/BOTH move is clamped)
        self.formulator_profile_token = None
        self.command_volume_ml = None
        self.target_percent = None
        self.profile_calibration_offset = None
        self.profile_calibration_slope = None
        self.profile_pwm_in_percent = None
        self.profile_pwm_out_percent = None
        self.relief_enabled = False
        self.status = "QUEUED"  # QUEUED, IN_PROGRESS, COMPLETED, FAILED
        self.actual_weight = None
        self.form_percent_pre_dispense = None
        self.form_percent_post_dispense = None
        self.form_speed_in_mms = None
        self.form_speed_out_mms = None
        self.form_time_in_s = None
        self.form_time_out_s = None
        self.timestamp = None
        self.job_duration_s = None
        self.calculated_step_size_percent = None
        self.settle_time_used_ms = None


class IntegratedDispenser:
    """Queue-based dispenser orchestrating CNC, formulator, and balance.
    
    Workflow for NORMAL mode (default action="BOTH"):
    1. Valve moves to UP position
    2. Draw fluid into formulator (IN)
    3. Valve moves to THRU position
    4. Record actuator position before OUT
    5. Tare balance
    6. Dispense fluid (OUT)
    7. Valve moves to CLOSED
    8. Remove fluid from tip post-dispense
    9. Read final weight from balance
    10. Record actuator position after dispense

    NORMAL mode also supports splitting fill and dispense into separate jobs via
    enqueue(action=...):
    - action="FILL": steps 1-2 only, then valve closes and holds the drawn fluid.
    - action="DISPENSE": opens the valve and dispenses volume_ml from whatever is
      currently held, computed as a %-delta move from the current actuator position
      (same calibrated volume<->% line, applied as a relative move). If the requested
      volume needs more travel than remains above the reference position, the move is
      skipped entirely (actuator stays put) and the job is flagged INSUFFICIENT_VOLUME
      instead of over- or under-dispensing.

    Workflow for PRIMING mode:
    - N cycles (default 1, set via enqueue(cycles=...)) of: Valve UP -> Move IN to target%
      -> Wait 7s -> Valve THRU -> Move OUT to target% -> Wait 7s
    - No volume calibration or balance reading

    Each job can specify its own operation_mode (NORMAL or PRIMING) via enqueue(operation_mode=...).
    """
    
    def __init__(self, formulator, balance, results_path=None):
        # self.cnc = cnc
        self.formulator = formulator
        self.balance = balance
        self.results_path = results_path or RESULTS_XLSX
        
        self.queue = []
        self.busy = False
        self.completed = 0
        self.failed = 0

    def _resolve_fluid_profile(self, fluid_profile_name):
        """Resolve profile config from FLUID_PROFILES by name (case-insensitive)."""
        key = (fluid_profile_name or FORMULATOR_FLUID_PROFILE).strip().upper()
        profile = FLUID_PROFILES.get(key)
        if profile is None:
            raise ValueError(f"Unknown fluid profile '{fluid_profile_name}'. Available: {', '.join(FLUID_PROFILES.keys())}")

        token = str(profile.get("formulator_profile", key)).strip().upper()
        offset = float(profile.get("calibration_offset", 0.0))
        slope = float(profile.get("calibration_slope", 1.0))
        pwm_in = max(0, min(100, int(profile.get("pwm_in_percent", FORMULATOR_PWM_PERCENT))))
        pwm_out = max(0, min(100, int(profile.get("pwm_out_percent", FORMULATOR_PWM_PERCENT))))
        relief_enabled = bool(profile.get("relief_enabled", False))

        return key, token, offset, slope, pwm_in, pwm_out, relief_enabled

    def _apply_profile_volume_correction(self, requested_volume_ml, profile_offset, profile_slope):
        """Apply inverse fluid TF to requested volume to get command volume.

        Firmware handles the volume->target% conversion with CALIBRATION_OFFSET/SLOPE.
        Main should only apply fluid correction once at the volume level.
        """
        if abs(profile_slope) < 1e-9:
            raise ValueError("Profile calibration_slope cannot be 0")

        command_volume = (requested_volume_ml - profile_offset) / profile_slope

        return max(0.0, command_volume)

    def _to_firmware_target_percent(self, command_volume_ml):
        """Convert firmware command volume to actuator target percent used by Pico."""
        return (command_volume_ml + CALIBRATION_OFFSET) / CALIBRATION_SLOPE

    def _to_firmware_delta_percent(self, command_volume_ml):
        """Convert a command volume delta to an actuator percent delta.

        Same linear mapping as _to_firmware_target_percent, but for a relative
        move from the current position rather than an absolute target — the
        offset cancels out since it's a fixed additive constant on both sides.
        """
        return command_volume_ml / CALIBRATION_SLOPE

    def _apply_profile_delta_correction(self, requested_volume_ml, profile_slope):
        """Fluid correction for a partial (delta) dispense — deliberately no profile_offset term.

        _apply_profile_volume_correction's profile_offset represents a one-time bonus
        that only shows up on a full round trip back to home (e.g. relief/valve-seating
        on arrival). A delta dispense stops mid-stroke and never reaches that event, so
        subtracting profile_offset here would silently short every partial dispense by
        about that amount.
        """
        if abs(profile_slope) < 1e-9:
            raise ValueError("Profile calibration_slope cannot be 0")
        return max(0.0, requested_volume_ml / profile_slope)

    def enqueue(self, volume_ml=None, container_id=None, location=None, fluid_profile=None, operation_mode=None, action=None, cycles=None):
        """Add a dispense job to the queue.

        Args:
            volume_ml: Volume to dispense in mL (required for NORMAL, ignored for PRIMING)
            container_id: Optional identifier for this job
            location: Optional CNC position name (reserved for future XY moves)
            fluid_profile: Optional profile override (e.g., WATER, GLYCERIN, BLUESIL)
            operation_mode: Optional mode ("NORMAL" or "PRIMING", default: "NORMAL")
            action: Optional NORMAL-mode sub-action ("FILL", "DISPENSE", or "BOTH", default: "BOTH")
                - BOTH: draw volume_ml in, then dispense it back out (original behavior)
                - FILL: draw volume_ml in and hold (valve closes, nothing dispensed)
                - DISPENSE: dispense volume_ml from whatever is currently held, computed as a
                  calibrated %-delta move from the current actuator position (not an absolute
                  volume command). If not enough travel remains above the reference position,
                  the move is skipped entirely (actuator stays put) and the job is flagged
                  INSUFFICIENT_VOLUME.
            cycles: Optional PRIMING-mode number of IN/OUT purge cycles (default: 1). Ignored
                for NORMAL mode.
        """
        job = DispenseJob(volume_ml, container_id, location, fluid_profile=fluid_profile, operation_mode=operation_mode, action=action, cycles=cycles)
        self.queue.append(job)
        location_note = f" at {location}" if location else ""
        vol_str = f"{volume_ml} mL" if volume_ml is not None else "N/A (PRIMING)"
        mode_note = f"mode={job.operation_mode}, action={job.action}" if job.operation_mode == "NORMAL" else f"mode={job.operation_mode}, cycles={job.cycles}"
        print(f"[QUEUE] Added: {job.container_id} ({vol_str}{location_note}, fluid={job.fluid_profile}, {mode_note})")
    
    async def process_queue(self):
        """Main queue processor - runs continuously."""
        print("[QUEUE] Processor started")
        
        while True:
            try:
                if self.queue and not self.busy:
                    # Check formulator duty cycle
                    if not self.formulator.is_ready():
                        cooldown = self.formulator.get_cooldown_time()
                        print(f"[QUEUE] ⚠ Formulator cooling down ({cooldown:.0f}s)...")
                        await asyncio.sleep(cooldown + 1)
                        continue
                    
                    # Get next job
                    self.busy = True
                    job = self.queue.pop(0)
                    await self._execute_dispense(job)
                    
                    if job.status == "COMPLETED":
                        self.completed += 1
                    elif job.status == "FAILED":
                        self.failed += 1
                    
                    self.busy = False
            except Exception as e:
                print(f"[QUEUE] ERROR: {e}")
                self.busy = False
            
            await asyncio.sleep(0.5)

    async def _do_fill(self, job, command_volume_ml, profile_token, pwm_in, pwm_out):
        """Draw fluid into the formulator and leave the valve in the correct holding state.

        For action="BOTH" the valve ends at THRU, ready for the immediate dispense that
        follows in the same job. For action="FILL" the valve ends at CLOSED, sealing the
        drawn fluid until a later DISPENSE job opens it.
        """
        end_valve = "THRU" if job.action == "BOTH" else "CLOSED"

        # -------- STEP 1: Move valve to the correct position before drawing --------
        # A fill normally moves UP in % (drawing fluid in through the intake path), so the
        # valve goes to UP. But if the actuator is already sitting above this fill's target
        # (e.g. left over from a prior job), the "IN" move actually has to travel DOWN to
        # reach it — that's physically a dispense motion, not an intake, so the valve needs
        # to be at THRU for it instead of UP.
        current_pos = self.formulator.get_position()
        if current_pos is not None and job.target_percent is not None and current_pos > job.target_percent:
            draw_valve = "THRU"
            print(
                f"[DISPENSE] Step 1: Actuator ({current_pos:.2f}%) is above fill target "
                f"({job.target_percent:.2f}%); moving valve to THRU instead of UP"
            )
        else:
            draw_valve = "UP"
            print("[DISPENSE] Step 1: Moving valve to UP position")
        self.formulator.valve_move(draw_valve)
        await asyncio.sleep(1)

        # -------- STEP 2: Draw fluid (IN) --------
        print(f"[DISPENSE] Step 2: Drawing {job.volume_ml} mL into formulator")
        time_in_start = time.time()
        ok_in = self.formulator.pump_volume(
            command_volume_ml,
            "IN",
            pwm_percent=pwm_in,
            viscosity_profile=profile_token,
        )
        job.form_time_in_s = time.time() - time_in_start

        if not ok_in:
            print("[DISPENSE] WARNING: Draw pump reported failure, but continuing")

        # Read step size and settle time from formulator status
        try:
            status_in = self.formulator.get_status()
            job.calculated_step_size_percent = status_in.get("STEP_SIZE")
            job.settle_time_used_ms = status_in.get("SETTLE_TIME")
            job.form_speed_in_mms = status_in.get("SPEED")
            if job.form_speed_in_mms is None:
                print("[DISPENSE] IN speed: N/A")
            else:
                print(f"[DISPENSE] IN speed: {job.form_speed_in_mms:.4f} mm/s")
        except Exception as e:
            print(f"[DISPENSE] WARNING: Could not read IN speed: {e}")
            job.form_speed_in_mms = None
        print(f"[DISPENSE] IN time: {job.form_time_in_s:.2f}s")
        await asyncio.sleep(2)  # Wait for formulator to fully complete

        # -------- STEP 3: Apply pressure relief if enabled, then move valve to its holding state --------
        if job.relief_enabled and job.volume_ml > 0.3:
            print(f"[DISPENSE] Step 3a: Applying pressure relief (CLOSED → OUT 1.2% → {end_valve})")
            # Move valve to CLOSED
            self.formulator.valve_move("CLOSED")
            await asyncio.sleep(5)
            # Get current position and move OUT by 1.0%
            try:
                current_pos = self.formulator.get_position()
                if current_pos is not None:
                    relief_target = current_pos - 1.2  # Move OUT by 1% for relief
                    self.formulator.move_to_percent_stepped(relief_target, "OUT", pwm_percent=pwm_out)
                    print(f"[DISPENSE] Relief: moved from {current_pos:.2f}% to {relief_target:.2f}%")
                else:
                    print("[DISPENSE] WARNING: Could not read position for relief calculation")
            except Exception as e:
                print(f"[DISPENSE] WARNING: Pressure relief failed: {e}")
            await asyncio.sleep(1)
            if end_valve == "THRU":
                self.formulator.valve_move("THRU")
                await asyncio.sleep(0.5)
            # else: relief already left the valve CLOSED
        else:
            print(f"[DISPENSE] Step 3: Moving valve to {end_valve} position")
            self.formulator.valve_move(end_valve)
            await asyncio.sleep(1)

        # -------- STEP 4: Record actuator position after fill --------
        try:
            job.form_percent_pre_dispense = self.formulator.get_position()
            if job.form_percent_pre_dispense is None:
                print("[DISPENSE] Form% after fill: N/A")
            else:
                print(f"[DISPENSE] Form% after fill: {job.form_percent_pre_dispense:.2f}%")
        except Exception as e:
            print(f"[DISPENSE] WARNING: Could not read Form% after fill: {e}")
            job.form_percent_pre_dispense = None

    async def _do_dispense(self, job, command_volume_ml, profile_token, pwm_out):
        """Dispense fluid via valve OUT, tare/weigh, and record position after.

        For action="BOTH" the valve is already at THRU and job.form_percent_pre_dispense was
        already recorded by _do_fill, so this continues straight to taring/dispensing using
        the full command_volume_ml via the firmware's volume-based PUMP command.

        For action="DISPENSE" the valve starts CLOSED (from an earlier FILL job), so it's
        opened here first and the current position is read fresh — it may have been set by
        a different job earlier in the queue. The dispense amount is then applied as a
        %-delta move from that live position rather than a volume command, since only the
        firmware's PUMP command knows an absolute home-relative volume — a partial dispense
        from mid-stroke has to move by percent. The volume feeding that %-delta is corrected
        via _apply_profile_delta_correction (slope only, no profile_offset) rather than the
        job's upfront command_volume_ml, since profile_offset is a full-round-trip-only bonus
        that a partial dispense never reaches. If the requested delta would move past the
        DEFAULT_HOME_POSITION reference (i.e. not enough fluid remains from the last fill),
        the move is skipped entirely — the actuator stays exactly where it is — and the job
        is flagged INSUFFICIENT_VOLUME instead of dispensing a different amount than asked for.
        """
        if job.action == "DISPENSE":
            print("[DISPENSE] Step: Moving valve to THRU position")
            self.formulator.valve_move("THRU")
            await asyncio.sleep(1)
            try:
                job.form_percent_pre_dispense = self.formulator.get_position()
                if job.form_percent_pre_dispense is None:
                    print("[DISPENSE] Form% before OUT: N/A")
                else:
                    print(f"[DISPENSE] Form% before OUT: {job.form_percent_pre_dispense:.2f}%")
            except Exception as e:
                print(f"[DISPENSE] WARNING: Could not read Form% before OUT: {e}")
                job.form_percent_pre_dispense = None

        # -------- Tare balance --------
        print("[DISPENSE] Step: Taring balance")
        self.balance.tare()
        await asyncio.sleep(2)

        # -------- Dispense fluid (OUT) --------
        print("[DISPENSE] Step: Dispensing fluid (OUT)")
        time_out_start = time.time()

        if job.action == "DISPENSE":
            current_pos = job.form_percent_pre_dispense
            if current_pos is None:
                raise RuntimeError("Could not read actuator position for delta dispense")
            delta_command_volume_ml = self._apply_profile_delta_correction(job.volume_ml, job.profile_calibration_slope)
            delta_percent = self._to_firmware_delta_percent(delta_command_volume_ml)
            target_percent = current_pos - delta_percent
            job.target_percent = target_percent
            if target_percent < DEFAULT_HOME_POSITION:
                print(
                    f"[DISPENSE] WARNING: Requested {job.volume_ml} mL needs {delta_percent:.2f}% of travel, "
                    f"but only {current_pos - DEFAULT_HOME_POSITION:.2f}% remains above the "
                    f"{DEFAULT_HOME_POSITION:.2f}% reference position. Not enough fluid remains — "
                    f"skipping this dispense (actuator stays at {current_pos:.2f}%) and flagging job "
                    f"INSUFFICIENT_VOLUME."
                )
                job.dispense_status = "INSUFFICIENT_VOLUME"
                ok_out = False
            else:
                ok_out = self.formulator.move_to_percent_stepped(
                    target_percent,
                    "OUT",
                    pwm_percent=pwm_out,
                    viscosity_profile=profile_token,
                )
        else:
            ok_out = self.formulator.pump_volume(
                command_volume_ml,
                "OUT",
                pwm_percent=pwm_out,
                viscosity_profile=profile_token,
            )

        job.form_time_out_s = time.time() - time_out_start
        if not ok_out and job.dispense_status != "INSUFFICIENT_VOLUME":
            print("[DISPENSE] WARNING: Dispense pump failed")
        try:
            status_out = self.formulator.get_status()
            job.form_speed_out_mms = status_out.get("SPEED")
            if job.form_speed_out_mms is None:
                print("[DISPENSE] OUT speed: N/A")
            else:
                print(f"[DISPENSE] OUT speed: {job.form_speed_out_mms:.4f} mm/s")
        except Exception as e:
            print(f"[DISPENSE] WARNING: Could not read OUT speed: {e}")
            job.form_speed_out_mms = None
        print(f"[DISPENSE] OUT time: {job.form_time_out_s:.2f}s")
        await asyncio.sleep(2)  # Wait for formulator to fully complete

        # -------- Move valve to CLOSED position --------
        print("[DISPENSE] Step: Moving valve to CLOSED position")
        self.formulator.valve_move("CLOSED")
        await asyncio.sleep(1)

        # -------- Read final weight --------
        print("[DISPENSE] Step: Reading final weight")
        weight = self.balance.read_weight(settle_time=10.0)
        job.actual_weight = weight
        print(f"[DISPENSE] Target: {job.volume_ml} mL, Actual: {weight:.3f} g")

        # -------- Record actuator position after dispense --------
        try:
            job.form_percent_post_dispense = self.formulator.get_position()
            if job.form_percent_post_dispense is None:
                print("[DISPENSE] Form% after OUT: N/A")
            else:
                print(f"[DISPENSE] Form% after OUT: {job.form_percent_post_dispense:.2f}%")
        except Exception as e:
            print(f"[DISPENSE] WARNING: Could not read Form% after OUT: {e}")
            job.form_percent_post_dispense = None

    async def _do_priming(self, job, profile_token, pwm_in, pwm_out):
        """Run PRIMING mode's repeated purge cycles, then return the actuator home.

        Each cycle: valve UP -> move IN to PRIMING_IN_TARGET_PERCENT -> wait 7s ->
        valve THRU -> move OUT to PRIMING_OUT_TARGET_PERCENT -> wait 7s. The number
        of cycles is job.cycles (set via enqueue(cycles=...), default 1). After all
        cycles, the actuator returns to DEFAULT_HOME_POSITION and the valve closes.
        """
        print(f"[DISPENSE] PRIMING MODE: {job.cycles} cycle(s)")
        print(f"[DISPENSE] IN target={PRIMING_IN_TARGET_PERCENT:.2f}%, OUT target={PRIMING_OUT_TARGET_PERCENT:.2f}%")

        for cycle in range(1, job.cycles + 1):
            print(f"\n[DISPENSE] === PRIMING CYCLE {cycle}/{job.cycles} ===")

            # Move valve to UP before IN
            print(f"[DISPENSE] Cycle {cycle}: Moving valve to UP")
            self.formulator.valve_move("UP")
            await asyncio.sleep(1)

            # Move IN to target percent (stepped)
            print(f"[DISPENSE] Cycle {cycle}: Moving IN to {PRIMING_IN_TARGET_PERCENT:.2f}%")
            ok_in = self.formulator.move_to_percent_stepped(
                PRIMING_IN_TARGET_PERCENT,
                "IN",
                pwm_percent=pwm_in,
                viscosity_profile=profile_token,
            )
            if not ok_in:
                print(f"[DISPENSE] WARNING: Cycle {cycle} IN move failed")

            # Wait 7 seconds
            print(f"[DISPENSE] Cycle {cycle}: Waiting 7s...")
            await asyncio.sleep(7)

            # Move valve to THRU before OUT
            print(f"[DISPENSE] Cycle {cycle}: Moving valve to THRU")
            self.formulator.valve_move("THRU")
            await asyncio.sleep(1)

            # Move OUT to target percent (stepped)
            print(f"[DISPENSE] Cycle {cycle}: Moving OUT to {PRIMING_OUT_TARGET_PERCENT:.2f}%")
            ok_out = self.formulator.move_to_percent_stepped(
                PRIMING_OUT_TARGET_PERCENT,
                "OUT",
                pwm_percent=pwm_out,
                viscosity_profile=profile_token,
            )
            if not ok_out:
                print(f"[DISPENSE] WARNING: Cycle {cycle} OUT move failed")

            # Wait 7 seconds before next cycle
            print(f"[DISPENSE] Cycle {cycle}: Waiting 7s...")
            await asyncio.sleep(7)

        # Move valve to UP before returning home
        print("[DISPENSE] Moving valve to UP")
        self.formulator.valve_move("UP")
        await asyncio.sleep(1)

        # Move IN to home position (stepped)
        print(f"[DISPENSE] Moving IN to {DEFAULT_HOME_POSITION:.2f}%")
        ok_in = self.formulator.move_to_percent_stepped(
            DEFAULT_HOME_POSITION,
            "IN",
            pwm_percent=pwm_in,
            viscosity_profile=profile_token,
        )
        if not ok_in:
            print("[DISPENSE] WARNING: IN move failed")

        # Wait 4 seconds
        print("[DISPENSE] Waiting 4s...")
        await asyncio.sleep(4)

        # Close valve at end
        self.formulator.valve_move("CLOSED")
        await asyncio.sleep(90)  # Long wait to ensure that the fluid has rested properly before next dispense, especially for high viscosity fluids like Siltech60.

        print(f"\n[DISPENSE] PRIMING MODE: All {job.cycles} cycle(s) completed")

    async def _execute_dispense(self, job):
        """Execute a single dispense operation.

        Args:
            job: DispenseJob object
        """
        job.status = "IN_PROGRESS"
        job.timestamp = time.time()
        
        print(f"\n{'='*60}")
        print(f"[DISPENSE] {job.container_id}")
        location_note = f" at {job.location}" if job.location else ""
        if job.volume_ml is not None:
            print(f"[DISPENSE] Target: {job.volume_ml} mL{location_note}")
        else:
            print(f"[DISPENSE] Target: PRIMING cycle{location_note}")

        profile_key, profile_token, profile_offset, profile_slope, pwm_in, pwm_out, relief_enabled = self._resolve_fluid_profile(job.fluid_profile)
        
        # Volume correction only needed for NORMAL mode
        if job.operation_mode == "NORMAL":
            command_volume_ml = self._apply_profile_volume_correction(job.volume_ml, profile_offset, profile_slope)
            if job.action in ("FILL", "BOTH"):
                target_percent = self._to_firmware_target_percent(command_volume_ml)
            else:
                # DISPENSE action: target percent is a delta from the live actuator
                # position, computed later once that position is known
                target_percent = None
        else:
            # PRIMING mode uses target percentages, not volumes
            command_volume_ml = None
            target_percent = None

        job.fluid_profile = profile_key
        job.formulator_profile_token = profile_token
        job.command_volume_ml = command_volume_ml
        job.target_percent = target_percent
        job.profile_calibration_offset = profile_offset
        job.profile_calibration_slope = profile_slope
        job.profile_pwm_in_percent = pwm_in
        job.profile_pwm_out_percent = pwm_out
        job.relief_enabled = relief_enabled

        print(f"[DISPENSE] Fluid profile: {profile_key} (firmware={profile_token})")
        print(f"[DISPENSE] Operation mode: {job.operation_mode} (action={job.action})")

        # Only print volume calibration for NORMAL mode
        if job.operation_mode == "NORMAL":
            target_str = f"{target_percent:.2f}%" if target_percent is not None else "computed from live position"
            print(
                f"[DISPENSE] Profile cal: command = desired*slope + offset | offset={profile_offset:.4f}, slope={profile_slope:.4f} | "
                f"Target={target_str} | Cmd vol={command_volume_ml:.4f} mL | PWM IN/OUT={pwm_in}/{pwm_out}%"
            )
        else:
            print(
                f"[DISPENSE] PRIMING targets: IN={PRIMING_IN_TARGET_PERCENT:.2f}%, OUT={PRIMING_OUT_TARGET_PERCENT:.2f}% | PWM IN/OUT={pwm_in}/{pwm_out}%"
            )
        print(f"{'='*60}")
        
        try:
            # Set formulator to job's operation mode
            if not self.formulator.set_operation_mode(job.operation_mode):
                raise RuntimeError(f"Failed to set formulator mode: {job.operation_mode}")
            await asyncio.sleep(0.5)

            # Mark this as the currently-selected fluid on the Pico (runtime state,
            # not firmware-fixed) -- also used as fallback if a bare command omits
            # the profile token.
            self.formulator.set_default_fluid(profile_token)
            
            if job.operation_mode == "NORMAL":
                # ========== NORMAL DISPENSING MODE ==========
                if job.action in ("FILL", "BOTH"):
                    await self._do_fill(job, command_volume_ml, profile_token, pwm_in, pwm_out)

                if job.action in ("DISPENSE", "BOTH"):
                    time.sleep(5)
                    await self._do_dispense(job, command_volume_ml, profile_token, pwm_out)

                job.status = "COMPLETED"

            elif job.operation_mode == "PRIMING":
                # ========== PRIMING MODE ==========
                await self._do_priming(job, profile_token, pwm_in, pwm_out)
                job.status = "COMPLETED"
            if job.timestamp is not None:
                job.job_duration_s = time.time() - job.timestamp
            print(f"[DISPENSE] ✓ COMPLETED: {job.container_id}")
            
        except Exception as e:
            job.status = "FAILED"
            if job.timestamp is not None:
                job.job_duration_s = time.time() - job.timestamp
            print(f"[DISPENSE] ✗ FAILED: {job.container_id}")
            print(f"[DISPENSE] Error: {e}")
        
        finally:
            try:
                self._write_result(job)
            except Exception as e:
                print(f"[DISPENSE] WARNING: Could not write result to Excel: {e}")
            print(f"{'='*60}\n")

    def _write_result(self, job):
        """Append a single dispense result to Excel."""
        # Calculate flow rates from speed (syringe diameter = 16mm)
        # Flow rate (mL/s) = (π/4 × diameter² × speed_mm/s) / 1000
        import math
        syringe_diameter_mm = 16
        syringe_area_mm2 = math.pi * (syringe_diameter_mm / 2) ** 2
        
        form_flowrate_in_mls = None
        form_flowrate_out_mls = None
        
        if job.form_speed_in_mms is not None:
            form_flowrate_in_mls = (syringe_area_mm2 * job.form_speed_in_mms) / 1000
        
        if job.form_speed_out_mms is not None:
            form_flowrate_out_mls = (syringe_area_mm2 * job.form_speed_out_mms) / 1000
        
        row = {
            "container_id": job.container_id,
            "volume_ml": job.volume_ml,
            "location": job.location or "",
            "status": job.status,
            "action": job.action,
            "cycles": job.cycles,
            "dispense_status": job.dispense_status,
            "target_form_percent": job.target_percent,
            "target_percent": job.target_percent,
            "form_percent_pre_dispense": job.form_percent_pre_dispense,
            "form_percent_post_dispense": job.form_percent_post_dispense,
            "form_speed_in_mms": job.form_speed_in_mms,
            "form_speed_out_mms": job.form_speed_out_mms,
            "form_time_in_s": job.form_time_in_s,
            "form_time_out_s": job.form_time_out_s,
            "form_flowrate_in_mls": form_flowrate_in_mls,
            "form_flowrate_out_mls": form_flowrate_out_mls,
            "actual_weight_g": job.actual_weight,
            "job_duration_s": job.job_duration_s,
            "command_volume_ml": job.command_volume_ml,
            "formulator_pwm_in_percent": job.profile_pwm_in_percent,
            "formulator_pwm_out_percent": job.profile_pwm_out_percent,
            "formulator_fluid_profile": job.fluid_profile,
            "formulator_profile_token": job.formulator_profile_token,
            "calculated_step_size_percent": job.calculated_step_size_percent,
            "settle_time_used_ms": job.settle_time_used_ms,
        }
        new_df = pd.DataFrame([row])
        if self.results_path.exists():
            existing_df = pd.read_excel(self.results_path)
            combined_df = pd.concat([existing_df, new_df], ignore_index=True)
        else:
            combined_df = new_df
        combined_df.to_excel(self.results_path, index=False)
    
    def get_queue_status(self):
        """Get current queue statistics.
        
        Returns:
            dict: Queue info (queue_length, completed, failed, busy)
        """
        return {
            "queue_length": len(self.queue),
            "completed": self.completed,
            "failed": self.failed,
            "busy": self.busy,
        }


# =====================================================
# SYSTEM MONITOR
# =====================================================

async def monitor_system(dispenser):
    """Periodically print system status."""
    print("[MONITOR] Started")
    
    while True:
        status = dispenser.get_queue_status()
        formulator_status = dispenser.formulator.get_status()
        
        queue_str = f"Queue: {status['queue_length']}"
        stats_str = f"Completed: {status['completed']}, Failed: {status['failed']}"
        busy_str = f"Status: {'BUSY' if status['busy'] else 'IDLE'}"
        
        # Safely extract formulator values with defaults
        form_pos = formulator_status.get('POS', None)
        form_duty = formulator_status.get('DUTY', None)
        form_ready = formulator_status.get('READY', None)
        
        # Format position display
        if form_pos is not None:
            pos_str = f"{form_pos:.1f}%"
        else:
            pos_str = "?"
        
        # Format duty display
        if form_duty is not None:
            duty_str = f"{form_duty:.1f}%"
        else:
            duty_str = "?"
        
        # Format ready status
        if form_ready is not None:
            ready_str = "RDY" if form_ready else "COOL"
        else:
            ready_str = "?"
        
        print(
            f"[MON] {queue_str} | {stats_str} | {busy_str} | "
            f"Form: {pos_str} ({duty_str} duty, {ready_str})"
        )
        
        await asyncio.sleep(5)


# =====================================================
# MAIN
# =====================================================

async def main():
    """Initialize system and run main loop."""
    
    print("\n" + "="*60)
    print("Integrated Formulator/CNC/Balance System")
    print("="*60 + "\n")
    
    # Load positions from CSV (optional for future XY moves)
    positions_file = Path(__file__).parent / "positions.csv"
    if positions_file.exists():
        positions_df = pd.read_csv(positions_file)
    else:
        print(f"[WARNING] Positions file not found: {positions_file}")
        print("[WARNING] Using empty positions DataFrame")
        positions_df = pd.DataFrame()
    
    # -------- Initialize hardware drivers --------
    # print("[INIT] Initializing CNC...")
    # cnc = CNC(
    #     config_path=CNC_CONFIG_PATH,
    #     cnc_type=CNC_TYPE,
    #     positions=positions_df,
    #     virtual=CNC_VIRTUAL,
    #     serial_port=CNC_PORT,
    # )
    # cnc.open()
    # await asyncio.sleep(1)
    
    print("[INIT] Initializing balance...")
    balance = Balance(
        serial_port=BALANCE_PORT,
        balance_id="balance1",
        baud_rate=BALANCE_BAUD,
        timeout=BALANCE_TIMEOUT,
    )
    balance.open()
    await asyncio.sleep(1)
    
    print("[INIT] Initializing formulator...")
    formulator = FormulatorDriver(
        host=FORMULATOR_HOST,
        port=FORMULATOR_TCP_PORT,
        pump_timeout_s=2000.0,
        formulator_id=FORMULATOR_ID,
    )
    formulator.open()
    await asyncio.sleep(1)
    sync_all_fluid_profiles(formulator)
    
    # -------- Create dispenser and start processing --------
    dispenser = IntegratedDispenser(
        # cnc,
        formulator,
        balance,
    )
    
    try:
        # Start background tasks
        queue_task = asyncio.create_task(dispenser.process_queue())
        monitor_task = asyncio.create_task(monitor_system(dispenser))
        
        # Example: Queue some dispense jobs (each job can specify its own mode)
        print("[MAIN] Queueing dispense jobs...")
        
        # # #Queue a PRIMING job (no volume needed)
        # print("[MAIN] Queueing 1 PRIMING job")
        # dispenser.enqueue(operation_mode="PRIMING")

        #Queue NORMAL jobs (default mode, volume required, action="BOTH")
        # print("[MAIN] Queueing NORMAL jobs")
        # for i in range(3):
        #     dispenser.enqueue(3)

        # Test pattern: fill once for 5.2 mL, then dispense 0.2 mL at a time, 10 times,
        # each computed as a %-delta move from wherever the actuator currently sits.
        print("[MAIN] Queueing FILL (5.2 mL) + 10x DISPENSE (0.2 mL) test pattern")
        for u in range(3):
            dispenser.enqueue(volume_ml=5.2, action="FILL")
            for i in range(10):
                dispenser.enqueue(volume_ml=0.2, action="DISPENSE")
        
        # Keep running until queue is empty
        while dispenser.queue or dispenser.busy:
            await asyncio.sleep(1) 
        
        print("\n[MAIN] All jobs completed!")
        print(f"[MAIN] Completed: {dispenser.completed}, Failed: {dispenser.failed}")
        
        # Cancel background tasks
        queue_task.cancel()
        monitor_task.cancel()
        
    except Exception as e:
        print(f"\n[MAIN] Error: {e}")
    
    finally:
        print("\n[MAIN] Shutting down...")
        # cnc.close()
        balance.close()
        formulator.close()
        print("[MAIN] Done")


if __name__ == "__main__":
    print("[STARTUP] Starting async event loop...")
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n[STARTUP] Interrupted by user")
    except Exception as e:
        print(f"\n[STARTUP] Fatal error: {e}")
