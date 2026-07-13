"""====================================================
Integrated Formulator/CNC/Balance Dispensing System - Pi5 Version
====================================================

Main orchestration file combining:
- CNC motion control (Genmitsu 4040 Pro via GRBL)
- Dispensing Formulator (Pico W via serial)
- Balance/scale readings (serial)

Queue-based workflow with async processing.

FILES USED:
============
Python Drivers (PC-side):
  • Drivers/cnc_api.py          - CNC motion control (G-code generation, GRBL communication)
  • Drivers/balance_api.py       - Balance/scale serial interface (read weight, tare, zero)
  • Drivers/formulator_driver.py - Pico formulator serial wrapper (send commands, receive status)

Pico Firmware (upload to Pico W):
  • Drivers/formulator_pico_api.py - MicroPython firmware for Pico (hardware control: valve, actuator)

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
    - Dispensing Formulator (Pico W): Connect via USB serial

2. Lock the tool (Formulator Pen) in place using the tool_changer.py script before running this file.

3. Before running main_dispense_system.py:
   Step A: Upload Pico firmware
   ───────────────────────────
   - Connect Pico W to PC via USB
   - Use Thonny IDE or mpremote to upload formulator_pico_api.py to Pico
   - Command example: mpremote cp Drivers/formulator_pico_api.py :main.py
   - Restart Pico (or it will auto-start main.py)
   
   Step B: Create/update config files
    ──────────────────────────────────
    - Update config.yaml with CNC settings and serial device path
    - Create positions.csv with named locations (VIAL_1, VIAL_2, WB, etc)
    - Update hardware serial settings below if any USB device changes
   
   Step C: Verify serial connections
   ─────────────────────────────────
   - Test CNC: Open terminal, verify GRBL responds to '?'
   - Test Balance: Run balance_api.py separately, verify tare/read works
   - Test Pico: Check serial terminal for "[SERIAL] Command handler ready"

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

FORMULATOR_PORT = "/dev/serial/by-id/usb-MicroPython_Board_in_FS_mode_e66368254f52132d-if00"
FORMULATOR_BAUD = 115200

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
    },
    "GLYCERIN": {
        "formulator_profile": "GLYCERIN",
        "calibration_offset": -0.0206,
        "calibration_slope": 1.2165,
        "pwm_in_percent": 30,
        "pwm_out_percent": 30,
        "relief_enabled": False,
    },
    "BLUESIL": {
        "formulator_profile": "BLUESIL",
        "calibration_offset": -0.2001,
        "calibration_slope": 0.9446,
        "pwm_in_percent": 25,
        "pwm_out_percent": 30,
        "relief_enabled": True,
    },
    "BLUESILV12": {
        "formulator_profile": "BLUESILV12",
        "calibration_offset": 0.2051,
        "calibration_slope": 0.7581,
        "pwm_in_percent": 25,
        "pwm_out_percent": 25,
        "relief_enabled": True,
    },
    "BLUESILV30": {
        "formulator_profile": "BLUESILV30",
        "calibration_offset": 0,
        "calibration_slope": 1,
        "pwm_in_percent": 25,
        "pwm_out_percent": 25,
        "relief_enabled": True,
    },
    "SILTECH60": {
        "formulator_profile": "SILTECH60",
        "calibration_offset": -0.0292,
        "calibration_slope": 0.9427,
        "pwm_in_percent": 25,
        "pwm_out_percent": 25,
        "relief_enabled": True,
    },
     "BLUESILV60": {
        "formulator_profile": "BLUESILV60",
        "calibration_offset": -0.0724,
        "calibration_slope": 0.9307,
        "pwm_in_percent": 25,
        "pwm_out_percent": 25,
        "relief_enabled": True,
    },
}

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

    def __init__(self, volume_ml=None, container_id=None, location=None, fluid_profile=None, operation_mode=None):
        self.location = location
        self.volume_ml = volume_ml
        self.container_id = container_id or f"VIAL_{time.time()}"
        self.fluid_profile = fluid_profile or FORMULATOR_FLUID_PROFILE
        self.operation_mode = operation_mode or "NORMAL"  # NORMAL or PRIMING
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
    
    Workflow for NORMAL mode (default):
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
    
    Workflow for PRIMING mode:
    - 4 cycles of: Valve UP -> Move IN to target% -> Wait 7s -> Valve THRU -> Move OUT to target% -> Wait 7s
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
        
    def enqueue(self, volume_ml=None, container_id=None, location=None, fluid_profile=None, operation_mode=None):
        """Add a dispense job to the queue.
        
        Args:
            volume_ml: Volume to dispense in mL (required for NORMAL, ignored for PRIMING)
            container_id: Optional identifier for this job
            location: Optional CNC position name (reserved for future XY moves)
            fluid_profile: Optional profile override (e.g., WATER, GLYCERIN, BLUESIL)
            operation_mode: Optional mode ("NORMAL" or "PRIMING", default: "NORMAL")
        """
        job = DispenseJob(volume_ml, container_id, location, fluid_profile=fluid_profile, operation_mode=operation_mode)
        self.queue.append(job)
        location_note = f" at {location}" if location else ""
        vol_str = f"{volume_ml} mL" if volume_ml is not None else "N/A (PRIMING)"
        print(f"[QUEUE] Added: {job.container_id} ({vol_str}{location_note}, fluid={job.fluid_profile}, mode={job.operation_mode})")
    
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
            target_percent = self._to_firmware_target_percent(command_volume_ml)
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
        print(f"[DISPENSE] Operation mode: {job.operation_mode}")
        
        # Only print volume calibration for NORMAL mode
        if job.operation_mode == "NORMAL":
            print(
                f"[DISPENSE] Profile cal: command = desired*slope + offset | offset={profile_offset:.4f}, slope={profile_slope:.4f} | "
                f"Target={target_percent:.2f}% | Cmd vol={command_volume_ml:.4f} mL | PWM IN/OUT={pwm_in}/{pwm_out}%"
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
            
            if job.operation_mode == "NORMAL":
                # ========== NORMAL DISPENSING MODE ==========
                
                # -------- STEP 1: Move valve to UP position --------
                print("[DISPENSE] Step 1: Moving valve to UP position")
                self.formulator.valve_move("UP")
                await asyncio.sleep(1)

                # # -------- STEP 1: Move to loading height --------
                # print("[DISPENSE] Step 1: Moving to loading height")
                # self.cnc.move_to_point(x=None, y=None, z=Z_LOAD, speed=Z_MOVE_SPEED)
                # await asyncio.sleep(1)
                
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

                # # -------- STEP 3: Move to dispense height --------
                # print("[DISPENSE] Step 3: Moving to dispense height")
                # self.cnc.move_to_point(x=None, y=None, z=Z_DISPENSE, speed=Z_MOVE_SPEED)
                # await asyncio.sleep(5)

                # -------- STEP 3: Apply pressure relief if enabled --------
                if job.relief_enabled and job.volume_ml > 0.3:
                    print("[DISPENSE] Step 3a: Applying pressure relief (CLOSED → OUT 1.2% → THRU)")
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
                    # Move valve to THRU
                    self.formulator.valve_move("THRU")
                    await asyncio.sleep(0.5)
                else:
                    # Just move valve to THRU
                    print("[DISPENSE] Step 3: Moving valve to THRU position")
                    self.formulator.valve_move("THRU")
                    await asyncio.sleep(1)

                # -------- STEP 4: Record actuator position before OUT --------
                try:
                    job.form_percent_pre_dispense = self.formulator.get_position()
                    if job.form_percent_pre_dispense is None:
                        print("[DISPENSE] Form% before OUT: N/A")
                    else:
                        print(f"[DISPENSE] Form% before OUT: {job.form_percent_pre_dispense:.2f}%")
                except Exception as e:
                    print(f"[DISPENSE] WARNING: Could not read Form% before OUT: {e}")
                    job.form_percent_pre_dispense = None


                # -------- STEP 5: Tare balance --------
                print("[DISPENSE] Step 5: Taring balance")
                self.balance.tare()
                await asyncio.sleep(2)

                # -------- STEP 6: Dispense fluid (OUT) --------
                print("[DISPENSE] Step 6: Dispensing fluid (OUT)")
                time_out_start = time.time()
                ok_out = self.formulator.pump_volume(
                    command_volume_ml,
                    "OUT",
                    pwm_percent=pwm_out,
                    viscosity_profile=profile_token,
                )
                job.form_time_out_s = time.time() - time_out_start
                if not ok_out:
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

                # -------- STEP 7: Move valve to CLOSED position --------
                print("[DISPENSE] Step 7: Moving valve to CLOSED position")
                self.formulator.valve_move("CLOSED")
                await asyncio.sleep(1)

                # # -------- STEP 8: Remove fluid from tip post dispense --------
                # print("[DISPENSE] Step 8: Removing fluid from tip")
                # self.cnc.move_to_point(x=None, y=13, z=None, speed=200)
                # await asyncio.sleep(3)
                # self.cnc.move_to_point(x=None, y=0, z=None, speed=200)
                # await asyncio.sleep(1)
                
                # -------- STEP 9: Read final weight --------
                print("[DISPENSE] Step 9: Reading final weight")
                weight = self.balance.read_weight(settle_time=10.0)
                job.actual_weight = weight
                print(f"[DISPENSE] Target: {job.volume_ml} mL, Actual: {weight:.3f} g")

                # -------- STEP 10: Record actuator position after dispense --------
                try:
                    job.form_percent_post_dispense = self.formulator.get_position()
                    if job.form_percent_post_dispense is None:
                        print("[DISPENSE] Form% after OUT: N/A")
                    else:
                        print(f"[DISPENSE] Form% after OUT: {job.form_percent_post_dispense:.2f}%")
                except Exception as e:
                    print(f"[DISPENSE] WARNING: Could not read Form% after OUT: {e}")
                    job.form_percent_post_dispense = None
                
                job.status = "COMPLETED"

            elif job.operation_mode == "PRIMING":
                # ========== PRIMING MODE ==========
                # 2 cycles of moved_to IN target, wait 7s, move_to OUT target, wait 7s
                
                print(f"[DISPENSE] PRIMING MODE: 2 cycles")
                print(f"[DISPENSE] IN target={PRIMING_IN_TARGET_PERCENT:.2f}%, OUT target={PRIMING_OUT_TARGET_PERCENT:.2f}%")
                
                for cycle in range(1, 2):
                    print(f"\n[DISPENSE] === PRIMING CYCLE {cycle}/2 ===")
                    
                    # Move valve to UP before IN
                    print(f"[DISPENSE] Cycle {cycle}: Moving valve to UP")
                    self.formulator.valve_move("UP")
                    await asyncio.sleep(1)
                    # self.cnc.move_to_point(x=None, y=None, z=Z_LOAD, speed=Z_MOVE_SPEED)
                    # await asyncio.sleep(2)

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
                    # self.cnc.move_to_point(x=None, y=None, z=Z_DISPENSE, speed=Z_MOVE_SPEED)
                    # await asyncio.sleep(2)
                    
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
                
                # Move valve to UP before IN
                print(f"[DISPENSE] Moving valve to UP")
                self.formulator.valve_move("UP")
                await asyncio.sleep(1)
                # self.cnc.move_to_point(x=None, y=None, z=Z_LOAD, speed=Z_MOVE_SPEED)
                # await asyncio.sleep(3)
                    
                # Move IN to target percent (stepped)
                print(f"[DISPENSE] Moving IN to {DEFAULT_HOME_POSITION:.2f}%")
                ok_in = self.formulator.move_to_percent_stepped(
                    DEFAULT_HOME_POSITION,
                    "IN",
                    pwm_percent=pwm_in,
                    viscosity_profile=profile_token,
                )
                if not ok_in:
                    print(f"[DISPENSE] WARNING: IN move failed")
                    
                # Wait 4 seconds
                print(f"[DISPENSE] Waiting 4s...")
                await asyncio.sleep(4)
                # self.cnc.move_to_point(x=None, y=None, z=Z_DISPENSE, speed=Z_MOVE_SPEED)
                # await asyncio.sleep(2)

                # Close valve at end
                self.formulator.valve_move("CLOSED")
                await asyncio.sleep(90) #Long wait to ensure that the fluid has rested properly before next dispense, especially for high viscosity fluids like Siltech60.
                
                print("\n[DISPENSE] PRIMING MODE: All 4 cycles completed")
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
        serial_port=FORMULATOR_PORT,
        baud_rate=FORMULATOR_BAUD,
        pump_timeout_s=2000.0,
    )
    formulator.open()
    await asyncio.sleep(1)
    
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
        
        #Queue a PRIMING job (no volume needed)
        print("[MAIN] Queueing 1 PRIMING job")
        dispenser.enqueue(operation_mode="PRIMING")
        
        #Queue NORMAL jobs (default mode, volume required)
        print("[MAIN] Queueing NORMAL jobs")
        for i in range(10): 
            dispenser.enqueue(0.15)
        
        
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
