"""====================================================
Formulator Dispenser Driver API (Pico W MicroPython)
Pico W + MG92B Valve + Actuonix L16 + DRV8871
====================================================

Driver-style API for powder formulation dispensing system with these main classes:
- Valve: Servo-controlled valve with positions (UP, CLOSED, THRU)
- Actuator: Linear actuator with position feedback and duty cycle protection
- CommandProcessor: Shared command parser/dispatcher (transport-agnostic)
- WiFiCommandHandler: Listens for PC commands over a WiFi TCP socket (primary transport)
- SerialCommandHandler: Listens for PC commands over USB serial (legacy/bench-debug fallback)

This firmware runs on the Pico W and receives commands from the Pi5 over WiFi.
Commands: VALVE:<pos>, PUMP:<volume>,<dir>[,<pwm>], POS, STATUS, READY?, RESET,
SETPROFILE:<name>,<IN|OUT>,<enabled>,<min>,<max>,<pause_ms>, SETFLUID:<name>

To use: Upload this file to the Pico (once, via mpremote), call main() at the end.
Only the WIFI_SSID/WIFI_PASSWORD/FORMULATOR_ID/STATIC_IP block below needs to differ
per physical unit -- everything else is shared firmware.
"""

import uasyncio as asyncio
from machine import Pin, PWM, ADC
import machine
import time
import sys
import network
import socket


# =====================================================
# USER-CONFIGURABLE PARAMETERS
# =====================================================
# These parameters should be redefined in your main code based on your 
# hardware setup and calibration data.

# Default calibration equation (inverted linear fit: percent = (volume_ml + offset) / slope)
# Calibrated from: 15% ≈ 1mL, 90% ≈ 7mL → 75% movement over 6mL range → 12.5%/mL slope
# Slope = 1/12.5 = 0.08; Offset = 15% × 0.08 = 1.2
# CALIBRATION_OFFSET = 0.5939  # Adjust based on your calibration
# CALIBRATION_SLOPE = 0.0771   # Adjust based on your calibration
CALIBRATION_OFFSET = 1.2     # Adjust based on your calibration
CALIBRATION_SLOPE = 0.08     # Adjust based on your calibration

# Default motor speed for dispensing operations (%)
DEFAULT_MOTOR_SPEED = 30

# Default home position for dispenser (% of actuator stroke)
DEFAULT_HOME_POSITION = 15.0

# Normal mode actuator window (% of stroke)
NORMAL_MIN_POSITION = 15.0
NORMAL_MAX_POSITION = 90.0

# Priming mode actuator window (% of stroke)
PRIMING_MIN_POSITION = 0.0
PRIMING_MAX_POSITION = 40.0

# Stepped motion profile by viscosity (step-size limits in ADC%)
DEFAULT_VISCOSITY_PROFILE = "BLUESILV12"  # Options: "WATER", "GLYCERIN", "BLUESIL", "BLUESILV12", "BLUESILV30"
INTER_STEP_PAUSE_MS = 3000
SETTLE_ONLY_ON_FINAL_STEP = True

# Dynamic stepping config: each move is split into the minimum number of steps
# such that every step is <= max_step_percent. If the resulting step size is
# below min_step_percent, the move is executed as a single step.
#These values could be possibly further optimized to improve throughput, I just chose safer values based on visual observation.
# VISCOSITY_IN_STEP_SIZE_LIMITS = {
#     "WATER": {"enabled": False, "min": 0.1, "max": 3.0, "pause_ms": 0},
#     "GLYCERIN": {"enabled": True, "min": 0.6, "max": 1.2, "pause_ms": 3000},
#     "BLUESIL": {"enabled": True, "min": 0.6, "max": 1.4, "pause_ms": 4000},
#     "BLUESILV12": {"enabled": True, "min": 0.6, "max": 3.6, "pause_ms": 5000},
#     # "BLUESILV12": {"enabled": True, "min": 0.6, "max": 1.2, "pause_ms": 5000},
#     "BLUESILV30": {"enabled": True, "min": 0.6, "max": 1.2, "pause_ms": 5000},
# }

# VISCOSITY_OUT_STEP_SIZE_LIMITS = {
#     "WATER": {"enabled": False, "min": 1.0, "max": 3.0, "pause_ms": 0},
#     "GLYCERIN": {"enabled": False, "min": 1.0, "max": 3.0, "pause_ms": 0},
#     "BLUESIL": {"enabled": True, "min": 0.6, "max": 42.0, "pause_ms": 4000},
#     "BLUESILV12": {"enabled": True, "min": 0.6, "max": 30.0, "pause_ms": 5000},
#     # "BLUESILV12": {"enabled": True, "min": 0.6, "max": 10.0, "pause_ms": 5000},
#     "BLUESILV30": {"enabled": True, "min": 0.6, "max": 8, "pause_ms": 5000},
# }

#### These values below are for short tubing tests

# VISCOSITY_IN_STEP_SIZE_LIMITS = {
#     "WATER": {"enabled": False, "min": 0.1, "max": 3.0, "pause_ms": 0},
#     "GLYCERIN": {"enabled": True, "min": 1.0, "max": 3.0, "pause_ms": 2000},
#     "BLUESIL": {"enabled": True, "min": 0.6, "max": 2.4, "pause_ms": 4000},
#     # "BLUESILV12": {"enabled": True, "min": 0.6, "max": 2.4, "pause_ms": 5000},
#     "BLUESILV12": {"enabled": True, "min": 0.6, "max": 1.2, "pause_ms": 10000},
#     "BLUESILV30": {"enabled": True, "min": 0.6, "max": 1.2, "pause_ms": 5000},
#     "SILTECH60": {"enabled": True, "min": 0.4, "max": 1.2, "pause_ms": 6000},
#     "BLUESILV60": {"enabled": True, "min": 0.4, "max": 1.0, "pause_ms": 14000},
# }

# VISCOSITY_OUT_STEP_SIZE_LIMITS = {
#     "WATER": {"enabled": False, "min": 1.0, "max": 3.0, "pause_ms": 0},
#     "GLYCERIN": {"enabled": False, "min": 1.0, "max": 3.0, "pause_ms": 0},
#     "BLUESIL": {"enabled": True, "min": 0.6, "max": 42.0, "pause_ms": 4000},
#     "BLUESILV12": {"enabled": True, "min": 0.6, "max": 20.0, "pause_ms": 5000},
#     # "BLUESILV12": {"enabled": True, "min": 0.6, "max": 10.0, "pause_ms": 5000},
#     "BLUESILV30": {"enabled": True, "min": 0.6, "max": 8, "pause_ms": 5000},
#     "SILTECH60": {"enabled": True, "min": 0.6, "max": 8, "pause_ms": 5000},
#     "BLUESILV60": {"enabled": True, "min": 0.6, "max": 8, "pause_ms": 7000},
# }

#### These values below are for pressurized tests with short tubing

VISCOSITY_IN_STEP_SIZE_LIMITS = {
    "WATER": {"enabled": False, "min": 0.1, "max": 3.0, "pause_ms": 0},
    "GLYCERIN": {"enabled": True, "min": 1.0, "max": 3.0, "pause_ms": 2000},
    "BLUESIL": {"enabled": True, "min": 0.6, "max": 2.4, "pause_ms": 4000},
    "BLUESILV12": {"enabled": True, "min": 0.3, "max": 3.6, "pause_ms": 3000},
    "BLUESILV30": {"enabled": True, "min": 0.6, "max": 1.2, "pause_ms": 5000},
    "SILTECH60": {"enabled": True, "min": 0.4, "max": 1.2, "pause_ms": 1500},
    "BLUESILV60": {"enabled": True, "min": 0.4, "max": 1.2, "pause_ms": 5000},
}

VISCOSITY_OUT_STEP_SIZE_LIMITS = {
    "WATER": {"enabled": False, "min": 1.0, "max": 3.0, "pause_ms": 0},
    "GLYCERIN": {"enabled": False, "min": 1.0, "max": 3.0, "pause_ms": 0},
    "BLUESIL": {"enabled": True, "min": 0.6, "max": 42.0, "pause_ms": 4000},
    "BLUESILV12": {"enabled": True, "min": 0.3, "max": 10.0, "pause_ms": 5000},
    # "BLUESILV12": {"enabled": True, "min": 0.6, "max": 10.0, "pause_ms": 5000},
    "BLUESILV30": {"enabled": True, "min": 0.6, "max": 8, "pause_ms": 5000},
    "SILTECH60": {"enabled": True, "min": 0.6, "max": 8, "pause_ms": 5000},
    "BLUESILV60": {"enabled": True, "min": 0.6, "max": 8, "pause_ms": 7000},
}

# Actuator hardware configuration (GPIO pins and basic parameters)
MOTOR_IN1_PIN = 4                    # DRV8871 IN1 control pin
MOTOR_IN2_PIN = 5                    # DRV8871 IN2 control pin
MOTOR_FEEDBACK_PIN = 28              # ADC feedback pin for position
MOTOR_PWM_FREQ = 1000                # PWM frequency in Hz
MOTOR_STROKE_MM = 50                # Actuator stroke length in mm
POSITION_TOLERANCE = 0.3             # Position tolerance in %
SETTLE_TIME_MS = 10000               # Time to settle in tolerance (ms)
MOVE_TIMEOUT_MS = 60000              # Maximum move duration (ms)
MAX_DUTY_CYCLE = 18.0                # Maximum duty cycle % in 5min window
ADC_FILTER_SIZE = 10                 # ADC sample count for filtering

#Valve Config

SERVO_PIN = 3                         # GPIO pin for servo signal

SERVO_POSITIONS = {
    "START": 68,      # Initial position
    "UP": 22,        # Intake position (open for fluid intake)
    "CLOSED": 68,    # Closed/neutral position
    "THRU": 112,     # Dispense position (through to output)
}

# =====================================================
# WIRELESS COMMUNICATION CONFIG
# =====================================================
# Per-unit identity: change these per physical Pico when running more than one.
# Addressing multiple Picos on one WiFi network is just "each gets its own IP" --
# no shared-bus addressing scheme needed. FORMULATOR_ID is only used for log lines.
FORMULATOR_ID = "formulator1"

WIFI_SSID = "AC-IoT-Cudy-18DC"
WIFI_PASSWORD = "31185981"

# Leave STATIC_IP as None to use DHCP (simplest -- pair with a DHCP reservation on
# your router/Pi5 AP keyed to this board's MAC address so the IP stays stable).
# Or set a tuple (ip, subnet, gateway, dns) to force a static IP on this Pico,
# e.g. ("192.168.4.101", "255.255.255.0", "192.168.4.1", "192.168.4.1").
STATIC_IP = None

FORMULATOR_TCP_PORT = 8888

# WiFi power-management: disabling this avoids a known CYW43439/MicroPython bug
# where the chip misses power-save rekeying and silently stops decrypting traffic.
WIFI_DISABLE_POWERSAVE = True

# Hardware watchdog: if the whole event loop wedges (known occasional issue with
# the Pico W WiFi stack), auto-reboot rather than staying silently frozen mid-move.
WATCHDOG_TIMEOUT_MS = 8000


# =====================================================
# VALVE CLASS
# =====================================================

class Valve:
    """Servo valve controller with smooth ramping between positions.
    
    Hardware: MG92B servo motor
    
    Args:
        servo_pin: GPIO pin connected to servo signal wire
        positions: Dict mapping position names to angles (degrees)
        pwm_freq: PWM frequency in Hz (default: 50)
        step_deg: Angle step size for smooth movement (default: 2)
        step_delay_ms: Delay between steps in milliseconds (default: 20)
    """

    def __init__(self, servo_pin=SERVO_PIN, positions=None, pwm_freq=50, step_deg=2, step_delay_ms=20):
        self.servo_pin = servo_pin
        self.pwm_freq = pwm_freq
        self.step_deg = step_deg
        self.step_delay_ms = step_delay_ms
        
        # Default valve positions (override if needed)
        self.positions = positions if positions is not None else SERVO_POSITIONS
        
        self.pwm = PWM(Pin(self.servo_pin))
        self.pwm.freq(self.pwm_freq)
        self.current_angle = None

    def _angle_to_duty(self, angle):
        """Convert servo angle (0-180°) to PWM duty cycle."""
        angle = max(0, min(180, angle))
        pulse_ms = 0.5 + (angle * 2 / 180.0)
        return int((pulse_ms / 20.0) * 65535)

    def write(self, angle):
        """Immediately set valve to specified angle."""
        self.pwm.duty_u16(self._angle_to_duty(angle))
        self.current_angle = angle
        print(f"[VALVE] Set to {angle}°")

    async def move(self, name):
        """Move valve smoothly to a named position.
        
        Args:
            name: Position name (e.g., "UP", "CLOSED", "THRU")
            
        Returns:
            bool: True if successful, False if position name not found
        """
        if name not in self.positions:
            print(f"[VALVE] Error: Unknown position '{name}'")
            return False

        target = self.positions[name]

        if self.current_angle is None:
            self.write(target)
            print(f"[VALVE] → {name} ({target}°)")
            return True

        step = self.step_deg if target > self.current_angle else -self.step_deg
        angle = self.current_angle

        while abs(angle - target) > abs(step):
            angle += step
            self.pwm.duty_u16(self._angle_to_duty(angle))
            await asyncio.sleep_ms(self.step_delay_ms)

        self.write(target)
        print(f"[VALVE] → {name} ({target}°)")
        return True


# =====================================================
# ACTUATOR CLASS
# =====================================================

class Actuator:
    """Linear actuator with ADC position feedback and duty cycle protection.

    Hardware: Actuonix L16 + DRV8871 motor driver
    
    All hardware and motion parameters are configured via module-level constants at the top of the file:
    MOTOR_IN1_PIN, MOTOR_IN2_PIN, MOTOR_FEEDBACK_PIN, MOTOR_PWM_FREQ, MOTOR_STROKE_MM,
    POSITION_TOLERANCE, SETTLE_TIME_MS, MOVE_TIMEOUT_MS, MAX_DUTY_CYCLE, ADC_FILTER_SIZE
    
    Args:
        in1_pin: GPIO pin for DRV8871 IN1 (default: MOTOR_IN1_PIN)
        in2_pin: GPIO pin for DRV8871 IN2 (default: MOTOR_IN2_PIN)
        feedback_pin: GPIO pin for ADC position feedback (default: MOTOR_FEEDBACK_PIN)
    """

    def __init__(self, in1_pin=None, in2_pin=None, feedback_pin=None):
        
        # Use module constants as defaults if not provided
        self.in1_pin = in1_pin if in1_pin is not None else MOTOR_IN1_PIN
        self.in2_pin = in2_pin if in2_pin is not None else MOTOR_IN2_PIN
        self.feedback_pin = feedback_pin if feedback_pin is not None else MOTOR_FEEDBACK_PIN
        self.motor_pwm_freq = MOTOR_PWM_FREQ
        self.stroke_mm = MOTOR_STROKE_MM
        
        # Limits will be set by set_operation_mode via _set_soft_limits_window
        self.operation_mode = "NORMAL"
        self.safe_min_percent = 0.0
        self.safe_max_percent = 100.0

        # Currently selected fluid/profile, settable at runtime from the Pi5 (SETFLUID)
        # instead of being hardcoded in firmware. Falls back to DEFAULT_VISCOSITY_PROFILE
        # when unset and no explicit profile is passed with a command.
        self.runtime_default_profile = None
        self.adc_max = 65535
        self.adc_min_usable = 0
        self.adc_max_usable = 65535
        
        # Motion parameters from module constants
        self.position_tolerance = POSITION_TOLERANCE
        self.settle_time_ms = SETTLE_TIME_MS
        self.move_timeout_ms = MOVE_TIMEOUT_MS
        
        # Duty cycle protection
        self.max_duty_cycle = MAX_DUTY_CYCLE
        self.duty_cycle_window_ms = 300000  # 5 minutes

        # Duty cycle tracking
        self.motor_on_time_ms = 0
        self.motor_on_start = None
        self.window_start = time.ticks_ms()
        self.is_motor_on = False
        
        # ADC filtering
        self.adc_filter_size = ADC_FILTER_SIZE
        
        # Initialize hardware
        self.in1 = PWM(Pin(self.in1_pin))
        self.in2 = PWM(Pin(self.in2_pin))
        self.in1.freq(self.motor_pwm_freq)
        self.in2.freq(self.motor_pwm_freq)

        self.adc = ADC(Pin(self.feedback_pin))
        self.adc_history = []
        self._last_adc_raw = None
        self._last_percent = None

        # Last move stats (To Calculate Speed)
        self.last_move_speed_mms = 0.0
        self.last_move_step_size_percent = None
        self.last_move_settle_time_ms = 0
        self.last_move_ok = None

        self.command_motor(0, 0)
        
        # Initialize limits for NORMAL mode
        self._set_soft_limits_window(NORMAL_MIN_POSITION, NORMAL_MAX_POSITION)
        
        print("[ACTUATOR] Initialized (motor stopped)")

    def _set_soft_limits_window(self, min_percent, max_percent):
        """Update active soft limit window and ADC bounds."""
        min_percent = max(0.0, min(100.0, float(min_percent)))
        max_percent = max(min_percent, min(100.0, float(max_percent)))

        self.safe_min_percent = min_percent
        self.safe_max_percent = max_percent
        self.adc_min_usable = int((self.safe_min_percent / 100.0) * self.adc_max)
        self.adc_max_usable = int((self.safe_max_percent / 100.0) * self.adc_max)

    def set_operation_mode(self, mode):
        """Set actuator operation mode and corresponding soft limits."""
        mode = str(mode).strip().upper()
        if mode == "NORMAL":
            self._set_soft_limits_window(NORMAL_MIN_POSITION, NORMAL_MAX_POSITION)
            self.operation_mode = "NORMAL"
        elif mode == "PRIMING":
            self._set_soft_limits_window(PRIMING_MIN_POSITION, PRIMING_MAX_POSITION)
            self.operation_mode = "PRIMING"
        else:
            raise ValueError("Invalid mode. Use NORMAL or PRIMING")

        print(
            f"[ACTUATOR] Mode={self.operation_mode} "
            f"(limits {self.safe_min_percent:.1f}% to {self.safe_max_percent:.1f}%)"
        )

    def _apply_soft_limits(self, percent):
        """Clamp percent to the usable ADC window."""
        target_adc = int((percent / 100.0) * self.adc_max)
        clamped_adc = max(self.adc_min_usable, min(self.adc_max_usable, target_adc))
        clamped_percent = (clamped_adc / self.adc_max) * 100.0
        return clamped_percent

    # -------- Duty cycle protection --------

    def get_duty_cycle(self):
        """Calculate current duty cycle percentage over the rolling window."""
        now = time.ticks_ms()
        window_elapsed = time.ticks_diff(now, self.window_start)

        if window_elapsed > self.duty_cycle_window_ms:
            self.window_start = now
            self.motor_on_time_ms = 0
            if self.is_motor_on and self.motor_on_start is not None:
                self.motor_on_start = now

        current_on_time = self.motor_on_time_ms
        if self.is_motor_on and self.motor_on_start is not None:
            current_on_time += time.ticks_diff(now, self.motor_on_start)

        return (current_on_time / self.duty_cycle_window_ms) * 100.0

    def can_run_motor(self):
        """Check if motor can run without exceeding duty cycle limit."""
        return self.get_duty_cycle() < self.max_duty_cycle

    def get_cooldown_time_s(self):
        """Calculate remaining cooldown time in seconds."""
        duty_cycle = self.get_duty_cycle()
        if duty_cycle < self.max_duty_cycle:
            return 0

        now = time.ticks_ms()
        window_elapsed = time.ticks_diff(now, self.window_start)

        required_off_time = self.motor_on_time_ms * ((100.0 / self.max_duty_cycle) - 1.0)
        elapsed_off_time = window_elapsed - self.motor_on_time_ms
        remaining_off_time = max(0, required_off_time - elapsed_off_time)
        return remaining_off_time / 1000.0

    # -------- Low-level I/O --------

    def _speed_to_duty(self, speed_percent):
        """Convert speed percentage to PWM duty cycle."""
        speed_percent = max(0, min(100, speed_percent))
        return int((speed_percent / 100.0) * 65535)

    def command_motor(self, direction, speed_percent):
        """Control motor direction and speed.
        
        Args:
            direction: 1 = extend, -1 = retract, 0 = stop
            speed_percent: Motor speed (0-100%)
        """
        duty = self._speed_to_duty(speed_percent)
        motor_will_be_on = (direction != 0 and speed_percent > 0)

        if motor_will_be_on and not self.is_motor_on:
            self.motor_on_start = time.ticks_ms()
            self.is_motor_on = True
        elif not motor_will_be_on and self.is_motor_on:
            if self.motor_on_start is not None:
                self.motor_on_time_ms += time.ticks_diff(time.ticks_ms(), self.motor_on_start)
                self.motor_on_start = None
            self.is_motor_on = False

        if direction > 0:
            self.in1.duty_u16(duty)
            self.in2.duty_u16(0)
        elif direction < 0:
            self.in1.duty_u16(0)
            self.in2.duty_u16(duty)
        else:
            self.in1.duty_u16(0)
            self.in2.duty_u16(0)

    def read_percent(self):
        """Read current position as percentage."""
        percent, _raw = self.read_position()
        return percent

    def read_position(self):
        """Read current position with ADC filtering.
        
        Returns:
            tuple: (percent, raw_adc) - position as % and raw ADC value
        """
        raw = self.adc.read_u16()

        self.adc_history.append(raw)
        if len(self.adc_history) > self.adc_filter_size:
            self.adc_history.pop(0)

        raw_f = int(sum(self.adc_history) / len(self.adc_history))
        percent = (raw_f / self.adc_max) * 100.0

        self._last_adc_raw = raw_f
        self._last_percent = percent
        return percent, raw_f

    # -------- Motion (continuous drive) --------

    async def move_to(self, target_percent, pwm_percent=None, settle_time_ms=None):
        """Move continuously to a target position percentage.

        Args:
            target_percent: Target position as % of stroke (0-100)
            pwm_percent: Motor speed as % (default: uses DEFAULT_MOTOR_SPEED)
            settle_time_ms: Optional settle override in milliseconds (for the final settle after reaching target band)

        Returns:
            bool: True if position reached and held within tolerance,
                  False on timeout or duty cycle limit
        """
        if pwm_percent is None:
            pwm_percent = DEFAULT_MOTOR_SPEED
        if settle_time_ms is None:
            settle_time_ms = self.settle_time_ms
        settle_time_ms = max(0, int(settle_time_ms))

        target = self._apply_soft_limits(target_percent)

        # Prime filter
        self.adc_history.clear()
        for _ in range(self.adc_filter_size):
            self.read_percent()

        start_time = time.ticks_ms()
        start_pos, start_adc = self.read_position()

        target_adc_est = int((target / 100.0) * self.adc_max)

        direction = 1 if target > start_pos else -1

        # If already at target, just settle
        if abs(start_pos - target) <= self.position_tolerance:
            self.command_motor(0, 0)
            await asyncio.sleep_ms(settle_time_ms)
            end_pos, end_adc = self.read_position()
            ok = abs(end_pos - target) <= self.position_tolerance
            self.last_move_speed_mms = 0.0
            self.last_move_ok = ok
            print(f"[ACTUATOR] Already at target: {end_pos:.1f}% (ADC {end_adc}) (ok={ok})")
            return ok

        print(
            f"[ACTUATOR] Moving {start_pos:.1f}% (ADC {start_adc}) "
            f"→ {target:.1f}% (ADC≈{target_adc_est}) at {pwm_percent}% PWM"
        )

        # Continuous drive
        self.command_motor(direction, pwm_percent)

        reached_time = None

        while True:
            now = time.ticks_ms()
            elapsed_ms = time.ticks_diff(now, start_time)

            if elapsed_ms > self.move_timeout_ms:
                self.command_motor(0, 0)
                current, current_adc = self.read_position()
                self.last_move_speed_mms = 0.0
                self.last_move_ok = False
                print(f"[ACTUATOR] WARNING: move timeout after {elapsed_ms}ms")
                print(f"[ACTUATOR] Final position: {current:.1f}% (ADC {current_adc}) (target: {target:.1f}%)")
                return False

            current, current_adc = self.read_position()

            # Overshoot safeguard: stop if we pass target in commanded direction,
            # then enter settle/final-check path (do not immediately fail).
            if reached_time is None:
                if direction > 0 and current > (target):
                    reached_time = now
                    self.command_motor(0, 0)
                    run_ms = max(1, elapsed_ms)
                    delta_percent = abs(current - start_pos)
                    delta_mm = (delta_percent / 100.0) * self.stroke_mm
                    self.last_move_speed_mms = delta_mm / (run_ms / 1000.0)
                    print(f"[ACTUATOR] Overshoot detected above target ({current:.1f}% > {target:.1f}%), stopping and settling")
                elif direction < 0 and current < (target):
                    reached_time = now
                    self.command_motor(0, 0)
                    run_ms = max(1, elapsed_ms)
                    delta_percent = abs(current - start_pos)
                    delta_mm = (delta_percent / 100.0) * self.stroke_mm
                    self.last_move_speed_mms = delta_mm / (run_ms / 1000.0)
                    print(f"[ACTUATOR] Overshoot detected below target ({current:.1f}% < {target:.1f}%), stopping and settling")

            error = abs(current - target)

            # First time entering tolerance -> stop and latch
            if reached_time is None and error <= self.position_tolerance:
                reached_time = now
                self.command_motor(0, 0)

                # Compute average speed
                run_ms = max(1, time.ticks_diff(reached_time, start_time))
                delta_percent = abs(current - start_pos)
                delta_mm = (delta_percent / 100.0) * self.stroke_mm
                self.last_move_speed_mms = delta_mm / (run_ms / 1000.0)
                self.last_move_step_size_percent = None  # Single move, no stepping
                self.last_move_settle_time_ms = settle_time_ms

                print(f"[ACTUATOR] Reached target band at {current:.1f}% (ADC {current_adc}) (settling)")

            # Physical limit guard (only if we have not already reached target band)
            if reached_time is None:
                if direction > 0 and current >= self.safe_max_percent:
                    self.command_motor(0, 0)
                    self.last_move_speed_mms = 0.0
                    self.last_move_ok = False
                    print(f"[ACTUATOR] Upper limit reached ({current:.1f}%, ADC {current_adc}), stopping")
                    return False
                if direction < 0 and current <= self.safe_min_percent:
                    self.command_motor(0, 0)
                    self.last_move_speed_mms = 0.0
                    self.last_move_ok = False
                    print(f"[ACTUATOR] Lower limit reached ({current:.1f}%, ADC {current_adc}), stopping")
                    return False

            # During settle window: wait full time, then check final position
            if reached_time is not None:
                if time.ticks_diff(now, reached_time) >= settle_time_ms:
                    # Check final position after full settle time
                    final_pos, final_adc = self.read_position()
                    final_error = abs(final_pos - target)
                    ok = final_error <= self.position_tolerance
                    self.last_move_ok = ok
                    if ok:
                        print(f"[ACTUATOR] Settled at {final_pos:.1f}% (ADC {final_adc}) (err {final_error:.2f}%)")
                    else:
                        print(f"[ACTUATOR] Final position {final_pos:.1f}% outside tolerance (err {final_error:.2f}%)")
                    return ok

            await asyncio.sleep_ms(20)

    async def move_to_stepped(self, target_percent, steps=None, pwm_percent=None,
                              inter_step_pause_ms=INTER_STEP_PAUSE_MS,
                              min_step_percent=1.0, max_step_percent=3.0,
                              settle_only_on_final_step=SETTLE_ONLY_ON_FINAL_STEP):
        """Move to target using segmented sub-moves.

        Useful for viscous fluids where commanded syringe motion leads fluid motion.
        Calculates total move speed from start to final position (excluding final settling time).
        """
        target = self._apply_soft_limits(target_percent)
        start_pos = self.read_percent()
        move_start_time = time.ticks_ms()  # Track total move start for speed calculation

        delta = abs(target - start_pos)

        # Optional fixed-step override for backward compatibility.
        if steps is not None:
            steps = max(1, int(steps))
        else:
            min_step_percent = max(0.4, float(min_step_percent))
            max_step_percent = max(min_step_percent, float(max_step_percent))

            if delta <= self.position_tolerance:
                steps = 1
            else:
                steps = int(delta / max_step_percent)
                if (delta - (steps * max_step_percent)) > 1e-9:
                    steps += 1
                steps = max(1, steps)
                planned_step = delta / steps
                if planned_step < min_step_percent:
                    steps = 1

        if steps == 1:
            return await self.move_to(target, pwm_percent=pwm_percent)

        planned_step = delta / steps if steps > 0 else delta
        print(
            f"[ACTUATOR] Stepped move: {start_pos:.1f}% → {target:.1f}% in {steps} steps "
            f"(~{planned_step:.2f}%/step, min={min_step_percent:.2f}%, max={max_step_percent:.2f}%)"
        )

        any_step_failed = False
        final_step_settle_ms = 0  # Track settle time of final step

        for step_idx in range(1, steps + 1):
            sub_target = start_pos + ((target - start_pos) * step_idx / steps)
            sub_target = self._apply_soft_limits(sub_target)
            is_final_step = (step_idx == steps)
            if settle_only_on_final_step and not is_final_step:
                step_settle_ms = 0
            else:
                step_settle_ms = self.settle_time_ms
            
            if is_final_step:
                final_step_settle_ms = step_settle_ms
            
            print(f"[ACTUATOR]   Step {step_idx}/{steps} -> {sub_target:.2f}%")
            ok = await self.move_to(sub_target, pwm_percent=pwm_percent, settle_time_ms=step_settle_ms)
            if not ok:
                print(f"[ACTUATOR] Step {step_idx}/{steps} failed")
                any_step_failed = True
            if step_idx < steps and inter_step_pause_ms > 0:
                await asyncio.sleep_ms(inter_step_pause_ms)

        final_pos, final_adc = self.read_position()
        final_error = abs(final_pos - target)
        final_ok = final_error <= self.position_tolerance

        # Calculate total move speed (start to finish, excluding final settling)
        move_end_time = time.ticks_ms()
        total_elapsed_ms = move_end_time - move_start_time - final_step_settle_ms
        if total_elapsed_ms < 1:
            total_elapsed_ms = 1
        
        total_delta_percent = abs(final_pos - start_pos)
        total_delta_mm = (total_delta_percent / 100.0) * self.stroke_mm
        self.last_move_speed_mms = total_delta_mm / (total_elapsed_ms / 1000.0)
        self.last_move_step_size_percent = planned_step
        self.last_move_settle_time_ms = final_step_settle_ms
        self.inter_step_settle_ms = inter_step_pause_ms
        
        print(f"[ACTUATOR] Stepped move total: {start_pos:.1f}% → {final_pos:.1f}% (distance {total_delta_mm:.2f}mm, time {total_elapsed_ms}ms excl. settle, speed {self.last_move_speed_mms:.4f} mm/s)")

        if final_ok:
            if any_step_failed:
                print(f"[ACTUATOR] Stepped move recovered by final step (final {final_pos:.2f}%, err {final_error:.2f}%)")
            return True

        print(f"[ACTUATOR] Stepped move failed at end (final {final_pos:.2f}%, target {target:.2f}%, err {final_error:.2f}%)")
        return False

    def set_default_profile(self, name):
        """Set the runtime-default fluid/viscosity profile (Pi5-controlled, not firmware-fixed)."""
        self.runtime_default_profile = str(name).strip().upper() if name else None
        print(f"[ACTUATOR] Default fluid profile -> {self.runtime_default_profile}")

    def set_viscosity_profile(self, direction, name, enabled, min_step, max_step, pause_ms):
        """Add or update a viscosity step-limit profile entry at runtime.

        Lets the Pi5 push/tune step-limit profiles (per fluid) without reflashing
        firmware -- the module-level tables below only serve as firmware defaults.
        """
        direction = str(direction).strip().upper()
        name = str(name).strip().upper()
        entry = {
            "enabled": bool(enabled),
            "min": max(0.01, float(min_step)),
            "max": max(float(min_step), float(max_step)),
            "pause_ms": max(0, int(pause_ms)),
        }
        if direction == "IN":
            VISCOSITY_IN_STEP_SIZE_LIMITS[name] = entry
        elif direction == "OUT":
            VISCOSITY_OUT_STEP_SIZE_LIMITS[name] = entry
        else:
            raise ValueError("direction must be IN or OUT")
        print(f"[ACTUATOR] Profile {name} {direction} updated: {entry}")

    def _resolve_step_size_limits(self, direction, viscosity_profile=None):
        """Resolve stepping enabled flag, min/max step size, and inter-step pause from profile."""
        profile = (viscosity_profile or self.runtime_default_profile or DEFAULT_VISCOSITY_PROFILE).strip().upper()
        if direction == "IN":
            limits = VISCOSITY_IN_STEP_SIZE_LIMITS.get(
                profile,
                VISCOSITY_IN_STEP_SIZE_LIMITS[DEFAULT_VISCOSITY_PROFILE],
            )
        else:
            limits = VISCOSITY_OUT_STEP_SIZE_LIMITS.get(
                profile,
                VISCOSITY_OUT_STEP_SIZE_LIMITS[DEFAULT_VISCOSITY_PROFILE],
            )

        steps_enabled = bool(limits.get("enabled", True))
        min_step = max(0.01, float(limits.get("min", 0.4)))
        max_step = max(min_step, float(limits.get("max", 3.0)))
        pause_ms = max(0, int(limits.get("pause_ms", INTER_STEP_PAUSE_MS)))
        return steps_enabled, min_step, max_step, pause_ms

    async def pump_vol(self, volume_ml, direction, calibration_offset=None, 
                       calibration_slope=None, home_position=None, pwm_percent=None,
                       viscosity_profile=None, in_steps=None, out_steps=None):
        """Move actuator to dispense specified volume using calibration.
        
        Args:
            volume_ml: Volume in mL to dispense
            direction: "IN" (draw in) or "OUT" (push out)
            calibration_offset: Calibration offset (default: CALIBRATION_OFFSET)
            calibration_slope: Calibration slope (default: CALIBRATION_SLOPE)
            home_position: Home position % (default: DEFAULT_HOME_POSITION)
            
        Returns:
            bool: True if move successful, False otherwise
        """
        if calibration_offset is None:
            calibration_offset = CALIBRATION_OFFSET
        if calibration_slope is None:
            calibration_slope = CALIBRATION_SLOPE
        if home_position is None:
            home_position = DEFAULT_HOME_POSITION
            
        # Calculate target position from volume using calibration equation
        target_percent = (volume_ml + calibration_offset) / calibration_slope
        target_percent = self._apply_soft_limits(target_percent)
        
        if direction == "IN":
            steps_enabled, min_step, max_step, pause_ms = self._resolve_step_size_limits("IN", viscosity_profile=viscosity_profile)
            print(
                f"[ACTUATOR] Pump {volume_ml} mL {direction} "
                f"(target: {target_percent:.1f}%, stepping: {steps_enabled}, min_step: {min_step:.2f}%, max_step: {max_step:.2f}%, pause_ms: {pause_ms})"
            )
            if in_steps is None and not steps_enabled:
                return await self.move_to(target_percent, pwm_percent=pwm_percent)
            return await self.move_to_stepped(
                target_percent,
                steps=in_steps,
                pwm_percent=pwm_percent,
                inter_step_pause_ms=pause_ms,
                min_step_percent=min_step,
                max_step_percent=max_step,
            )
        elif direction == "OUT":
            steps_enabled, min_step, max_step, pause_ms = self._resolve_step_size_limits("OUT", viscosity_profile=viscosity_profile)
            print(
                f"[ACTUATOR] Pump {volume_ml} mL {direction} "
                f"(dispense target: {home_position:.1f}%, stepping: {steps_enabled}, min_step: {min_step:.2f}%, max_step: {max_step:.2f}%, pause_ms: {pause_ms})"
            )
            if out_steps is None and not steps_enabled:
                return await self.move_to(home_position, pwm_percent=pwm_percent)
            return await self.move_to_stepped(
                home_position,
                steps=out_steps,
                pwm_percent=pwm_percent,
                inter_step_pause_ms=pause_ms,
                min_step_percent=min_step,
                max_step_percent=max_step,
            )
        else:
            print(f"[ACTUATOR] Error: Invalid direction '{direction}' (use 'IN' or 'OUT')")
            return False


# =====================================================
# DISPENSER CLASS -> Old so not really used anymore, but keeping for reference
# =====================================================

class Dispenser: 
    """Queue-based dispenser combining valve and actuator control.
    
    Args:
        valve: Valve instance
        actuator: Actuator instance
    """
    
    def __init__(self, valve, actuator):
        self.valve = valve
        self.actuator = actuator
        self.queue = []
        self.busy = False
        self.completed = 0

    def enqueue(self, volume_ml):
        """Add a dispense operation to the queue.

        Args:
            volume_ml: Volume in mL to dispense
        """
        vol = float(volume_ml)
        self.queue.append(vol)
        print(f"[DISPENSER] Queued {vol:.2f} mL (queue length: {len(self.queue)})")

    async def process_queue(self):
        """Process queued dispense operations continuously."""
        while True:
            if self.queue and not self.busy:
                if not self.actuator.can_run_motor():
                    cooldown = self.actuator.get_cooldown_time_s()
                    duty = self.actuator.get_duty_cycle()
                    print(f"[DISPENSER] ⚠ Duty cycle at {duty:.1f}% - waiting {cooldown:.0f}s")
                    await asyncio.sleep_ms(int(cooldown * 1000))
                    continue

                self.busy = True
                volume_ml = self.queue.pop(0)
                await self._dispense(volume_ml)
                self.completed += 1
                self.busy = False

            await asyncio.sleep_ms(500)

    async def _dispense(self, volume_ml):
        """Execute a single dispense operation.
        
        Args:
            volume_ml: Volume in mL to dispense
        """
        print(f"\n=== DISPENSE {volume_ml:.2f} mL ===")

        start_pos, start_adc = self.actuator.read_position()
        print(f"[DISPENSER] IN:  {start_pos:.2f}% (ADC {start_adc})")
        ok_in = await self.actuator.pump_vol(volume_ml, "IN")
        await asyncio.sleep(1)

        mid_pos, mid_adc = self.actuator.read_position()
        print(f"[DISPENSER] OUT: {mid_pos:.2f}% (ADC {mid_adc})")
        ok_out = await self.actuator.pump_vol(volume_ml, "OUT")
        await asyncio.sleep(1)

        print(f"=== DONE (in={ok_in}, out={ok_out}) ===\n")


# =====================================================
# COMMAND PROCESSOR (shared, transport-agnostic)
# =====================================================

class CommandProcessor:
    """Parses and dispatches PC commands against a Valve + Actuator.

    Shared by both transports (WiFi and legacy serial) so the command set only
    needs to be maintained in one place.

    Args:
        valve: Valve instance
        actuator: Actuator instance
    """

    def __init__(self, valve, actuator):
        self.valve = valve
        self.actuator = actuator
        self.last_error = None
        self.reset_requested = False

    async def _process_command(self, command):
        """Process a single command and return response."""

        if command.startswith("VALVE:"):
            position = command.split(":")[1].strip()
            ok = await self.valve.move(position)
            return "OK" if ok else "ERROR"
        
        elif command.startswith("PUMP:"):
            try:
                parts = command.split(":")[1].split(",")
                volume = float(parts[0].strip())
                direction = parts[1].strip().upper()
                pwm_percent = None
                viscosity_profile = None
                explicit_steps = None
                if len(parts) >= 3 and parts[2].strip() != "":
                    pwm_percent = float(parts[2].strip())
                if len(parts) >= 4 and parts[3].strip() != "":
                    token = parts[3].strip().upper()
                    try:
                        explicit_steps = int(token)
                    except ValueError:
                        viscosity_profile = token

                in_steps = explicit_steps if direction == "IN" else None
                out_steps = explicit_steps if direction == "OUT" else None

                ok = await self.actuator.pump_vol(
                    volume,
                    direction,
                    pwm_percent=pwm_percent,
                    viscosity_profile=viscosity_profile,
                    in_steps=in_steps,
                    out_steps=out_steps,
                )
                if ok:
                    return "OK"
                current, _ = self.actuator.read_position()
                if direction == "OUT":
                    if current <= (DEFAULT_HOME_POSITION + self.actuator.position_tolerance):
                        return "OK"
                if direction == "IN":
                    target_percent = (volume + CALIBRATION_OFFSET) / CALIBRATION_SLOPE
                    target_percent = self.actuator._apply_soft_limits(target_percent)
                    if abs(current - target_percent) <= self.actuator.position_tolerance:
                        return "OK"
                return "ERROR"
            except (IndexError, ValueError) as e:
                return "ERROR:Invalid format - use PUMP:<volume>,<IN|OUT>[,<pwm>][,<LOW|MEDIUM|HIGH|steps>]"

        elif command.startswith("MODE:"):
            try:
                mode = command.split(":", 1)[1].strip().upper()
                self.actuator.set_operation_mode(mode)
                return "OK"
            except Exception as e:
                return f"ERROR:{e}"

        elif command.startswith("STEP:"):
            try:
                parts = command.split(":", 1)[1].split(",")
                target_percent = float(parts[0].strip())
                direction = parts[1].strip().upper()
                pwm_percent = None
                viscosity_profile = None
                explicit_steps = None

                if len(parts) >= 3 and parts[2].strip() != "":
                    pwm_percent = float(parts[2].strip())
                if len(parts) >= 4 and parts[3].strip() != "":
                    token = parts[3].strip().upper()
                    try:
                        explicit_steps = int(token)
                    except ValueError:
                        viscosity_profile = token

                if direction not in ("IN", "OUT"):
                    return "ERROR:Direction must be IN or OUT"

                in_steps = explicit_steps if direction == "IN" else None
                out_steps = explicit_steps if direction == "OUT" else None
                steps_enabled, min_step, max_step, pause_ms = self.actuator._resolve_step_size_limits(
                    direction,
                    viscosity_profile=viscosity_profile,
                )

                target_percent = self.actuator._apply_soft_limits(target_percent)

                if ((direction == "IN" and in_steps is None) or (direction == "OUT" and out_steps is None)) and not steps_enabled:
                    ok = await self.actuator.move_to(target_percent, pwm_percent=pwm_percent)
                else:
                    ok = await self.actuator.move_to_stepped(
                        target_percent,
                        steps=in_steps if direction == "IN" else out_steps,
                        pwm_percent=pwm_percent,
                        inter_step_pause_ms=pause_ms,
                        min_step_percent=min_step,
                        max_step_percent=max_step,
                    )

                if ok:
                    return "OK"
                current, _ = self.actuator.read_position()
                if abs(current - target_percent) <= self.actuator.position_tolerance:
                    return "OK"
                return "ERROR"
            except (IndexError, ValueError):
                return "ERROR:Invalid format - use STEP:<target%>,<IN|OUT>[,<pwm>][,<LOW|MEDIUM|HIGH|steps>]"
        
        elif command == "POS":
            pos = self.actuator.read_percent()
            return f"POS:{pos:.2f}"
        
        elif command == "STATUS":
            pos, adc = self.actuator.read_position()
            duty = self.actuator.get_duty_cycle()
            can_run = self.actuator.can_run_motor()
            speed = self.actuator.last_move_speed_mms
            step_size = self.actuator.last_move_step_size_percent if self.actuator.last_move_step_size_percent is not None else 0
            settle_time = self.actuator.inter_step_settle_ms if hasattr(self.actuator, 'inter_step_settle_ms') else self.actuator.last_move_settle_time_ms
            return f"STATUS:POS={pos:.2f},ADC={adc},DUTY={duty:.1f},READY={can_run},SPEED={speed:.4f},STEP_SIZE={step_size:.4f},SETTLE_TIME={settle_time}"
        
        elif command == "READY?":
            ready = self.actuator.can_run_motor()
            cooldown = 0 if ready else self.actuator.get_cooldown_time_s()
            return f"READY:{ready},COOLDOWN={cooldown:.1f}"

        elif command.startswith("SETPROFILE:"):
            try:
                parts = command.split(":", 1)[1].split(",")
                name = parts[0].strip()
                direction = parts[1].strip().upper()
                enabled = parts[2].strip() in ("1", "true", "True", "TRUE")
                min_step = float(parts[3].strip())
                max_step = float(parts[4].strip())
                pause_ms = int(parts[5].strip())
                self.actuator.set_viscosity_profile(direction, name, enabled, min_step, max_step, pause_ms)
                return "OK"
            except (IndexError, ValueError) as e:
                return f"ERROR:Invalid format - use SETPROFILE:<name>,<IN|OUT>,<enabled 0|1>,<min>,<max>,<pause_ms> ({e})"

        elif command.startswith("SETFLUID:"):
            name = command.split(":", 1)[1].strip()
            self.actuator.set_default_profile(name if name else None)
            return "OK"

        elif command == "RESET":
            self.reset_requested = True
            return "OK"

        else:
            return f"ERROR:Unknown command '{command}'"


# =====================================================
# SERIAL COMMAND HANDLER (legacy / USB bench-debug fallback)
# =====================================================

class SerialCommandHandler(CommandProcessor):
    """Listen for commands over USB serial. Kept as a fallback debug transport --
    WiFiCommandHandler below is the primary transport used in main()."""

    async def listen(self):
        """Main loop: listen for commands and respond."""
        print("[SERIAL] Command handler ready (fallback transport)")
        print()

        while True:
            # Check for incoming data
            if sys.stdin in __import__('select').select([sys.stdin], [], [], 0)[0]:
                try:
                    line = sys.stdin.readline().strip()
                    if not line:
                        continue

                    response = await self._process_command(line)
                    print(response)
                    if self.reset_requested:
                        await asyncio.sleep_ms(200)
                        machine.reset()

                except Exception as e:
                    self.last_error = str(e)
                    print(f"ERROR:{e}")

            await asyncio.sleep_ms(50)


# =====================================================
# WIFI COMMAND HANDLER (primary transport)
# =====================================================

class WiFiCommandHandler(CommandProcessor):
    """Listen for commands from the Pi5 over a WiFi TCP socket.

    Holds one persistent client connection at a time (matching how the Pi5-side
    driver opens once and reuses the link, same as the previous serial workflow).
    A background task keeps re-associating with WiFi if the link drops.
    """

    def __init__(self, valve, actuator, ssid=WIFI_SSID, password=WIFI_PASSWORD,
                 port=FORMULATOR_TCP_PORT, static_ip=STATIC_IP):
        super().__init__(valve, actuator)
        self.ssid = ssid
        self.password = password
        self.port = port
        self.static_ip = static_ip
        self.wlan = network.WLAN(network.STA_IF)

    async def connect_wifi(self):
        """Bring up WiFi and block until associated (retries indefinitely)."""
        self.wlan.active(True)
        if WIFI_DISABLE_POWERSAVE:
            self.wlan.config(pm=0xa11140)  # disable power-save: avoids a known
            # CYW43439/MicroPython bug where the chip misses power-save rekeying
            # and silently stops decrypting traffic.
        if self.static_ip is not None:
            self.wlan.ifconfig(self.static_ip)

        while not self.wlan.isconnected():
            print(f"[WIFI] Connecting to '{self.ssid}'...")
            self.wlan.connect(self.ssid, self.password)
            for _ in range(20):  # ~10s per attempt
                if self.wlan.isconnected():
                    break
                await asyncio.sleep_ms(500)

        print(f"[WIFI] Connected. IP={self.wlan.ifconfig()[0]}")

    async def _reconnect_monitor(self):
        """Background task: re-associate if the WiFi link drops."""
        while True:
            if not self.wlan.isconnected():
                print("[WIFI] Link dropped, reconnecting...")
                await self.connect_wifi()
            await asyncio.sleep_ms(2000)

    async def listen(self):
        """Accept one client at a time and process newline-terminated commands."""
        await self.connect_wifi()
        asyncio.create_task(self._reconnect_monitor())

        addr = socket.getaddrinfo("0.0.0.0", self.port)[0][-1]
        server = socket.socket()
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(addr)
        server.listen(1)
        server.setblocking(False)

        print(f"[{FORMULATOR_ID}] WiFi command handler ready on port {self.port}")
        print("[WIFI] Available commands:")
        print("  VALVE:<UP|CLOSED|THRU>     - Move valve to position")
        print("  PUMP:<volume>,<IN|OUT>[,<pwm>][,<profile|steps>]")
        print("  MODE:<NORMAL|PRIMING>      - Set actuator mode and soft limits")
        print("  STEP:<target%>,<IN|OUT>[,<pwm>][,<profile|steps>]")
        print("  SETPROFILE:<name>,<IN|OUT>,<enabled>,<min>,<max>,<pause_ms>")
        print("  SETFLUID:<name>            - Set runtime-default fluid profile")
        print("  POS / STATUS / READY?      - Telemetry")
        print("  RESET                      - Reboot the Pico")
        print()

        while True:
            conn = None
            try:
                conn, remote_addr = server.accept()
            except OSError:
                pass  # no pending connection

            if conn is not None:
                conn.setblocking(False)
                print(f"[WIFI] Client connected: {remote_addr}")
                await self._handle_connection(conn)

            await asyncio.sleep_ms(50)

    async def _handle_connection(self, conn):
        """Service one client connection until it disconnects or errors out."""
        buf = b""
        try:
            while True:
                data = None
                try:
                    data = conn.recv(256)
                except OSError as e:
                    if e.args[0] in (11, 35):  # EAGAIN/EWOULDBLOCK -- no data yet
                        data = None
                    else:
                        raise

                if data:
                    buf += data
                    while b"\n" in buf:
                        line, buf = buf.split(b"\n", 1)
                        line = line.decode().strip()
                        if not line:
                            continue
                        try:
                            response = await self._process_command(line)
                        except Exception as e:
                            self.last_error = str(e)
                            response = f"ERROR:{e}"
                        conn.send((response + "\n").encode())
                        if self.reset_requested:
                            await asyncio.sleep_ms(200)  # let the response flush out
                            machine.reset()
                elif data == b"":
                    print("[WIFI] Client disconnected")
                    return

                await asyncio.sleep_ms(20)
        except OSError as e:
            print(f"[WIFI] Connection error: {e}")
        finally:
            try:
                conn.close()
            except OSError:
                pass


async def wdt_feeder(wdt):
    """Keep feeding the hardware watchdog for as long as the event loop is alive.

    If the whole loop wedges (known occasional Pico W WiFi stack issue), this task
    stops running too, the watchdog times out, and the board auto-reboots instead
    of sitting silently frozen mid-move.
    """
    while True:
        wdt.feed()
        await asyncio.sleep_ms(1000)


# =====================================================
# MAIN (Pico firmware entry point)
# =====================================================

async def main():
    """Initialize hardware and start command handler."""
    print(f"\n=== Formulator Firmware ({FORMULATOR_ID}) ===\n")
    await asyncio.sleep(1)

    # Initialize hardware
    valve = Valve()
    actuator = Actuator(in1_pin=4, in2_pin=5)  # Match hardware wiring
    actuator.set_operation_mode("NORMAL")
    wifi_handler = WiFiCommandHandler(valve, actuator)

    wdt = machine.WDT(timeout=WATCHDOG_TIMEOUT_MS)
    asyncio.create_task(wdt_feeder(wdt))

    await valve.move("CLOSED")

    # Main loop: listen for Pi5 commands over WiFi
    await wifi_handler.listen()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n[FIRMWARE] Shutdown")

