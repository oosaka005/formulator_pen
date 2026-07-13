"""====================================================
Formulator Driver - Python API for communicating with Pico formulator
====================================================

Wrapper for serial communication with the Pico running formulator_api.py firmware.
Provides a clean interface to control valve, actuator, and monitor system status.
"""

import serial
import time


class FormulatorDriver:
    """Python driver for Pico-based formulator system.
    
    Args:
        serial_port: Serial device path (for example "/dev/serial/by-id/...")
        baud_rate: Serial baud rate (default: 115200)
        timeout: Serial read timeout in seconds (default: 2.0)
    """
    
    def __init__(self, serial_port, baud_rate=115200, timeout=2.0, pump_timeout_s=700.0):
        self.serial_port = serial_port
        self.baud_rate = baud_rate
        self.timeout = timeout
        self.pump_timeout_s = float(pump_timeout_s)
        self.connection = None
        self.formulator_id = "formulator1"

    def open(self):
        """Open serial connection to Pico."""
        if self.connection and self.connection.is_open:
            return
        self.connection = serial.Serial(
            port=self.serial_port,
            baudrate=self.baud_rate,
            timeout=self.timeout
        )
        time.sleep(2)  # Wait for Pico to initialize
        print(f"[{self.formulator_id}] Connected to {self.serial_port}")

    def close(self):
        """Close serial connection."""
        if self.connection and self.connection.is_open:
            self.connection.close()
        print(f"[{self.formulator_id}] Disconnected")

    def _ensure_open(self):
        """Check connection is open."""
        if not self.connection or not self.connection.is_open:
            raise RuntimeError(f"[{self.formulator_id}] Not connected. Call open() first.")

    def _send_command(self, command, timeout_s=3.0):
        """Send command to Pico and wait for response.
        
        Args:
            command: Command string (e.g., "VALVE:CLOSED")
            timeout_s: Max seconds to wait for matching response
            
        Returns:
            str: Response from Pico (last line received)
        """
        self._ensure_open()

        # Clear stale buffered lines from previous commands/logging
        while self.connection.in_waiting > 0:
            self.connection.read(self.connection.in_waiting)

        self.connection.write((command + "\n").encode())

        # Determine expected response prefix for this command
        if command == "POS":
            expected_prefixes = ("POS:", "ERROR")
        elif command == "STATUS":
            expected_prefixes = ("STATUS:", "ERROR")
        elif command == "READY?":
            expected_prefixes = ("READY:", "ERROR")
        else:
            expected_prefixes = ("OK", "ERROR")

        start_time = time.time()
        last_nonempty = ""

        while time.time() - start_time < timeout_s:
            line = self.connection.readline().decode(errors="ignore").strip()
            if not line:
                continue

            last_nonempty = line

            # Ignore firmware debug/info lines and keep waiting for actual response
            if line.startswith("["):
                continue

            if line.startswith(expected_prefixes):
                if line.startswith("ERROR"):
                    print(f"[{self.formulator_id}] {line}")
                return line

        if last_nonempty:
            return last_nonempty
        return ""
    
    def _send_command_wait_for_completion(self, command, timeout_s=None):
        """Send command and wait for completion (for async operations like pump).
        
        Args:
            command: Command string (e.g., "PUMP:3,IN")
            timeout_s: Maximum time to wait for completion
            
        Returns:
            bool: True if successful, False if failed or timed out
        """
        self._ensure_open()
        if timeout_s is None:
            timeout_s = self.pump_timeout_s
        
        # Clear any stale data in the buffer before sending new command
        while self.connection.in_waiting > 0:
            self.connection.read(self.connection.in_waiting)
        
        print(f"[{self.formulator_id}] Sending: {command}")
        self.connection.write((command + "\n").encode())
        self.connection.flush()
        
        start_time = time.time()
        last_progress_log = start_time
        
        while time.time() - start_time < timeout_s:
            if self.connection.in_waiting > 0:
                response = self.connection.readline().decode().strip()
                if response:
                    # Only print important messages, not every status update
                    if response == "OK":
                        return True
                    elif response == "ERROR":
                        print(f"[{self.formulator_id}] ERROR received")
                        return False
                    elif response.startswith("ERROR:"):
                        print(f"[{self.formulator_id}] {response}")
                        return False
                    else:
                        print(f"[{self.formulator_id}] {response}")
            else:
                now = time.time()
                if now - last_progress_log >= 3:
                    # elapsed = int(now - start_time)
                    # print(f"[{self.formulator_id}] Waiting for completion... ({elapsed}s)")
                    last_progress_log = now
            
            time.sleep(0.1)
        
        # Timed out
        print(f"[{self.formulator_id}] Timeout waiting for response")
        return False

    def valve_move(self, position):
        """Move valve to specified position.
        
        Args:
            position: "UP", "CLOSED", or "THRU"
            
        Returns:
            bool: True if successful, False otherwise
        """
        response = self._send_command(f"VALVE:{position}")
        ok = response == "OK"
        if ok:
            print(f"[{self.formulator_id}] Valve → {position}")
        return ok

    def set_operation_mode(self, mode):
        """Set firmware operation mode.

        Args:
            mode: "NORMAL" or "PRIMING"

        Returns:
            bool: True if successful
        """
        mode = str(mode).strip().upper()
        response = self._send_command(f"MODE:{mode}")
        ok = response == "OK"
        if ok:
            print(f"[{self.formulator_id}] Mode → {mode}")
        return ok

    def move_to_percent_stepped(self, target_percent, direction, pwm_percent=None,
                                viscosity_profile=None, steps=None, timeout_s=None):
        """Move actuator to explicit target % using firmware stepped motion path.

        Args:
            target_percent: Target position in percent (0-100, clamped by firmware mode limits)
            direction: "IN" or "OUT" (selects viscosity step profile)
            pwm_percent: Optional motor speed percent
            viscosity_profile: Optional profile token (e.g., WATER, GLYCERIN, BLUESIL)
            steps: Optional explicit number of steps override
            timeout_s: Optional command completion timeout

        Returns:
            bool: True when movement completes
        """
        direction = str(direction).strip().upper()
        if direction not in ("IN", "OUT"):
            raise ValueError("direction must be 'IN' or 'OUT'")

        token = ""
        if steps is not None:
            token = str(int(steps))
        elif viscosity_profile is not None:
            token = str(viscosity_profile).strip().upper()

        if pwm_percent is None:
            if token:
                command = f"STEP:{float(target_percent):.4f},{direction},,{token}"
            else:
                command = f"STEP:{float(target_percent):.4f},{direction}"
        else:
            if token:
                command = f"STEP:{float(target_percent):.4f},{direction},{float(pwm_percent):.2f},{token}"
            else:
                command = f"STEP:{float(target_percent):.4f},{direction},{float(pwm_percent):.2f}"

        ok = self._send_command_wait_for_completion(command, timeout_s=timeout_s)

        speed_note = f" @ {pwm_percent}%" if pwm_percent is not None else ""
        profile_note = f" ({viscosity_profile})" if viscosity_profile else ""
        if ok:
            print(f"[{self.formulator_id}] ✓ Stepped move complete: {target_percent}% {direction}{speed_note}{profile_note}")
        else:
            print(f"[{self.formulator_id}] ✗ Stepped move failed: {target_percent}% {direction}{speed_note}{profile_note}")

        return ok

    def pump_volume(self, volume_ml, direction, pwm_percent=None, viscosity_profile=None,
                    home_position=10.0, home_tolerance=2.0, timeout_s=None):
        """Pump specified volume and wait for completion.
        
        Args:
            volume_ml: Volume in mL to pump
            direction: "IN" (draw in) or "OUT" (push out)
            pwm_percent: Optional PWM percent (0-100) for actuator speed
            viscosity_profile: Optional fluid profile token (e.g., "WATER", "GLYCERIN", "BLUESIL")
            home_position: Not used (kept for API compatibility)
            home_tolerance: Not used (kept for API compatibility)
            timeout_s: Optional override for command completion timeout in seconds
            
        Returns:
            bool: True when movement completes (regardless of settle status)
        """
        if viscosity_profile is not None:
            viscosity_profile = str(viscosity_profile).strip().upper()
            if pwm_percent is None:
                command = f"PUMP:{volume_ml},{direction},,{viscosity_profile}"
            else:
                command = f"PUMP:{volume_ml},{direction},{pwm_percent},{viscosity_profile}"
        else:
            if pwm_percent is None:
                command = f"PUMP:{volume_ml},{direction}"
            else:
                command = f"PUMP:{volume_ml},{direction},{pwm_percent}"
        
        # Send pump command and wait for completion
        # Tolerance is used by firmware as stopping point; once stopped = success
        ok = self._send_command_wait_for_completion(command, timeout_s=timeout_s)
        
        speed_note = f" @ {pwm_percent}%" if pwm_percent is not None else ""
        profile_note = f" ({viscosity_profile})" if viscosity_profile else ""
        if ok:
            print(f"[{self.formulator_id}] ✓ Pump complete: {volume_ml} mL {direction}{speed_note}{profile_note}")
        else:
            print(f"[{self.formulator_id}] ✗ Pump failed: {volume_ml} mL {direction}{speed_note}{profile_note}")
        
        return ok

    def get_position(self):
        """Get current actuator position as percentage.
        
        Returns:
            float: Position as % of stroke, or None on error
        """
        response = self._send_command("POS")
        try:
            if response.startswith("POS:"):
                return float(response.split(":")[1])
        except (IndexError, ValueError):
            pass
        return None

    def get_status(self):
        """Get system status.
        
        Returns:
            dict: Status info (pos, adc, duty, ready), or empty dict on error
        """
        response = self._send_command("STATUS")
        status = {}
        
        try:
            if response.startswith("STATUS:"):
                parts = response.split(":")[1].split(",")
                for part in parts:
                    key, val = part.split("=")
                    try:
                        status[key] = float(val)
                    except ValueError:
                        status[key] = val == "True"
        except (IndexError, ValueError):
            pass
        
        return status

    def is_ready(self):
        """Check if system is ready for commands (duty cycle OK).
        
        Returns:
            bool: True if ready, False if in cooldown
        """
        response = self._send_command("READY?")
        try:
            if response.startswith("READY:"):
                ready_str = response.split(":")[1].split(",")[0]
                return ready_str == "True"
        except (IndexError, ValueError):
            pass
        return False

    def get_cooldown_time(self):
        """Get remaining cooldown time if in duty cycle protection.
        
        Returns:
            float: Cooldown time in seconds, or 0 if ready
        """
        response = self._send_command("READY?")
        try:
            if response.startswith("READY:"):
                cooldown_str = response.split(":")[1].split(",")[1].split("=")[1]
                return float(cooldown_str)
        except (IndexError, ValueError):
            pass
        return 0.0
