"""====================================================
Formulator Driver - Python API for communicating with Pico formulator
====================================================

Wrapper for WiFi (TCP socket) communication with the Pico W running
pico_api_step.py firmware. Provides a clean interface to control valve,
actuator, and monitor system status, and to push runtime-tunable config
(viscosity step-limit profiles, currently selected fluid) down to the Pico
without ever needing to reflash firmware.

To control multiple Picos, just instantiate one FormulatorDriver per Pico
(each with its own host/port) -- addressing is handled by each Pico having
its own IP on the WiFi network, no shared-bus addressing scheme needed.
"""

import socket
import time


class FormulatorDriver:
    """Python driver for a Pico W-based formulator system, over WiFi.

    Args:
        host: Pico's IP address or hostname on the WiFi network
        port: TCP port the Pico's WiFiCommandHandler is listening on (default: 8888)
        timeout: Socket read timeout in seconds (default: 2.0)
        pump_timeout_s: Max seconds to wait for a pump/step move to complete
        formulator_id: Label used in log lines (useful when running multiple Picos)
    """

    def __init__(self, host, port=8888, timeout=2.0, pump_timeout_s=700.0, formulator_id="formulator1"):
        self.host = host
        self.port = port
        self.timeout = timeout
        self.pump_timeout_s = float(pump_timeout_s)
        self.connection = None
        self._recv_buf = b""
        self.formulator_id = formulator_id

    def open(self):
        """Open TCP connection to the Pico."""
        if self.connection:
            return
        sock = socket.create_connection((self.host, self.port), timeout=self.timeout)
        self.connection = sock
        self._recv_buf = b""
        time.sleep(0.5)  # brief settle after connect
        print(f"[{self.formulator_id}] Connected to {self.host}:{self.port}")

    def close(self):
        """Close TCP connection."""
        if self.connection:
            try:
                self.connection.close()
            except OSError:
                pass
            self.connection = None
        print(f"[{self.formulator_id}] Disconnected")

    def _ensure_open(self):
        """Check connection is open."""
        if not self.connection:
            raise RuntimeError(f"[{self.formulator_id}] Not connected. Call open() first.")

    def _readline(self, deadline):
        """Read one newline-terminated line from the socket, buffering partial reads.

        Args:
            deadline: time.time() value after which to give up

        Returns:
            str: decoded line (without newline), or "" on timeout/disconnect
        """
        while b"\n" not in self._recv_buf:
            remaining = deadline - time.time()
            if remaining <= 0:
                return ""
            self.connection.settimeout(max(0.05, remaining))
            try:
                chunk = self.connection.recv(256)
            except socket.timeout:
                return ""
            except OSError:
                return ""
            if not chunk:
                # Peer closed the connection -- drop the stale socket so the
                # next command raises clearly instead of hanging forever.
                self.connection = None
                return ""
            self._recv_buf += chunk

        line, self._recv_buf = self._recv_buf.split(b"\n", 1)
        return line.decode(errors="ignore").strip()

    def _send_command(self, command, timeout_s=3.0):
        """Send command to Pico and wait for response.

        Args:
            command: Command string (e.g., "VALVE:CLOSED")
            timeout_s: Max seconds to wait for matching response

        Returns:
            str: Response from Pico (last line received)
        """
        self._ensure_open()

        self._recv_buf = b""
        self.connection.sendall((command + "\n").encode())

        if command == "POS":
            expected_prefixes = ("POS:", "ERROR")
        elif command == "STATUS":
            expected_prefixes = ("STATUS:", "ERROR")
        elif command == "READY?":
            expected_prefixes = ("READY:", "ERROR")
        else:
            expected_prefixes = ("OK", "ERROR")

        deadline = time.time() + timeout_s
        last_nonempty = ""

        while time.time() < deadline:
            line = self._readline(deadline)
            if not line:
                if self.connection is None:
                    break
                continue

            last_nonempty = line

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

        self._recv_buf = b""
        print(f"[{self.formulator_id}] Sending: {command}")
        self.connection.sendall((command + "\n").encode())

        deadline = time.time() + timeout_s

        while time.time() < deadline:
            response = self._readline(deadline)
            if self.connection is None:
                print(f"[{self.formulator_id}] Connection dropped while waiting for completion")
                return False
            if not response:
                continue

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

        print(f"[{self.formulator_id}] Timeout waiting for response")
        return False

    def send_raw(self, command, timeout_s=5.0):
        """Send an arbitrary raw command string and return the raw response line.

        Bypasses the typed helper methods below -- intended for interactive
        testing/debugging of the WiFi link (see wifi_link_test.py), not normal use.
        """
        return self._send_command(command, timeout_s=timeout_s)

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

    def set_default_fluid(self, name):
        """Set the Pico's runtime-default fluid/viscosity profile.

        Lets PUMP/STEP commands omit the profile token and fall back to
        whatever fluid is "currently selected" from the Pi5 side.

        Args:
            name: Profile token (e.g., "BLUESILV12"), or None/"" to clear

        Returns:
            bool: True if successful
        """
        token = str(name).strip().upper() if name else ""
        response = self._send_command(f"SETFLUID:{token}")
        ok = response == "OK"
        if ok:
            print(f"[{self.formulator_id}] Default fluid → {token or '(none)'}")
        return ok

    def set_viscosity_profile(self, name, direction, min_step, max_step, pause_ms, enabled=True):
        """Push (add or update) a viscosity step-limit profile entry to the Pico at runtime.

        This is how step-limit profiles are changed without reflashing firmware --
        the authoritative values should live in main_dispense_system.py's
        FLUID_PROFILES and get synced down via this call.

        Args:
            name: Profile token (e.g., "BLUESILV12")
            direction: "IN" or "OUT"
            min_step: Minimum step size (%)
            max_step: Maximum step size (%)
            pause_ms: Inter-step pause (ms)
            enabled: Whether stepped motion is enabled for this profile/direction

        Returns:
            bool: True if successful
        """
        direction = str(direction).strip().upper()
        if direction not in ("IN", "OUT"):
            raise ValueError("direction must be 'IN' or 'OUT'")

        token = str(name).strip().upper()
        command = (
            f"SETPROFILE:{token},{direction},{1 if enabled else 0},"
            f"{float(min_step):.4f},{float(max_step):.4f},{int(pause_ms)}"
        )
        response = self._send_command(command)
        ok = response == "OK"
        if ok:
            print(f"[{self.formulator_id}] Profile {token} {direction} synced (enabled={enabled}, min={min_step}, max={max_step}, pause_ms={pause_ms})")
        else:
            print(f"[{self.formulator_id}] Failed to sync profile {token} {direction}: {response}")
        return ok

    def sync_fluid_profile(self, name, in_min_step, in_max_step, in_pause_ms, in_enabled,
                           out_min_step, out_max_step, out_pause_ms, out_enabled):
        """Push both IN and OUT step-limit profile entries for a fluid in one call.

        Returns:
            bool: True only if both directions synced successfully
        """
        ok_in = self.set_viscosity_profile(name, "IN", in_min_step, in_max_step, in_pause_ms, enabled=in_enabled)
        ok_out = self.set_viscosity_profile(name, "OUT", out_min_step, out_max_step, out_pause_ms, enabled=out_enabled)
        return ok_in and ok_out

    def reset(self):
        """Reboot the Pico over the WiFi link (equivalent of an mpremote reset).

        The TCP connection will drop as the board reboots -- call open() again
        (after giving it a few seconds to reassociate with WiFi) to reconnect.
        """
        self._ensure_open()
        response = self._send_command("RESET", timeout_s=2.0)
        ok = response == "OK"
        if ok:
            print(f"[{self.formulator_id}] Reset requested")
        self.close()
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
