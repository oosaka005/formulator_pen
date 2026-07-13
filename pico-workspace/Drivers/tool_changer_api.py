#!/usr/bin/env python3
# Tool changer control via Arduino firmware (START/STOP/HOLD/STATUS)
import time
import threading
import serial

POLL = 0.01          # seconds between switch checks
TIMEOUT = 8.0        # max wait time for full move
RPM_AWAY = 100       # RPM toward LOCK side (tune as needed)
RPM_TOWARD = -100    # RPM toward UNLOCK side (tune as needed)
MOVE_PRE_TIME = 1.0  # initial fixed move duration (s) before waiting for switch

# Fixed connection settings (copied from equipment_config.yaml)
DEFAULT_AXIS = "X"
DEFAULT_PORT = "COM14"
DEFAULT_BAUD = 9600
DEFAULT_TIMEOUT = 2.0


class ToolChanger:
    """Arduino-based tool changer (uses configured axis + TOOL_SW)."""

    def __init__(self, port: str = DEFAULT_PORT, baudrate: int = DEFAULT_BAUD, timeout: float = DEFAULT_TIMEOUT):
        self.axis = DEFAULT_AXIS
        self.configured_port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial_connection = None
        self.port = None
        self.io_lock = threading.Lock()

    def open(self, port: str | None = None):
        with self.io_lock:
            try:
                if self.serial_connection and self.serial_connection.is_open:
                    self.close()

                port = port or self.configured_port
                if not port:
                    print("ERROR: No port specified")
                    return False

                self.serial_connection = serial.Serial(
                    port=port,
                    baudrate=self.baudrate,
                    timeout=self.timeout,
                    write_timeout=self.timeout,
                )

                time.sleep(2)
                self.port = port
                return True
            except serial.SerialException as e:
                print(f"ERROR: Serial connection error: {e}")
                return False
            except Exception as e:
                print(f"ERROR: Unexpected error connecting to Arduino: {e}")
                return False

    def close(self):
        with self.io_lock:
            if self.serial_connection and self.serial_connection.is_open:
                try:
                    self.serial_connection.close()
                except Exception as e:
                    print(f"ERROR: Error disconnecting: {e}")
            self.serial_connection = None
            self.port = None

    def _send_command(self, command):
        with self.io_lock:
            if not self.serial_connection or not self.serial_connection.is_open:
                print("ERROR: Arduino not connected")
                return "ERROR: Arduino not connected"

            try:
                if self.serial_connection.in_waiting > 0:
                    self.serial_connection.read_all()

                command_bytes = (command + "\n").encode("utf-8")
                self.serial_connection.write(command_bytes)
                self.serial_connection.flush()

                response_lines = []
                start_time = time.time()
                while time.time() - start_time < self.timeout:
                    if self.serial_connection.in_waiting > 0:
                        line = self.serial_connection.readline().decode("utf-8").strip()
                        if line:
                            response_lines.append(line)
                            if command == "STATUS" and line.startswith("{") and line.endswith("}"):
                                return line
                            if line == "OK" or line.startswith("ERROR"):
                                return line
                    time.sleep(0.05)

                if response_lines:
                    return response_lines[-1]
                print(f"ERROR: Timeout waiting for response to command: {command}")
                return "ERROR: Timeout"
            except serial.SerialException as e:
                print(f"ERROR: Serial error sending command '{command}': {e}")
                return f"ERROR: Serial error - {e}"
            except Exception as e:
                print(f"ERROR: Unexpected error sending command '{command}': {e}")
                return f"ERROR: {e}"

    def _status(self):
        return self._send_command("STATUS")

    def _tool_switch_engaged(self) -> bool:
        status = self._status()
        if not (status and status.startswith("{") and status.endswith("}")):
            return False
        return '"TOOL_SW":0' in status.replace(" ", "")

    def _ensure_hold(self):
        status = self._status()
        if status and '"EN":0' in status.replace(" ", ""):
            self._hold()

    def _start(self, rpm: int):
        if rpm == 0 or abs(rpm) > 3000:
            raise ValueError("RPM must be between -3000 and 3000 (non-zero).")
        self._send_command(f"START,{self.axis},{int(rpm)},1,1")

    def _stop(self):
        self._send_command(f"STOP,{self.axis}")

    def _hold(self):
        self._send_command(f"HOLD,{self.axis}")

    def _release(self):
        self._send_command(f"RELEASE,{self.axis}")

    def unlock(self) -> bool:
        """Run TOWARD until tool switch engages, then release."""
        self._ensure_hold()
        self._start(RPM_TOWARD)
        time.sleep(MOVE_PRE_TIME)

        start = time.monotonic()
        while not self._tool_switch_engaged() and (time.monotonic() - start) < TIMEOUT:
            time.sleep(POLL)

        success = self._tool_switch_engaged()
        self._stop()
        self._release()
        return success

    def lock(self) -> bool:
        """Run AWAY until tool switch engages, then hold."""
        self._ensure_hold()
        self._start(RPM_AWAY)
        time.sleep(MOVE_PRE_TIME)

        start = time.monotonic()
        while not self._tool_switch_engaged() and (time.monotonic() - start) < TIMEOUT:
            time.sleep(POLL)

        success = self._tool_switch_engaged()
        self._stop()
        self._hold()
        return success