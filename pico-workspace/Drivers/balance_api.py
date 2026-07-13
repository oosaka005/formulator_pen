"""====================================================
Lab Dispensing System (ADC% Calibration - Continuous DC Drive)
Pico W + MG92B Valve + Actuonix L16 + DRV8871
+ Platform Actuator (P16-50-256-12-S)
====================================================

- Balance control module for reading weight and sending tare/zero commands to a serial-connected balance.
"""
import serial
import time

class Balance:
    
    def __init__(self, serial_port, balance_id, baud_rate=9600, timeout=1.0):
        self.balance_id = balance_id
        self.serial_port = serial_port
        self.baud_rate = baud_rate
        self.timeout = timeout
        self.connection = None

    def open(self):
        if self.connection and self.connection.is_open:
            return
        self.connection = serial.Serial(
            port=self.serial_port,
            baudrate=self.baud_rate,
            timeout=self.timeout
        )
        self._clear_buffers()
        print(f"[{self.balance_id}] Connected to {self.serial_port}")

    def _clear_buffers(self):
        """Drop any buffered serial data before a new command."""
        if not self.connection:
            return

        try:
            self.connection.reset_input_buffer()
        except Exception:
            pass

        try:
            self.connection.reset_output_buffer()
        except Exception:
            pass

        while self.connection.in_waiting:
            self.connection.read(self.connection.in_waiting)

    def _ensure_open(self):
        if not self.connection or not self.connection.is_open:
            raise RuntimeError(f"[{self.balance_id}] Not connected. Call open() first.")
    
    def read_weight(self, settle_time: float = 5.0):
        """Read weight from balance after optional settling delay."""
        self._ensure_open()
        if settle_time > 0:
            time.sleep(settle_time)

        self._clear_buffers()
        self.connection.write(b"R")
        try:
            self.connection.flush()
        except Exception:
            pass

        deadline = time.monotonic() + self.timeout
        accumulated = ""

        while time.monotonic() < deadline:
            if self.connection.in_waiting:
                char = self.connection.read(1).decode()
                if char == "g":
                    break
                if char not in ("\r", "\n"):
                    accumulated += char
            else:
                time.sleep(0.01)

        if not accumulated:
            raise RuntimeError(f"[{self.balance_id}] Timeout waiting for weight response")

        try:
            return float(accumulated.replace("\r", "").replace("\n", "").strip())
        except ValueError:
            raise RuntimeError(f"Failed to parse: {accumulated}")
    
    def tare(self):
        """Execute tare command"""
        self._ensure_open()
        self._clear_buffers()
        self.connection.write(b"T")
        try:
            self.connection.flush()
        except Exception:
            pass
        print(f"[{self.balance_id}] Sent TARE (T)")
        time.sleep(1)
    
    def zero(self):
        """Execute zero command"""
        self._ensure_open()
        self._clear_buffers()
        self.connection.write(b"Z")
        try:
            self.connection.flush()
        except Exception:
            pass
        print(f"[{self.balance_id}] Sent ZERO (Z)")
        time.sleep(1)

    def close(self):
        """Close connection"""
        if self.connection and self.connection.is_open:
            self.connection.close()
            print(f"[{self.balance_id}] Disconnected")