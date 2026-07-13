"""
Valve Calibration Manual Override
Command a target angle and move the valve with stepped motion.
"""

import uasyncio as asyncio
from machine import Pin, PWM

# ===== CONFIGURATION =====
SERVO_PIN = 3  # PWM pin for servo control.
SERVO_PWM_FREQ = 50  # Hz (standard servo frequency)
SERVO_STEP_DEG = 2  # Degrees per motion step
SERVO_STEP_DELAY_MS = 20  # Delay between motion steps

# Current calibrated positions (update these as you calibrate)
SERVO_POSITIONS = {
    "START": 68,      # Initial position
    "UP": 112,        # Intake position (open for fluid intake)
    "CLOSED": 68,    # Closed/neutral position
    "THRU": 22,     # Dispense position (through to output)
}

# ===== VALVE CLASS =====
class Valve:
    """
    Controls a servo valve with angle control for calibration
    """
    def __init__(
        self,
        pin_num=SERVO_PIN,
        freq=SERVO_PWM_FREQ,
        step_deg=SERVO_STEP_DEG,
        step_delay_ms=SERVO_STEP_DELAY_MS,
    ):
        self.pwm = PWM(Pin(pin_num))
        self.pwm.freq(freq)  # 50 Hz for servo
        self.step_deg = max(1, int(step_deg))
        self.step_delay_ms = max(0, int(step_delay_ms))
        self.current_angle = None

    def _angle_to_duty(self, angle):
        """Convert angle (0-180°) to PWM duty cycle"""
        angle = max(0, min(180, angle))  # Clamp to 0-180
        pulse_width_ms = 0.5 + (angle*2 / 180.0)  # 1-2ms range
        duty = int((pulse_width_ms / 20.0) * 65535)
        return duty

    def write(self, angle):
        """Move servo to specified angle (0-180)"""
        duty = self._angle_to_duty(angle)
        self.pwm.duty_u16(duty)
        self.current_angle = angle
        print(f"Valve: Moved to {angle}°")

    async def move_to_angle_stepped(self, target_angle):
        """Move servo to target angle using stepped motion."""
        target_angle = max(0, min(180, int(target_angle)))

        if self.current_angle is None:
            self.write(target_angle)
            return

        if target_angle == self.current_angle:
            return

        step = self.step_deg if target_angle > self.current_angle else -self.step_deg
        angle = self.current_angle

        while abs(target_angle - angle) > abs(step):
            angle += step
            self.pwm.duty_u16(self._angle_to_duty(angle))
            self.current_angle = angle
            await asyncio.sleep_ms(self.step_delay_ms)

        self.write(target_angle)

    async def move_to_position(self, position_name):
        """Move valve to a predefined position using stepped motion."""
        if position_name not in SERVO_POSITIONS:
            print(f"Error: Unknown position '{position_name}'")
            return False

        angle = SERVO_POSITIONS[position_name]
        await self.move_to_angle_stepped(angle)
        return True

# ===== CALIBRATION TEST =====
async def calibration_test():
    """Manual override loop for commanding valve angles during calibration."""
    print("=== Valve Manual Override Calibration ===")
    print("Enter an angle (0-180) to move the valve.")
    print("Commands: q=quit, status=current angle, or a named position (START/UP/CLOSED/THRU)")
    print()

    # Create valve
    valve = Valve()

    # Start in CLOSED for safety
    await valve.move_to_position("CLOSED")

    while True:
        user_cmd = input("Command angle or position: ").strip()

        if not user_cmd:
            continue

        cmd_upper = user_cmd.upper()
        cmd_lower = user_cmd.lower()

        if cmd_lower in ("q", "quit", "exit"):
            break

        if cmd_lower == "status":
            if valve.current_angle is None:
                print("Current angle: unknown")
            else:
                print(f"Current angle: {valve.current_angle}°")
            continue

        if cmd_upper in SERVO_POSITIONS:
            target_angle = SERVO_POSITIONS[cmd_upper]
            print(f"Moving to {cmd_upper} ({target_angle}°)")
            await valve.move_to_position(cmd_upper)
            print(f"Commanded: {target_angle}° | Landed: {valve.current_angle}°")
            continue

        try:
            target_angle = int(float(user_cmd))
        except ValueError:
            print("Invalid command. Enter 0-180, START/UP/CLOSED/THRU, status, or q.")
            continue

        if target_angle < 0 or target_angle > 180:
            print("Angle out of range. Enter a value between 0 and 180.")
            continue

        await valve.move_to_angle_stepped(target_angle)
        print(f"Commanded: {target_angle}° | Landed: {valve.current_angle}°")

    print("Manual override complete. Returning valve to CLOSED.")
    await valve.move_to_position("CLOSED")

# ===== MAIN FUNCTION =====
async def main():
    """Run valve manual override calibration."""
    try:
      await calibration_test()

    except KeyboardInterrupt:
      print("\n❌ Test interrupted")
      # Ensure valve is in safe position
      valve = Valve()
      await valve.move_to_position("CLOSED")
      await asyncio.sleep(0.5)

# ===== RUN TEST =====
asyncio.run(main())