"""====================================================
Minimal standalone ADC pin diagnostic (runs ON the Pico)
====================================================

Isolates whether a specific GPIO/ADC pin itself is reading correctly, with
NONE of pico_api_step.py's other code involved (no Actuator, no filtering, no
WiFi, no valve). If this still reads wrong, the problem is upstream of all the
software in pico_api_step.py -- it's the pin, the wiring, or the potentiometer.

This does NOT touch main.py -- it won't interfere with your real firmware.

Usage (run directly without deploying, from the Pi5/PC side):
    mpremote connect <path> run adc_pin_diagnostic.py

Or copy it over temporarily and run interactively via Thonny/mpremote repl.

How to use this to isolate the fault:
  1. Run with PIN = 27 (the currently-wired feedback pin, moved here from GP28).
     Move the actuator/potentiometer by hand and watch the raw/voltage values.
  2. If it's still wrong: physically move the SAME feedback wire to another
     ADC-capable pin (GP26, or back to GP28 -- avoid GP29, it's shared with
     the WiFi chip on Pico W), change PIN below to match, and re-run.
       - Still wrong on the new pin too -> problem follows the wire, so it's
         the wiring/potentiometer/power to the pot, not the Pico's ADC input.
       - Correct on the new pin -> GP27 itself (or its trace/solder joint) is
         the fault; move the permanent wiring and update MOTOR_FEEDBACK_PIN in
         Drivers/pico_api_step.py to match.
  3. Cross-check with a multimeter directly on the pin header while moving the
     actuator by hand -- if the multimeter also shows a stuck/wrong voltage,
     it's fully confirmed electrical (wiring/pot/power), not a Pico fault at all.
"""

from machine import Pin, ADC
import time

PIN = 27  # change to 26 or 28 to test a different ADC-capable pin (not 29 on Pico W)

adc = ADC(Pin(PIN))

print(f"Reading raw ADC on GP{PIN} every 0.2s. Move the actuator/pot by hand.")
print("Press Ctrl-C to stop.\n")

last_raw = None
try:
    while True:
        raw = adc.read_u16()          # 0-65535
        voltage = (raw / 65535) * 3.3  # assumes 3.3V reference, standard for Pico
        percent = (raw / 65535) * 100.0

        note = ""
        if last_raw is not None and abs(raw - last_raw) < 50:
            note = "  (basically unchanged)"
        last_raw = raw

        print(f"GP{PIN}: raw={raw:5d}  voltage={voltage:.3f}V  percent={percent:6.2f}%{note}")
        time.sleep(0.2)
except KeyboardInterrupt:
    print("\nStopped.")
