"""====================================================
Formulator WiFi Link Tester
====================================================

Standalone diagnostic tool for checking the Pi5 <-> Pico W WiFi link after
uploading Drivers/pico_api_step.py to the Pico. Uses the real FormulatorDriver
(same code path as main_dispense_system.py), so a pass here means the actual
production driver can talk to the actual production firmware.

Usage:
    python wifi_link_test.py                     # uses DEFAULT_HOST below
    python wifi_link_test.py --host 192.168.4.101 # or override per-run

Before running:
    - Upload pico_api_step.py to the Pico (mpremote cp ... :main.py) and power it on
    - Watch its USB serial console for "[WIFI] Connected. IP=..." to get the IP
    - Edit DEFAULT_HOST below to that IP (or pass --host each time instead)
"""

import argparse
import sys
import time
from pathlib import Path

# Edit this to the Pico's IP once you know it (see "Before running" above).
# Overridden by --host on the command line if you pass one.
DEFAULT_HOST = "192.168.10.166"

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from Drivers.formulator_driver import FormulatorDriver


PASS = "PASS"
FAIL = "FAIL"


def check_ok(driver, command, label, results):
    """Send a command, expect a literal 'OK' response, record + print pass/fail."""
    response = driver.send_raw(command)
    ok = response == "OK"
    results.append((label, ok, response))
    print(f"  [{PASS if ok else FAIL}] {label:<28} -> {command!r:35} => {response!r}")
    return ok


def check_prefix(driver, command, prefix, label, results):
    """Send a command, expect the response to start with `prefix`."""
    response = driver.send_raw(command)
    ok = response.startswith(prefix)
    results.append((label, ok, response))
    print(f"  [{PASS if ok else FAIL}] {label:<28} -> {command!r:35} => {response!r}")
    return ok


def check_error(driver, command, label, results):
    """Send a command, expect an ERROR response (used to confirm bad input is rejected)."""
    response = driver.send_raw(command)
    ok = response.startswith("ERROR")
    results.append((label, ok, response))
    print(f"  [{PASS if ok else FAIL}] {label:<28} -> {command!r:35} => {response!r}")
    return ok


def run_smoke_test(driver):
    """Exercise every non-destructive command. Safe to run anytime -- no fluid
    is dispensed and the actuator never moves (only the valve, which just
    switches flow path, does)."""
    print("\n=== Smoke test: telemetry, mode, valve, profile/fluid commands ===")
    results = []

    check_prefix(driver, "READY?", "READY:", "Ready check", results)
    check_prefix(driver, "STATUS", "STATUS:", "Status telemetry", results)
    check_prefix(driver, "POS", "POS:", "Position readback", results)

    check_ok(driver, "MODE:NORMAL", "Set mode NORMAL", results)

    check_ok(driver, "VALVE:CLOSED", "Valve -> CLOSED", results)
    check_ok(driver, "VALVE:UP", "Valve -> UP", results)
    check_ok(driver, "VALVE:THRU", "Valve -> THRU", results)
    check_ok(driver, "VALVE:CLOSED", "Valve -> CLOSED (restore)", results)

    check_ok(driver, "SETFLUID:WATER", "Set default fluid", results)
    check_ok(driver, "SETFLUID:", "Clear default fluid", results)

    # Uses a throwaway profile name so this never touches a real fluid's config.
    check_ok(driver, "SETPROFILE:SMOKETEST,IN,1,0.5,3.0,500", "Push test profile (IN)", results)
    check_ok(driver, "SETPROFILE:SMOKETEST,OUT,1,0.5,3.0,500", "Push test profile (OUT)", results)

    check_error(driver, "BOGUS_COMMAND", "Unknown command rejected", results)

    passed = sum(1 for _, ok, _ in results if ok)
    print(f"\nSmoke test: {passed}/{len(results)} passed")
    if passed != len(results):
        print("Failed checks:")
        for label, ok, response in results:
            if not ok:
                print(f"  - {label}: got {response!r}")
    return passed == len(results)


def run_actuator_move_test(driver):
    """Optional: exercises real actuator motion. Confirms first.

    Uses the driver's typed move_to_percent_stepped() rather than a raw
    send_raw("STEP:...") call -- that method routes through
    _send_command_wait_for_completion(), which blocks for as long as the move
    actually takes (up to the firmware's own move_timeout_ms + settle_time_ms,
    i.e. up to ~70s), the same way main_dispense_system.py does in production.
    A raw send_raw() here would use the generic 5s command timeout, which is
    far too short for real motion and would falsely report failure while the
    Pico is still mid-move -- and desync later checks against stale responses.
    """
    print("\n=== Actuator movement test ===")
    print("This will PHYSICALLY MOVE the actuator a small amount in PRIMING mode")
    print("(0-40% window) and back. Make sure it's safe to move before continuing.")
    answer = input("Proceed? [y/N]: ").strip().lower()
    if answer != "y":
        print("Skipped.")
        return None

    results = []
    check_ok(driver, "MODE:PRIMING", "Set mode PRIMING", results)

    start_pos = driver.send_raw("POS")
    print(f"  Start position: {start_pos}")

    # Valve must be open on the correct path before commanding fluid motion,
    # same as the real dispense workflow (UP before drawing IN, THRU before OUT) --
    # moving the actuator against a CLOSED valve pushes/draws against a blocked
    # line instead of moving fluid.
    check_ok(driver, "VALVE:UP", "Valve -> UP (before IN move)", results)

    print("  Moving IN to 10% (waiting for the full move + settle to complete)...")
    ok_in = driver.move_to_percent_stepped(10.0, "IN", pwm_percent=25, viscosity_profile="BLUESILV12")
    results.append(("Step move IN to 10%", ok_in, "completed" if ok_in else "did not complete/settle"))
    print(f"  [{PASS if ok_in else FAIL}] Step move IN to 10%")
    check_prefix(driver, "POS", "POS:", "Position after IN move", results)

    check_ok(driver, "VALVE:THRU", "Valve -> THRU (before OUT move)", results)

    print("  Moving OUT to 2% (waiting for the full move + settle to complete)...")
    ok_out = driver.move_to_percent_stepped(2.0, "OUT", pwm_percent=25, viscosity_profile="BLUESILV12")
    results.append(("Step move OUT to 2%", ok_out, "completed" if ok_out else "did not complete/settle"))
    print(f"  [{PASS if ok_out else FAIL}] Step move OUT to 2%")
    check_prefix(driver, "POS", "POS:", "Position after OUT move", results)

    check_ok(driver, "VALVE:CLOSED", "Valve -> CLOSED (restore)", results)
    check_ok(driver, "MODE:NORMAL", "Restore mode NORMAL", results)

    passed = sum(1 for _, ok, _ in results if ok)
    print(f"\nActuator move test: {passed}/{len(results)} passed")
    return passed == len(results)


def run_position_monitor(driver):
    """Continuously poll and print STATUS (position %, raw ADC counts, duty)
    without moving anything. Purely read-only diagnostic.

    Useful for isolating a feedback-wiring/hardware fault from a software issue:
    move the actuator BY HAND (if safe to do so) while this runs and watch
    whether the printed position/ADC value tracks the real physical movement
    at all. If it never changes no matter how far you move it, that points at
    the feedback wire/connector rather than anything in this codebase. If it
    changes but inconsistently/noisily, that's a different class of problem
    (wiring picking up noise, bad connection, etc.) than a clean disconnect.
    """
    print("\n=== Live position monitor (read-only, moves nothing) ===")
    print("Polling STATUS repeatedly. If safe, move the actuator by hand and watch")
    print("whether POS/ADC below tracks the real physical movement.")
    print("Press Ctrl-C to stop.\n")

    last_percent = None
    unchanged_count = 0
    try:
        while True:
            status = driver.send_raw("STATUS")
            note = ""
            try:
                percent = float(status.split("POS=")[1].split(",")[0])
                if last_percent is not None and abs(percent - last_percent) < 0.05:
                    unchanged_count += 1
                else:
                    unchanged_count = 0
                last_percent = percent
                if unchanged_count >= 10:
                    note = "  <-- unchanged for 5+ readings"
            except (IndexError, ValueError):
                note = "  <-- could not parse POS from response"
            print(f"{status}{note}")
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\nStopped.")


def run_reset_test(driver, host, port, formulator_id):
    """Exercises RESET end-to-end: sends it, then confirms the Pico actually
    reboots and comes back up on WiFi (not just that it accepted the command)."""
    print("\n=== Reset test ===")
    print("This will reboot the Pico. The current connection will drop and the")
    print("board will take several seconds to reconnect to WiFi afterward.")
    answer = input("Proceed? [y/N]: ").strip().lower()
    if answer != "y":
        print("Skipped.")
        return None

    ok_reset = driver.reset()
    print(f"  RESET command acknowledged: {ok_reset}")
    if not ok_reset:
        return False

    print("  Waiting for Pico to reboot and reconnect to WiFi...")
    time.sleep(5)

    for attempt in range(1, 11):
        try:
            driver.open()
            ready = driver.send_raw("READY?")
            if ready.startswith("READY:"):
                print(f"  [PASS] Pico back online after reset (attempt {attempt}) -> {ready!r}")
                return True
        except (OSError, RuntimeError):
            pass
        print(f"  ... retry {attempt}/10")
        time.sleep(2)

    print("  [FAIL] Pico did not come back online after reset within ~25s")
    return False


def run_interactive_console(driver):
    """Free-form raw command entry, for exploring things the automated tests don't cover."""
    print("\n=== Interactive console ===")
    print("Type any raw command (e.g. VALVE:UP, PUMP:1,IN,25,WATER, STATUS).")
    print("Type 'help' for the command reference, 'quit' to return to the menu.\n")

    help_text = """
  VALVE:<UP|CLOSED|THRU>
  PUMP:<volume_ml>,<IN|OUT>[,<pwm>][,<profile|steps>]
  MODE:<NORMAL|PRIMING>
  STEP:<target_percent>,<IN|OUT>[,<pwm>][,<profile|steps>]
  SETPROFILE:<name>,<IN|OUT>,<enabled 0|1>,<min>,<max>,<pause_ms>
  SETFLUID:<name>
  POS
  STATUS
  READY?
  RESET
"""

    while True:
        try:
            command = input("pico> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break

        if not command:
            continue
        if command.lower() in ("quit", "exit"):
            break
        if command.lower() == "help":
            print(help_text)
            continue
        if command.upper() == "RESET":
            print("Use the dedicated 'Reset test' menu option instead (it verifies recovery).")
            continue

        try:
            response = driver.send_raw(command)
            print(f"<<< {response!r}")
        except (OSError, RuntimeError) as e:
            print(f"<<< connection error: {e}")


def main():
    parser = argparse.ArgumentParser(description="Test the Pi5 <-> Pico WiFi link")
    parser.add_argument("--host", default=DEFAULT_HOST, help="Pico's IP address (default: DEFAULT_HOST in this file)")
    parser.add_argument("--port", type=int, default=8888, help="TCP port (default: 8888)")
    parser.add_argument("--timeout", type=float, default=3.0, help="Socket timeout in seconds")
    parser.add_argument("--formulator-id", default="formulator1", help="Label for log lines")
    args = parser.parse_args()

    if args.host == "192.168.4.101" and DEFAULT_HOST == "192.168.4.101":
        print("DEFAULT_HOST is still the placeholder IP -- edit it near the top of this")
        print("file (or pass --host <ip>) before running.")
        sys.exit(1)

    driver = FormulatorDriver(
        host=args.host,
        port=args.port,
        timeout=args.timeout,
        # Generous on purpose: a stepped move can hit MOVE_TIMEOUT_MS (60s) on
        # more than one struggling step before giving up, so a tight timeout here
        # can misreport "stalled" when the Pico is still legitimately working.
        pump_timeout_s=300.0,
        formulator_id=args.formulator_id,
    )

    print(f"Connecting to {args.host}:{args.port}...")
    driver.open()

    try:
        while True:
            print("\n=== Formulator WiFi Link Tester ===")
            print("  1) Automated smoke test (safe, no actuator motion)")
            print("  2) Actuator movement test (real motion, confirms first)")
            print("  3) Reset test (reboots the Pico, confirms it comes back)")
            print("  4) Interactive console (raw commands)")
            print("  5) Live position monitor (read-only, move actuator by hand)")
            print("  6) Quit")
            choice = input("Choose: ").strip()

            if choice == "1":
                run_smoke_test(driver)
            elif choice == "2":
                run_actuator_move_test(driver)
            elif choice == "3":
                run_reset_test(driver, args.host, args.port, args.formulator_id)
            elif choice == "4":
                run_interactive_console(driver)
            elif choice == "5":
                run_position_monitor(driver)
            elif choice == "6":
                break
            else:
                print("Unrecognized choice.")
    finally:
        driver.close()


if __name__ == "__main__":
    main()
