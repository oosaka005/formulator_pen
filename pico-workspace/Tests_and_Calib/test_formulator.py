"""
Comprehensive Formulator Driver Test
Tests all FormulatorDriver functions using the actual driver class
"""

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent))

from Drivers.formulator_driver import FormulatorDriver
import time

FORMULATOR_PORT = "/dev/serial/by-id/usb-MicroPython_Board_in_FS_mode_e66368254f52132d-if00"
BAUD_RATE = 115200
PUMP_TEST_VOLUME_ML = 3.0  # Use smaller volume for testing
RUN_VALVE_TESTS = False  # Set True when valve hardware is connected

print("=" * 60)
print("Formulator Driver Test Suite")
print("=" * 60)

try:
    # Initialize driver
    print(f"\n[TEST 1] Initializing FormulatorDriver on {FORMULATOR_PORT}...")
    formulator = FormulatorDriver(
        serial_port=FORMULATOR_PORT,
        baud_rate=BAUD_RATE,
        timeout=2.0
    )
    formulator.open()
    print("[SUCCESS] Driver initialized and connected")
    time.sleep(2)
    
    # Test is_ready
    print("\n[TEST 2] Testing is_ready()...")
    ready = formulator.is_ready()
    print(f"  Result: {'Ready' if ready else 'Not ready (cooling down)'}")
    
    if not ready:
        cooldown = formulator.get_cooldown_time()
        print(f"  Cooldown time: {cooldown:.1f}s")
        if cooldown > 10:
            print(f"  [WARNING] Long cooldown detected. Consider waiting or power-cycling Pico.")
    
    # Test get_status
    print("\n[TEST 3] Testing get_status()...")
    status = formulator.get_status()
    print(f"  Status: {status}")
    
    # Test get_position
    print("\n[TEST 4] Testing get_position()...")
    pos = formulator.get_position()
    print(f"  Position: {pos:.2f}%")
    
    # Test valve movements
    if RUN_VALVE_TESTS:
        print("\n[TEST 5] Testing valve_move()...")
        
        print("  Moving valve to UP")
        ok = formulator.valve_move("UP")
        print(f"  Result: {'OK' if ok else 'FAILED'}")
        time.sleep(1)
        
        print("  Moving valve to CLOSED")
        ok = formulator.valve_move("CLOSED")
        print(f"  Result: {'OK' if ok else 'FAILED'}")
        time.sleep(1)
        
        print("  Moving valve to THRU")
        ok = formulator.valve_move("THRU")
        print(f"  Result: {'OK' if ok else 'FAILED'}")
        time.sleep(1)
        
        print("  Moving valve back to CLOSED")
        ok = formulator.valve_move("CLOSED")
        print(f"  Result: {'OK' if ok else 'FAILED'}")
        time.sleep(1)
    else:
        print("\n[TEST 5] Skipping valve_move() (valve disconnected)")
    
    # Test pump operations (only if ready)
    if formulator.is_ready():
        print(f"\n[TEST 6] Testing pump_volume() with {PUMP_TEST_VOLUME_ML} mL...")
        
        
        time.sleep(2)
        
        print(f"\n  Pumping {PUMP_TEST_VOLUME_ML} mL OUT")
        pos_before_out = formulator.get_position()
        ok_out = formulator.pump_volume(PUMP_TEST_VOLUME_ML, "OUT", pwm_percent=50)
        print(f"  Result: {'SUCCESS' if ok_out else 'FAILED'}")
        
        time.sleep(1)
        pos_after_out = formulator.get_position()
        print(f"  Position after OUT: {pos_after_out:.2f}%")
        delta_out = pos_after_out - pos_before_out
        print(f"  Delta OUT: {delta_out:.2f}%")
        if abs(delta_out) < 1.0:
            print("  [WARNING] Actuator position change < 1% (no visible movement)")
    else:
        print("\n[TEST 6] SKIPPED - Formulator not ready (duty cycle protection)")
        cooldown = formulator.get_cooldown_time()
        print(f"  Wait {cooldown:.1f}s or power-cycle Pico to reset")
    
    # Final status
    print("\n[TEST 7] Final status check...")
    status = formulator.get_status()
    print(f"  Status: {status}")
    
    print("\n" + "=" * 60)
    print("[SUCCESS] All driver tests completed!")
    print("=" * 60)
    
except Exception as e:
    print(f"\n[ERROR] Test failed: {e}")
    import traceback
    traceback.print_exc()
    
finally:
    try:
        formulator.close()
        print("\n[CLEANUP] Driver closed.")
    except:
        pass

print("\nTest complete!")
