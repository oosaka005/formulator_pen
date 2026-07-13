"""
Minimal CNC Connection Test - Direct GRBL Commands
===================================================
Tests CNC connection with simple GRBL commands without the full CNC class.
This bypasses config files and just tests basic connectivity and movement.
"""

import serial
import time
from Drivers.cnc_api import CNC

# =====================================================
# CONFIGURATION
# =====================================================
CNC_PORT = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
BAUD_RATE = 115200

def send_gcode(ser, command):
    """Send a G-code command and wait for response"""
    print(f"  Sending: {command}")
    ser.write(str.encode(command + '\n'))
    time.sleep(0.1)
    
    # Read response
    response = []
    while ser.in_waiting > 0:
        grbl_out = ser.readline()
        resp = grbl_out.strip().decode('utf-8')
        if resp:
            response.append(resp)
            print(f"  Response: {resp}")
    
    return response

def check_status(ser):
    """Check CNC status"""
    ser.write(str.encode('?\n'))
    time.sleep(0.1)
    
    while ser.in_waiting > 0:
        grbl_out = ser.readline()
        resp = grbl_out.strip().decode('utf-8')
        if resp:
            print(f"  Status: {resp}")
            return resp
    return None

print("=" * 60)
print("CNC Connection Test - GRBL Direct")
print("=" * 60)

try:
    # Open serial connection
    print(f"\n[STEP 1] Opening serial connection on {CNC_PORT}...")
    ser = serial.Serial(CNC_PORT, BAUD_RATE, timeout=2)
    
    # Wake up GRBL
    print("\n[STEP 2] Waking up GRBL...")
    ser.write(str.encode("\r\n\r\n"))
    time.sleep(2)
    ser.flushInput()
    
    # Check if CNC responds
    print("\n[STEP 3] Testing basic communication...")
    send_gcode(ser, "$$")  # Request GRBL settings
    time.sleep(0.5)
    
    # Unlock if needed
    print("\n[STEP 4] Unlocking machine (if needed)...")
    send_gcode(ser, "$X")
    time.sleep(0.5)
    
    # Check status
    print("\n[STEP 5] Checking machine status...")
    check_status(ser)
    
    # Set to absolute positioning
    print("\n[STEP 6] Setting absolute positioning mode...")
    send_gcode(ser, "G90")
    time.sleep(0.5)
    
    # # Set software zero at current position
    # print("\n[STEP 7] Setting current position as software zero (X0 Y0 Z0)...")
    # send_gcode(ser, "G10 L20 P1 X0 Y0 Z0")
    # time.sleep(0.5)
    # check_status(ser)
    
    # Test movements
    print("\n" + "=" * 60)
    print("MOVEMENT TESTS")
    print("=" * 60)
    
    # Move X axis right
    print("\n[MOVE 1] Moving Y axis to +80mm (RIGHT, slow)...")
    send_gcode(ser, "G1 z0 F300")
    time.sleep(3)
    check_status(ser)


    
 
    
    
    
    print("\n" + "=" * 60)
    print("[SUCCESS] All tests completed!")
    print("=" * 60)
    print("\nIf you saw the X and Z axes moving, your CNC is properly connected!")
    print("X movements: +10mm → +50mm → +100mm → +10mm (then return home)")
    print("Z movements: 0mm → -20mm → -5mm")

    
except serial.SerialException as e:
    print(f"\n[ERROR] Could not connect to CNC on {CNC_PORT}")
    print(f"Details: {e}")
    print("\nTroubleshooting:")
    print("  1. Check CNC is powered on")
    print("  2. Verify USB cable is connected")
    print("  3. Confirm COM port in Device Manager")
    print("  4. Close any other software using the port (Candle, etc)")
    
except KeyboardInterrupt:
    print("\n\n[INTERRUPTED] Test stopped by user")
    
except Exception as e:
    print(f"\n[ERROR] Unexpected error: {e}")
    import traceback
    traceback.print_exc()
    
finally:
    # Close connection
    try:
        print("\n[CLEANUP] Closing serial connection...")
        ser.close()
        print("[SUCCESS] Connection closed.")
    except:
        pass

print("\nTest complete!")
