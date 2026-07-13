# Tool Changer Test - Lock/Unlock Functionality
import sys
import os
import time

# Add the src directory to the path to enable relative imports
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from Drivers.tool_changer_api import ToolChanger

# ====== Configuration ======
VIRTUAL_MODE = False  # Set to True for dry run
SERIAL_PORT = "COM6"  # e.g., "COM14" to override config
DO_LOCK = True
DO_UNLOCK = False


# ===========================================
# Main execution
# ===========================================
if __name__ == "__main__":
    print(f"Tool Changer Test (Virtual: {VIRTUAL_MODE})")
    
    try:
        if VIRTUAL_MODE:
            print("[INFO] Running in virtual mode - no actual hardware control")
            raise SystemExit(0)

        tc = ToolChanger()
        if not tc.open(port=SERIAL_PORT):
            print("[ERROR] Failed to connect to tool changer")
            raise SystemExit(1)

        print("[INFO] STATUS ->", tc._send_command("STATUS"))

        if DO_LOCK:
            input("(Enter) start lock...")
            start_lock = time.monotonic()
            lock_ok = tc.lock()
            lock_elapsed = time.monotonic() - start_lock
            print(f"[LOCK] result={'OK' if lock_ok else 'TIMEOUT'} elapsed={lock_elapsed:.3f}s status={tc._send_command('STATUS')}")

        if DO_UNLOCK:
            input("(Enter) start unlock...")
            start_unlock = time.monotonic()
            unlock_ok = tc.unlock()
            unlock_elapsed = time.monotonic() - start_unlock
            print(f"[UNLOCK] result={'OK' if unlock_ok else 'TIMEOUT'} elapsed={unlock_elapsed:.3f}s status={tc._send_command('STATUS')}")

        print("[DONE]")

    except Exception as e:
        print(f"[ERROR] Tool changer test failed: {e}")
        print("Check if:")
        print("1. Tool changer is connected via USB")
        print("2. Serial port is correct")
        print("3. Set VIRTUAL_MODE=False for real hardware testing")
    finally:
        try:
            tc.close()
        except Exception:
            pass
