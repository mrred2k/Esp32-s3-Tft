Import("env")
import time
import sys
import serial
import serial.tools.list_ports

NORMAL_PORT = "COM3"
BOOTLOADER_PORT = "COM4"
TIMEOUT_S = 20

def get_ports():
    return [p.device for p in serial.tools.list_ports.comports()]

def wait_for_port(port, timeout=TIMEOUT_S):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if port in get_ports():
            return round(timeout - (deadline - time.time()), 1)
        time.sleep(0.3)
    return None

def before_upload(source, target, env):
    ports = get_ports()
    if BOOTLOADER_PORT in ports:
        print(f"\n[pre-upload] {BOOTLOADER_PORT} already in bootloader mode - ready!")
        return
    if NORMAL_PORT in ports:
        print(f"\n[pre-upload] Triggering bootloader via 1200bps touch on {NORMAL_PORT}...")
        try:
            ser = serial.Serial()
            ser.port = NORMAL_PORT
            ser.baudrate = 1200
            ser.dtr = False
            ser.open()
            time.sleep(0.1)
            ser.close()
        except Exception as e:
            # Native USB CDC often throws here but the touch still works - continue waiting
            print(f"[pre-upload] Touch sent (port closed with: {e})")
        print(f"[pre-upload] Waiting for {BOOTLOADER_PORT}...")
        elapsed = wait_for_port(BOOTLOADER_PORT)
        if elapsed is not None:
            print(f"[pre-upload] {BOOTLOADER_PORT} appeared after {elapsed}s - ready!")
            time.sleep(0.5)
        else:
            print(f"[pre-upload] ERROR: {BOOTLOADER_PORT} did not appear within {TIMEOUT_S}s!")
    else:
        print(f"[pre-upload] ERROR: Neither {NORMAL_PORT} nor {BOOTLOADER_PORT} found!")

def after_upload(source, target, env):
    print(f"\n[post-upload] Flash done! Press RST on the board, then the monitor will open.")
    print(f"[post-upload] Waiting for {NORMAL_PORT} to appear (max {TIMEOUT_S}s)...")
    elapsed = wait_for_port(NORMAL_PORT)
    if elapsed is not None:
        print(f"[post-upload] {NORMAL_PORT} appeared after {elapsed}s - opening monitor!")
    else:
        print(f"[post-upload] WARNING: {NORMAL_PORT} did not appear within {TIMEOUT_S}s. Try pressing RST.")

env.AddPreAction("upload", before_upload)
env.AddPostAction("upload", after_upload)
