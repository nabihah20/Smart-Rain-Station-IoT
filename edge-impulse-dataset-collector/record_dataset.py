"""
Edge Impulse Audio Dataset Collector & WAV Slicer
Target: ESP32-S3 AMOLED (ES7210 16kHz 16-bit Mono PCM)
Saves 1.0-second WAV files directly into Edge Impulse formatted folders:
  - dataset/tip/
  - dataset/knock/
  - dataset/background/
"""

import os
import sys
import time
import wave
import serial
import serial.tools.list_ports

SAMPLE_RATE = 16000
BYTES_PER_SAMPLE = 2  # 16-bit mono
CLIP_DURATION_SEC = 1.0  # 1-second audio clips for Edge Impulse
CLIP_BYTES = int(SAMPLE_RATE * BYTES_PER_SAMPLE * CLIP_DURATION_SEC)  # 32,000 bytes

DATASET_ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "dataset")
CLASSES = ["tip", "knock", "background"]

for c in CLASSES:
    os.makedirs(os.path.join(DATASET_ROOT, c), exist_ok=True)

def list_existing_counts():
    counts = {}
    for c in CLASSES:
        path = os.path.join(DATASET_ROOT, c)
        files = [f for f in os.listdir(path) if f.endswith(".wav")]
        counts[c] = len(files)
    return counts

def delete_all_dataset():
    for c in CLASSES:
        path = os.path.join(DATASET_ROOT, c)
        for f in os.listdir(path):
            if f.endswith(".wav"):
                try:
                    os.remove(os.path.join(path, f))
                except Exception as e:
                    print(f"Error removing {f}: {e}")
    print("[SUCCESS] Cleared all recorded samples from dataset folders.")

def delete_last_sample(class_name):
    path = os.path.join(DATASET_ROOT, class_name)
    files = [f for f in os.listdir(path) if f.endswith(".wav")]
    if not files:
        print(f"No samples found in {class_name} to delete.")
        return
    files.sort(key=lambda x: os.path.getmtime(os.path.join(path, x)))
    last_file = files[-1]
    os.remove(os.path.join(path, last_file))
    print(f"[REMOVED] Deleted last sample: {last_file}")

def save_wav(filepath, pcm_bytes):
    with wave.open(filepath, "wb") as wf:
        wf.setnchannels(1)  # Mono
        wf.setsampwidth(BYTES_PER_SAMPLE)  # 16-bit (2 bytes)
        wf.setframerate(SAMPLE_RATE)  # 16 kHz
        wf.writeframes(pcm_bytes)

def record_clip(ser, class_name):
    # Flush any lingering bytes
    ser.reset_input_buffer()
    
    # Start stream command
    ser.write(b"S")
    time.sleep(0.05)
    
    # Wait for ESP32 sync token
    line = ser.readline().decode('utf-8', errors='ignore')
    while "[ESP32_STREAM_START]" not in line:
        line = ser.readline().decode('utf-8', errors='ignore')

    print(f"  -> Recording 1.0s clip for [{class_name.upper()}]... (Action now!)")

    # Read exactly 1 second of audio
    captured = bytearray()
    while len(captured) < CLIP_BYTES:
        chunk = ser.read(min(CLIP_BYTES - len(captured), 4096))
        if chunk:
            captured.extend(chunk)

    # Stop stream command
    ser.write(b"X")
    ser.reset_input_buffer()

    # Generate filename
    timestamp = int(time.time() * 1000)
    filename = f"{class_name}_{timestamp}.wav"
    filepath = os.path.join(DATASET_ROOT, class_name, filename)

    save_wav(filepath, captured)
    print(f"  [SAVED] {filepath} ({len(captured)} bytes)")

def main():
    port = "COM8"
    baud = 921600

    print("==================================================================")
    print("   EDGE IMPULSE AUDIO DATASET COLLECTOR - ESP32-S3 AMOLED        ")
    print(f"   Sample Rate: 16,000 Hz | 16-bit Mono | Format: .WAV           ")
    print("==================================================================")

    counts = list_existing_counts()
    print("Current Dataset Counts:")
    for c, cnt in counts.items():
        print(f"  - {c.ljust(12)}: {cnt} samples")
    print("==================================================================")
    print("Connecting to ESP32 on", port, "at", baud, "baud...")

    try:
        ser = serial.Serial(port, baud, timeout=2.0)
        time.sleep(1.0)
    except Exception as e:
        print(f"\n[ERROR] Could not open port {port}: {e}")
        print("Please check that the Arduino Serial Monitor is closed and board is plugged in.")
        sys.exit(1)

    print("[CONNECTED] ESP32 Ready!\n")

    try:
        while True:
            counts = list_existing_counts()
            print("------------------------------------------------------------------")
            print(f"Choose action to record a 1-second sample:")
            print(f"  [1] TIP BUCKET   (Current: {counts['tip']} samples)      - Tip left or right bucket")
            print(f"  [2] STEEL KNOCK  (Current: {counts['knock']} samples)    - Tap or knock the steel frame")
            print(f"  [3] BACKGROUND   (Current: {counts['background']} samples)   - Room silence, funnel water drops")
            print(f"  [u] Undo/Delete last tip sample")
            print(f"  [c] CLEAR ALL recordings and restart from 0")
            print(f"  [q] Quit")
            print("------------------------------------------------------------------")

            choice = input("Enter choice [1/2/3/u/c/q]: ").strip().lower()

            if choice == "1":
                record_clip(ser, "tip")
            elif choice == "2":
                record_clip(ser, "knock")
            elif choice == "3":
                record_clip(ser, "background")
            elif choice == "u":
                delete_last_sample("tip")
            elif choice == "c":
                confirm = input("Are you sure you want to delete ALL samples and restart? (y/n): ").strip().lower()
                if confirm == "y":
                    delete_all_dataset()
            elif choice == "q":
                print("Exiting collector tool. Goodbye!")
                break
            else:
                print("Invalid input. Please enter 1, 2, 3, or q.")
            print()

    finally:
        try:
            ser.write(b"X")
            ser.close()
        except:
            pass

if __name__ == "__main__":
    main()

