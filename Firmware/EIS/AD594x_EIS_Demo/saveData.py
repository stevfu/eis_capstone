import serial
import csv
from datetime import datetime
 
# --- Configuration ---
port = 'COM5'  # Change to your Arduino port (e.g., '/dev/ttyACM0' on Linux/Mac)
baudrate = 115200
csv_filename = f"arduino_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
 
# --- Connect to Serial ---
try:
    ser = serial.Serial(port, baudrate, timeout=1)
    print(f"Connected to {port} at {baudrate} baud.")
except serial.SerialException:
    print(f"Failed to connect on port {port}.")
    exit()
 
# --- Create CSV File ---
with open(csv_filename, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(["Voltage (V)", "Current (A)", "Time (s)"])  # Column headers
 
    try:
        print("Logging data. Press Ctrl+C to stop.")
        while True:
            line = ser.readline().decode('utf-8').strip()
            if line:
                try:
                    data = line.split(",")
                    if len(data) == 3:
                        writer.writerow(data)
                        print("Logged:", data)
                    else:
                        print("Skipping malformed line:", line)
                except Exception as e:
                    print("Error parsing line:", e)
    except KeyboardInterrupt:
        print("\nLogging stopped by user.")
    finally:
        ser.close()
        print(f"Serial connection closed. Data saved to '{csv_filename}'.")