import time
import threading
import u12
from File_Setup import write_csv

def log_voltages(d: u12.U12, path, start_time, interval=1.0):
    """Read AI0-AI2 and write a timestamped row to the CSV every `interval` seconds."""
    voltages = []

    for channel in range(3):
        reading = d.eAnalogIn(channel)
        #print(f"AI{channel}: {reading['voltage']} V")
        voltages.append(reading['voltage'])

    elapsed_time = time.time() - start_time
    write_csv(path, [elapsed_time] + voltages)

    # Schedule the next call
    threading.Timer(interval, log_voltages, args=[d, path, start_time, interval]).start()
