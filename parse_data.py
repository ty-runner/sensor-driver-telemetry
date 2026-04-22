

import csv
import os
import re
input_file = r".\Desktop\Github\sensor-driver-telemetry\ks_data.txt"
output_file = ".\Desktop\Github\sensor-driver-telemetry\sensor_data_indoor.csv"

# Patterns
temp_pattern = re.compile(r"Temp:\s*([\d.]+)")
press_pattern = re.compile(r"Pressure:\s*([\d.]+)")
hum_pattern = re.compile(r"Humidity:\s*([\d.]+)")

# Prepare CSV (append mode)
file_exists = os.path.isfile(output_file)
csvfile = open(output_file, "a", newline="")
writer = csv.writer(csvfile)

# Write header if new file
if not file_exists:
    writer.writerow(["temperature_C", "pressure_Pa", "humidity_percent"])

temp = None
press = None
hum = None

with open(input_file, "r") as infile:
    for line in infile:
        if "Temp:" in line:
            match = temp_pattern.search(line)
            if match:
                temp = float(match.group(1))

        elif "Pressure:" in line:
            match = press_pattern.search(line)
            if match:
                press = float(match.group(1))

        elif "Humidity:" in line:
            match = hum_pattern.search(line)
            if match:
                hum = float(match.group(1))

        # Once all three are captured → write row
        if temp is not None and press is not None and hum is not None:
            writer.writerow([temp, press, hum])
            csvfile.flush()

            # Reset for next reading
            temp, press, hum = None, None, None

print("Done. Data appended to:", output_file)