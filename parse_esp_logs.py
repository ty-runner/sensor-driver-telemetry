import re
import csv

# Input / Output files
input_file = "esp_data.txt"
output_file = "metrics.csv"

# Regex patterns
e2e_pattern = re.compile(r"E2E Latency:\s+(\d+)\s+us")
exec_pattern = re.compile(r"Loop Exec Time:\s+(\d+)\s+us")
period_pattern = re.compile(r"Loop Period.*:\s+(\d+)\s+us")

# Storage
data = []
current_entry = {
    "e2e_latency": None,
    "loop_exec_time": None,
    "loop_period": None
}

with open(input_file, "r") as f:
    for line in f:
        # Match E2E Latency
        e2e_match = e2e_pattern.search(line)
        if e2e_match:
            current_entry["e2e_latency"] = int(e2e_match.group(1))

        # Match Loop Exec Time
        exec_match = exec_pattern.search(line)
        if exec_match:
            current_entry["loop_exec_time"] = int(exec_match.group(1))

        # Match Loop Period
        period_match = period_pattern.search(line)
        if period_match:
            current_entry["loop_period"] = int(period_match.group(1))

            # Assume full set complete → save row
            data.append(current_entry.copy())

            # Reset for next entry
            current_entry = {
                "e2e_latency": None,
                "loop_exec_time": None,
                "loop_period": None
            }

# Write CSV
with open(output_file, "w", newline="") as csvfile:
    writer = csv.writer(csvfile)
    
    # Header
    writer.writerow(["E2E Latency (us)", "Loop Exec Time (us)", "Loop Period (us)"])
    
    # Rows
    for entry in data:
        writer.writerow([
            entry["e2e_latency"],
            entry["loop_exec_time"],
            entry["loop_period"]
        ])

print(f"CSV written to {output_file}")