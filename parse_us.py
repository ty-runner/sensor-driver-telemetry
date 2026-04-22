import re
import csv

# Input/output files
input_file = "up_data.txt"
output_file = "up_metrics.csv"

# Compile regex patterns for Exec, E2E, Jitter
exec_pattern = re.compile(r"Exec:\s*([\d.]+)\s*s")
e2e_pattern = re.compile(r"E2E:\s*([\d.]+)\s*s")
jitter_pattern = re.compile(r"Jitter:\s*([\d.]+)\s*s")

# Open the input file and output CSV
with open(input_file, 'r') as f_in, open(output_file, 'w', newline='') as f_out:
    writer = csv.writer(f_out)
    # Write header
    writer.writerow(["Exec_us", "E2E_us", "Jitter_us"])

    # Process each line
    for line in f_in:
        exec_match = exec_pattern.search(line)
        e2e_match = e2e_pattern.search(line)
        jitter_match = jitter_pattern.search(line)

        # Skip line if any metric is missing
        if not (exec_match and e2e_match and jitter_match):
            continue

        # Convert from seconds to microseconds
        exec_us = float(exec_match.group(1)) * 1e6
        e2e_us = float(e2e_match.group(1)) * 1e6
        jitter_us = float(jitter_match.group(1)) * 1e6

        # Write row to CSV
        writer.writerow([exec_us, e2e_us, jitter_us])

print(f"Parsed metrics saved to {output_file}")