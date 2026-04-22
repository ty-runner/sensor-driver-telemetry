import re
import csv

input_file = "ks_data.txt"
output_file = "ks_metrics.csv"

e2e_pattern = re.compile(r"E2E Latency:\s+(\d+)\s+us")
exec_pattern = re.compile(r"Loop Exec Time:\s+(\d+)\s+us")
period_pattern = re.compile(r"Loop Period.*:\s+(\d+)\s+us")

data = []
current_entry = {
    "e2e_latency": None,
    "loop_exec_time": None,
    "loop_period": None
}

with open(input_file, "r") as f:
    for line in f:
        e2e_match = e2e_pattern.search(line)
        if e2e_match:
            current_entry["e2e_latency"] = int(e2e_match.group(1))

        exec_match = exec_pattern.search(line)
        if exec_match:
            current_entry["loop_exec_time"] = int(exec_match.group(1))

        period_match = period_pattern.search(line)
        if period_match:
            current_entry["loop_period"] = int(period_match.group(1))

        if (current_entry["e2e_latency"] is not None and
            current_entry["loop_exec_time"] is not None):

            data.append(current_entry.copy())

            # reset
            current_entry = {
                "e2e_latency": None,
                "loop_exec_time": None,
                "loop_period": None
            }

# Write CSV
with open(output_file, "w", newline="") as csvfile:
    writer = csv.writer(csvfile)

    writer.writerow(["E2E Latency (us)", "Loop Exec Time (us)", "Loop Period (us)"])

    for entry in data:
        writer.writerow([
            entry["e2e_latency"],
            entry["loop_exec_time"],
            entry["loop_period"]
        ])

print(f"CSV written to {output_file}")