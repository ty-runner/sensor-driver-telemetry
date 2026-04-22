import socket
import struct
import csv
import os
import joblib
import pandas as pd

# -----------------------------
# Load trained model
# -----------------------------
model = joblib.load("rf_sensor_model.pkl")  # your trained RF model
feature_cols = ["temperature", "pressure", "humidity"]

# -----------------------------
# UDP setup
# -----------------------------
fmt = "<Q i I I H"
filename = "sensor_data_PREDICTIONS.csv"

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", 5005))
print("Listening on UDP 5005...")

# CSV file setup
file_exists = os.path.isfile(filename)
csvfile = open(filename, "a", newline="")
writer = csv.writer(csvfile)
if not file_exists:
    writer.writerow(["temperature", "humidity", "pressure", "predicted_label"])

# -----------------------------
# Receive loop
# -----------------------------
while True:
    data, addr = sock.recvfrom(1024)

    if len(data) != struct.calcsize(fmt):
        print("Unexpected size:", len(data))
        continue

    ts, temp, hum, press, crc = struct.unpack(fmt, data)
    temp_c = temp / 100.0

    # -----------------------------
    # Make prediction
    # -----------------------------
    sample_df = pd.DataFrame([[temp_c, press, hum]], columns=feature_cols)
    pred_label = model.predict(sample_df)[0]  # 0=Indoor, 1=Outdoor
    label_str = "Indoor" if pred_label == 0 else "Outdoor"
    probs = model.predict_proba(sample_df)[0]
    print(f"Prob Indoor: {probs[0]:.2f}, Prob Outdoor: {probs[1]:.2f}")
    print(f"Timestamp: {ts}")
    print(f"Temp (C): {temp_c}")
    print(f"Humidity: {hum}")
    print(f"Pressure: {press}")
    print(f"Predicted: {label_str}")
    print("------")

    # -----------------------------
    # Write to CSV
    # -----------------------------
    #writer.writerow([temp_c, hum, press, label_str])
    #writer.writerow([temp_c, press, hum])
    csvfile.flush()