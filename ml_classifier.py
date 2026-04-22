import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

from sklearn.model_selection import train_test_split
from sklearn.ensemble import RandomForestClassifier
from sklearn.metrics import accuracy_score, confusion_matrix, classification_report
from sklearn.preprocessing import StandardScaler
import joblib

# =========================================================
# CONFIG
# =========================================================
CITY = "San Diego"
ROLL_WINDOW = 20

# =========================================================
# STEP 1: LOAD INDOOR DATA (Intel Lab Dataset)
# =========================================================
print("Loading indoor data...")

indoor = pd.read_csv(
    "data.txt",
    sep=r"\s+",
    header=None,
    names=["date", "time", "epoch", "mote_id", "temperature", "humidity", "light", "voltage"]
)

indoor = indoor[["temperature", "humidity"]].copy()
indoor = indoor[
    (indoor["humidity"].between(0, 100)) &
    (indoor["temperature"].between(-40, 85))  # realistic sensor range
]
# Synthetic pressure baseline (stable indoor assumption)
indoor["pressure"] = 101068

indoor["label"] = 0

# Optional downsample
indoor = indoor.sample(n=min(len(indoor), 5000), random_state=42)

print("Indoor samples:", len(indoor))

# =========================================================
# STEP 2: LOAD OUTDOOR DATA (Kaggle split files)
# =========================================================
print("Loading outdoor data (split sensors)...")

temp = pd.read_csv("temperature.csv")
press = pd.read_csv("pressure.csv")
humid = pd.read_csv("humidity.csv")

# Normalize column names
temp.columns = temp.columns.str.lower()
press.columns = press.columns.str.lower()
humid.columns = humid.columns.str.lower()

# Identify city column dynamically
cities = [c for c in temp.columns if c != "datetime"]

if CITY.lower() not in [c.lower() for c in cities]:
    raise ValueError(f"{CITY} not found. Available cities: {cities[:10]} ...")

# Find exact column match
city_col = next(c for c in temp.columns if c.lower() == CITY.lower())

# Extract city-specific data
temp_sd = temp[["datetime", city_col]].rename(columns={city_col: "temperature"})
temp_sd["temperature"] = temp_sd["temperature"].astype(float) - 273.15
press_sd = press[["datetime", city_col]].rename(columns={city_col: "pressure"})
press_sd["pressure"] = press_sd["pressure"].astype(float) * 100
humid_sd = humid[["datetime", city_col]].rename(columns={city_col: "humidity"})

# Merge sensors
outdoor = temp_sd.merge(press_sd, on="datetime")
outdoor = outdoor.merge(humid_sd, on="datetime")

outdoor = outdoor.dropna()

# Label
outdoor["label"] = 1
print(outdoor)
# Balance dataset
outdoor = outdoor.sample(n=min(len(outdoor), len(indoor)), random_state=42)

print("Outdoor samples:", len(outdoor))


# =========================================================
# STEP 3: COMBINE
# =========================================================
df = pd.concat([indoor, outdoor], ignore_index=True)
df = df.sample(frac=1, random_state=42).reset_index(drop=True)

print("Total dataset size:", len(df))


# =========================================================
# STEP 4: FEATURE ENGINEERING (IMPORTANT SIGNAL BOOST)
# =========================================================
print("Engineering features...")

df = df.sort_index()



# =========================================================
# STEP 5: FEATURES + SCALING
# =========================================================
features = [
    "temperature",
    "humidity",
    "pressure"
]

X = df[features]
y = df["label"]

scaler = StandardScaler()
X_scaled = scaler.fit_transform(X)


# =========================================================
# STEP 6: TRAIN / TEST SPLIT
# =========================================================
X_train, X_test, y_train, y_test = train_test_split(
    X_scaled, y,
    test_size=0.2,
    random_state=42,
    stratify=y
)

print("Train size:", len(X_train))
print("Test size:", len(X_test))


# =========================================================
# STEP 7: TRAIN MODEL
# =========================================================
print("Training model...")

model = RandomForestClassifier(
    n_estimators=200,
    random_state=42,
    max_depth=None
)

model.fit(X_train, y_train)


# =========================================================
# STEP 8: EVALUATION
# =========================================================
y_pred = model.predict(X_test)

print("\n=== RESULTS ===")
print("Accuracy:", accuracy_score(y_test, y_pred))
print("\nConfusion Matrix:\n", confusion_matrix(y_test, y_pred))
print("\nClassification Report:\n", classification_report(y_test, y_pred))


# =========================================================
# STEP 9: VISUALIZATION
# =========================================================
print("Plotting...")

plt.figure(figsize=(8, 6))

colors = {0: "blue", 1: "red"}
labels = {0: "Indoor", 1: "Outdoor"}

for label in df["label"].unique():
    subset = df[df["label"] == label]
    plt.scatter(
        subset["humidity"],
        subset["temperature"],
        c=colors[label],
        label=labels[label],
        alpha=0.3
    )

plt.xlabel("Humidity")
plt.ylabel("Temperature")
plt.title(f"Indoor vs Outdoor Classification ({CITY})")
plt.legend()
plt.show()


# =========================================================
# STEP 10: SAVE MODEL
# =========================================================
joblib.dump(model, "rf_sensor_model.pkl")
joblib.dump(scaler, "scaler.pkl")

print("Model + scaler saved!")