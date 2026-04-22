# sensor_classifier.py
import pandas as pd
from sklearn.model_selection import train_test_split
from sklearn.linear_model import LogisticRegression
from sklearn.ensemble import RandomForestClassifier
from sklearn.metrics import accuracy_score, confusion_matrix, classification_report
import matplotlib.pyplot as plt
import joblib


# -----------------------------
# Step 1: Load and combine datasets
# -----------------------------
indoor = pd.read_csv("sensor_data_indoor.csv")
outdoor = pd.read_csv("sensor_data_outdoor.csv")
print(indoor.describe())
print(outdoor.describe())
indoor["label"] = 0  # indoor
outdoor["label"] = 1  # outdoor

df = pd.concat([indoor, outdoor], ignore_index=True)

# -----------------------------
# Step 2: Balance the dataset
# -----------------------------
# Downsample outdoor to match indoor count
#outdoor_sampled = outdoor.sample(n=len(indoor), random_state=42)
outdoor_sampled = outdoor.sample(n=len(indoor))
df_balanced = pd.concat([indoor, outdoor_sampled], ignore_index=True)

# Optional: shuffle
df_balanced = df_balanced.sample(frac=1, random_state=42).reset_index(drop=True)

# -----------------------------
# Step 3: Train/test split
# -----------------------------
feature_cols = ["temperature", "pressure", "humidity"]
X = df_balanced[feature_cols]
y = df_balanced["label"]

X_train, X_test, y_train, y_test = train_test_split(
    X, y, test_size=0.2, random_state=42
)
print(X_train.describe())
# -----------------------------
# Step 4: Train a simple model
# -----------------------------
# Logistic Regression baseline
log_model = LogisticRegression()
log_model.fit(X_train, y_train)

# -----------------------------
# Step 5: Evaluate Logistic Regression
# -----------------------------
y_pred = log_model.predict(X_test)

print("=== Logistic Regression ===")
print("Accuracy:", accuracy_score(y_test, y_pred))
print("Confusion Matrix:\n", confusion_matrix(y_test, y_pred))
print("Classification Report:\n", classification_report(y_test, y_pred))

# -----------------------------
# Optional Step 6: Stronger model
# -----------------------------
rf_model = RandomForestClassifier(n_estimators=100, random_state=42)
rf_model.fit(X_train, y_train)
y_pred_rf = rf_model.predict(X_test)

print("\n=== Random Forest ===")
print("Accuracy:", accuracy_score(y_test, y_pred_rf))
print("Confusion Matrix:\n", confusion_matrix(y_test, y_pred_rf))
print("Classification Report:\n", classification_report(y_test, y_pred_rf))

# -----------------------------
# Step 8: Quick sanity check visualization
# -----------------------------
# Map labels to colors manually for clarity
colors = {0: "blue", 1: "red"}
labels = {0: "Indoor", 1: "Outdoor"}

plt.figure(figsize=(8, 6))

for label in df["label"].unique():
    subset = df[df["label"] == label]
    plt.scatter(
        subset["humidity"],
        subset["temperature"],
        c=colors[label],
        label=labels[label],
        alpha=0.4
    )

plt.xlabel("Humidity (%)")
plt.ylabel("Temperature (°C)")
plt.title("Indoor vs Outdoor Sensor Data")
plt.legend()  # Add legend
plt.show()

# Save Random Forest model
joblib.dump(rf_model, "rf_sensor_model.pkl")