import os
import json
import cv2
import numpy as np
import tensorflow as tf

# -----------------------------
# SETTINGS
# -----------------------------
MODEL_PATH = "models/best_model.keras"
CLASS_NAMES_PATH = "models/class_names.json"
IMG_SIZE = (256, 256)
CAMERA_INDEX = 0
LOCK_THRESHOLD = 0.80   # confidence needed to say "LOCKED"

# Center box size
BOX_W = 220
BOX_H = 220

# -----------------------------
# CHECK FILES
# -----------------------------
if not os.path.exists(MODEL_PATH):
    raise FileNotFoundError(f"Model not found: {MODEL_PATH}")

if not os.path.exists(CLASS_NAMES_PATH):
    raise FileNotFoundError(f"Class names file not found: {CLASS_NAMES_PATH}")

# -----------------------------
# LOAD MODEL AND CLASS NAMES
# -----------------------------
model = tf.keras.models.load_model(MODEL_PATH)

with open(CLASS_NAMES_PATH, "r") as f:
    class_names = json.load(f)

print("Loaded model:", MODEL_PATH)
print("Loaded classes:", class_names)

# -----------------------------
# CAMERA
# -----------------------------
cap = cv2.VideoCapture(CAMERA_INDEX)

if not cap.isOpened():
    raise RuntimeError("Could not open camera.")

print("Press 'q' to quit.")

while True:
    ret, frame = cap.read()
    if not ret:
        print("Could not read frame.")
        break

    display = frame.copy()
    h, w, _ = frame.shape

    # Center box coordinates
    x1 = (w - BOX_W) // 2
    y1 = (h - BOX_H) // 2
    x2 = x1 + BOX_W
    y2 = y1 + BOX_H

    # Crop ROI (region of interest)
    roi = frame[y1:y2, x1:x2]

    # Make sure ROI is valid
    if roi.size != 0:
        # Preprocess for CNN
        roi_resized = cv2.resize(roi, IMG_SIZE)
        roi_rgb = cv2.cvtColor(roi_resized, cv2.COLOR_BGR2RGB)
        roi_scaled = roi_rgb.astype("float32") / 255.0
        roi_input = np.expand_dims(roi_scaled, axis=0)

        # Predict
        preds = model.predict(roi_input, verbose=0)[0]
        pred_idx = int(np.argmax(preds))
        pred_class = class_names[pred_idx]
        confidence = float(preds[pred_idx])

        # Decide lock state
        if confidence >= LOCK_THRESHOLD:
            color = (0, 255, 0)
            status = f"LOCKED: {pred_class} ({confidence:.2f})"
        else:
            color = (0, 255, 255)
            status = f"SEARCHING: {pred_class} ({confidence:.2f})"

        # Draw box
        cv2.rectangle(display, (x1, y1), (x2, y2), color, 2)

        # Draw label
        cv2.putText(
            display,
            status,
            (x1, max(30, y1 - 10)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            color,
            2
        )

    # Show instructions
    cv2.putText(
        display,
        "Place object in center box | Press q to quit",
        (20, 35),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (255, 255, 255),
        2
    )

    cv2.imshow("Live CNN Camera Classifier", display)

    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()