import os
import json
import cv2
import numpy as np
import tensorflow as tf
import matplotlib.pyplot as plt

# ------------------ SETTINGS ------------------
MODEL_PATH = "models/best_model.keras"          # use best saved model
CLASS_NAMES_PATH = "models/class_names.json"
IMG_PATH = r"C:\Users\samst\Desktop\SP_2026_Vision\data\cnn\test\fire\Fire_frame_00017.jpg"          # change this to your test image
IMG_SIZE = (256, 256)

# ------------------ CHECK FILES ------------------
if not os.path.exists(MODEL_PATH):
    raise FileNotFoundError(f"Model not found: {MODEL_PATH}")

if not os.path.exists(CLASS_NAMES_PATH):
    raise FileNotFoundError(f"Class names file not found: {CLASS_NAMES_PATH}")

if not os.path.exists(IMG_PATH):
    raise FileNotFoundError(f"Image not found: {IMG_PATH}")

# ------------------ LOAD MODEL AND CLASSES ------------------
model = tf.keras.models.load_model(MODEL_PATH)

with open(CLASS_NAMES_PATH, "r") as f:
    class_names = json.load(f)

print("Loaded classes:", class_names)

# ------------------ LOAD AND PREPROCESS IMAGE ------------------
img = cv2.imread(IMG_PATH)

if img is None:
    raise ValueError(f"Could not read image: {IMG_PATH}")

img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
img_resized = cv2.resize(img, IMG_SIZE)
img_scaled = img_resized.astype("float32") / 255.0
img_input = np.expand_dims(img_scaled, axis=0)

# ------------------ PREDICT ------------------
predictions = model.predict(img_input)[0]
pred_index = int(np.argmax(predictions))
pred_class = class_names[pred_index]
confidence = float(predictions[pred_index])

# ------------------ SHOW RESULT ------------------
plt.imshow(img_resized)
plt.title(f"Predicted: {pred_class} | Confidence: {confidence:.4f}")
plt.axis("off")
plt.show()

print("\nPrediction probabilities:")
for i, prob in enumerate(predictions):
    print(f"{class_names[i]}: {float(prob):.4f}")

print(f"\nFinal Prediction: {pred_class}")
print(f"Confidence: {confidence:.4f}")