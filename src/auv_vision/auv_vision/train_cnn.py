import os
import json
import tensorflow as tf
import matplotlib.pyplot as plt
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Conv2D, Dense, Flatten, MaxPooling2D, Input

# ------------------ SETTINGS ------------------
IMG_SIZE = (256, 256)
BATCH_SIZE = 32
EPOCHS = 10

TRAIN_DIR = "data/cnn/train"
VAL_DIR = "data/cnn/val"
TEST_DIR = "data/cnn/test"

MODEL_SAVE_PATH = "models/task_classifier.keras"
BEST_MODEL_PATH = "models/best_model.keras"
CLASS_NAMES_PATH = "models/class_names.json"
LOG_DIR = "logs"

# ------------------ DEBUG PATH CHECK ------------------
print("Current working directory:", os.getcwd())
print("Train path exists:", os.path.exists(TRAIN_DIR))
print("Val path exists:", os.path.exists(VAL_DIR))
print("Test path exists:", os.path.exists(TEST_DIR))

# ------------------ LOAD DATA ------------------
train_ds = tf.keras.utils.image_dataset_from_directory(
    TRAIN_DIR,
    image_size=IMG_SIZE,
    batch_size=BATCH_SIZE
)

val_ds = tf.keras.utils.image_dataset_from_directory(
    VAL_DIR,
    image_size=IMG_SIZE,
    batch_size=BATCH_SIZE
)

test_ds = tf.keras.utils.image_dataset_from_directory(
    TEST_DIR,
    image_size=IMG_SIZE,
    batch_size=BATCH_SIZE,
    shuffle=False
)

class_names = train_ds.class_names
num_classes = len(class_names)

print("Classes:", class_names)
print("Number of classes:", num_classes)

# Save class names so prediction script stays in sync
os.makedirs("models", exist_ok=True)
with open(CLASS_NAMES_PATH, "w") as f:
    json.dump(class_names, f)

# ------------------ NORMALIZATION ------------------
train_ds = train_ds.map(lambda x, y: (x / 255.0, y))
val_ds = val_ds.map(lambda x, y: (x / 255.0, y))
test_ds = test_ds.map(lambda x, y: (x / 255.0, y))

# ------------------ DATA AUGMENTATION ------------------
data_augmentation = tf.keras.Sequential([
    tf.keras.layers.RandomFlip("horizontal"),
    tf.keras.layers.RandomRotation(0.1),
    tf.keras.layers.RandomZoom(0.1),
])

train_ds = train_ds.map(lambda x, y: (data_augmentation(x, training=True), y))

# Improve pipeline performance
AUTOTUNE = tf.data.AUTOTUNE
train_ds = train_ds.prefetch(buffer_size=AUTOTUNE)
val_ds = val_ds.prefetch(buffer_size=AUTOTUNE)
test_ds = test_ds.prefetch(buffer_size=AUTOTUNE)

# ------------------ MODEL ------------------
model = Sequential([
    Input(shape=(256, 256, 3)),
    Conv2D(16, (3, 3), activation='relu'),
    MaxPooling2D(),

    Conv2D(32, (3, 3), activation='relu'),
    MaxPooling2D(),

    Conv2D(64, (3, 3), activation='relu'),
    MaxPooling2D(),

    Flatten(),
    Dense(256, activation='relu'),
    Dense(num_classes, activation='softmax')
])

model.compile(
    optimizer='adam',
    loss='sparse_categorical_crossentropy',
    metrics=['accuracy']
)

model.summary()

# ------------------ CALLBACKS ------------------
tensorboard_callback = tf.keras.callbacks.TensorBoard(log_dir=LOG_DIR)

checkpoint_callback = tf.keras.callbacks.ModelCheckpoint(
    BEST_MODEL_PATH,
    monitor="val_loss",
    save_best_only=True
)

# ------------------ TRAIN ------------------
history = model.fit(
    train_ds,
    validation_data=val_ds,
    epochs=EPOCHS,
    callbacks=[tensorboard_callback, checkpoint_callback]
)

# Save final model
model.save(MODEL_SAVE_PATH)
print(f"Final model saved to: {MODEL_SAVE_PATH}")
print(f"Best model saved to: {BEST_MODEL_PATH}")
print(f"Class names saved to: {CLASS_NAMES_PATH}")

# ------------------ PLOT TRAINING CURVES ------------------
plt.figure(figsize=(12, 5))

plt.subplot(1, 2, 1)
plt.plot(history.history['accuracy'], label='train_accuracy')
plt.plot(history.history['val_accuracy'], label='val_accuracy')
plt.title('Accuracy')
plt.xlabel('Epoch')
plt.ylabel('Accuracy')
plt.legend()

plt.subplot(1, 2, 2)
plt.plot(history.history['loss'], label='train_loss')
plt.plot(history.history['val_loss'], label='val_loss')
plt.title('Loss')
plt.xlabel('Epoch')
plt.ylabel('Loss')
plt.legend()

plt.tight_layout()
plt.show()

# ------------------ TEST EVALUATION ------------------
test_loss, test_accuracy = model.evaluate(test_ds)
print("Test Loss:", test_loss)
print("Test Accuracy:", test_accuracy)