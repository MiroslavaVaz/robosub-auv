#!/usr/bin/env python3

import os
import json
import cv2
import numpy as np
import tensorflow as tf
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
sys.path.append("/home/vision/Desktop/RoboSub_Vision/.venv/lib/python3.9/site-packages")  # Adjust this path as needed

MODEL_PATH = "home/robosub/Desktop/RoboSub_Vision_2025-2026/.venv/lib/python3.10/site-packages/auv_vision/models/cnn_model.h5" 
CLASS_NAMES_PATH = "home/robosub/Desktop/RoboSub_Vision_2025-2026/Image_Processing/models/best_models.keras"
IMG_SIZE = (256, 256)
CAMERA_INDEX = 0
LOCK_THRESHOLD = 0.80

BOX_W = 220
BOX_H = 220


class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')

        if not os.path.exists(MODEL_PATH):
            raise FileNotFoundError(f"Model not found: {MODEL_PATH}")

        if not os.path.exists(CLASS_NAMES_PATH):
            raise FileNotFoundError(f"Class names file not found: {CLASS_NAMES_PATH}")

        self.model = tf.keras.models.load_model(MODEL_PATH)

        with open(CLASS_NAMES_PATH, "r") as f:
            self.class_names = json.load(f)

        self.get_logger().info(f"Loaded model: {MODEL_PATH}")
        self.get_logger().info(f"Loaded classes: {self.class_names}")

        self.publisher_ = self.create_publisher(String, 'vision_data', 10)

        self.cap = cv2.VideoCapture(CAMERA_INDEX)
        if not self.cap.isOpened():
            raise RuntimeError("Could not open camera.")

        self.timer = self.create_timer(0.1, self.process_frame)

    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning("Could not read frame.")
            return

        display = frame.copy()
        h, w, _ = frame.shape

        x1 = (w - BOX_W) // 2
        y1 = (h - BOX_H) // 2
        x2 = x1 + BOX_W
        y2 = y1 + BOX_H

        roi = frame[y1:y2, x1:x2]

        status = "NO_VALID_ROI"
        pred_class = "none"
        confidence = 0.0

        if roi.size != 0:
            roi_resized = cv2.resize(roi, IMG_SIZE)
            roi_rgb = cv2.cvtColor(roi_resized, cv2.COLOR_BGR2RGB)
            roi_scaled = roi_rgb.astype("float32") / 255.0
            roi_input = np.expand_dims(roi_scaled, axis=0)

            preds = self.model.predict(roi_input, verbose=0)[0]
            pred_idx = int(np.argmax(preds))
            pred_class = self.class_names[pred_idx]
            confidence = float(preds[pred_idx])

            if confidence >= LOCK_THRESHOLD:
                color = (0, 255, 0)
                status = "LOCKED"
            else:
                color = (0, 255, 255)
                status = "SEARCHING"

            cv2.rectangle(display, (x1, y1), (x2, y2), color, 2)

            label = f"{status}: {pred_class} ({confidence:.2f})"
            cv2.putText(
                display,
                label,
                (x1, max(30, y1 - 10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                color,
                2
            )

        cv2.putText(
            display,
            "Place object in center box | Press q to quit",
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2
        )

        msg = String()
        msg.data = f"status:{status},class:{pred_class},confidence:{confidence:.3f}"
        self.publisher_.publish(msg)

        cv2.imshow("Live CNN Camera Classifier", display)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            self.get_logger().info("Shutting down vision node...")
            self.cap.release()
            cv2.destroyAllWindows()
            rclpy.shutdown()

    def destroy_node(self):
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    rclpy.spin(node)
    node.destroy_node()


if __name__ == '__main__':
    main()