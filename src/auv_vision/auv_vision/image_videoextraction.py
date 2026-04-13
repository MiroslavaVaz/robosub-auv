import cv2
from pathlib import Path

# ---------------------------------
# SETTINGS
# ---------------------------------
VIDEO_ROOT = Path(r"C:\Users\samst\Desktop\SP_2026_Vision\data\videos_raw\Blood")
OUTPUT_DIR = Path(r"C:\Users\samst\Desktop\SP_2026_Vision\data\images_raw\Blood")

VALID_EXTENSIONS = {".mov", ".mp4", ".avi", ".mkv"}
FRAME_SKIP = 10
IMG_SIZE = (256, 256)

# ---------------------------------
# CHECK ROOT FOLDERS
# ---------------------------------
if not VIDEO_ROOT.exists():
    raise FileNotFoundError(f"Video folder not found: {VIDEO_ROOT}")

if not OUTPUT_DIR.exists():
    raise FileNotFoundError(f"Output folder not found: {OUTPUT_DIR}")

# ---------------------------------
# PROCESS VIDEOS
# ---------------------------------
video_files = [
    video_path for video_path in VIDEO_ROOT.iterdir()
    if video_path.is_file() and video_path.suffix.lower() in VALID_EXTENSIONS
]

if not video_files:
    raise FileNotFoundError(f"No video files found in: {VIDEO_ROOT}")

for video_path in video_files:
    print(f"\nProcessing video: {video_path.name}")

    cap = cv2.VideoCapture(str(video_path))

    if not cap.isOpened():
        print(f"[ERROR] Could not open {video_path}")
        continue

    frame_idx = 0
    saved_idx = 0
    video_name = video_path.stem

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        if frame_idx % FRAME_SKIP == 0:
            try:
                frame_resized = cv2.resize(frame, IMG_SIZE)
            except Exception as e:
                print(f"[ERROR] Resize failed at frame {frame_idx}: {e}")
                frame_idx += 1
                continue

            filename = f"{video_name}_frame_{saved_idx:05d}.jpg"
            save_path = OUTPUT_DIR / filename

            success = cv2.imwrite(str(save_path), frame_resized)
            if success:
                saved_idx += 1
            else:
                print(f"[ERROR] Failed to save image: {save_path}")

        frame_idx += 1

    cap.release()
    print(f"Saved {saved_idx} frames from {video_path.name}")

print("\nDone.")