import cv2
import numpy as np
from pathlib import Path
import random

# ---------------------------------
# SETTINGS
# ---------------------------------
INPUT_DIR = Path(r"C:\Users\samst\Desktop\SP_2026_Vision\data\images_raw\Firetruck")
OUTPUT_DIR = Path(r"C:\Users\samst\Desktop\SP_2026_Vision\data\augmented_images\Firetruck")

VALID_EXTENSIONS = {".jpg", ".jpeg", ".png"}
IMG_SIZE = (256, 256)

AUGS_PER_IMAGE = 8
ROTATION_RANGE = 35
TRANSLATE_PIXELS = 40
SCALE_RANGE = 0.20
BRIGHTNESS_RANGE = (0.7, 1.3)
CONTRAST_RANGE = (0.75, 1.25)
COLOR_RANGE = (0.8, 1.2)

random.seed()
np.random.seed()

# ---------------------------------
# AUGMENTATION FUNCTIONS
# ---------------------------------
def rotate_scale_translate(image, angle, scale, tx, ty):
    h, w = image.shape[:2]
    center = (w // 2, h // 2)
    matrix = cv2.getRotationMatrix2D(center, angle, scale)
    matrix[0, 2] += tx
    matrix[1, 2] += ty
    return cv2.warpAffine(
        image,
        matrix,
        (w, h),
        flags=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_REFLECT_101
    )

def adjust_brightness_contrast(image, brightness_factor, contrast_factor):
    img = image.astype(np.float32)
    img = img * contrast_factor
    img = img * brightness_factor
    img = np.clip(img, 0, 255)
    return img.astype(np.uint8)

def adjust_color_channels(image, b_gain, g_gain, r_gain):
    img = image.astype(np.float32).copy()
    img[:, :, 0] *= b_gain
    img[:, :, 1] *= g_gain
    img[:, :, 2] *= r_gain
    img = np.clip(img, 0, 255)
    return img.astype(np.uint8)

def add_gaussian_noise(image, sigma=10):
    noise = np.random.normal(0, sigma, image.shape).astype(np.float32)
    img = image.astype(np.float32) + noise
    img = np.clip(img, 0, 255)
    return img.astype(np.uint8)

def random_blur(image):
    k = random.choice([3, 5])
    return cv2.GaussianBlur(image, (k, k), 0)

def random_crop_and_resize(image, crop_scale=0.85):
    h, w = image.shape[:2]
    new_h = int(h * random.uniform(crop_scale, 1.0))
    new_w = int(w * random.uniform(crop_scale, 1.0))

    y1 = random.randint(0, h - new_h)
    x1 = random.randint(0, w - new_w)

    cropped = image[y1:y1+new_h, x1:x1+new_w]
    return cv2.resize(cropped, (w, h))

# ---------------------------------
# CHECK FOLDERS
# ---------------------------------
if not INPUT_DIR.exists():
    raise FileNotFoundError(f"Input folder not found: {INPUT_DIR}")

if not OUTPUT_DIR.exists():
    raise FileNotFoundError(f"Output folder not found: {OUTPUT_DIR}")

image_files = [
    p for p in INPUT_DIR.iterdir()
    if p.is_file() and p.suffix.lower() in VALID_EXTENSIONS
]

print(f"Found {len(image_files)} image(s) in {INPUT_DIR}")

if not image_files:
    raise FileNotFoundError(f"No images found in: {INPUT_DIR}")

# ---------------------------------
# PROCESS IMAGES
# ---------------------------------
for img_path in image_files:
    image = cv2.imread(str(img_path))

    if image is None:
        print(f"[WARNING] Could not read: {img_path.name}")
        continue

    image = cv2.resize(image, IMG_SIZE)
    base_name = img_path.stem

    for i in range(AUGS_PER_IMAGE):
        aug = image.copy()

        # Stronger geometric changes
        angle = random.uniform(-ROTATION_RANGE, ROTATION_RANGE)
        scale = random.uniform(1.0 - SCALE_RANGE, 1.0 + SCALE_RANGE)
        tx = random.randint(-TRANSLATE_PIXELS, TRANSLATE_PIXELS)
        ty = random.randint(-TRANSLATE_PIXELS, TRANSLATE_PIXELS)
        aug = rotate_scale_translate(aug, angle, scale, tx, ty)

        # Random crop sometimes
        if random.random() < 0.5:
            aug = random_crop_and_resize(aug, crop_scale=0.75)

        # Brightness/contrast
        brightness_factor = random.uniform(*BRIGHTNESS_RANGE)
        contrast_factor = random.uniform(*CONTRAST_RANGE)
        aug = adjust_brightness_contrast(aug, brightness_factor, contrast_factor)

        # Mild color shifts
        b_gain = random.uniform(*COLOR_RANGE)
        g_gain = random.uniform(*COLOR_RANGE)
        r_gain = random.uniform(*COLOR_RANGE)
        aug = adjust_color_channels(aug, b_gain, g_gain, r_gain)

        # Horizontal flip sometimes
        if random.random() < 0.5:
            aug = cv2.flip(aug, 1)

        # Blur sometimes
        if random.random() < 0.3:
            aug = random_blur(aug)

        # Noise sometimes
        if random.random() < 0.3:
            aug = add_gaussian_noise(aug, sigma=random.uniform(5, 15))

        save_name = f"{base_name}_aug_{i:03d}.jpg"
        save_path = OUTPUT_DIR / save_name

        success = cv2.imwrite(str(save_path), aug)
        if not success:
            print(f"[WARNING] Failed to save: {save_path}")

    print(f"Augmented: {img_path.name}")

print("\nDone.")