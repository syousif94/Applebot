"""
Test OSNet ReID model with multiple images.

Usage:
  # Compare specific images:
  uv run python test_reid.py image1.jpg image2.jpg image3.jpg

  # Compare all images in a folder:
  uv run python test_reid.py ./test_images/

  # Use webcam to capture test shots (press SPACE to capture, Q to quit):
  uv run python test_reid.py --webcam
"""

import sys
import os
import numpy as np
import coremltools as ct
from PIL import Image
from itertools import combinations

MODEL_PATH = os.path.join(os.path.dirname(__file__), "..", "RoboCar", "Models", "OSNetReID.mlpackage")
INPUT_SIZE = (128, 256)  # width x height


def load_model():
    print(f"Loading model from {MODEL_PATH}")
    return ct.models.MLModel(MODEL_PATH)


def get_embedding(model, image_path):
    """Get 512-dim embedding from a person crop image."""
    img = Image.open(image_path).convert("RGB")
    img_resized = img.resize(INPUT_SIZE, Image.LANCZOS)
    result = model.predict({"image": img_resized})
    return result["embedding"].flatten()


def cosine_similarity(a, b):
    """Cosine similarity between two L2-normalized vectors."""
    return float(np.dot(a, b))


def collect_images(paths):
    """Collect image file paths from arguments (files or directories)."""
    extensions = {".jpg", ".jpeg", ".png", ".bmp", ".webp", ".heic"}
    images = []
    for p in paths:
        if os.path.isdir(p):
            for f in sorted(os.listdir(p)):
                if os.path.splitext(f)[1].lower() in extensions:
                    images.append(os.path.join(p, f))
        elif os.path.isfile(p):
            images.append(p)
        else:
            print(f"Warning: {p} not found, skipping")
    return images


def capture_webcam_images():
    """Capture images from webcam for testing."""
    try:
        import cv2
    except ImportError:
        print("OpenCV not installed. Run: uv add opencv-python")
        sys.exit(1)

    output_dir = os.path.join(os.path.dirname(__file__), "test_captures")
    os.makedirs(output_dir, exist_ok=True)

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Could not open webcam")
        sys.exit(1)

    print("Webcam capture mode:")
    print("  SPACE = capture image")
    print("  Q     = quit and compare all captures")
    print()

    captured = []
    count = 0
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Draw crosshair and info
        h, w = frame.shape[:2]
        cv2.putText(frame, f"Captured: {count} | SPACE=capture Q=quit",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.rectangle(frame, (w//4, h//8), (3*w//4, 7*h//8), (0, 255, 0), 2)
        cv2.imshow("OSNet ReID Test - Capture", frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord(' '):
            count += 1
            path = os.path.join(output_dir, f"capture_{count:03d}.jpg")
            cv2.imwrite(path, frame)
            captured.append(path)
            print(f"  Captured #{count}: {path}")
        elif key == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    return captured


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    # Webcam mode
    if "--webcam" in sys.argv:
        images = capture_webcam_images()
        if len(images) < 2:
            print("Need at least 2 captures to compare")
            sys.exit(1)
    else:
        images = collect_images(sys.argv[1:])

    if len(images) < 2:
        print(f"Need at least 2 images to compare, got {len(images)}")
        sys.exit(1)

    print(f"\nFound {len(images)} images:")
    for img in images:
        print(f"  {os.path.basename(img)}")

    # Load model and compute embeddings
    model = load_model()
    print("\nComputing embeddings...")

    embeddings = {}
    for img_path in images:
        name = os.path.basename(img_path)
        embeddings[name] = get_embedding(model, img_path)
        norm = np.linalg.norm(embeddings[name])
        print(f"  {name}: norm={norm:.4f}")

    # Pairwise similarity matrix
    names = list(embeddings.keys())
    print(f"\n{'Pairwise Cosine Similarity':=^60}")
    print()

    # Header
    max_name = max(len(n) for n in names)
    header = " " * (max_name + 2)
    for n in names:
        header += f"{n[:12]:>13}"
    print(header)

    # Matrix
    for i, n1 in enumerate(names):
        row = f"{n1:<{max_name}}  "
        for j, n2 in enumerate(names):
            sim = cosine_similarity(embeddings[n1], embeddings[n2])
            if i == j:
                row += f"{'1.000':>13}"
            else:
                # Color coding via threshold
                marker = "✓" if sim > 0.6 else "✗"
                row += f"{sim:>11.4f} {marker}"
        print(row)

    # Sorted pairs
    print(f"\n{'All Pairs (sorted by similarity)':=^60}")
    print()
    pairs = []
    for (n1, e1), (n2, e2) in combinations(embeddings.items(), 2):
        sim = cosine_similarity(e1, e2)
        pairs.append((n1, n2, sim))
    pairs.sort(key=lambda x: -x[2])

    for n1, n2, sim in pairs:
        bar_len = int(sim * 30) if sim > 0 else 0
        bar = "█" * bar_len + "░" * (30 - bar_len)
        match = "SAME PERSON" if sim > 0.6 else "DIFFERENT" if sim < 0.4 else "UNCERTAIN"
        print(f"  {n1} ↔ {n2}")
        print(f"    {bar} {sim:.4f}  [{match}]")
        print()

    print("Thresholds: >0.6 = likely same person, <0.4 = likely different")
    print("Note: These thresholds may need tuning for your use case.")


if __name__ == "__main__":
    main()
