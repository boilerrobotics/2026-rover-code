import argparse
import os
from pathlib import Path
import time

os.environ.setdefault("YOLO_CONFIG_DIR", "/tmp/ultralytics")

import cv2
from ultralytics import YOLO

def parse_args():
    parser = argparse.ArgumentParser(
        description="Rover Object Detection Prototype (Supports Custom Models & Video/Webcam Feeds)"
    )
    parser.add_argument(
        "--model",
        "-m",
        type=str,
        default="yolov8n.pt",
        help="Path to YOLO model weights (.pt file), e.g. 'yolov8n.pt' or custom 'hammer_best.pt'.",
    )
    parser.add_argument(
        "--source",
        "-s",
        type=str,
        default=str(Path(__file__).parent / "data" / "test_image2.jpg"),
        help="Input source: '0' for live webcam, a video file path ('test.mp4'), or an image path.",
    )
    parser.add_argument(
        "--conf",
        "-c",
        type=float,
        default=0.25,
        help="Confidence threshold for detections (0.0 to 1.0).",
    )
    parser.add_argument(
        "--fov",
        type=float,
        default=60.0,
        help="Horizontal field of view (FOV) of the camera in degrees (default: 60.0).",
    )
    parser.add_argument(
        "--save",
        action="store_true",
        help="Save output to result.jpg (for images) or result.mp4 (for video).",
    )
    parser.add_argument(
        "--no-show",
        action="store_true",
        help="Disable interactive OpenCV window (useful in headless environments).",
    )
    return parser.parse_args()


def process_frame(frame, model, conf=0.25, fov_deg=60.0):
    """
    Runs YOLO inference on a single frame and draws rover targeting annotations:
    - Bounding boxes and labels
    - Center line of frame
    - Target center point
    - Steering angle offset (degrees)
    - Relative distance proxy
    """
    annotated_img = frame.copy()
    img_height, img_width = annotated_img.shape[:2]
    img_center_x = img_width // 2

    # Draw vertical reference line down the center of the frame
    cv2.line(
        annotated_img,
        (img_center_x, 0),
        (img_center_x, img_height),
        (255, 0, 0),
        1,
    )

    # Run inference on numpy array
    results = model.predict(source=frame, conf=conf, verbose=False)
    detections = []

    for result in results:
        boxes = result.boxes.xyxy.cpu().numpy()
        class_ids = result.boxes.cls.cpu().numpy()
        confidences = result.boxes.conf.cpu().numpy()

        for box, class_id, confidence in zip(boxes, class_ids, confidences):
            x1, y1, x2, y2 = map(int, box)
            label = model.names[int(class_id)]
            conf_val = float(confidence)

            # Target center coordinates
            obj_center_x = (x1 + x2) // 2
            obj_center_y = (y1 + y2) // 2

            # Calculate steering angle offset relative to camera optical center
            pixel_offset_x = obj_center_x - img_center_x
            approx_angle_deg = (pixel_offset_x / (img_width / 2.0)) * (fov_deg / 2.0)

            # Rough distance proxy: ratio of image height to bounding box height
            box_height = max(y2 - y1, 1)
            relative_distance = img_height / box_height

            detections.append(
                {
                    "label": label,
                    "confidence": conf_val,
                    "box": (x1, y1, x2, y2),
                    "center": (obj_center_x, obj_center_y),
                    "angle_deg": approx_angle_deg,
                    "rel_dist": relative_distance,
                }
            )

            # bounding box and center point
            cv2.rectangle(annotated_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.circle(
                annotated_img, (obj_center_x, obj_center_y), 5, (0, 0, 255), -1
            )

            # metrics/annotations
            text = f"{label} {conf_val:.2f} ({approx_angle_deg:+.1f} deg)"
            cv2.putText(
                annotated_img,
                text,
                (x1, max(y1 - 10, 15)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2,
            )

    return annotated_img, detections


def run_video_feed(cap, model, conf, fov_deg, show=True, save=False):
    """Processes video or webcam feed frame-by-frame."""
    writer = None
    if save:
        width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        writer = cv2.VideoWriter("result.mp4", fourcc, fps, (width, height))

    prev_time = time.time()

    print("Starting video feed. Press 'q' in the window to quit.")
    try:
        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                print("End of video stream or cannot fetch frame.")
                break

            # process detection + rover math
            annotated_frame, detections = process_frame(
                frame, model, conf=conf, fov_deg=fov_deg
            )

            # fps calculation
            curr_time = time.time()
            fps = 1.0 / max(curr_time - prev_time, 1e-5)
            prev_time = curr_time

            cv2.putText(
                annotated_frame,
                f"FPS: {fps:.1f}",
                (20, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 255),
                2,
            )

            for det in detections:
                print(
                    f"[{det['label']}] Conf: {det['confidence']:.2f} | "
                    f"Angle: {det['angle_deg']:+.1f}° | RelDist: {det['rel_dist']:.2f}"
                )

            if writer is not None:
                writer.write(annotated_frame)

            if show:
                cv2.imshow("Rover Object Detection Feed", annotated_frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
    finally:
        cap.release()
        if writer is not None:
            writer.release()
            print("Saved video result to result.mp4")
        if show:
            cv2.destroyAllWindows()


def run_image(image_path, model, conf, fov_deg, show=True, save=True):
    """Processes a single static image."""
    img = cv2.imread(str(image_path))
    if img is None:
        print(f"Error: Could not read image at {image_path}")
        return

    annotated_img, detections = process_frame(
        img, model, conf=conf, fov_deg=fov_deg
    )

    for det in detections:
        print(
            f"Detected {det['label']} ({det['confidence']:.2f}) | "
            f"Box: {list(det['box'])} | "
            f"Angle: {det['angle_deg']:+.1f} deg | "
            f"Rel Dist: {det['rel_dist']:.2f}"
        )

    if save:
        output_path = Path("result.jpg")
        cv2.imwrite(str(output_path), annotated_img)
        print(f"Saved annotated image to {output_path}")

    if show:
        try:
            cv2.imshow("Rover Detection - Static Image", annotated_img)
            print("Press any key to close the image window...")
            cv2.waitKey(0)
            cv2.destroyAllWindows()
        except cv2.error:
            pass


def main():
    args = parse_args()

    # load model + config
    print(f"Loading YOLO model from: {args.model}")
    model = YOLO(args.model)
    print(f"Loaded classes: {model.names}")

    # determine input source -> need to change later to only video feed
    # different types of sources mainly for testing
    source_str = args.source
    show_window = not args.no_show

    if source_str.isdigit():
        camera_idx = int(source_str)
        print(f"Opening webcam device index: {camera_idx}")
        cap = cv2.VideoCapture(camera_idx)
        if not cap.isOpened():
            print(f"Error: Could not open camera {camera_idx}")
            return
        run_video_feed(
            cap,
            model,
            conf=args.conf,
            fov_deg=args.fov,
            show=show_window,
            save=args.save,
        )
    else:
        source_path = Path(source_str)
        video_extensions = {".mp4", ".avi", ".mov", ".mkv"}

        if source_path.suffix.lower() in video_extensions:
            print(f"Opening video file: {source_path}")
            cap = cv2.VideoCapture(str(source_path))
            if not cap.isOpened():
                print(f"Error: Could not open video file at {source_path}")
                return
            run_video_feed(
                cap,
                model,
                conf=args.conf,
                fov_deg=args.fov,
                show=show_window,
                save=args.save,
            )
        else:
            print(f"Processing static image: {source_path}")
            run_image(
                source_path,
                model,
                conf=args.conf,
                fov_deg=args.fov,
                show=show_window,
                save=args.save or True,
            )


if __name__ == "__main__":
    main()
