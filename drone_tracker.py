import time
import sys
import cv2
import numpy as np
import logging
import threading

# from camera_manager import RealSenseCamera
from yolo_detector import YOLODetector
from utils import FPSCounter, draw_detections, draw_fps, draw_detection_stats

# ==================== CONFIGURATION ====================
# Input Source Configuration
RUN_ON_TEST = True                     # Set to True to run on test video, False for RealSense camera
TEST_VIDEO_PATH = "data/test/test_720p.mp4"  # Path to test video file (used when RUN_ON_TEST=True)
VIDEO_PLAYBACK_FPS = 30                 # Fixed FPS for video playback (set to 0 to use video's native FPS)

# Model Configuration
MODEL_PATH = "models/weights/best.pt"

# Camera Configuration (RealSense)
CAMERA_WIDTH = 1280                  # Camera resolution width
CAMERA_HEIGHT = 720                  # Camera resolution height
CAMERA_FPS = 30                      # Target FPS

# Detection Configuration
MIN_CONFIDENCE = 0.50                # Minimum detection confidence (0.0 - 1.0)

# Output Configuration
SERIAL_PORT = "COM3"                 # Serial port for ESP32
BAUD_RATE = 115200                   # Serial baud rate
SEND_HZ = 30                         # Data output rate (Hz)
ENABLE_SMOOTHING = True              # Enable EMA smoothing filter
SMOOTH_ALPHA = 0.25                  # Smoothing facto--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------r-------------- (lower = smoother)

# Display Configuration
SHOW_DISPLAY = True                  # Show visual feedback window
VERBOSE_OUTPUT = False               # Print extra debug info
# =======================================================

try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False
    print("[WARN] pyserial not installed - console output only")


class SmoothingFilter:
    """Exponential Moving Average (EMA) filter for smooth tracking"""
    def __init__(self, alpha=0.25):
        self.alpha = alpha if ENABLE_SMOOTHING else 0.0
        self.x = 0.0
        self.y = 0.0
        self.has_data = False
    
    def update(self, x, y):
        if self.alpha <= 0.0:
            return x, y
            
        if not self.has_data:
            self.x = x
            self.y = y
            self.has_data = True
        else:
            self.x = self.alpha * x + (1.0 - self.alpha) * self.x
            self.y = self.alpha * y + (1.0 - self.alpha) * self.y
        
        return self.x, self.y
    
    def reset(self):
        self.has_data = False


class SerialConnection:
    """Handles serial communication with ESP32"""
    def __init__(self, port, baud):
        self.ser = None
        if not SERIAL_AVAILABLE:
            return
        
        try:
            self.ser = serial.Serial(port, baudrate=baud, timeout=0)
            time.sleep(0.5)  # Allow ESP32 to reset
            print(f"[INFO] Serial connected: {port} @ {baud} baud")
            # RICHARD ADDED CODE
            self.running = True
            self.thread = threading.Thread(target=self._read_serial, daemon=True)
            self.thread.start()
            # END RICHARD ADDED CODE
        except Exception as e:
            print(f"[WARN] Could not open serial port: {e}")
            print("[INFO] Will output to console only")
    
    def send(self, data_string):
        """Send data via serial"""
        if self.ser and self.ser.writable():
            try:
                self.ser.write(data_string.encode('ascii'))
            except Exception as e:
                if VERBOSE_OUTPUT:
                    print(f"[WARN] Serial write error: {e}")
    
    def close(self):
        if self.ser:
            self.running = False
            self.ser.close()

    
    def _read_serial(self):
        """Continuously read and print incoming data from ESP32."""
        while getattr(self, "running", False) and self.ser:
            try:
                data = self.ser.readline().decode('ascii', errors='ignore').strip()
                if data:
                    print(f"[ESP32] {data}")
            except Exception:
                pass
            time.sleep(0.001)  # minimal delay to prevent CPU hogging

def select_best_detection(detections):
    """
    Select best detection from list of detections for gimbal tracking
    Prioritizes 'uav' class, then highest confidence

    Args:
        detections: List of detection dicts from YOLODetector.detect()

    Returns:
        Best detection dict or None
    """
    if not detections:
        return None

    # Prioritize 'uav' class, then by confidence
    best_detection = None
    best_score = -1.0

    for det in detections:
        class_name = det['class_name'].lower()
        confidence = det['confidence']

        # Score: prioritize UAV/drone classes, then confidence
        is_uav = 'uav' in class_name or 'drone' in class_name
        score = (2.0 if is_uav else 1.0) * confidence

        if score > best_score:
            best_detection = det
            best_score = score

    return best_detection


def format_output(found, dx, dy, confidence):
    """
    Format tracking data for ESP32
    Output: found,dx,dy,confidence
    """
    return f"{1 if found else 0},{dx:.4f},{dy:.4f},{confidence:.4f}\n"


def draw_visualization(frame, detection, frame_center, smoothed_coords, fps):
    """Draw gimbal tracking visualization on frame"""
    h, w = frame.shape[:2]
    cx, cy = frame_center

    # Draw center crosshair
    cross_size = 30
    cv2.line(frame, (int(cx - cross_size), int(cy)),
             (int(cx + cross_size), int(cy)), (255, 255, 255), 2)
    cv2.line(frame, (int(cx), int(cy - cross_size)),
             (int(cx), int(cy + cross_size)), (255, 255, 255), 2)
    cv2.circle(frame, (int(cx), int(cy)), 5, (255, 255, 255), -1)

    if detection:
        # Extract detection info
        center = detection['center']
        det_x, det_y = center
        conf = detection['confidence']
        class_name = detection['class_name']

        # Draw detection point
        cv2.circle(frame, (int(det_x), int(det_y)), 12, (0, 255, 0), -1)
        cv2.circle(frame, (int(det_x), int(det_y)), 15, (0, 255, 0), 2)

        # Draw line from center to target
        cv2.line(frame, (int(cx), int(cy)),
                 (int(det_x), int(det_y)), (0, 255, 0), 2)

        # Status text
        status = "LOCKED"
        status_color = (0, 255, 0)

        # Detection info
        info_text = f"{class_name} | Conf: {conf:.2f}"
        cv2.putText(frame, info_text, (int(det_x) + 20, int(det_y) - 15),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        # Smoothed coordinates
        if smoothed_coords:
            dx_smooth, dy_smooth = smoothed_coords
            coord_text = f"dx:{dx_smooth:+.3f} dy:{dy_smooth:+.3f}"
            cv2.putText(frame, coord_text, (int(det_x) + 20, int(det_y) + 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    else:
        status = "SEARCHING"
        status_color = (0, 0, 255)

    # Main status
    cv2.putText(frame, status, (20, 60),
               cv2.FONT_HERSHEY_SIMPLEX, 1.8, status_color, 4)

    # FPS counter
    fps_text = f"FPS: {fps:.1f}"
    cv2.putText(frame, fps_text, (w - 150, 30),
               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    # Instructions
    cv2.putText(frame, "Press 'q' to quit | 's' to screenshot",
               (20, h - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

    return frame


def main():
    print("=" * 70)
    print("  DRONE TRACKING DATA EXTRACTOR")
    if RUN_ON_TEST:
        print(f"  For ESP32 Gimbal Motor Control (Test Video Mode)")
    else:
        print(f"  For ESP32 Gimbal Motor Control (RealSense Camera)")
    print("=" * 70)
    print()

    # Setup logging
    logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
    logger = logging.getLogger(__name__)

    # Initialize YOLO detector
    try:
        detector = YOLODetector(
            model_path=MODEL_PATH,
            confidence_threshold=MIN_CONFIDENCE,
            device='auto'
        )
        logger.info(f"YOLO detector initialized with confidence threshold: {MIN_CONFIDENCE}")
    except Exception as e:
        print(f"[ERROR] Failed to load YOLO model: {e}")
        print(f"[TIP] Make sure model file exists at: {MODEL_PATH}")
        return

    # Initialize input source (camera or video file)
    camera = None
    video_cap = None
    video_fps = 0.0

    if RUN_ON_TEST:
        # Initialize video capture from file
        logger.info(f"Loading test video: {TEST_VIDEO_PATH}")
        video_cap = cv2.VideoCapture(TEST_VIDEO_PATH)

        if not video_cap.isOpened():
            print(f"[ERROR] Failed to open test video: {TEST_VIDEO_PATH}")
            print("[TIP] Make sure the video file exists at the specified path")
            return

        # Get video properties
        video_width = int(video_cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        video_height = int(video_cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        video_native_fps = video_cap.get(cv2.CAP_PROP_FPS)
        video_frame_count = int(video_cap.get(cv2.CAP_PROP_FRAME_COUNT))

        # Use fixed FPS if configured, otherwise use video's native FPS
        if VIDEO_PLAYBACK_FPS > 0:
            video_fps = VIDEO_PLAYBACK_FPS
            logger.info(f"Video loaded: {video_width}x{video_height} @ {video_native_fps:.1f}fps (native), "
                       f"will play at {video_fps:.1f}fps (configured)")
        else:
            video_fps = video_native_fps
            logger.info(f"Video loaded: {video_width}x{video_height} @ {video_fps:.1f}fps, {video_frame_count} frames")
    else:
        # Initialize RealSense camera
        logger.info(f"Initializing RealSense camera: {CAMERA_WIDTH}x{CAMERA_HEIGHT} @ {CAMERA_FPS}fps")

        camera = RealSenseCamera(width=CAMERA_WIDTH, height=CAMERA_HEIGHT, fps=CAMERA_FPS)

        if not camera.initialize():
            print(f"[ERROR] Failed to initialize RealSense camera")
            print("[TIP] Make sure RealSense camera is connected")
            return

        logger.info(f"Camera resolution: {CAMERA_WIDTH}x{CAMERA_HEIGHT}")

    # Initialize serial connection
    serial_conn = SerialConnection(SERIAL_PORT, BAUD_RATE)

    # Initialize smoothing filter
    smoother = SmoothingFilter(alpha=SMOOTH_ALPHA)

    # Initialize FPS counter
    fps_counter = FPSCounter()
    
    # Timing control
    send_period = 1.0 / SEND_HZ
    next_send_time = time.perf_counter()

    # Video playback timing (for test mode)
    if RUN_ON_TEST and video_fps > 0:
        frame_period = 1.0 / video_fps  # Time between frames
        next_frame_time = time.perf_counter()
    else:
        frame_period = 0.0
        next_frame_time = 0.0

    print()
    print("[INFO] System ready!")
    if RUN_ON_TEST and video_fps > 0:
        print(f"[INFO] Video playback FPS: {video_fps:.1f}")
    print(f"[INFO] Output format: found,dx,dy,confidence")
    print(f"[INFO] Update rate: {SEND_HZ} Hz")
    print("-" * 70)
    print()

    frame_count = 0

    try:
        while True:
            # Video playback timing control (throttle to match video FPS)
            if RUN_ON_TEST and video_fps > 0:
                current_time = time.perf_counter()
                time_to_wait = next_frame_time - current_time

                # If we're ahead of schedule, wait
                if time_to_wait > 0:
                    time.sleep(time_to_wait)

                # Schedule next frame
                next_frame_time = time.perf_counter() + frame_period

            # Get frame from input source
            if RUN_ON_TEST:
                # Read frame from video file
                ret, color_frame = video_cap.read()

                if not ret:
                    print("\n[INFO] End of video reached")
                    break

                depth_frame = None  # No depth data from video file
            else:
                # Get frame from RealSense camera
                color_frame, depth_frame = camera.get_frame()

                if color_frame is None:
                    logger.warning("No frame received from camera")
                    continue

            # Update FPS counter
            fps_counter.update()
            current_fps = fps_counter.get_fps()

            # Get frame dimensions and calculate center
            h, w = color_frame.shape[:2]
            center_x = w / 2.0
            center_y = h / 2.0

            # Run YOLO detection
            detections = detector.detect(color_frame)

            # Select best detection for gimbal tracking
            best_detection = select_best_detection(detections)

            # Calculate tracking data
            found = False
            dx_out = 0.0
            dy_out = 0.0
            conf_out = 0.0
            smoothed_coords = None

            if best_detection:
                # Extract center and confidence from detection dict
                det_x, det_y = best_detection['center']
                conf = best_detection['confidence']
                class_name = best_detection['class_name']

                # Calculate normalized offsets
                # Right = positive, Left = negative
                dx_raw = (det_x - center_x) / center_x
                # Up = positive, Down = negative
                dy_raw = (center_y - det_y) / center_y

                # Clamp to valid range
                dx_raw = max(-1.0, min(1.0, dx_raw))
                dy_raw = max(-1.0, min(1.0, dy_raw))

                # Apply smoothing
                dx_smooth, dy_smooth = smoother.update(dx_raw, dy_raw)

                # Output values
                dx_out = dx_smooth
                dy_out = dy_smooth
                conf_out = conf
                found = True
                smoothed_coords = (dx_smooth, dy_smooth)

                if VERBOSE_OUTPUT:
                    print(f"[DETECT] {class_name} @ ({det_x:.0f},{det_y:.0f}) "
                          f"conf={conf:.2f} dx={dx_out:+.3f} dy={dy_out:+.3f}")
            else:
                smoother.reset()

            # Send data at fixed rate
            current_time = time.perf_counter()
            if current_time >= next_send_time:
                output_string = format_output(found, dx_out, dy_out, conf_out)

                # Send via serial
                serial_conn.send(output_string)

                # Print to console
                sys.stdout.write(output_string)
                sys.stdout.flush()

                next_send_time = current_time + send_period

            # Visualization
            if SHOW_DISPLAY:
                annotated_frame = draw_visualization(color_frame, best_detection,
                                          (center_x, center_y),
                                          smoothed_coords, current_fps)

                window_title = "Drone Tracker - Data Output (Test Video)" if RUN_ON_TEST else "Drone Tracker - Data Output (RealSense)"
                cv2.imshow(window_title, annotated_frame)

                # Minimal delay for GUI event handling (timing is controlled by sleep above)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    print("\n[INFO] Quit command received")
                    break
                elif key == ord('s'):
                    filename = f"screenshot_{int(time.time())}.jpg"
                    cv2.imwrite(filename, annotated_frame)
                    print(f"[INFO] Screenshot saved: {filename}")

            frame_count += 1
    
    except KeyboardInterrupt:
        print("\n[INFO] Interrupted by user (Ctrl+C)")

    except Exception as e:
        print(f"\n[ERROR] Unexpected error: {e}")
        import traceback
        traceback.print_exc()

    finally:
        # Cleanup
        print("\n[INFO] Shutting down...")

        # Send final "no target" signal
        final_output = format_output(False, 0, 0, 0)
        serial_conn.send(final_output)
        sys.stdout.write(final_output)

        # Stop input source
        if RUN_ON_TEST and video_cap is not None:
            video_cap.release()
        elif camera is not None:
            camera.stop()

        cv2.destroyAllWindows()
        serial_conn.close()

        print(f"[INFO] Processed {frame_count} frames")
        print("[INFO] Shutdown complete")


if __name__ == "__main__":
    main()
