import cv2
import json
import sys
import os

def detect_csi_cameras():
    available_cameras = []
    for i in range(10):  # 최대 10개의 카메라를 확인
        print(f"Checking camera {i}...", file=sys.stderr)
        cap = cv2.VideoCapture(f"nvarguscamerasrc sensor-id={i} ! video/x-raw(memory:NVMM), width=1280, height=720, format=NV12, framerate=30/1 ! nvvidconv ! video/x-raw, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink", cv2.CAP_GSTREAMER)
        if cap.isOpened():
            print(f"Camera {i} opened successfully", file=sys.stderr)
            ret, _ = cap.read()
            if ret:
                print(f"Successfully read frame from camera {i}", file=sys.stderr)
                available_cameras.append(i)
            else:
                print(f"Failed to read frame from camera {i}", file=sys.stderr)
            cap.release()
        else:
            print(f"Failed to open camera {i}", file=sys.stderr)
    return available_cameras

if __name__ == "__main__":
    print("Starting camera detection...", file=sys.stderr)
    cameras = detect_csi_cameras()
    print(f"Detected cameras: {cameras}", file=sys.stderr)
    
    # Save JSON to file
    output_file = os.path.join(os.path.dirname(__file__), 'detected_cameras.json')
    with open(output_file, 'w') as f:
        json.dump(cameras, f)
    
    print(f"Camera detection completed. Results saved to {output_file}", file=sys.stderr)
    
