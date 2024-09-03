import cv2
import json

def detect_csi_cameras():
    available_cameras = []
    for i in range(10):  # 최대 10개의 카메라를 확인
        cap = cv2.VideoCapture(f"nvarguscamerasrc sensor-id={i} ! video/x-raw(memory:NVMM), width=1280, height=720, format=NV12, framerate=30/1 ! nvvidconv ! video/x-raw, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink", cv2.CAP_GSTREAMER)
        if cap.isOpened():
            available_cameras.append(i)
            cap.release()
    return available_cameras

if __name__ == "__main__":
    cameras = detect_csi_cameras()
    print(json.dumps(cameras))  # JSON 형식으로 출력