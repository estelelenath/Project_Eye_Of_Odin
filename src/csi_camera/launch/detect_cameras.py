import cv2
import json
import sys
import os
import glob


def quick_check_camera(device):
    """
    (A) 방식
    OpenCV 기본 백엔드(v4l2 등)를 사용하여 
    단순히 '장치가 열리는지'만 빠르게 확인.
    """
    try:
        cap = cv2.VideoCapture(device)
        if cap.isOpened():
            cap.release()
            return True
    except Exception as e:
    	print(f"빠른 체크 중 오류 발생: {device}, {e}", file=sys.stderr)
    
    return False
	
def deep_check_camera(device):
    """
    (B) 방식
    GStreamer 파이프라인으로 열어서 프레임을 실제로 읽어본다.
    Jetson 환경에서 실제로 사용할 파이프라인 예시.
    """
    try:
        pipeline = (
            f"v4l2src device={device} ! "
            "image/jpeg, width=1280, height=720, framerate=30/1 ! "
            "jpegdec ! videoconvert ! video/x-raw, format=BGR ! "
            "appsink"
        )

        cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        if cap.isOpened():
            ret, _ = cap.read()
            cap.release()
            if ret:
                print(f"MJPG 파이프라인 성공: {device}", file=sys.stderr)
                return True
                
        # MJPG가 실패하면 YUYV 시도
        pipeline = (
            f"v4l2src device={device} ! "
            "video/x-raw, format=YUY2, width=640, height=480, framerate=30/1 ! "
            "videoconvert ! video/x-raw, format=BGR ! "
            "appsink"
        )
        
        cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        if cap.isOpened():
            ret, _ = cap.read()
            cap.release()
            if ret:
                print(f"YUYV 파이프라인 성공: {device}", file=sys.stderr)
                return True
        
    except Exception as e:
        print(f"GStreamer 확인 중 오류 발생: {device}, {e}", file=sys.stderr)
    return False

	
def detect_all_cameras():
    available_cameras = {
		'csi': [], 
		'usb': []
	}
    
    # CSI 카메라 감지
	# Jetson 보드에서 최대 4개의 CSI 카메라를 지원한다고 가정
    for i in range(4):
        print(f"CSI 카메라 {i} 확인 중...", file=sys.stderr)
        try:
            pipeline = (
                f"nvarguscamerasrc sensor-id={i} ! "
                "video/x-raw(memory:NVMM), width=1280, height=720, format=NV12, "
                "framerate=30/1 ! nvvidconv ! video/x-raw, format=BGRx ! "
                "videoconvert ! video/x-raw, format=BGR ! appsink"
            )
            cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
            if cap.isOpened():
                ret, _ = cap.read()
                if ret:
                    available_cameras['csi'].append(i)
                cap.release()
        except Exception as e:
            print(f"CSI 카메라 {i} 확인 중 오류 발생: {e}", file=sys.stderr)

    # USB 카메라 감지
    video_devices = glob.glob('/dev/video*')
    for device in video_devices:
        print(f"USB 장치 {device} 빠른 체크 중...", file=sys.stderr)
        try:
            device_id = int(device.replace('/dev/video', ''))
            
            # 빠른 체크 (quick_check_camera)
            if not quick_check_camera(device_id):
                continue
            
            print(f"USB 장치 {device} GStreamer 확인 중...", file=sys.stderr)
            
            # GStreamer 확인 (deep_check_camera)
            if deep_check_camera(device):
                available_cameras['usb'].append(device_id)
        except Exception as e:
            print(f"USB 장치 {device} 확인 중 오류 발생: {e}", file=sys.stderr)

    return available_cameras

if __name__ == "__main__":
    print("카메라 감지 시작...", file=sys.stderr)
    cameras = detect_all_cameras()
    print(f"감지된 카메라: {cameras}", file=sys.stderr)

    output_file = os.path.join(os.path.dirname(__file__), 'detected_cameras.json')
    with open(output_file, 'w') as f:
        json.dump(cameras, f)
    print(f"카메라 감지 완료. 결과가 {output_file}에 저장됨", file=sys.stderr)
