# 먼저 stdout stderr 때문에 에러가 났습니다.
# 모든 결과(print("..."))가 stdout으로 나오는 바람에, [0],[1],[2],... 이렇게 실행가능한 카메라 검색결과만 원했지만, [0],[1],[2],... + Gst 정보라던가 다른 정보도 포함되어 오류가 발생했습니다.
# 다만 print("...") 를 사용하지않았으면 아마 오류가 없었을지도... 괜히 디버그한다고 막 사용하다...
# 그리고 [0],[1],[2] 이런 정보는 json 파일로 정보를 런치파일로 전달합니다

import cv2  # OpenCV 라이브러리를 임포트합니다. 카메라 접근과 이미지 처리에 사용됩니다.
import json  # JSON 데이터를 다루기 위한 라이브러리입니다.
import sys  # 시스템 특정 매개변수와 함수를 사용하기 위한 모듈입니다.
import os  # 운영체제와 상호 작용하기 위한 모듈입니다.

def detect_csi_cameras():
    available_cameras = []  # 사용 가능한 카메라의 ID를 저장할 리스트를 초기화합니다.
    for i in range(10):  # 최대 10개의 카메라를 확인합니다.
        print(f"카메라 {i} 확인 중...", file=sys.stderr)  # 디버그 정보를 stderr로 출력합니다.
        # GStreamer 파이프라인을 사용하여 카메라를 열려고 시도합니다.
        # 여기서 보통 VideoCapture하고 (0) 이렇게 카메라 번호를 사용하지만, GStreamer_pipeline 을 사용하려면 VideoCapture(GStreamer_pipeline, cv2.CAP_GSTREAMER) 를 사용해야합니다. 
        # GStreamer_pipeline 이건 nvidia 플랫폼의 비디오를 켤때 필요한 각종 파라메터 정보이며, (센서아이디, 이미지 사이즈 등등) 그리고 cv2.CAP_GSTREAMER 은 하드웨어가속을 사용하여 카메라를 켜겠다는 말입니다. 이건 빼도 상관없습니다.
        # 이렇게도 쓸수있겠죠...
        # cap = cv2.VideoCapture(gstreamer_pipeline(sensor_id=self.sensor_id, framerate=self.framerate), cv2.CAP_GSTREAMER) 
        cap = cv2.VideoCapture(f"nvarguscamerasrc sensor-id={i} ! video/x-raw(memory:NVMM), width=1280, height=720, format=NV12, framerate=30/1 ! nvvidconv ! video/x-raw, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink", cv2.CAP_GSTREAMER)
        if cap.isOpened():  # 카메라가 성공적으로 열렸는지 확인합니다.
            print(f"카메라 {i} 열기 성공", file=sys.stderr)
            ret, _ = cap.read()  # 프레임을 읽어봅니다.
            if ret:  # 프레임을 성공적으로 읽었는지 확인합니다.
                print(f"카메라 {i}에서 프레임 읽기 성공", file=sys.stderr)
                available_cameras.append(i)  # 사용 가능한 카메라 목록에 추가합니다.
            else:
                print(f"카메라 {i}에서 프레임 읽기 실패", file=sys.stderr)
            cap.release()  # 카메라 리소스를 해제합니다.
        else:
            print(f"카메라 {i} 열기 실패", file=sys.stderr)
    return available_cameras  # 사용 가능한 카메라 목록을 반환합니다.

if __name__ == "__main__":
    print("카메라 감지 시작...", file=sys.stderr)
    cameras = detect_csi_cameras()  # 카메라 감지 함수를 실행합니다.
    print(f"감지된 카메라: {cameras}", file=sys.stderr)
    
    # JSON을 파일로 저장합니다.
    # 파일경로
    output_file = os.path.join(os.path.dirname(__file__), 'detected_cameras.json')
    # output_file = os.path.join(os.path.dirname(__file__), 'detect_cameras.json'), 
    # 만약에 지금 보고있는 코드가 있는 파일이름이 detect_cameras.py 이라고 가정한다면, 이 파일이 위치한 디렉토리에서 detected_cameras.json 파일의 경로를 생성하는 코드입니다.
    # __file__: -> /home/user/project/detect_cameras.py
    # os.path.dirname(__file__) -> /home/user/project
    # os.path.join(os.path.dirname(__file__), 'detect_cameras.json') -> /home/user/project/detect_cameras.json

    # + info
    # 현재 파일이 위치한 디렉터리 경로 -> current_dir = os.path.dirname(__file__)
    # 상위 디렉터리 접근? -> parent_dir = os.path.dirname(current_dir)
    # 두 레벨 위 디렉터리로 이동하기 -> two_levels_up = os.path.dirname(os.path.dirname(__file__))
    # 다른 디렉터리로 접근
    # parent_dir = os.path.dirname(__file__)
    # target_dir = os.path.join(parent_dir, '..', 'project2')
    # then : /home/user/project2/
    with open(output_file, 'w') as f:
        json.dump(cameras, f)  # 감지된 카메라 목록을 JSON 형식으로 파일에 저장합니다.
    
    print(f"카메라 감지 완료. 결과가 {output_file}에 저장됨", file=sys.stderr)