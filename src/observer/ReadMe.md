Requirement

sudo apt install uvicorn
sudo pip install fastapi uvicorn[standard] jinja2 aiofiles
pip install pyzmq

Run


USB Connection Environment

# 1. Robot) 먼저 ROS2 시스템 실행
ros2 launch project_eye_of_odin project_eye_of_odin.launch.py

# 2. Robot) odin_bridge 실행 (새 터미널)
ros2 launch odin_bridge odin_bridge.launch.py jetson_ids:='["001"]'

# 3. Robot) FastAPI 서버 실행 (새 터미널)
cd src/observer
uvicorn app.main:app --reload --host 0.0.0.0 --port 8000

동일 머신(로컬) 테스트 시 "Jetson Side Check"
http:// 127.0.0.1:8000

원격 접속(임베디드 ↔ Host PC) 시 "Host PC Side Check"
http://<임베디드-IP>:8000
http://192.168.55.1:8000



For Wifi



For 5G
future solution: VPN(WireGuard/OpenVPN)