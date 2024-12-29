# bridge/zmq_bridge.py
import zmq
import zmq.asyncio
import asyncio
import json
import logging
from datetime import datetime
from typing import Dict, Set, Optional
from fastapi import WebSocket

############################################################
# 메시지 구조 스펙
############################################################
TOPIC_SPECS = {
    "camera": {
        "num_parts": 3,         # 3개 멀티파트: [topic, json, jpeg bytes]
        "json_index": 1,        # 2번째 part = JSON
        "binary_index": 2       # 3번째 part = 바이너리(JPEG)
    },
    "lidar": {
        "num_parts": 2,         # 2개 멀티파트: [topic, json]
        "json_index": 1,        # 2번째 part = JSON
        "binary_index": None    # 바이너리 없음
    }
}

class ZMQBridge:
    """
    ZMQ SUB로부터 여러 토픽 메시지를 받아 저장하고,
    연결된 WebSocket 클라이언트에 실시간 전송.
    * 카메라, 라이다 모두 FPS(전송 주기) 제한을 둠.
    """

    def __init__(self, address: str = "tcp://127.0.0.1:5555"):
        self._logger = logging.getLogger("ZMQBridge")

        # ZMQ(asyncio) 설정
        self._context = zmq.asyncio.Context()
        self._socket = self._context.socket(zmq.SUB)
        self._socket.connect(address)
        # 모든 토픽 수신 (camera, lidar)
        self._socket.setsockopt_string(zmq.SUBSCRIBE, "")

        # WebSocket 클라이언트 풀(동적 토픽 확장 가능)
        self._active_websockets: Dict[str, Set[WebSocket]] = {}

        # 최근 데이터 캐시: topic별 마지막 메시지
        self._last_data: Dict[str, Optional[dict]] = {}

        # 카메라 전송 주기 제한(예: 0.1초 = 최대 10FPS)
        self._camera_send_interval = 0.1
        self._last_camera_send_time = 0.0

        # 라이다 전송 주기 제한(예: 0.2초 = 최대 5FPS)
        self._lidar_send_interval = 0.2
        self._last_lidar_send_time = 0.0

        self._logger.info(f"ZMQBridge init: connected to {address}")

    async def start(self):
        """
        FastAPI에서 @app.on_event("startup") 이후,
        asyncio.create_task(bridge.start()) 형태로 호출해주면,
        메인 이벤트 루프에서 ZMQ 메시지를 계속 수신한다.
        """
        self._logger.info("ZMQ Subscriber started.")
        while True:
            try:
                parts = await self._socket.recv_multipart()
                if not parts:
                    continue

                topic = parts[0].decode()
                if topic not in TOPIC_SPECS:
                    # 정의되지 않은 토픽이면 로그 남기고 스킵
                    self._logger.warning(f"Unknown topic '{topic}'. Skipping.")
                    continue

                spec = TOPIC_SPECS[topic]
                if len(parts) != spec["num_parts"]:
                    self._logger.warning(
                        f"Expected {spec['num_parts']} parts for topic '{topic}', got {len(parts)}. Skipping."
                    )
                    continue

                await self._handle_message(topic, parts, spec)

            except Exception as e:
                self._logger.error(f"ZMQ Error: {e}")
                await asyncio.sleep(1)  # 재시도 대기

    async def _handle_message(self, topic: str, parts: list, spec: dict):
        """
        토픽별 스펙에 맞춰 JSON 파싱, 바이너리 데이터 추출.
        캐시에 업데이트 후, 해당 토픽의 전송 주기가 만족되면 브로드캐스트.
        """
        now_str = datetime.utcnow().isoformat()
        event_loop = asyncio.get_event_loop()

        try:
            # JSON 파싱
            json_part = parts[spec["json_index"]]
            try:
                parsed_json = json.loads(json_part.decode("utf-8"))
            except Exception as e:
                self._logger.warning(f"JSON parse error for topic '{topic}': {e}")
                return  # 파싱 실패 시 스킵

            # 바이너리가 있는 경우
            binary_data = None
            if spec["binary_index"] is not None:
                binary_data = parts[spec["binary_index"]]

            # 캐시 초기화
            if topic not in self._last_data:
                self._last_data[topic] = None

            # 카메라 처리
            if topic == "camera":
                self._last_data[topic] = {
                    "metadata": parsed_json,
                    "image": binary_data,
                    "timestamp": now_str
                }
                # FPS 제한
                current_time = event_loop.time()
                if (current_time - self._last_camera_send_time) >= self._camera_send_interval:
                    self._last_camera_send_time = current_time
                    await self._broadcast_camera()
                else:
                    # 아직 전송 주기가 안 지났으므로, 브로드캐스트 스킵
                    pass

            # 라이다 처리
            elif topic == "lidar":
                self._last_data[topic] = {
                    "data": parsed_json,
                    "timestamp": now_str
                }
                current_time = event_loop.time()
                if (current_time - self._last_lidar_send_time) >= self._lidar_send_interval:
                    self._last_lidar_send_time = current_time
                    await self._broadcast_lidar(topic)
                else:
                    pass

        except Exception as e:
            self._logger.error(f"Handle message error for topic '{topic}': {e}")

    async def register_websocket(self, topic: str, websocket: WebSocket):
        """
        WebSocket 연결 -> 해당 토픽 클라이언트 세트에 등록.
        필요 시 최신 데이터 즉시 전송.
        """
        await websocket.accept()
        if topic not in self._active_websockets:
            self._active_websockets[topic] = set()
        self._active_websockets[topic].add(websocket)

        self._logger.info(f"WebSocket connected: {topic} (total: {len(self._active_websockets[topic])})")

        # 연결 직후, 기존 데이터 있으면 즉시 전송(단, FPS 제한은 무시하고 "최신"을 한번 보내줄 수 있음)
        if topic in self._last_data and self._last_data[topic] is not None:
            if topic == "camera":
                await self._broadcast_camera(single_ws=websocket)
            else:
                await self._broadcast_lidar(topic, single_ws=websocket)

        try:
            while True:
                await asyncio.sleep(1)
        except Exception:
            self._logger.warning(f"WebSocket disconnected: {topic}")
        finally:
            self._active_websockets[topic].discard(websocket)

    async def _broadcast_camera(self, single_ws: Optional[WebSocket] = None):
        """
        카메라 데이터 = JSON(메타데이터+타임스탬프) + JPEG.
        single_ws가 주어지면 그 웹소켓만, 없으면 전체에게 전송.
        """
        data = self._last_data.get("camera")
        if not data:
            return

        disconnected = set()
        targets = [single_ws] if single_ws else list(self._active_websockets.get("camera", set()))

        send_dict = {
            "metadata": data["metadata"],
            "timestamp": data["timestamp"]
        }
        for ws in targets:
            try:
                # 1) JSON 전송
                await ws.send_json(send_dict)
                # 2) 바이너리(JPEG)
                await ws.send_bytes(data["image"])
            except Exception as e:
                self._logger.error(f"Broadcast Camera Error: {e}")
                disconnected.add(ws)

        for ws in disconnected:
            self._active_websockets["camera"].discard(ws)

    async def _broadcast_lidar(self, topic: str, single_ws: Optional[WebSocket] = None):
        """
        LiDAR(또는 다른 토픽)의 데이터 = 단일 JSON.
        single_ws가 주어지면 그 웹소켓만, 없으면 전체에게 전송.
        """
        data = self._last_data.get(topic)
        if not data:
            return

        disconnected = set()
        targets = [single_ws] if single_ws else list(self._active_websockets.get(topic, set()))

        for ws in targets:
            try:
                await ws.send_json(data)
            except Exception as e:
                self._logger.error(f"Broadcast {topic} Error: {e}")
                disconnected.add(ws)

        for ws in disconnected:
            self._active_websockets[topic].discard(ws)

    def get_connected_clients(self) -> dict:
        """
        현재 토픽별 연결된 WebSocket 클라이언트 수
        """
        return {t: len(ws_set) for t, ws_set in self._active_websockets.items()}

    def stop(self):
        """
        앱 종료 시 자원 정리
        """
        self._socket.close()
        self._context.term()
        self._logger.info("ZMQBridge stopped.")

# 전역 싱글톤(필요 시 main.py 등에서 직접 생성해도 무방)
bridge = ZMQBridge()
