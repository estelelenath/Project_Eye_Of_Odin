# observer/app/main.py

import asyncio
from fastapi import FastAPI, WebSocket
from fastapi.templating import Jinja2Templates
from fastapi.staticfiles import StaticFiles
from fastapi.responses import HTMLResponse
from fastapi.requests import Request
import logging
from pathlib import Path

from bridge.zmq_bridge import bridge  # 싱글톤 인스턴스 import

# FastAPI 앱 생성
app = FastAPI(title="Project Eye of Odin - Observer")

# 로깅 설정
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("Observer")

# 템플릿 & 정적 파일 설정
BASE_DIR = Path(__file__).resolve().parent
templates = Jinja2Templates(directory=str(BASE_DIR / "templates"))
app.mount("/static", StaticFiles(directory=str(BASE_DIR / "static")), name="static")

# Startup/Shutdown 이벤트 핸들러
@app.on_event("startup")
async def startup_event():
    """
    앱 시작 시 ZMQ 브릿지 시작
    """
    logger.info("Starting ZMQ Bridge...")
    # bridge.start()를 비동기로 실행
    asyncio.create_task(bridge.start())
    logger.info("ZMQ Bridge started")

@app.on_event("shutdown")
def shutdown_event():
    """
    앱 종료 시 ZMQ 브릿지 정리
    """
    logger.info("Stopping ZMQ Bridge...")
    bridge.stop()
    logger.info("ZMQ Bridge stopped")

# WebSocket 라우트
@app.websocket("/ws/camera")
async def websocket_camera_endpoint(websocket: WebSocket):
    """
    카메라 WebSocket 연결
    클라이언트는 이 endpoint로 연결하여 실시간 카메라 영상을 받음
    """
    await bridge.register_websocket("camera", websocket)

@app.websocket("/ws/lidar")
async def websocket_lidar_endpoint(websocket: WebSocket):
    """
    LiDAR WebSocket 연결
    클라이언트는 이 endpoint로 연결하여 실시간 LiDAR 데이터를 받음
    """
    await bridge.register_websocket("lidar", websocket)

# HTTP 라우트 - 대시보드 페이지
@app.get("/", response_class=HTMLResponse)
async def dashboard(request: Request):
    """
    메인 대시보드 페이지 렌더링
    """
    return templates.TemplateResponse(
        "dashboard.html",
        {
            "request": request,
            "connected_clients": bridge.get_connected_clients()
        }
    )

# 상태 확인 API
@app.get("/api/status")
def get_status():
    """
    브릿지 상태 정보 반환
    """
    return {
        "status": "running",
        "connected_clients": bridge.get_connected_clients()
    }

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(
        "main:app",
        host="0.0.0.0",
        port=8000,
        reload=True,
        reload_dirs=["app", "bridge"]
    )




# from fastapi import FastAPI, Request
# from fastapi.staticfiles import StaticFiles
# from fastapi.templating import Jinja2Templates

# app = FastAPI(title="Project Odin Dashboard")

# # 정적 파일 마운트
# app.mount("/static", StaticFiles(directory="app/static"), name="static")

# # 템플릿 설정
# templates = Jinja2Templates(directory="app/templates")

# @app.get("/")
# async def root(request: Request):
#     return templates.TemplateResponse(
#         "dashboard.html",
#         {"request": request, "title": "Project Odin Dashboard"}
#     )