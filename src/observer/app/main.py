from fastapi import FastAPI, Request
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates

app = FastAPI(title="Project Odin Dashboard")

# 정적 파일 마운트
app.mount("/static", StaticFiles(directory="app/static"), name="static")

# 템플릿 설정
templates = Jinja2Templates(directory="app/templates")

@app.get("/")
async def root(request: Request):
    return templates.TemplateResponse(
        "dashboard.html",
        {"request": request, "title": "Project Odin Dashboard"}
    )