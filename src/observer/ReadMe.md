Requirement

sudo apt install uvicorn
sudo pip install fastapi uvicorn[standard] jinja2 aiofiles
pip install pyzmq

Run

uvicorn app.main:app --reload --host 0.0.0.0 --port 8000




