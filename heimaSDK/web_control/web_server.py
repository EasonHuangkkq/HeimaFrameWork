import asyncio
import json
import socket
import threading
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles
from fastapi.responses import HTMLResponse
import uvicorn
import os

app = FastAPI()

# Make sure static folder exists
static_dir = os.path.join(os.path.dirname(__file__), "static")
if not os.path.exists(static_dir):
    os.makedirs(static_dir)

app.mount("/static", StaticFiles(directory=static_dir), name="static")

# UDP Socket setup
udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
C_PORT = 8888
C_HOST = "127.0.0.1"
PY_PORT = 8889

connected_clients = set()
clients_lock = threading.Lock()
loop_ref = None

def udp_listener_thread():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", PY_PORT))
    while True:
        try:
            data, _ = sock.recvfrom(2048)
            msg = data.decode('utf-8')
            if msg.startswith("batch|"):
                motors = msg.split('|')[1:]
                out_list = []
                for m in motors:
                    if not m: continue
                    parts = m.split(':')
                    if len(parts) >= 6:
                        out_list.append({
                            "alias": int(parts[0]),
                            "pos": float(parts[1]),
                            "vel": float(parts[2]),
                            "tor": float(parts[3]),
                            "temp": int(parts[4]),
                            "err": int(parts[5])
                        })
                
                out_str = json.dumps({"type": "batch", "data": out_list})
                if loop_ref and not loop_ref.is_closed():
                    with clients_lock:
                        for client in list(connected_clients):
                            asyncio.run_coroutine_threadsafe(client.send_text(out_str), loop_ref)
        except Exception as e:
            print(f"UDP Recv Error: {e}")

@app.on_event("startup")
async def startup_event():
    global loop_ref
    loop_ref = asyncio.get_running_loop()
    threading.Thread(target=udp_listener_thread, daemon=True).start()

@app.get("/")
async def get():
    index_path = os.path.join(static_dir, "index.html")
    with open(index_path, "r", encoding="utf-8") as f:
        return HTMLResponse(f.read())

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    with clients_lock:
        connected_clients.add(websocket)
    print("WebSocket connected!")
    try:
        while True:
            data = await websocket.receive_text()
            msg_obj = json.loads(data)
            cmd = msg_obj.get("cmd", "")
            alias = msg_obj.get("alias", 0)
            val = msg_obj.get("val", 0.0)
            
            udp_msg = f"{cmd}:{alias}:{val}"
            try:
                udp_sock.sendto(udp_msg.encode('utf-8'), (C_HOST, C_PORT))
            except Exception as e:
                print(f"UDP Error: {e}")
                
    except WebSocketDisconnect:
        with clients_lock:
            if websocket in connected_clients:
                connected_clients.remove(websocket)
        print("WebSocket disconnected")

if __name__ == "__main__":
    uvicorn.run("web_server:app", host="0.0.0.0", port=8000, reload=True)
