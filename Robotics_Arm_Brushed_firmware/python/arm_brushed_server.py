from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
import queue
import threading
import json
import serial
import serial.tools.list_ports
import uvicorn
import sys
import time
import asyncio

app = FastAPI()

HEARTBEAT_INTERVAL = 3
HEARTBEAT_TIMEOUT = 2

class ConnectionManager:
    def __init__(self):
        self.connections = []
        self.loop:asyncio.AbstractEventLoop | None = None

    def set_event_loop(self, loop):
        self.loop = loop

    def broadcast(self, message):
        for conn in self.connections:
            if self.loop is not None and self.loop.is_running():
                self.loop.create_task(conn.send_text(message))
    
    def add(self, socket):
        self.connections.append(socket)

    def remove(self, socket):
        if socket in self.connections:
            self.connections.remove(socket)

class ThreadState:
    def __init__(self):
        self.terminated = False
        self.activeThread = None
        self.serialConnected = False
        self.serial = None

def fetchPortList():
   return [ port.device for port in serial.tools.list_ports.comports()]

origins = ["*"]

app.add_middleware(
    CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

messageQueue = queue.Queue()
threadState = ThreadState()
connectionManager = ConnectionManager()
defaultConnection = fetchPortList()

def startSerialThread():
    if threadState.activeThread is None:
        t = threading.Thread(target=forwardCommandLoop)
        threadState.activeThread = t
        t.start()

def forwardCommandLoop():
    serialPort = None

    while True:
        if threadState.terminated:
            break
        message = None
        try:
            message = messageQueue.get(block=False)
        except queue.Empty:
            if serialPort is not None:
                continue
            time.sleep(0.5)
        
        print(f"msg: {message}")
        if serialPort is None:
            port = None
            if threadState.serial is None:
                ports = fetchPortList()
                if len(ports) > 0:
                    port = ports[0]
                    threadState.serial = port
            else:
                port = threadState.serial

            if port is not None:
                try:
                    serialPort = serial.Serial(port, 115200, timeout=1, write_timeout=1)
                    threadState.serialConnected = True
                    print(f"New serial connection to {port}")
                    connectionManager.broadcast(json.dumps({"serial":"SERIAL_CONNECT", "port":port}))
                except serial.SerialException as e:
                    print("Failed connection")
        if message is not None:
            if "port" in message:
                if serialPort is not None:
                    serialPort.close()
                serialPort = None
                port =message["port"]
                threadState.serial = port
            elif serialPort is not None:
                if not serialPort.is_open:
                    connectionManager.broadcast(json.dumps({
                        "serial":"SERIAL_DISCONNECT"
                    }))
                    serialPort = None
                    continue
                command = message["command"].strip()
                if command[0] == "p":
                    angle = command[1:]
                    try:
                        intAngle = int(angle)
                        command = f"p{intAngle:03}"
                    except ValueError:
                        pass
                
                try:
                    encoded = command.encode()
                    serialPort.write(encoded)
                    
                except serial.SerialException as e:
                    serialPort = None
                    connectionManager.broadcast(json.dumps({
                        "serial":"SERIAL_DISCONNECT"
                    }))
                    threadState.serialConnected = False
                    print("Serial Disconnected")

    if serialPort is not None:
        serialPort.close()

async def ws_heartbeat_task(socket):
    try:
        while True:
            await asyncio.sleep(HEARTBEAT_INTERVAL)
            await socket.send_json({"conn":"HEARTBEAT"})
    except asyncio.CancelledError:
        pass

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    connectionManager.set_event_loop(asyncio.get_running_loop())
    await websocket.accept()
    heartbeat_task = asyncio.create_task(ws_heartbeat_task(websocket))
    connectionManager.add(websocket)
    try:
        if threadState.serialConnected:
            await websocket.send_json({"serial":"SERIAL_CONNECT", "port":threadState.serial})
        while True:
            try:
                data = await asyncio.wait_for(websocket.receive_text(), HEARTBEAT_INTERVAL+HEARTBEAT_TIMEOUT)
            except asyncio.TimeoutError:
                print("Heartbeat timeout")
                break
            message = json.loads(data)
            if "conn" not in message:
                messageQueue.put(message)
            # print(data)

    except WebSocketDisconnect:
        pass

    heartbeat_task.cancel()
    connectionManager.remove(websocket)
    print("Client Disconnect")

@app.get("/api/ports")
def get_ports():
    ports = fetchPortList()
    return {"ports":ports}

@app.post("/api/connect")
def set_port(port_name:str):
    messageQueue.put({"port":port_name})

app.mount("/", StaticFiles(directory="frontend", html=True), name="frontend")


if __name__ == "__main__":
    startSerialThread()
    uvicorn.run(app, host="0.0.0.0", port=5000)
    threadState.terminated = True
