#!/usr/bin/env python3
"""
Prototype ROV control server
- REST endpoints to start/stop ROS nodes on remote hosts via SSH
- WebSocket /ws/telemetry to push telemetry to connected UIs
- Serves static web UI from ../web

Configure remote hosts via environment variables:
ORANGEPI_HOST, ORANGEPI_USER, ORANGEPI_KEY
WINDOWS_HOST, WINDOWS_USER, WINDOWS_KEY

Run: python3 tools/rov_control_server.py
"""
import os
import asyncio
from fastapi import FastAPI, WebSocket, WebSocketDisconnect, Request
from fastapi.responses import JSONResponse, HTMLResponse
from fastapi.staticfiles import StaticFiles
import paramiko
from typing import List

app = FastAPI()

WEB_ROOT = os.path.join(os.path.dirname(__file__), '..', 'web')
app.mount('/static', StaticFiles(directory=WEB_ROOT), name='static')

ORANGEPI_HOST = os.environ.get('ORANGEPI_HOST', '192.168.1.100') # update the correct IP
ORANGEPI_USER = os.environ.get('ORANGEPI_USER', 'pi') # update the correct username
ORANGEPI_KEY = os.environ.get('ORANGEPI_KEY')  # path to private key 

WINDOWS_HOST = os.environ.get('WINDOWS_HOST', 'windows-host') # Note: Windows SSH server must be configured and running, or use WSL with an SSH server inside it
WINDOWS_USER = os.environ.get('WINDOWS_USER', 'user') # For Windows, key-based auth is less common; you may need to set up an SSH server that supports it, or use password auth (not implemented here for security reasons). If using WSL, you can set up key-based auth inside the WSL environment and point WINDOWS_KEY to that.
WINDOWS_KEY = os.environ.get('WINDOWS_KEY') # path to private key for Windows SSH, if applicable. If using WSL, this would be the key for the WSL SSH server.

ROS_DOMAIN = os.environ.get('ROS_DOMAIN_ID', '42') # ensure both Orange Pi and Windows host use the same ROS_DOMAIN_ID for communication


class ConnectionError(Exception):
    pass


async def ssh_exec(host: str, user: str, key_path: str, cmd: str, timeout: int = 10):
    """Execute cmd over SSH using paramiko in a thread to avoid blocking async loop."""
    def _run():
        client = paramiko.SSHClient()
        client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        try:
            if key_path:
                client.connect(hostname=host, username=user, key_filename=key_path, timeout=5)
            else:
                client.connect(hostname=host, username=user, timeout=5)
            stdin, stdout, stderr = client.exec_command(cmd)
            out = stdout.read().decode(errors='ignore')
            err = stderr.read().decode(errors='ignore')
            client.close()
            return out, err
        except Exception as e:
            raise ConnectionError(str(e))

    return await asyncio.to_thread(_run)


@app.get('/')
async def index():
    path = os.path.join(WEB_ROOT, 'index.html')
    with open(path, 'r', encoding='utf-8') as f:
        return HTMLResponse(f.read())


@app.post('/api/orangepi/start_nodes')
async def start_orangepi_nodes():
    # start nodes using ros2 launch under bash -lc so environment sourcing works
    cmd = f"bash -lc 'source ~/ros2_ws/install/local_setup.bash >/dev/null 2>&1; export ROS_DOMAIN_ID={ROS_DOMAIN}; nohup ros2 launch wu25 start_nodes.launch.py >/tmp/rov_nodes.log 2>&1 &'"
    try:
        out, err = await ssh_exec(ORANGEPI_HOST, ORANGEPI_USER, ORANGEPI_KEY, cmd)
        return JSONResponse({'ok': True, 'out': out, 'err': err})
    except ConnectionError as e:
        return JSONResponse({'ok': False, 'error': str(e)}, status_code=500)


@app.post('/api/orangepi/stop_nodes')
async def stop_orangepi_nodes():
    # attempt to kill thrusters and bno_node processes
    cmd = "bash -lc 'pkill -f bno_node || true; pkill -f thrusters || true; pkill -f ros2 || true'"
    try:
        out, err = await ssh_exec(ORANGEPI_HOST, ORANGEPI_USER, ORANGEPI_KEY, cmd)
        return JSONResponse({'ok': True, 'out': out, 'err': err})
    except ConnectionError as e:
        return JSONResponse({'ok': False, 'error': str(e)}, status_code=500)


@app.post('/api/windows/start_joy')
async def start_windows_joy():
    # Example: run joy node inside WSL on the Windows host
    cmd = f"wsl -d Ubuntu-22.04 -- bash -lc 'export ROS_DOMAIN_ID={ROS_DOMAIN}; source /opt/ros/jazzy/setup.bash >/dev/null 2>&1; nohup ros2 run joy joy_node >/tmp/joy.log 2>&1 &'"
    try:
        out, err = await ssh_exec(WINDOWS_HOST, WINDOWS_USER, WINDOWS_KEY, cmd)
        return JSONResponse({'ok': True, 'out': out, 'err': err})
    except ConnectionError as e:
        return JSONResponse({'ok': False, 'error': str(e)}, status_code=500)


@app.post('/api/windows/stop_joy')
async def stop_windows_joy():
    cmd = "wsl -d Ubuntu-22.04 -- bash -lc 'pkill -f joy_node || true'"
    try:
        out, err = await ssh_exec(WINDOWS_HOST, WINDOWS_USER, WINDOWS_KEY, cmd)
        return JSONResponse({'ok': True, 'out': out, 'err': err})
    except ConnectionError as e:
        return JSONResponse({'ok': False, 'error': str(e)}, status_code=500)


# telemetry WebSocket manager
class TelemetryManager:
    def __init__(self):
        self.clients: List[WebSocket] = []

    async def connect(self, ws: WebSocket):
        await ws.accept()
        self.clients.append(ws)

    def disconnect(self, ws: WebSocket):
        if ws in self.clients:
            self.clients.remove(ws)

    async def broadcast(self, message: str):
        for ws in list(self.clients):
            try:
                await ws.send_text(message)
            except Exception:
                self.disconnect(ws)


tm = TelemetryManager()


@app.websocket('/ws/telemetry')
async def websocket_telemetry(ws: WebSocket):
    await tm.connect(ws)
    try:
        while True:
            # this server only broadcasts telemetry pushed via REST POST; keep connection open
            await ws.receive_text()
    except WebSocketDisconnect:
        tm.disconnect(ws)


@app.post('/api/telemetry')
async def post_telemetry(request: Request):
    data = await request.body()
    await tm.broadcast(data.decode())
    return JSONResponse({'ok': True})


@app.post('/api/pid')
async def set_pid_params(request: Request):
    """Set PID gains on the running thruster_node via ros2 param set over SSH."""
    try:
        data = await request.json()
    except Exception:
        return JSONResponse({'ok': False, 'error': 'Invalid JSON'}, status_code=400)

    allowed = ['rot_kp', 'rot_ki', 'rot_kd', 'rot_i_zone', 'rot_max_output',
               'depth_kp', 'depth_ki', 'depth_kd']
    set_cmds = []
    for name in allowed:
        if name in data:
            val = data[name]
            if isinstance(val, (int, float)):
                set_cmds.append(f'ros2 param set /thruster_node {name} {float(val)}')

    if not set_cmds:
        return JSONResponse({'ok': False, 'error': 'No valid parameters provided'}, status_code=400)

    joined = ' && '.join(set_cmds)
    cmd = (f"bash -lc 'source ~/ros2_ws/install/local_setup.bash >/dev/null 2>&1; "
           f"export ROS_DOMAIN_ID={ROS_DOMAIN}; {joined}'")
    try:
        out, err = await ssh_exec(ORANGEPI_HOST, ORANGEPI_USER, ORANGEPI_KEY, cmd)
        return JSONResponse({'ok': True, 'out': out, 'err': err})
    except ConnectionError as e:
        return JSONResponse({'ok': False, 'error': str(e)}, status_code=500)


if __name__ == '__main__':
    import uvicorn
    uvicorn.run('tools.rov_control_server:app', host='0.0.0.0', port=8080, log_level='info')
