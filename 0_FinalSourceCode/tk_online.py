import os
import sys
import time
import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from motion_controller import MotionController

HOST = "0.0.0.0"
PORT = 8000

HTML_PAGE = """<!DOCTYPE html>
<html lang="zh-CN">
<head>
  <meta charset="UTF-8" />
  <title>WheelFight Remote Control</title>
  <style>
    body { font-family: Arial, sans-serif; background: #f3f3f3; color: #222; }
    .box { width: 560px; margin: 20px auto; padding: 18px; background: #fff; border-radius: 10px; box-shadow: 0 0 16px rgba(0,0,0,0.08); }
    .row { display: flex; justify-content: space-between; margin-bottom: 12px; }
    .row button { width: 150px; padding: 12px; font-size: 16px; }
    .panel { margin-top: 18px; padding: 14px; border: 1px solid #ddd; border-radius: 8px; background: #fafafa; }
    .panel div { margin-bottom: 8px; }
    .status { font-weight: bold; }
  </style>
</head>
<body>
  <div class="box">
    <h2>WheelFight Remote Control</h2>
    <div class="row">
      <button onclick="send('forward')">前进</button>
      <button onclick="send('backward')">后退</button>
    </div>
    <div class="row">
      <button onclick="send('left')">左转</button>
      <button onclick="send('stop')">停止</button>
      <button onclick="send('right')">右转</button>
    </div>
    <div class="row">
      <button onclick="enterPlatform('ahead')">前上台</button>
      <button onclick="enterPlatform('behind')">后上台</button>
    </div>
    <div class="panel">
      <div>
        <label for="speed_input">速度参数:</label>
        <input id="speed_input" type="number" min="0" value="400" style="width:100px; padding:6px; font-size:14px;" />
        <span style="color:#666;">仅使用正速度</span>
      </div>
      <div class="status">Fence code: <span id="fence">--</span></div>
      <div class="status">Edge code: <span id="edge">--</span></div>
      <div class="status">Enemy code: <span id="enemy">--</span></div>
      <div class="status">当前指令: <span id="current_cmd">--</span></div>
      <div>Sensor raw:</div>
      <div id="sensors">--</div>
      <div>提示：按浏览器地址栏回车刷新当前页面，支持远程控制。</div>
    </div>
  </div>
  <script>
    function getSpeed() {
      const value = parseInt(document.getElementById('speed_input').value, 10);
      return Number.isFinite(value) && value > 0 ? value : 400;
    }
    function send(cmd) {
      const speed = getSpeed();
      fetch(`/command?cmd=${cmd}&speed=${speed}`)
        .then(r => r.text())
        .then(t => {
          console.log(t);
          document.getElementById('current_cmd').innerText = t;
        })
        .catch(e => console.error(e));
    }
    function enterPlatform(mode) {
      fetch(`/platform?mode=${mode}`)
        .then(r => r.text())
        .then(t => {
          console.log(t);
          document.getElementById('current_cmd').innerText = t;
        })
        .catch(e => console.error(e));
    }
    function updateStatus() {
      fetch('/status')
        .then(r => r.json())
        .then(data => {
          document.getElementById('fence').innerText = data.fence_code;
          document.getElementById('edge').innerText = data.edge_code;
          document.getElementById('enemy').innerText = data.enemy_code;
          document.getElementById('sensors').innerText = data.raw_text;
          document.getElementById('current_cmd').innerText = data.current_command || '--';
        })
        .catch(err => console.error('status error', err));
    }
    setInterval(updateStatus, 1000);
    updateStatus();
  </script>
</body>
</html>
"""


class RemoteHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        parsed = urlparse(self.path)
        path = parsed.path
        params = parse_qs(parsed.query)

        if path == "/":
            self._respond_text(HTML_PAGE, content_type="text/html; charset=utf-8")
            return

        if path == "/command":
            cmd = params.get("cmd", [""])[0]
            speed = params.get("speed", [""])[0]
            result = self.server.controller.handle_command(cmd, speed)
            self._respond_text(result)
            return

        if path == "/platform":
            mode = params.get("mode", [""])[0]
            result = self.server.controller.handle_platform(mode)
            self._respond_text(result)
            return

        if path == "/status":
            status = self.server.controller.current_status()
            self._respond_json(status)
            return

        self.send_error(404, "Not Found")

    def log_message(self, format, *args):
        # 避免过多日志输出
        return

    def _respond_text(self, text, content_type="text/plain; charset=utf-8"):
        body = text.encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _respond_json(self, data):
        body = json.dumps(data, ensure_ascii=False).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)


class RemoteServer:
    def __init__(self):
        self.controller = MotionController()
        try:
            self.controller.uptech.ADC_IO_Open()
        except Exception as exc:
            print("Warning: ADC_IO_Open failed:", exc)

        self.last_fence_code = None
        self.last_edge_code = None
        self.last_enemy_code = None
        self.current_command = "--"
        self.controller.uptech.ADC_IO_SetAllIOMode(0)
        self.fence_status = {
            "fence_code": -1,
            "edge_code": -1,
            "enemy_code": -1,
            "raw_text": "--",
            "current_command": self.current_command,
        }
        self._update_thread = threading.Thread(target=self._poll_status, daemon=True)
        self._update_thread.start()

    def handle_command(self, cmd, speed_value):
        try:
            speed = abs(int(speed_value))
        except (TypeError, ValueError):
            speed = 400
        if speed == 0:
            speed = 400

        action = cmd.lower()
        print(f"收到远程命令: {action} speed={speed}")
        if action == "forward":
            self.controller.move_cmd(speed, speed)
            self.current_command = f"前进({speed})"
            return self.current_command
        if action == "backward":
            self.controller.move_cmd(-speed, -speed)
            self.current_command = f"后退({speed})"
            return self.current_command
        if action == "left":
            self.controller.move_cmd(-speed, speed)
            self.current_command = f"左转({speed})"
            return self.current_command
        if action == "right":
            self.controller.move_cmd(speed, -speed)
            self.current_command = f"右转({speed})"
            return self.current_command
        if action == "stop":
            self.controller.move_cmd(0, 0)
            self.current_command = "停止"
            return self.current_command
        self.current_command = "未知指令"
        return "unknown command"

    def handle_platform(self, mode):
        action = mode.lower()
        print(f"收到上台请求: {action}")
        self.controller.move_cmd(0, 0)
        if action == "ahead":
            self.controller.go_up_ahead_platform()
            self.current_command = "前上台"
            return "前上台"
        if action == "behind":
            self.controller.go_up_behind_platform()
            self.current_command = "后上台"
            return "后上台"
        self.current_command = "未知上台模式"
        return "unknown platform mode"

    def fence_detect(self):
        try:
            io_0 = self.controller.uptech.ADC_IO_GetInputLevel(0)
            io_1 = self.controller.uptech.ADC_IO_GetInputLevel(1)
            io_2 = self.controller.uptech.ADC_IO_GetInputLevel(2)
            io_3 = self.controller.uptech.ADC_IO_GetInputLevel(3)
            ad_0 = self.controller.uptech.ADC_Get_Channel(0)
            ad_1 = self.controller.uptech.ADC_Get_Channel(1)
            ad_2 = self.controller.uptech.ADC_Get_Channel(2)
            ad_3 = self.controller.uptech.ADC_Get_Channel(3)
        except Exception as exc:
            print("Fence detect read failed:", exc)
            return -1, "sensor read failed"

        raw_text = f"IO[0-3]={io_0},{io_1},{io_2},{io_3} AD[0-3]={ad_0},{ad_1},{ad_2},{ad_3}"
        FD = 150
        RD = 150
        BD = 150
        LD = 150

        if io_2 == 0 and io_1 == 1 and io_3 == 1 and ad_0 > FD and ad_1 < RD and ad_2 < BD and ad_3 < LD:
            return 1, raw_text
        if io_3 == 0 and io_0 == 1 and io_2 == 1 and ad_0 < FD and ad_1 > RD and ad_2 < BD and ad_3 < LD:
            return 2, raw_text
        if io_0 == 0 and io_1 == 1 and io_3 == 1 and ad_0 < FD and ad_1 < RD and ad_2 > BD and ad_3 < LD:
            return 3, raw_text
        if io_1 == 0 and io_0 == 1 and io_2 == 1 and ad_0 < FD and ad_1 < RD and ad_2 < BD and ad_3 > LD:
            return 4, raw_text
        if io_1 == 1 and io_2 == 1 and ad_0 > FD and ad_1 < RD and ad_2 < BD and ad_3 > LD:
            return 5, raw_text
        if io_2 == 1 and io_3 == 1 and ad_0 > FD and ad_1 > RD and ad_2 < BD and ad_3 < LD:
            return 6, raw_text
        if io_0 == 1 and io_3 == 1 and ad_0 < FD and ad_1 > RD and ad_2 > BD and ad_3 < LD:
            return 7, raw_text
        if io_0 == 1 and io_1 == 1 and ad_0 < FD and ad_1 < RD and ad_2 > BD and ad_3 > LD:
            return 8, raw_text
        if ad_0 > FD and ad_1 < RD and ad_2 > BD and ad_3 < LD:
            return 9, raw_text
        if ad_0 < FD and ad_1 > RD and ad_2 < BD and ad_3 > LD:
            return 10, raw_text
        if ad_0 > FD and ad_1 > RD and ad_2 < BD and ad_3 > LD:
            return 11, raw_text
        if ad_0 > FD and ad_1 > RD and ad_2 > BD and ad_3 < LD:
            return 12, raw_text
        if ad_0 > FD and ad_1 < RD and ad_2 > BD and ad_3 > LD:
            return 13, raw_text
        if ad_0 < FD and ad_1 > RD and ad_2 > BD and ad_3 > LD:
            return 14, raw_text
        if io_0 == 0 and io_1 == 0 and ad_0 < FD and ad_1 < RD:
            return 15, raw_text
        if io_0 == 0 and io_3 == 0 and ad_0 < FD and ad_3 < LD:
            return 16, raw_text
        if io_1 == 0 and io_2 == 0 and ad_1 < RD and ad_2 < BD:
            return 17, raw_text
        if io_2 == 0 and io_3 == 0 and ad_2 < BD and ad_3 < LD:
            return 18, raw_text
        return 101, raw_text

    def edge_detect(self):
        try:
            io_4 = self.controller.uptech.ADC_IO_GetInputLevel(4)
            io_5 = self.controller.uptech.ADC_IO_GetInputLevel(5)
            io_6 = self.controller.uptech.ADC_IO_GetInputLevel(6)
            io_7 = self.controller.uptech.ADC_IO_GetInputLevel(7)
            ad_0 = self.controller.uptech.ADC_Get_Channel(0)
        except Exception as exc:
            print("Edge detect read failed:", exc)
            return -1

        if io_4 == 0 and io_5 == 0 and io_6 == 0 and io_7 == 0:
            return 0
        if io_4 == 1 and io_5 == 0 and io_6 == 0 and io_7 == 0:
            return 1
        if io_4 == 0 and io_5 == 1 and io_6 == 0 and io_7 == 0:
            return 2
        if io_4 == 0 and io_5 == 0 and io_6 == 1 and io_7 == 0:
            return 3
        if io_4 == 0 and io_5 == 0 and io_6 == 0 and io_7 == 1:
            return 4
        if io_4 == 1 and io_5 == 1 and io_6 == 0 and io_7 == 0:
            return 5
        if io_4 == 0 and io_5 == 0 and io_6 == 1 and io_7 == 1:
            return 6
        if io_4 == 1 and io_5 == 0 and io_6 == 0 and io_7 == 1:
            return 7
        if io_4 == 0 and io_5 == 1 and io_6 == 1 and io_7 == 0:
            return 8
        if io_4 == 1 and io_5 == 1 and io_6 == 1 and io_7 == 1 and ad_0 > 1000:
            return 9
        if io_4 == 1 and io_5 == 1 and io_6 == 1 and io_7 == 1 and ad_0 <= 1000:
            return 10
        return 102

    def enemy_detect(self):
        try:
            io_0 = self.controller.uptech.ADC_IO_GetInputLevel(0)
            io_1 = self.controller.uptech.ADC_IO_GetInputLevel(1)
            io_2 = self.controller.uptech.ADC_IO_GetInputLevel(2)
            io_3 = self.controller.uptech.ADC_IO_GetInputLevel(3)
            ad_0 = self.controller.uptech.ADC_Get_Channel(0)
        except Exception as exc:
            print("Enemy detect read failed:", exc)
            return -1

        if io_0 == 1 and io_1 == 1 and io_2 == 1 and io_3 == 1:
            return 0
        if io_0 == 0:
            if ad_0 < 1000:
                return 1
            return 11
        if io_1 == 0:
            return 2
        if io_2 == 0:
            return 3
        if io_3 == 0:
            return 4
        return 103

    def _poll_status(self):
        while True:
            fence_code, raw_text = self.fence_detect()
            edge_code = self.edge_detect()
            enemy_code = self.enemy_detect()

            if fence_code != self.last_fence_code:
                self.last_fence_code = fence_code
                print(f"Fence state changed: {fence_code} | {raw_text}")
            if edge_code != self.last_edge_code:
                self.last_edge_code = edge_code
                print(f"Edge state changed: {edge_code}")
            if enemy_code != self.last_enemy_code:
                self.last_enemy_code = enemy_code
                print(f"Enemy state changed: {enemy_code}")

            self.fence_status = {
                "fence_code": fence_code,
                "edge_code": edge_code,
                "enemy_code": enemy_code,
                "raw_text": raw_text,
                "current_command": self.current_command,
            }
            time.sleep(0.5)

    def current_status(self):
        self.fence_status["current_command"] = self.current_command
        return self.fence_status

    def close(self):
        try:
            self.controller.move_cmd(0, 0)
        except Exception:
            pass
        try:
            self.controller.uptech.CDS_Close()
        except Exception:
            pass


def run_server():
    remote = RemoteServer()
    server = ThreadingHTTPServer((HOST, PORT), RemoteHandler)
    server.controller = remote
    print(f"Remote control server running on http://{HOST}:{PORT}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopping server...")
    finally:
        remote.close()
        server.server_close()
        print("Server stopped.")


if __name__ == "__main__":
    run_server()
