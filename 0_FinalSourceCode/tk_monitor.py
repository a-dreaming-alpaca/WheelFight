"""Read-only web monitor for the WheelFight match controller.

This process deliberately does not open the Mega serial port or the UpTech
ADC interface. The match controller is the sole hardware owner and publishes
an atomic JSON snapshot for this monitor to display.
"""

from __future__ import annotations

import argparse
import json
import os
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer


DEFAULT_HOST = "0.0.0.0"
DEFAULT_PORT = 8001
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_STATUS_FILE = os.path.join(BASE_DIR, "runtime", "match_status.json")
DEFAULT_STALE_SECONDS = 2.0


HTML_PAGE = """<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>WheelFight 运行监控</title>
  <style>
    :root { color-scheme: dark; font-family: system-ui, sans-serif; }
    body { margin: 0; background: #0d1117; color: #e6edf3; }
    main { width: min(1180px, calc(100vw - 24px)); margin: 18px auto 40px; }
    header { display: flex; justify-content: space-between; gap: 16px;
             align-items: baseline; margin-bottom: 14px; }
    h1 { margin: 0; font-size: 24px; }
    h2 { margin: 0 0 10px; font-size: 16px; color: #c9d1d9; }
    .stamp { color: #8b949e; font-variant-numeric: tabular-nums; }
    .grid { display: grid; grid-template-columns: repeat(2, minmax(0, 1fr));
            gap: 12px; }
    section { border: 1px solid #30363d; border-radius: 9px; padding: 12px;
              background: #161b22; }
    .wide { grid-column: 1 / -1; }
    .rows { display: grid; grid-template-columns: repeat(2, minmax(0, 1fr));
            gap: 7px; }
    .row, .ir { border: 1px solid #30363d; border-radius: 7px; padding: 8px;
                background: #0d1117; }
    .row { display: flex; justify-content: space-between; gap: 12px; }
    .label { color: #8b949e; font-size: 13px; }
    .value { font-weight: 650; text-align: right; overflow-wrap: anywhere;
             font-variant-numeric: tabular-nums; }
    .ir-grid { display: grid; grid-template-columns: repeat(4, minmax(0, 1fr));
               gap: 7px; }
    .ir strong { display: block; margin-bottom: 5px; }
    .ir small { color: #8b949e; font-variant-numeric: tabular-nums; }
    .ok { color: #3fb950; } .warn { color: #d29922; } .bad { color: #f85149; }
    pre { margin: 0; white-space: pre-wrap; overflow-wrap: anywhere;
          font: 13px/1.5 ui-monospace, monospace; }
    @media (max-width: 760px) {
      header { display: block; } .grid { grid-template-columns: 1fr; }
      .wide { grid-column: auto; } .rows { grid-template-columns: 1fr; }
      .ir-grid { grid-template-columns: repeat(2, minmax(0, 1fr)); }
    }
  </style>
</head>
<body>
<main>
  <header><h1>WheelFight 运行监控</h1><div id="stamp" class="stamp">等待状态…</div></header>
  <div class="grid">
    <section><h2>状态机与动作</h2><div id="match" class="rows"></div></section>
    <section><h2>安全与台面</h2><div id="safety" class="rows"></div></section>
    <section><h2>Mega 通信</h2><div id="link" class="rows"></div></section>
    <section><h2>摄像头识别</h2><div id="vision" class="rows"></div></section>
    <section class="wide"><h2>12 路红外测距（A0 正前，顺时针）</h2><div id="ir" class="ir-grid"></div></section>
    <section><h2>灰度与数字量原始值</h2><div id="raw" class="rows"></div></section>
    <section><h2>目标簇</h2><pre id="clusters">--</pre></section>
  </div>
</main>
<script>
  const bearings = [0, 30, 60, 90, 120, 150, 180, -150, -120, -90, -60, -30];
  const at = (obj, path, fallback='--') => {
    let value = obj;
    for (const part of path.split('.')) value = value && value[part];
    return value === null || value === undefined || value === '' ? fallback : value;
  };
  const fmt = (value) => typeof value === 'number' && !Number.isInteger(value)
    ? value.toFixed(3) : String(value);
  function renderRows(id, rows) {
    const root = document.getElementById(id); root.innerHTML = '';
    for (const [labelText, rawValue, className=''] of rows) {
      const row = document.createElement('div'); row.className = 'row';
      const label = document.createElement('span'); label.className = 'label';
      const value = document.createElement('span'); value.className = `value ${className}`;
      label.textContent = labelText; value.textContent = fmt(rawValue);
      row.append(label, value); root.appendChild(row);
    }
  }
  function render(data) {
    const monitor = data.monitor || {};
    const sensor = data.sensor || {};
    const link = data.sensor_link || {};
    const vision = data.vision || {};
    const backend = data.vision_backend || {};
    const statusClass = monitor.file_ok && !monitor.stale ? 'ok' : (monitor.file_ok ? 'warn' : 'bad');
    document.getElementById('stamp').className = `stamp ${statusClass}`;
    document.getElementById('stamp').textContent = monitor.file_ok
      ? `状态文件 ${fmt(at(data, 'monitor.age_seconds'))} 秒前更新`
      : at(data, 'monitor.error');
    renderRows('match', [
      ['状态', at(data, 'state')], ['原因', at(data, 'state_reason')],
      ['已收到开赛手势', at(data, 'match_started')], ['比赛计时/s', at(data, 'match_elapsed')],
      ['左/右指令', `${at(data, 'command.left')}/${at(data, 'command.right')}`],
      ['动作', at(data, 'command.label')], ['铲子', at(data, 'shovel_pose')],
      ['控制进程运行', at(data, 'match_running')]
    ]);
    renderRows('safety', [
      ['台面状态', at(data, 'sensor.platform_state')],
      ['前/后在台上', `${at(data, 'sensor.front_on_platform')}/${at(data, 'sensor.rear_on_platform')}`],
      ['前左/前右边缘', `${at(data, 'sensor.front_left_edge')}/${at(data, 'sensor.front_right_edge')}`],
      ['后方高物体', at(data, 'sensor.rear_high_object')],
      ['左/右开赛手势', `${at(data, 'sensor.start_left_hand_near')}/${at(data, 'sensor.start_right_hand_near')}`],
      ['传感器帧年龄/s', at(data, 'sensor.age')]
    ]);
    renderRows('link', [
      ['端口', at(data, 'sensor_link.port')], ['已连接', at(data, 'sensor_link.connected')],
      ['帧率/Hz', at(data, 'sensor_link.rate_hz')], ['序号', at(data, 'sensor.sequence')],
      ['有效帧', at(data, 'sensor_link.valid_frames')], ['坏帧', at(data, 'sensor_link.invalid_frames')],
      ['CRC 错误', at(data, 'sensor_link.checksum_errors')], ['丢帧', at(data, 'sensor_link.dropped_frames')],
      ['重复帧', at(data, 'sensor_link.duplicate_frames')], ['串口错误', at(data, 'sensor_link.last_error')]
    ]);
    renderRows('vision', [
      ['分类', at(data, 'vision.classification')], ['黄绿色占比', at(data, 'vision.gain_color_ratio')],
      ['红色占比', at(data, 'vision.harmful_color_ratio')], ['置信度', at(data, 'vision.confidence')],
      ['红色X分数', at(data, 'vision.red_x_score')], ['红色X确认', at(data, 'vision.red_x_detected')],
      ['最佳交叉角度/°', at(data, 'vision.red_x_angle_deg')],
      ['结果年龄/s', at(data, 'vision.age')],
      ['后端健康', at(data, 'vision_backend.healthy')], ['摄像头序号', at(data, 'vision_backend.camera_index')],
      ['可用模式', at(data, 'vision_available')], ['错误', at(data, 'vision.error', at(data, 'vision_backend.last_error'))]
    ]);
    const raw = sensor.raw_analog || [], filtered = sensor.filtered_analog || [];
    const active = sensor.infrared_active || [], irRoot = document.getElementById('ir');
    irRoot.innerHTML = '';
    for (let i = 0; i < 12; i++) {
      const card = document.createElement('div'); card.className = `ir ${active[i] ? 'warn' : ''}`;
      const title = document.createElement('strong'); title.textContent = `A${i} · ${bearings[i]}°`;
      const detail = document.createElement('small');
      detail.textContent = `原始 ${raw[i] ?? '--'} · 滤波 ${filtered[i] ?? '--'} · 触发 ${active[i] ?? '--'}`;
      card.append(title, detail); irRoot.appendChild(card);
    }
    const digital = sensor.raw_digital || [];
    renderRows('raw', [
      ['A12 前灰度', raw[12] ?? '--'], ['A13 后灰度', raw[13] ?? '--'],
      ['A12 前滤波', filtered[12] ?? '--'], ['A13 后滤波', filtered[13] ?? '--'],
      ['DI0 前左', digital[0] ?? '--'], ['DI1 前右', digital[1] ?? '--'],
      ['DI2 后方', digital[2] ?? '--']
    ]);
    document.getElementById('clusters').textContent = JSON.stringify(sensor.clusters || [], null, 2);
  }
  async function refresh() {
    try { const response = await fetch('/status', {cache: 'no-store'}); render(await response.json()); }
    catch (error) { document.getElementById('stamp').textContent = `读取失败：${error}`; }
  }
  setInterval(refresh, 500); refresh();
</script>
</body>
</html>
"""


class StatusReader:
    def __init__(
        self,
        status_file: str = DEFAULT_STATUS_FILE,
        stale_seconds: float = DEFAULT_STALE_SECONDS,
    ) -> None:
        self.status_file = status_file
        self.stale_seconds = stale_seconds

    def read(self) -> dict:
        try:
            with open(self.status_file, "r", encoding="utf-8") as fp:
                status = json.load(fp)
            if not isinstance(status, dict):
                raise ValueError("status root must be a JSON object")
            timestamp = status.get("timestamp")
            age = (
                max(0.0, time.time() - timestamp)
                if isinstance(timestamp, (int, float))
                else None
            )
            status["monitor"] = {
                "file_ok": True,
                "age_seconds": age,
                "stale": age is None or age > self.stale_seconds,
                "error": "",
            }
            return status
        except FileNotFoundError:
            error = f"状态文件不存在：{self.status_file}"
        except Exception as exc:
            error = f"状态文件读取失败：{exc}"
        return {
            "monitor": {
                "file_ok": False,
                "age_seconds": None,
                "stale": True,
                "error": error,
            }
        }


class MonitorHandler(BaseHTTPRequestHandler):
    def do_GET(self) -> None:
        if self.path == "/" or self.path.startswith("/?"):
            self._respond(HTML_PAGE.encode("utf-8"), "text/html; charset=utf-8")
            return
        if self.path == "/status" or self.path.startswith("/status?"):
            body = json.dumps(
                self.server.status_reader.read(), ensure_ascii=False
            ).encode("utf-8")
            self._respond(body, "application/json; charset=utf-8")
            return
        self.send_error(404, "Not Found")

    def log_message(self, format: str, *args) -> None:
        return

    def _respond(self, body: bytes, content_type: str) -> None:
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="WheelFight read-only web monitor")
    parser.add_argument("--host", default=DEFAULT_HOST)
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--status-file", default=DEFAULT_STATUS_FILE)
    return parser.parse_args()


def run_monitor() -> None:
    args = parse_args()
    server = ThreadingHTTPServer((args.host, args.port), MonitorHandler)
    server.status_reader = StatusReader(args.status_file)
    print(f"WheelFight monitor: http://{args.host}:{args.port}")
    print(f"Read-only status source: {os.path.abspath(args.status_file)}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopping monitor...")
    finally:
        server.server_close()


if __name__ == "__main__":
    run_monitor()
