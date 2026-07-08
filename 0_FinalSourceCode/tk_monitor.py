import json
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from uptech import UpTech


HOST = "0.0.0.0"
PORT = 8001
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
STATUS_FILE = os.path.join(BASE_DIR, "runtime", "match_status.json")
POLL_INTERVAL = 0.5
STALE_STATUS_SECONDS = 2.0

FD = 350
RD = 250
BD = 280
LD = 200

HTML_PAGE = """<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1.0" />
  <title>WheelFight Monitor</title>
  <style>
    :root {
      color-scheme: light;
      font-family: Arial, sans-serif;
      background: #f4f6f8;
      color: #17202a;
    }
    body {
      margin: 0;
      min-height: 100vh;
    }
    main {
      width: min(980px, calc(100vw - 24px));
      margin: 16px auto;
    }
    header {
      display: flex;
      align-items: baseline;
      justify-content: space-between;
      gap: 12px;
      margin-bottom: 12px;
    }
    h1 {
      margin: 0;
      font-size: 24px;
      font-weight: 700;
    }
    .stamp {
      color: #566573;
      font-size: 14px;
      white-space: nowrap;
    }
    .grid {
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 12px;
    }
    section {
      background: #fff;
      border: 1px solid #d9e0e7;
      border-radius: 8px;
      padding: 12px;
      box-shadow: 0 1px 4px rgba(17, 24, 39, 0.05);
    }
    h2 {
      margin: 0 0 10px;
      font-size: 16px;
    }
    .pairs {
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 8px;
    }
    .item {
      display: flex;
      justify-content: space-between;
      gap: 10px;
      padding: 7px 8px;
      border: 1px solid #e6ebf0;
      border-radius: 6px;
      background: #fbfcfd;
      min-height: 22px;
    }
    .label {
      color: #5d6d7e;
      font-size: 13px;
    }
    .value {
      font-variant-numeric: tabular-nums;
      font-weight: 700;
      overflow-wrap: anywhere;
      text-align: right;
    }
    .wide {
      grid-column: 1 / -1;
    }
    .ok {
      color: #1e8449;
    }
    .warn {
      color: #b9770e;
    }
    .bad {
      color: #a93226;
    }
    @media (max-width: 720px) {
      header {
        display: block;
      }
      .grid {
        grid-template-columns: 1fr;
      }
      .pairs {
        grid-template-columns: 1fr;
      }
    }
  </style>
</head>
<body>
  <main>
    <header>
      <h1>WheelFight Monitor</h1>
      <div class="stamp" id="updated">--</div>
    </header>
    <div class="grid">
      <section>
        <h2>Codes</h2>
        <div class="pairs" id="codes"></div>
      </section>
      <section>
        <h2>Match State</h2>
        <div class="pairs" id="match"></div>
      </section>
      <section>
        <h2>IO</h2>
        <div class="pairs" id="io"></div>
      </section>
      <section>
        <h2>ADC</h2>
        <div class="pairs" id="adc"></div>
      </section>
      <section class="wide">
        <h2>Status</h2>
        <div class="pairs" id="status"></div>
      </section>
    </div>
  </main>
  <script>
    function clsForStatus(data) {
      if (data.last_error) return 'bad';
      if (!data.status_file_ok || data.status_age === null || data.status_age > 2) return 'warn';
      return 'ok';
    }
    function valueText(value) {
      if (value === null || value === undefined || value === '') return '--';
      if (typeof value === 'number') return Number.isInteger(value) ? String(value) : value.toFixed(3);
      return String(value);
    }
    function renderPairs(id, data, names) {
      const root = document.getElementById(id);
      root.innerHTML = '';
      names.forEach((name) => {
        const row = document.createElement('div');
        row.className = 'item';
        const label = document.createElement('span');
        label.className = 'label';
        label.textContent = name;
        const value = document.createElement('span');
        value.className = 'value';
        value.textContent = valueText(data[name]);
        row.appendChild(label);
        row.appendChild(value);
        root.appendChild(row);
      });
    }
    function renderIndexed(id, data, prefix) {
      const names = Object.keys(data || {}).sort((a, b) => Number(a) - Number(b));
      const values = {};
      names.forEach((name) => values[`${prefix}${name}`] = data[name]);
      renderPairs(id, values, Object.keys(values));
    }
    function updateStatus() {
      fetch('/status')
        .then((response) => response.json())
        .then((data) => {
          document.getElementById('updated').textContent = new Date().toLocaleTimeString();
          renderPairs('codes', data.codes || {}, ['fence', 'edge', 'enemy', 'stage', 'slip']);
          renderPairs('match', data.match || {}, [
            'state',
            'state_reason',
            'match_running',
            'last_stage',
            'action_label',
            'action_index',
            'action_total',
            'stun_time',
            'tag_id'
          ]);
          renderIndexed('io', data.raw_io || {}, 'IO');
          renderIndexed('adc', data.raw_adc || {}, 'AD');
          renderPairs('status', data, ['status_file_ok', 'status_age', 'last_error']);
          document.getElementById('status').className = `pairs ${clsForStatus(data)}`;
        })
        .catch((error) => {
          document.getElementById('updated').textContent = `status error: ${error}`;
        });
    }
    setInterval(updateStatus, 500);
    updateStatus();
  </script>
</body>
</html>
"""


def code_platform(adc):
    ad_4 = adc[4]
    ad_5 = adc[5]
    if ad_4 + ad_5 > 7000:
        return 0
    if (ad_4 <= 3500) != (ad_5 <= 3500):
        return 2
    return 1


def code_fence(io, adc):
    io_0, io_1, io_2, io_3 = io[0], io[1], io[2], io[3]
    ad_0, ad_1, ad_2, ad_3 = adc[0], adc[1], adc[2], adc[3]

    if io_2 == 0 and io_1 == 1 and io_3 == 1 and ad_0 > FD and ad_1 < RD and ad_2 < BD and ad_3 < LD:
        return 1
    if io_3 == 0 and io_0 == 1 and io_2 == 1 and ad_0 < FD and ad_1 > RD and ad_2 < BD and ad_3 < LD:
        return 2
    if io_0 == 0 and io_1 == 1 and io_3 == 1 and ad_0 < FD and ad_1 < RD and ad_2 > BD and ad_3 < LD:
        return 3
    if io_1 == 0 and io_0 == 1 and io_2 == 1 and ad_0 < FD and ad_1 < RD and ad_2 < BD and ad_3 > LD:
        return 4
    if io_1 == 1 and io_2 == 1 and ad_0 > FD and ad_1 < RD and ad_2 < BD and ad_3 > LD:
        return 5
    if io_2 == 1 and io_3 == 1 and ad_0 > FD and ad_1 > RD and ad_2 < BD and ad_3 < LD:
        return 6
    if io_0 == 1 and io_3 == 1 and ad_0 < FD and ad_1 > RD and ad_2 > BD and ad_3 < LD:
        return 7
    if io_0 == 1 and io_1 == 1 and ad_0 < FD and ad_1 < RD and ad_2 > BD and ad_3 > LD:
        return 8
    if ad_0 > FD and ad_1 < RD and ad_2 > BD and ad_3 < LD:
        return 9
    if ad_0 < FD and ad_1 > RD and ad_2 < BD and ad_3 > LD:
        return 10
    if ad_0 > FD and ad_1 > RD and ad_2 < BD and ad_3 > LD:
        return 11
    if ad_0 > FD and ad_1 > RD and ad_2 > BD and ad_3 < LD:
        return 12
    if ad_0 > FD and ad_1 < RD and ad_2 > BD and ad_3 > LD:
        return 13
    if ad_0 < FD and ad_1 > RD and ad_2 > BD and ad_3 > LD:
        return 14
    if io_0 == 0 and io_1 == 0 and ad_0 < FD and ad_1 < RD:
        return 15
    if io_0 == 0 and io_3 == 0 and ad_0 < FD and ad_3 < LD:
        return 16
    if io_1 == 0 and io_2 == 0 and ad_1 < FD and ad_2 < RD:
        return 17
    if io_2 == 0 and io_3 == 0 and ad_2 < FD and ad_3 < LD:
        return 18
    return 101


def code_edge(io):
    io_4, io_5, io_6, io_7 = io[4], io[5], io[6], io[7]
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
    return 102


def code_enemy(io, adc, tag_id):
    io_0, io_1, io_2, io_3 = io[0], io[1], io[2], io[3]
    ad_0 = adc[0]
    if io_0 == 1 and io_1 == 1 and io_2 == 1 and io_3 == 1:
        return 0
    if io_0 == 0:
        if tag_id != 2:
            if ad_0 < 1000:
                return 1
            return 11
        return 5
    if io_1 == 0:
        return 2
    if io_2 == 0:
        return 3
    if io_3 == 0:
        return 4
    return 103


def code_slip(io, adc):
    io_4, io_5, io_6, io_7 = io[4], io[5], io[6], io[7]
    ad_4 = adc[4]
    ad_5 = adc[5]
    if io_4 == 1 and io_7 == 1 and io_5 == 0 and io_6 == 0:
        return 0
    if io_5 == 1 and io_6 == 1 and io_4 == 0 and io_7 == 0:
        return 1
    if ad_5 < 3500:
        return 2
    if ad_4 < 3500:
        return 3
    return 105


class Monitor:
    def __init__(self):
        self.uptech = UpTech()
        self.last_error = ""
        self.status = self._empty_status("starting")
        self._lock = threading.Lock()
        self._last_print_key = None
        try:
            self.uptech.ADC_IO_Open()
        except Exception as exc:
            self.last_error = f"ADC_IO_Open failed: {exc}"
            print(self.last_error)
        self._thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._thread.start()

    def _empty_status(self, error=""):
        return {
            "raw_io": {str(i): None for i in range(8)},
            "raw_adc": {str(i): None for i in range(7)},
            "codes": {
                "fence": None,
                "edge": None,
                "enemy": None,
                "stage": None,
                "slip": None,
            },
            "match": {},
            "status_age": None,
            "status_file_ok": False,
            "last_error": error,
        }

    def _read_match_status(self):
        try:
            with open(STATUS_FILE, "r", encoding="utf-8") as fp:
                match = json.load(fp)
            timestamp = match.get("timestamp")
            age = time.time() - timestamp if isinstance(timestamp, (int, float)) else None
            return match, age, True, ""
        except FileNotFoundError:
            return {}, None, False, "match status file not found"
        except Exception as exc:
            return {}, None, False, f"match status read failed: {exc}"

    def _read_sensors(self):
        raw_io = {str(i): self.uptech.ADC_IO_GetInputLevel(i) for i in range(8)}
        raw_adc = {str(i): self.uptech.ADC_Get_Channel(i) for i in range(7)}
        io = [raw_io[str(i)] for i in range(8)]
        adc = [raw_adc[str(i)] for i in range(7)]
        return raw_io, raw_adc, io, adc

    def _collect_status(self):
        match, age, file_ok, file_error = self._read_match_status()
        try:
            raw_io, raw_adc, io, adc = self._read_sensors()
            tag_id = match.get("tag_id", -1)
            status = {
                "raw_io": raw_io,
                "raw_adc": raw_adc,
                "codes": {
                    "fence": code_fence(io, adc),
                    "edge": code_edge(io),
                    "enemy": code_enemy(io, adc, tag_id),
                    "stage": code_platform(adc),
                    "slip": code_slip(io, adc),
                },
                "match": match,
                "status_age": age,
                "status_file_ok": file_ok,
                "last_error": file_error,
            }
            self.last_error = file_error
            return status
        except Exception as exc:
            self.last_error = f"sensor read failed: {exc}"
            status = self._empty_status(self.last_error)
            status["match"] = match
            status["status_age"] = age
            status["status_file_ok"] = file_ok
            return status

    def _print_if_changed(self, status):
        match_for_key = dict(status["match"])
        match_for_key.pop("timestamp", None)
        key = json.dumps(
            {
                "raw_io": status["raw_io"],
                "raw_adc": status["raw_adc"],
                "codes": status["codes"],
                "match": match_for_key,
                "last_error": status["last_error"],
            },
            sort_keys=True,
            ensure_ascii=False,
        )
        if key == self._last_print_key:
            return
        self._last_print_key = key

        match = status.get("match", {})
        codes = status.get("codes", {})
        io_line = " ".join(f"IO{i}={status['raw_io'].get(str(i))}" for i in range(8))
        adc_line = " ".join(f"AD{i}={status['raw_adc'].get(str(i))}" for i in range(7))
        code_line = " ".join(f"{name}={codes.get(name)}" for name in ("fence", "edge", "enemy", "stage", "slip"))
        match_line = (
            f"state={match.get('state', '--')} "
            f"reason={match.get('state_reason', '--')} "
            f"running={match.get('match_running', '--')} "
            f"action={match.get('action_label', '--')} "
            f"step={match.get('action_index', '--')}/{match.get('action_total', '--')} "
            f"tag={match.get('tag_id', '--')}"
        )
        age = status.get("status_age")
        age_text = "--" if age is None else f"{age:.2f}s"
        print(f"[{time.strftime('%H:%M:%S')}] {code_line} age={age_text}")
        print(f"  {match_line}")
        print(f"  {io_line}")
        print(f"  {adc_line}")
        if status.get("last_error"):
            print(f"  error={status['last_error']}")

    def _poll_loop(self):
        while True:
            status = self._collect_status()
            with self._lock:
                self.status = status
            self._print_if_changed(status)
            time.sleep(POLL_INTERVAL)

    def current_status(self):
        with self._lock:
            return json.loads(json.dumps(self.status, ensure_ascii=False))

    def close(self):
        try:
            self.uptech.ADC_IO_Close()
        except Exception:
            pass


class MonitorHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/":
            self._respond_text(HTML_PAGE, "text/html; charset=utf-8")
            return
        if self.path == "/status":
            self._respond_json(self.server.monitor.current_status())
            return
        self.send_error(404, "Not Found")

    def log_message(self, format, *args):
        return

    def _respond_text(self, text, content_type):
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


def run_monitor():
    monitor = Monitor()
    server = ThreadingHTTPServer((HOST, PORT), MonitorHandler)
    server.monitor = monitor
    print(f"WheelFight monitor running on http://{HOST}:{PORT}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopping monitor...")
    finally:
        monitor.close()
        server.server_close()
        print("Monitor stopped.")


if __name__ == "__main__":
    run_monitor()
