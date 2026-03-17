#!/usr/bin/env python3
import argparse
import json
import math
import os
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse

try:
    import rospy
    from gazebo_msgs.msg import ModelStates
    from mavros_msgs.msg import State
    from nexus_swarm_sim.msg import UwbRange
    ROS_IMPORTS_AVAILABLE = True
except ImportError:
    rospy = None
    ModelStates = None
    State = None
    UwbRange = None
    ROS_IMPORTS_AVAILABLE = False

try:
    from rospkg import RosPack
except ImportError:
    RosPack = None


def resolve_host_ip():
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        udp_socket.connect(("8.8.8.8", 80))
        return udp_socket.getsockname()[0]
    except OSError:
        try:
            return socket.gethostbyname(socket.gethostname())
        except OSError:
            return "127.0.0.1"
    finally:
        udp_socket.close()


class SwarmDashboard:
    def __init__(self, demo_mode=False, host=None, port=None, drone_prefix=None):
        self.demo_mode = demo_mode
        if not self.demo_mode and not ROS_IMPORTS_AVAILABLE:
            raise RuntimeError("ROS dependencies are not available. Use --demo to run without ROS.")

        if self.demo_mode:
            self.drone_prefix = drone_prefix or "nexus"
            self.host = host or "0.0.0.0"
            self.port = int(port or 8787)
        else:
            self.drone_prefix = drone_prefix or rospy.get_param("~drone_prefix", rospy.get_param("/drone_prefix", "nexus"))
            self.host = host or rospy.get_param("~host", "0.0.0.0")
            self.port = int(port or rospy.get_param("~port", 8787))
        self.machine_hostname = socket.gethostname()
        self.machine_ip = resolve_host_ip()
        self.package_root = self._resolve_package_root()
        self.nexus_svg_path = os.path.join(self.package_root, "docs", "nexus.svg")
        self._lock = threading.Lock()
        self._discovered = set()
        self._vehicle_subscribers = {}
        self._vehicle_states = {}
        self._vehicle_positions = {}
        self._vehicle_fcu_urls = {}
        self._vehicle_last_seen = {}
        self._uwb_links = {}

        if not self.demo_mode:
            rospy.Subscriber("/gazebo/model_states", ModelStates, self._model_states_callback, queue_size=1)
        else:
            self._seed_demo_state()
            self._demo_thread = threading.Thread(target=self._demo_loop)
            self._demo_thread.daemon = True
            self._demo_thread.start()

        self._http_server = ThreadingHTTPServer((self.host, self.port), self._make_handler())
        self._http_thread = threading.Thread(target=self._http_server.serve_forever)
        self._http_thread.daemon = True
        self._http_thread.start()

        self._log(
            "[SwarmDashboard] Web UI ready at http://%s:%d (hostname=%s, ip=%s)",
            self.host if self.host != "0.0.0.0" else "localhost",
            self.port,
            self.machine_hostname,
            self.machine_ip,
        )
        if not self.demo_mode:
            rospy.on_shutdown(self._shutdown)

    def _shutdown(self):
        self._http_server.shutdown()
        self._http_server.server_close()

    def _resolve_package_root(self):
        if RosPack is not None:
            try:
                return RosPack().get_path("nexus_swarm_sim")
            except Exception:
                pass
        return str(Path(__file__).resolve().parents[1])

    def _log(self, message, *args):
        if not self.demo_mode and rospy is not None:
            rospy.loginfo(message, *args)
        else:
            print(message % args if args else message, flush=True)

    def _seed_demo_state(self):
        now = time.time()
        for index in range(5):
            drone_id = f"{self.drone_prefix}{index}"
            self._discovered.add(drone_id)
            self._vehicle_states[drone_id] = {
                "connected": True,
                "armed": False,
                "guided": False,
                "mode": "STABILIZE",
                "system_status": 3,
            }
            self._vehicle_positions[drone_id] = {
                "x": round(index * 2.0, 3),
                "y": round(math.sin(index * 0.5), 3),
                "z": 0.2,
            }
            self._vehicle_fcu_urls[drone_id] = f"tcp://127.0.0.1:{5760 + 10 * index}"
            self._vehicle_last_seen[drone_id] = now

    def _demo_loop(self):
        start = time.time()
        while True:
            now = time.time()
            with self._lock:
                sorted_ids = sorted(self._discovered)
                for index, drone_id in enumerate(sorted_ids):
                    phase = now - start + index * 0.35
                    self._vehicle_positions[drone_id] = {
                        "x": round(index * 2.0 + math.cos(phase) * 0.35, 3),
                        "y": round(math.sin(phase * 0.7) * 1.4, 3),
                        "z": round(0.2 + abs(math.sin(phase * 0.4)) * 0.12, 3),
                    }
                    self._vehicle_last_seen[drone_id] = now

                self._uwb_links = {}
                for left_index, src_id in enumerate(sorted_ids):
                    for dst_id in sorted_ids[left_index + 1:]:
                        src = self._vehicle_positions[src_id]
                        dst = self._vehicle_positions[dst_id]
                        dx = src["x"] - dst["x"]
                        dy = src["y"] - dst["y"]
                        dz = src["z"] - dst["z"]
                        distance_2d = math.sqrt(dx * dx + dy * dy)
                        distance_3d = math.sqrt(dx * dx + dy * dy + dz * dz)
                        key = tuple(sorted((src_id, dst_id)))
                        self._uwb_links[key] = {
                            "src": key[0],
                            "dst": key[1],
                            "distance_2d": round(distance_2d, 3),
                            "distance_3d": round(distance_3d, 3),
                            "los": True,
                            "rssi_dbm": round(-52.0 - distance_3d * 6.0, 2),
                            "quality": round(max(0.3, 1.0 - distance_3d / 20.0), 3),
                            "age_sec": 0.0,
                            "timestamp": now,
                        }
            time.sleep(0.2)

    def _make_handler(self):
        dashboard = self

        class DashboardHandler(BaseHTTPRequestHandler):
            def do_GET(self):
                parsed = urlparse(self.path)
                if parsed.path == "/":
                    self._send_html(dashboard.render_html())
                    return
                if parsed.path == "/api/state":
                    self._send_json(dashboard.snapshot())
                    return
                if parsed.path == "/assets/nexus.svg":
                    self._send_file(dashboard.nexus_svg_path, "image/svg+xml")
                    return
                self.send_error(404, "Not found")

            def log_message(self, format_string, *args):
                return

            def _send_html(self, body):
                encoded = body.encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(encoded)))
                self.end_headers()
                self.wfile.write(encoded)

            def _send_json(self, payload):
                encoded = json.dumps(payload).encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(encoded)))
                self.end_headers()
                self.wfile.write(encoded)

            def _send_file(self, path, content_type):
                if not os.path.exists(path):
                    self.send_error(404, "Not found")
                    return
                with open(path, "rb") as handle:
                    data = handle.read()
                self.send_response(200)
                self.send_header("Content-Type", content_type)
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)

        return DashboardHandler

    def _model_states_callback(self, msg):
        current_time = time.time()
        with self._lock:
            for index, name in enumerate(msg.name):
                if not name.startswith(self.drone_prefix):
                    continue
                pose = msg.pose[index]
                self._vehicle_positions[name] = {
                    "x": round(pose.position.x, 3),
                    "y": round(pose.position.y, 3),
                    "z": round(pose.position.z, 3),
                }
                self._vehicle_last_seen[name] = current_time
                if name in self._discovered:
                    continue
                self._discovered.add(name)
                self._vehicle_states[name] = {
                    "connected": False,
                    "armed": False,
                    "guided": False,
                    "mode": "N/A",
                    "system_status": 0,
                }
                self._vehicle_fcu_urls[name] = "N/A"
                self._vehicle_subscribers[name] = [
                    rospy.Subscriber(
                        f"/{name}/mavros/state",
                        State,
                        self._mavros_state_callback,
                        callback_args=name,
                        queue_size=10,
                    ),
                    rospy.Subscriber(
                        f"/{name}/uwb/range",
                        UwbRange,
                        self._uwb_range_callback,
                        callback_args=name,
                        queue_size=50,
                    ),
                ]
                rospy.loginfo("[SwarmDashboard] Tracking vehicle %s", name)

    def _mavros_state_callback(self, msg, drone_id):
        with self._lock:
            self._vehicle_states[drone_id] = {
                "connected": bool(msg.connected),
                "armed": bool(msg.armed),
                "guided": bool(msg.guided),
                "mode": msg.mode or "N/A",
                "system_status": int(msg.system_status),
            }
            self._vehicle_last_seen[drone_id] = time.time()
            self._vehicle_fcu_urls[drone_id] = rospy.get_param(f"/{drone_id}/mavros/fcu_url", "N/A")

    def _uwb_range_callback(self, msg, _drone_id):
        key = tuple(sorted((msg.src_id, msg.dst_id)))
        with self._lock:
            self._uwb_links[key] = {
                "src": key[0],
                "dst": key[1],
                "distance_2d": round(float(msg.distance_2d), 3),
                "distance_3d": round(float(msg.distance_3d), 3),
                "los": bool(msg.los),
                "rssi_dbm": round(float(msg.rssi), 2),
                "quality": round(float(msg.quality), 3),
                "age_sec": 0.0,
                "timestamp": time.time(),
            }

    def snapshot(self):
        now = time.time()
        with self._lock:
            vehicles = []
            for drone_id in sorted(self._discovered):
                position = self._vehicle_positions.get(drone_id, {"x": 0.0, "y": 0.0, "z": 0.0})
                state = self._vehicle_states.get(drone_id, {})
                index = self._extract_index(drone_id)
                vehicles.append(
                    {
                        "id": drone_id,
                        "label": f"NEXUS #{index + 1}" if index >= 0 else drone_id.upper(),
                        "namespace": f"/{drone_id}",
                        "host_ip": self.machine_ip,
                        "hostname": self.machine_hostname,
                        "fcu_url": self._vehicle_fcu_urls.get(drone_id, "N/A"),
                        "position": position,
                        "state": state,
                        "last_seen_sec": round(now - self._vehicle_last_seen.get(drone_id, now), 2),
                    }
                )

            links = []
            for key in sorted(self._uwb_links):
                link = dict(self._uwb_links[key])
                link["age_sec"] = round(now - link.pop("timestamp"), 2)
                if link["age_sec"] <= 5.0:
                    links.append(link)

            extents = self._compute_extents(vehicles)
            return {
                "generated_at": round(now, 3),
                "host": {
                    "hostname": self.machine_hostname,
                    "ip": self.machine_ip,
                    "web_url": f"http://{self.machine_ip}:{self.port}",
                },
                "summary": {
                    "vehicle_count": len(vehicles),
                    "link_count": len(links),
                    "drone_prefix": self.drone_prefix,
                },
                "map": extents,
                "vehicles": vehicles,
                "links": links,
            }

    @staticmethod
    def _extract_index(drone_id):
        suffix = ""
        for char in reversed(drone_id):
            if not char.isdigit():
                break
            suffix = char + suffix
        return int(suffix) if suffix else -1

    @staticmethod
    def _compute_extents(vehicles):
        if not vehicles:
            return {"min_x": -5.0, "max_x": 5.0, "min_y": -5.0, "max_y": 5.0}
        xs = [vehicle["position"]["x"] for vehicle in vehicles]
        ys = [vehicle["position"]["y"] for vehicle in vehicles]
        min_x = min(xs) - 2.0
        max_x = max(xs) + 2.0
        min_y = min(ys) - 2.0
        max_y = max(ys) + 2.0
        if math.isclose(min_x, max_x):
            min_x -= 2.0
            max_x += 2.0
        if math.isclose(min_y, max_y):
            min_y -= 2.0
            max_y += 2.0
        return {
            "min_x": round(min_x, 2),
            "max_x": round(max_x, 2),
            "min_y": round(min_y, 2),
            "max_y": round(max_y, 2),
        }

    @staticmethod
    def render_html():
        return """<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>Nexus Swarm Dashboard</title>
  <style>
    :root {
      --bg: #0b1117;
      --panel: rgba(13, 21, 31, 0.82);
      --panel-strong: rgba(18, 28, 41, 0.94);
      --panel-soft: rgba(18, 28, 41, 0.72);
      --line: rgba(136, 173, 205, 0.18);
      --grid: rgba(82, 122, 158, 0.12);
      --text: #e7f1f8;
      --muted: #8ea3b7;
      --accent: #ff6a3d;
      --accent-soft: rgba(255, 106, 61, 0.16);
      --ok: #3dd68c;
      --warn: #ffb347;
      --shadow: 0 24px 54px rgba(0, 0, 0, 0.34);
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      color: var(--text);
      font-family: "Segoe UI", "Helvetica Neue", Arial, sans-serif;
      background:
        radial-gradient(circle at top left, rgba(255,106,61,0.12), transparent 20rem),
        radial-gradient(circle at bottom right, rgba(61,214,140,0.08), transparent 26rem),
        linear-gradient(140deg, #081017 0%, #0d1620 55%, #0f1b26 100%);
    }
    .page {
      min-height: 100vh;
      padding: 18px;
      display: grid;
      gap: 16px;
    }
    .topbar {
      display: grid;
      grid-template-columns: minmax(0, 1fr) auto;
      gap: 14px;
      align-items: stretch;
    }
    .hero, .badge {
      border: 1px solid var(--line);
      background: linear-gradient(180deg, rgba(20,31,45,0.96), rgba(14,23,34,0.84));
      box-shadow: var(--shadow);
      backdrop-filter: blur(18px);
    }
    .hero {
      border-radius: 22px;
      padding: 20px 22px;
    }
    .eyebrow {
      margin: 0 0 8px;
      color: var(--accent);
      text-transform: uppercase;
      letter-spacing: 0.18em;
      font-size: 0.74rem;
      font-weight: 800;
    }
    h1 {
      margin: 0;
      font-size: clamp(2rem, 4vw, 3rem);
      letter-spacing: -0.03em;
      line-height: 0.95;
    }
    .sub {
      margin: 10px 0 0;
      color: var(--muted);
      max-width: 56rem;
      line-height: 1.5;
    }
    .stats {
      display: grid;
      gap: 10px;
      min-width: 220px;
    }
    .badge {
      border-radius: 18px;
      padding: 12px 14px;
      display: grid;
      place-items: center;
      font-weight: 800;
      font-size: 0.9rem;
    }
    .shell {
      display: grid;
      grid-template-columns: 330px minmax(0, 1fr) 320px;
      gap: 16px;
      min-height: calc(100vh - 160px);
    }
    .panel {
      border: 1px solid var(--line);
      border-radius: 22px;
      background: linear-gradient(180deg, rgba(16,25,36,0.92), rgba(10,17,25,0.84));
      box-shadow: var(--shadow);
      backdrop-filter: blur(16px);
      overflow: hidden;
    }
    .stack {
      display: grid;
      gap: 16px;
      align-content: start;
    }
    .panel-header {
      padding: 16px 18px 0;
    }
    .section-title {
      margin: 0;
      font-size: 0.72rem;
      text-transform: uppercase;
      letter-spacing: 0.18em;
      font-weight: 800;
      color: var(--muted);
    }
    .section-sub {
      margin: 8px 0 0;
      color: var(--muted);
      font-size: 0.9rem;
      line-height: 1.45;
    }
    .map-panel {
      display: grid;
      grid-template-rows: auto 1fr auto;
    }
    #mapWrap {
      position: relative;
      margin: 14px 16px 0;
      min-height: 680px;
      border-radius: 20px;
      overflow: hidden;
      border: 1px solid var(--line);
      background:
        linear-gradient(var(--grid) 1px, transparent 1px),
        linear-gradient(90deg, var(--grid) 1px, transparent 1px),
        radial-gradient(circle at center, rgba(255,255,255,0.02), rgba(255,255,255,0)),
        linear-gradient(180deg, rgba(7,11,17,0.88), rgba(12,18,27,0.96));
      background-size: 42px 42px, 42px 42px, 100% 100%, 100% 100%;
    }
    #mapSvg {
      width: 100%;
      height: 100%;
      display: block;
    }
    .map-footer {
      display: flex;
      justify-content: space-between;
      gap: 12px;
      padding: 14px 18px 18px;
      font-size: 0.88rem;
      color: var(--muted);
    }
    .detail-body, .meta-body {
      padding: 14px 18px 18px;
    }
    .grid {
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 10px;
    }
    .kv, .link-card, .vehicle-item {
      border: 1px solid rgba(136,173,205,0.14);
      background: linear-gradient(180deg, rgba(21,32,46,0.98), rgba(13,21,31,0.98));
    }
    .kv {
      padding: 12px;
      border-radius: 16px;
    }
    .kv small,
    .link-card small {
      display: block;
      margin-bottom: 6px;
      font-size: 0.66rem;
      text-transform: uppercase;
      letter-spacing: 0.12em;
      color: var(--muted);
    }
    .kv strong,
    .link-card strong {
      display: block;
      font-size: 0.94rem;
      line-height: 1.35;
      word-break: break-word;
    }
    .detail-shell {
      display: grid;
      gap: 14px;
    }
    .link-stack {
      display: grid;
      gap: 8px;
    }
    .link-card {
      border-radius: 16px;
      padding: 12px;
    }
    .vehicles-card {
      display: grid;
      grid-template-rows: auto 1fr;
    }
    #vehicleList {
      padding: 14px 16px 16px;
      display: grid;
      gap: 10px;
      max-height: calc(100vh - 260px);
      overflow: auto;
    }
    .vehicle-item {
      border-radius: 18px;
      padding: 14px;
      cursor: pointer;
      text-align: left;
      transition: transform 0.16s ease, border-color 0.16s ease, background 0.16s ease;
    }
    .vehicle-item:hover,
    .vehicle-item.active {
      transform: translateY(-1px);
      border-color: rgba(255,106,61,0.42);
      background: linear-gradient(180deg, rgba(37,27,25,0.98), rgba(18,24,34,0.98));
    }
    .vehicle-title {
      display: flex;
      justify-content: space-between;
      align-items: center;
      gap: 10px;
      margin-bottom: 8px;
      font-weight: 800;
    }
    .vehicle-meta {
      display: grid;
      gap: 4px;
      color: var(--muted);
      font-size: 0.88rem;
    }
    .pill {
      display: inline-flex;
      align-items: center;
      gap: 6px;
      padding: 4px 8px;
      border-radius: 999px;
      font-size: 0.72rem;
      font-weight: 800;
      letter-spacing: 0.08em;
      text-transform: uppercase;
      background: rgba(136,173,205,0.1);
      color: var(--muted);
    }
    .pill.ok {
      background: rgba(61,214,140,0.14);
      color: var(--ok);
    }
    .muted {
      color: var(--muted);
    }
    @media (max-width: 1320px) {
      .shell {
        grid-template-columns: 300px minmax(0, 1fr);
      }
      .right-stack {
        grid-column: 1 / -1;
      }
      #vehicleList {
        max-height: none;
      }
    }
    @media (max-width: 980px) {
      .topbar,
      .shell {
        grid-template-columns: 1fr;
      }
      .stats {
        grid-template-columns: repeat(3, minmax(0, 1fr));
        min-width: 0;
      }
    }
    @media (max-width: 720px) {
      .page {
        padding: 12px;
      }
      .stats,
      .grid {
        grid-template-columns: 1fr;
      }
      #mapWrap {
        min-height: 460px;
      }
    }
  </style>
</head>
<body>
  <div class="page">
    <header class="topbar">
      <section class="hero">
        <p class="eyebrow">Nexus Swarm Sim</p>
        <h1>Mission Control</h1>
        <p class="sub">Live tactical view of active vehicles, namespace topology, FCU endpoints, and UWB links across the swarm.</p>
      </section>
      <section class="stats">
        <div class="badge" id="hostBadge">Host: loading</div>
        <div class="badge" id="countBadge">Vehicles: 0</div>
        <div class="badge" id="linkBadge">Links: 0</div>
      </section>
    </header>

    <main class="shell">
      <section class="stack left-stack">
        <article class="panel detail-card">
          <div class="panel-header">
            <p class="section-title">Selected Vehicle</p>
            <p class="section-sub">Focused telemetry, namespace identity, FCU route, and recent UWB relationships.</p>
          </div>
          <div class="detail-body" id="vehicleDetail">
            <div class="muted">Select a vehicle on the map or from the roster.</div>
          </div>
        </article>

        <article class="panel meta-card">
          <div class="panel-header">
            <p class="section-title">System</p>
            <p class="section-sub">Host metadata and overall swarm session state.</p>
          </div>
          <div class="meta-body">
            <div class="grid" id="systemGrid"></div>
          </div>
        </article>
      </section>

      <section class="panel map-panel">
        <div class="panel-header">
          <p class="section-title">Tactical Map</p>
          <p class="section-sub">Vehicles are rendered from a top-down perspective. Click any marker to sync the detail panel.</p>
        </div>
        <div id="mapWrap">
          <svg id="mapSvg" viewBox="0 0 1200 680" preserveAspectRatio="xMidYMid meet"></svg>
        </div>
        <div class="map-footer">
          <span id="statusText">Waiting for swarm data...</span>
          <span id="clockText">Last refresh: --</span>
        </div>
      </section>

      <section class="stack right-stack">
        <article class="panel vehicles-card">
          <div class="panel-header">
            <p class="section-title">Active Vehicles</p>
            <p class="section-sub">Compact roster with live state, namespace, and current position.</p>
          </div>
          <div id="vehicleList"></div>
        </article>
      </section>
    </main>
  </div>

  <script>
    const state = {
      selected: null,
      latest: null,
      iconHref: '/assets/nexus.svg'
    };

    function formatNumber(value) {
      return Number.isFinite(value) ? value.toFixed(2) : '--';
    }

    function mapPoint(pos, extents, width, height, padding) {
      const rangeX = Math.max(extents.max_x - extents.min_x, 1);
      const rangeY = Math.max(extents.max_y - extents.min_y, 1);
      const x = padding + ((pos.x - extents.min_x) / rangeX) * (width - padding * 2);
      const y = height - padding - ((pos.y - extents.min_y) / rangeY) * (height - padding * 2);
      return { x, y };
    }

    function renderSystem(data) {
      const systemGrid = document.getElementById('systemGrid');
      systemGrid.innerHTML = '';
      const rows = [
        ['Hostname', data.host.hostname],
        ['Host IP', data.host.ip],
        ['Prefix', data.summary.drone_prefix],
        ['Vehicles', String(data.summary.vehicle_count)],
      ];
      rows.forEach(([label, value]) => {
        const item = document.createElement('div');
        item.className = 'kv';
        item.innerHTML = `<small>${label}</small><strong>${value}</strong>`;
        systemGrid.appendChild(item);
      });
      document.getElementById('hostBadge').textContent = `Host: ${data.host.ip}`;
      document.getElementById('countBadge').textContent = `Vehicles: ${data.summary.vehicle_count}`;
      document.getElementById('linkBadge').textContent = `Links: ${data.summary.link_count}`;
      document.getElementById('statusText').textContent = data.summary.vehicle_count
        ? 'Dashboard streaming live ROS swarm state.'
        : 'Waiting for active vehicles...';
      document.getElementById('clockText').textContent = `Last refresh: ${new Date().toLocaleTimeString()}`;
    }

    function renderVehicleList(data) {
      const list = document.getElementById('vehicleList');
      list.innerHTML = '';
      data.vehicles.forEach(vehicle => {
        const item = document.createElement('button');
        item.className = `vehicle-item${state.selected === vehicle.id ? ' active' : ''}`;
        item.type = 'button';
        item.onclick = () => {
          state.selected = vehicle.id;
          renderVehicleList(state.latest);
          renderVehicleDetail(state.latest);
          renderMap(state.latest);
        };
        item.innerHTML = `
          <div class="vehicle-title">
            <span>${vehicle.label}</span>
            <span class="pill ${vehicle.state.connected ? 'ok' : ''}">${vehicle.state.connected ? 'CONNECTED' : 'NO FCU'}</span>
          </div>
          <div class="muted">${vehicle.namespace}</div>
          <div class="muted">x=${formatNumber(vehicle.position.x)} y=${formatNumber(vehicle.position.y)} z=${formatNumber(vehicle.position.z)}</div>
        `;
        list.appendChild(item);
      });
    }

    function renderVehicleDetail(data) {
      const detail = document.getElementById('vehicleDetail');
      const selected = data.vehicles.find(vehicle => vehicle.id === state.selected) || data.vehicles[0];
      if (!selected) {
        detail.className = 'muted';
        detail.textContent = 'Select a vehicle on the map or from the list.';
        return;
      }
      state.selected = selected.id;
      const relatedLinks = data.links.filter(link => link.src === selected.id || link.dst === selected.id);
      detail.className = '';
      detail.innerHTML = `
        <div class="detail-shell">
          <div class="grid">
            <div class="kv"><small>Vehicle</small><strong>${selected.label}</strong></div>
            <div class="kv"><small>Namespace</small><strong>${selected.namespace}</strong></div>
            <div class="kv"><small>Host IP</small><strong>${selected.host_ip}</strong></div>
            <div class="kv"><small>FCU URL</small><strong>${selected.fcu_url}</strong></div>
            <div class="kv"><small>Mode</small><strong>${selected.state.mode || 'N/A'}</strong></div>
            <div class="kv"><small>Connected</small><strong>${selected.state.connected ? 'True' : 'False'}</strong></div>
            <div class="kv"><small>Armed</small><strong>${selected.state.armed ? 'True' : 'False'}</strong></div>
            <div class="kv"><small>Position</small><strong>${formatNumber(selected.position.x)}, ${formatNumber(selected.position.y)}, ${formatNumber(selected.position.z)}</strong></div>
          </div>
          <div>
            <p class="section-title" style="margin:0 0 10px;">UWB Neighbors</p>
            <div class="link-stack">
              ${relatedLinks.length ? relatedLinks.map(link => `
                <div class="link-card">
                  <small>${link.src} ↔ ${link.dst}</small>
                  <strong>${formatNumber(link.distance_3d)} m | ${link.los ? 'LOS' : 'NLOS'} | RSSI ${formatNumber(link.rssi_dbm)} dBm | age ${formatNumber(link.age_sec)} s</strong>
                </div>
              `).join('') : '<div class="muted">No recent UWB links for this vehicle.</div>'}
            </div>
          </div>
        </div>
      `;
    }

    function renderMap(data) {
      const svg = document.getElementById('mapSvg');
      const width = 1200;
      const height = 680;
      const padding = 70;
      svg.innerHTML = '';

      const defs = document.createElementNS('http://www.w3.org/2000/svg', 'defs');
      defs.innerHTML = `
        <filter id="shadow" x="-50%" y="-50%" width="200%" height="200%">
          <feDropShadow dx="0" dy="10" stdDeviation="12" flood-color="rgba(31,41,55,0.28)"/>
        </filter>
      `;
      svg.appendChild(defs);

      data.links.forEach(link => {
        const src = data.vehicles.find(vehicle => vehicle.id === link.src);
        const dst = data.vehicles.find(vehicle => vehicle.id === link.dst);
        if (!src || !dst) return;
        const p1 = mapPoint(src.position, data.map, width, height, padding);
        const p2 = mapPoint(dst.position, data.map, width, height, padding);
        const line = document.createElementNS('http://www.w3.org/2000/svg', 'line');
        line.setAttribute('x1', p1.x);
        line.setAttribute('y1', p1.y);
        line.setAttribute('x2', p2.x);
        line.setAttribute('y2', p2.y);
        line.setAttribute('stroke', link.los ? '#d24d2f' : '#64748b');
        line.setAttribute('stroke-width', state.selected && (state.selected === link.src || state.selected === link.dst) ? '5' : '3');
        line.setAttribute('stroke-dasharray', link.los ? '0' : '10 8');
        line.setAttribute('stroke-linecap', 'round');
        svg.appendChild(line);

        const label = document.createElementNS('http://www.w3.org/2000/svg', 'text');
        label.setAttribute('x', (p1.x + p2.x) / 2);
        label.setAttribute('y', (p1.y + p2.y) / 2 - 10);
        label.setAttribute('text-anchor', 'middle');
        label.setAttribute('font-size', '18');
        label.setAttribute('font-weight', '700');
        label.setAttribute('fill', '#22313f');
        label.textContent = `${formatNumber(link.distance_3d)} m`;
        svg.appendChild(label);
      });

      data.vehicles.forEach(vehicle => {
        const point = mapPoint(vehicle.position, data.map, width, height, padding);
        const group = document.createElementNS('http://www.w3.org/2000/svg', 'g');
        group.style.cursor = 'pointer';
        group.onclick = () => {
          state.selected = vehicle.id;
          renderVehicleList(state.latest);
          renderVehicleDetail(state.latest);
          renderMap(state.latest);
        };

        if (state.selected === vehicle.id) {
          const halo = document.createElementNS('http://www.w3.org/2000/svg', 'circle');
          halo.setAttribute('cx', point.x);
          halo.setAttribute('cy', point.y - 16);
          halo.setAttribute('r', '42');
          halo.setAttribute('fill', 'rgba(210,77,47,0.16)');
          group.appendChild(halo);
        }

        const image = document.createElementNS('http://www.w3.org/2000/svg', 'image');
        image.setAttributeNS('http://www.w3.org/1999/xlink', 'href', state.iconHref);
        image.setAttribute('x', point.x - 30);
        image.setAttribute('y', point.y - 58);
        image.setAttribute('width', '60');
        image.setAttribute('height', '60');
        image.setAttribute('filter', 'url(#shadow)');
        group.appendChild(image);

        const namePlate = document.createElementNS('http://www.w3.org/2000/svg', 'rect');
        namePlate.setAttribute('x', point.x - 52);
        namePlate.setAttribute('y', point.y + 8);
        namePlate.setAttribute('width', '104');
        namePlate.setAttribute('height', '28');
        namePlate.setAttribute('rx', '14');
        namePlate.setAttribute('fill', vehicle.state.connected ? 'rgba(255,255,255,0.98)' : 'rgba(231,236,242,0.98)');
        namePlate.setAttribute('stroke', state.selected === vehicle.id ? '#d24d2f' : 'rgba(31,41,55,0.14)');
        group.appendChild(namePlate);

        const label = document.createElementNS('http://www.w3.org/2000/svg', 'text');
        label.setAttribute('x', point.x);
        label.setAttribute('y', point.y + 27);
        label.setAttribute('text-anchor', 'middle');
        label.setAttribute('font-size', '15');
        label.setAttribute('font-weight', '700');
        label.setAttribute('fill', '#1f2937');
        label.textContent = vehicle.label;
        group.appendChild(label);

        const stateText = document.createElementNS('http://www.w3.org/2000/svg', 'text');
        stateText.setAttribute('x', point.x);
        stateText.setAttribute('y', point.y - 72);
        stateText.setAttribute('text-anchor', 'middle');
        stateText.setAttribute('font-size', '13');
        stateText.setAttribute('font-weight', '700');
        stateText.setAttribute('fill', vehicle.state.connected ? '#177245' : '#64748b');
        stateText.textContent = vehicle.state.connected ? vehicle.state.mode : 'NO FCU';
        group.appendChild(stateText);

        svg.appendChild(group);
      });
    }

    async function refresh() {
      try {
        const response = await fetch('/api/state', { cache: 'no-store' });
        const data = await response.json();
        state.latest = data;
        if (!state.selected && data.vehicles.length) {
          state.selected = data.vehicles[0].id;
        }
        renderSystem(data);
        renderVehicleList(data);
        renderVehicleDetail(data);
        renderMap(data);
      } catch (error) {
        document.getElementById('statusText').textContent = `Dashboard error: ${error}`;
      }
    }

    refresh();
    setInterval(refresh, 1000);
  </script>
</body>
</html>
"""


def main():
    parser = argparse.ArgumentParser(description="Nexus Swarm web dashboard")
    parser.add_argument("--demo", action="store_true", help="Run with synthetic demo data instead of ROS topics")
    parser.add_argument("--host", default=None, help="HTTP bind host")
    parser.add_argument("--port", type=int, default=None, help="HTTP bind port")
    parser.add_argument("--drone-prefix", default=None, help="Vehicle prefix for demo mode or ROS override")
    args, _ = parser.parse_known_args()

    auto_demo = not ROS_IMPORTS_AVAILABLE
    demo_mode = args.demo or auto_demo

    if demo_mode:
        if auto_demo and not args.demo:
            print("[SwarmDashboard] ROS modules not found. Starting in demo mode.", flush=True)
        dashboard = SwarmDashboard(
            demo_mode=True,
            host=args.host,
            port=args.port,
            drone_prefix=args.drone_prefix,
        )
        try:
            while True:
                time.sleep(1.0)
        except KeyboardInterrupt:
            dashboard._shutdown()
        return

    rospy.init_node("swarm_dashboard", anonymous=False)
    SwarmDashboard(
        demo_mode=False,
        host=args.host,
        port=args.port,
        drone_prefix=args.drone_prefix,
    )
    rospy.spin()


if __name__ == "__main__":
    main()
