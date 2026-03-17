import math
import socket
import threading
import time
from http.server import ThreadingHTTPServer

try:
    import rospy
    from gazebo_msgs.msg import ModelStates
    from mavros_msgs.msg import State
    from nexus_swarm_sim.msg import UwbRange
    from sensor_msgs.msg import NavSatFix

    ROS_IMPORTS_AVAILABLE = True
except ImportError:
    rospy = None
    ModelStates = None
    State = None
    UwbRange = None
    NavSatFix = None
    ROS_IMPORTS_AVAILABLE = False

try:
    from dashboard.web import create_handler
except ImportError:
    from .web import create_handler


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
        self._lock = threading.Lock()
        self._discovered = set()
        self._vehicle_subscribers = {}
        self._vehicle_states = {}
        self._vehicle_positions = {}
        self._vehicle_gps = {}
        self._vehicle_headings = {}
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

        self._http_server = ThreadingHTTPServer((self.host, self.port), create_handler(self))
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

    def _log(self, message, *args):
        if not self.demo_mode and rospy is not None:
            rospy.loginfo(message, *args)
        else:
            print(message % args if args else message, flush=True)

    def _seed_demo_state(self):
        base_lat = -35.3632621
        base_lon = 149.1652374
        now = time.time()
        for index in range(1, 6):
            slot = index - 1
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
                "x": round(slot * 2.0, 3),
                "y": round(math.sin(slot * 0.5), 3),
                "z": 0.2,
            }
            self._vehicle_gps[drone_id] = {
                "lat": round(base_lat + slot * 0.00005, 7),
                "lon": round(base_lon + slot * 0.00005, 7),
                "alt": 0.2,
            }
            self._vehicle_fcu_urls[drone_id] = f"tcp://127.0.0.1:{5760 + 10 * slot}"
            self._vehicle_headings[drone_id] = round((slot * 18.0) % 360.0, 1)
            self._vehicle_last_seen[drone_id] = now

    def _demo_loop(self):
        base_lat = -35.3632621
        base_lon = 149.1652374
        start = time.time()
        while True:
            now = time.time()
            with self._lock:
                sorted_ids = sorted(self._discovered)
                for index, drone_id in enumerate(sorted_ids):
                    phase = now - start + index * 0.35
                    dx = math.cos(phase) * 0.35
                    dy = math.sin(phase * 0.7) * 1.4
                    self._vehicle_positions[drone_id] = {
                        "x": round(index * 2.0 + dx, 3),
                        "y": round(dy, 3),
                        "z": round(0.2 + abs(math.sin(phase * 0.4)) * 0.12, 3),
                    }
                    self._vehicle_gps[drone_id] = {
                        "lat": round(base_lat + index * 0.00005 + dx * 0.000009, 7),
                        "lon": round(base_lon + index * 0.00005 + dy * 0.000009, 7),
                        "alt": round(0.2 + abs(math.sin(phase * 0.4)) * 0.12, 3),
                    }
                    heading_deg = (math.degrees(math.atan2(math.cos(phase * 0.7) * 0.98, -math.sin(phase))) + 360.0) % 360.0
                    self._vehicle_headings[drone_id] = round(heading_deg, 1)
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
                self._vehicle_headings[name] = round(self._yaw_from_quaternion(pose.orientation), 1)
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
                self._vehicle_gps[name] = None
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
                    rospy.Subscriber(
                        f"/{name}/mavros/global_position/global",
                        NavSatFix,
                        self._gps_callback,
                        callback_args=name,
                        queue_size=10,
                    ),
                ]
                self._log("[SwarmDashboard] Tracking vehicle %s", name)

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

    def _gps_callback(self, msg, drone_id):
        if math.isnan(msg.latitude) or math.isnan(msg.longitude):
            return
        with self._lock:
            self._vehicle_gps[drone_id] = {
                "lat": round(msg.latitude, 7),
                "lon": round(msg.longitude, 7),
                "alt": round(msg.altitude, 3),
            }

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
                        "label": f"NEXUS #{index}" if index >= 0 else drone_id.upper(),
                        "namespace": f"/{drone_id}",
                        "host_ip": self.machine_ip,
                        "hostname": self.machine_hostname,
                        "fcu_url": self._vehicle_fcu_urls.get(drone_id, "N/A"),
                        "position": position,
                        "gps": self._vehicle_gps.get(drone_id),
                        "heading_deg": self._vehicle_headings.get(drone_id),
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
                "map": self._compute_extents(vehicles),
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
    def _yaw_from_quaternion(orientation):
        siny_cosp = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
        return (math.degrees(math.atan2(siny_cosp, cosy_cosp)) + 360.0) % 360.0
