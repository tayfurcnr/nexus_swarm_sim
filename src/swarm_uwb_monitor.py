#!/usr/bin/env python3
"""
Dynamic Drone UWB Simulation Monitor

This utility dynamically discovers drones in Gazebo (starting with a prefix)
and processes UWB range data from per-drone topics.

Subscribed Topics:
  - /gazebo/model_states       (Discovery)
  - /<vehicle>/uwb/range       (UwbRange)
  - /<vehicle>/uwb/raw_signal  (RawUWBSignal)

Monitor Output Topics:
  - /uwb_sim/distance/{pair}    (ground truth 3D distance)
  - /uwb_sim/distance2d/{pair}  (ground truth 2D horizontal distance)
  - /uwb_sim/nearby_ranges      (filtered neighbors within threshold — JSON)
"""

import rospy
import math
import json
import sys
import os
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float64, String
from nexus_swarm_sim.msg import UwbRange, RawUWBSignal

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
HELPER_DIR = os.path.join(os.path.dirname(SCRIPT_DIR), "scripts")
if HELPER_DIR not in sys.path:
    sys.path.insert(0, HELPER_DIR)

from vehicle_naming import (
    model_name_to_public_id,
    model_name_to_ros_namespace,
    vehicle_sort_key,
)


class DynamicUWBMonitor:
    """Monitor drones and per-drone UWB data dynamically discovered from Gazebo"""

    def __init__(self):
        drone_prefix = rospy.get_param('/drone_prefix', 'nexus')
        self.model_prefix = rospy.get_param('~model_prefix', drone_prefix)
        self.discovered_drones = set()
        self.model_to_public_id = {}
        
        self.drone_states = {}
        self.local_positions = {}
        self.latest_uwb_ranges = {}    # key: "src_dst"  → {msg, timestamp}
        self.latest_raw_signals = {}   # key: "src_dst"  → {msg, timestamp}
        self.refresh_hz = rospy.get_param('~refresh_hz', 4.0)
        self.use_terminal_ui = rospy.get_param('~use_terminal_ui', True)
        self.last_render = None

        # Ground truth distance publishers
        self.dist_pub = {}
        self.dist2d_pub = {}

        # Nearby ranges publisher
        self.nearby_pub    = rospy.Publisher('/uwb_sim/nearby_ranges', String, queue_size=5)
        self.near_threshold = rospy.get_param('~near_threshold', 3.0)

        self.debug_mode = rospy.get_param('~debug_mode', False)

        # Discovery Subscriber
        rospy.Subscriber('/gazebo/model_states', ModelStates, self._discovery_callback)
        rospy.loginfo(f"[SwarmUwbMonitor] Initialized. Watching for models starting with '{self.model_prefix}'")

    def _discovery_callback(self, msg):
        """Detect new models in Gazebo and setup subscribers dynamically"""
        for name in msg.name:
            public_id = model_name_to_public_id(name, self.model_prefix)
            if public_id == name or public_id in self.discovered_drones:
                continue
            if name.startswith(self.model_prefix):
                self._setup_new_drone(name)

    def _setup_new_drone(self, model_name):
        """Initialize data structures and subscribers for a newly discovered drone"""
        drone_id = model_name_to_public_id(model_name, self.model_prefix)
        drone_ns = model_name_to_ros_namespace(model_name, self.model_prefix)
        rospy.loginfo(f"[UWBMonitor] Discovering new drone: {drone_id} (model={model_name}, ns={drone_ns})")
        self.discovered_drones.add(drone_id)
        self.model_to_public_id[model_name] = drone_id
        
        self.drone_states[drone_id] = State()
        self.local_positions[drone_id] = PoseStamped()

        # Subscribe to drone state (MAVROS)
        rospy.Subscriber(
            f'{drone_ns}/mavros/state',
            State,
            self._state_callback,
            callback_args=drone_id
        )

        # Subscribe to local position (MAVROS)
        rospy.Subscriber(
            f'{drone_ns}/mavros/local_position/pose',
            PoseStamped,
            self._local_position_callback,
            callback_args=drone_id
        )

        # Subscribe to per-drone processed range topic (Simulator)
        rospy.Subscriber(
            f'{drone_ns}/uwb/range',
            UwbRange,
            self._uwb_range_callback,
            callback_args=drone_id
        )

        # Subscribe to per-drone raw signal topic (Simulator)
        rospy.Subscriber(
            f'{drone_ns}/uwb/raw_signal',
            RawUWBSignal,
            self._raw_signal_callback,
            callback_args=drone_id
        )

    # ------------------------------------------------------------------ #
    #  Callbacks                                                           #
    # ------------------------------------------------------------------ #

    def _state_callback(self, msg, drone_ns):
        self.drone_states[drone_ns] = msg

    def _local_position_callback(self, msg, drone_ns):
        self.local_positions[drone_ns] = msg

    def _uwb_range_callback(self, msg, drone_ns):
        key = f"{msg.src_id}_{msg.dst_id}"
        self.latest_uwb_ranges[key] = {
            'msg': msg,
            'timestamp': rospy.Time.now()
        }

    def _raw_signal_callback(self, msg, drone_ns):
        key = f"{msg.src_id}_{msg.dst_id}"
        self.latest_raw_signals[key] = {
            'msg': msg,
            'timestamp': rospy.Time.now()
        }

    # ------------------------------------------------------------------ #
    #  Helpers                                                             #
    # ------------------------------------------------------------------ #

    def get_drone_position(self, drone_ns):
        if drone_ns in self.local_positions:
            pos = self.local_positions[drone_ns].pose.position
            return [pos.x, pos.y, pos.z]
        return [0.0, 0.0, 0.0]

    def get_distance_between_drones(self, ns1, ns2):
        p1, p2 = self.get_drone_position(ns1), self.get_drone_position(ns2)
        return math.sqrt(sum((a - b) ** 2 for a, b in zip(p1, p2)))

    def get_distance_2d_between_drones(self, ns1, ns2):
        p1, p2 = self.get_drone_position(ns1), self.get_drone_position(ns2)
        return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

    def _ensure_pair_publishers(self, src, dst):
        key = f"{src}_{dst}"
        topic_key = key.replace("/", "_")
        if key not in self.dist_pub:
            self.dist_pub[key] = rospy.Publisher(f'/uwb_sim/distance/{topic_key}', Float64, queue_size=5)
            self.dist2d_pub[key] = rospy.Publisher(f'/uwb_sim/distance2d/{topic_key}', Float64, queue_size=5)
        return key

    def _render_snapshot(self, lines):
        snapshot = "\n".join(lines)
        if not self.use_terminal_ui:
            rospy.loginfo("\n%s", snapshot)
            return

        if snapshot == self.last_render:
            return

        sys.stdout.write("\033[2J\033[H")
        sys.stdout.write(snapshot + "\n")
        sys.stdout.flush()
        self.last_render = snapshot

    def print_drone_status(self):
        if not self.discovered_drones:
            rospy.loginfo_throttle(5, "[UWBMonitor] Waiting for drones to be discovered...")
            return

        lines = []
        lines.append("=" * 80)
        lines.append(f"DRONE POSITION STATUS ({len(self.discovered_drones)} discovered)")
        lines.append("=" * 80)

        # Sort for consistent display
        sorted_drones = sorted(self.discovered_drones, key=lambda vehicle_id: vehicle_sort_key(vehicle_id, self.model_prefix))
        positions = {ns: self.get_drone_position(ns) for ns in sorted_drones}

        for ns in sorted_drones:
            pos = positions[ns]
            state = self.drone_states.get(ns)
            tick = "CONNECTED" if state and state.connected else "DISCONNECTED"
            lines.append(
                f"{tick:12} {ns.upper():8} | Position: ({pos[0]:7.2f}, {pos[1]:7.2f}, {pos[2]:7.2f}) m"
            )

        lines.append("")
        lines.append("UWB RANGE MEASUREMENTS (Dynamic links)")
        lines.append("=" * 80)

        nearby = {}
        # Iterate over all possible unique pairs
        for i, src in enumerate(sorted_drones):
            for dst in sorted_drones[i+1:]:
                src_pos = positions[src]
                dst_pos = positions[dst]
                dx = src_pos[0] - dst_pos[0]
                dy = src_pos[1] - dst_pos[1]
                dz = src_pos[2] - dst_pos[2]
                dist_3d = math.sqrt(dx * dx + dy * dy + dz * dz)
                dist_2d = math.sqrt(dx * dx + dy * dy)

                key = self._ensure_pair_publishers(src, dst)

                self.dist_pub[key].publish(Float64(data=dist_3d))
                self.dist2d_pub[key].publish(Float64(data=dist_2d))

                uwb_msg_src, age_src = self._get_latest_range(src, dst)
                uwb_msg_dst, age_dst = self._get_latest_range(dst, src)

                meas_str = "NO MEAS"
                if uwb_msg_src:
                    los = "LOS" if uwb_msg_src.los else "NLOS"
                    meas_str = f"meas={uwb_msg_src.distance_3d:.3f}m ({los}, age={age_src:.2f}s)"
                elif uwb_msg_dst:
                    los = "LOS" if uwb_msg_dst.los else "NLOS"
                    meas_str = f"meas={uwb_msg_dst.distance_3d:.3f}m ({los}, age={age_dst:.2f}s, reverse)"
                
                lines.append(f"  {src} ↔ {dst:8} | GT: {dist_3d:7.3f}m | UWB: {meas_str}")

                # Accumulate nearby info
                if dist_2d <= self.near_threshold:
                    entry = {'dist_2d': round(dist_2d, 4), 'dist_3d': round(dist_3d, 4)}
                    nearby.setdefault(src, {})[dst] = entry
                    nearby.setdefault(dst, {})[src] = entry

        self.nearby_pub.publish(String(data=json.dumps(nearby)))

        lines.append("=" * 80)
        self._render_snapshot(lines)

    def _get_latest_range(self, src, dst):
        key = f"{src}_{dst}"
        if key in self.latest_uwb_ranges:
            data = self.latest_uwb_ranges[key]
            return data['msg'], (rospy.Time.now() - data['timestamp']).to_sec()
        return None, None


def main():
    rospy.init_node('swarm_uwb_monitor', anonymous=True)
    monitor = DynamicUWBMonitor()
    
    rate = rospy.Rate(monitor.refresh_hz)
    while not rospy.is_shutdown():
        monitor.print_drone_status()
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
