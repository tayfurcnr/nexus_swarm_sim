#!/usr/bin/env python3
"""
Dynamic Drone UWB Simulation Monitor

This utility dynamically discovers drones in Gazebo (starting with a prefix)
and processes UWB range data from per-drone topics.

Subscribed Topics:
  - /gazebo/model_states       (Discovery)
  - /<drone>/uwb/range         (UwbRange)
  - /<drone>/uwb/raw_signal    (RawUWBSignal)

Monitor Output Topics:
  - /uwb_sim/distance/{pair}    (ground truth 3D distance)
  - /uwb_sim/distance2d/{pair}  (ground truth 2D horizontal distance)
  - /uwb_sim/nearby_ranges      (filtered neighbors within threshold — JSON)
"""

import rospy
import math
import json
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float64, String
from nexus_swarm_sim.msg import UwbRange, RawUWBSignal
import time


class DynamicUWBMonitor:
    """Monitor drones and per-drone UWB data dynamically discovered from Gazebo"""

    def __init__(self):
        drone_prefix = rospy.get_param('/drone_prefix', 'nexus')
        self.model_prefix = rospy.get_param('~model_prefix', drone_prefix)
        self.discovered_drones = set()
        
        self.drone_states = {}
        self.local_positions = {}
        self.latest_uwb_ranges = {}    # key: "src_dst"  → {msg, timestamp}
        self.latest_raw_signals = {}   # key: "src_dst"  → {msg, timestamp}

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
            if name.startswith(self.model_prefix) and name not in self.discovered_drones:
                self._setup_new_drone(name)

    def _setup_new_drone(self, drone_ns):
        """Initialize data structures and subscribers for a newly discovered drone"""
        rospy.loginfo(f"[UWBMonitor] Discovering new drone: {drone_ns}")
        self.discovered_drones.add(drone_ns)
        
        self.drone_states[drone_ns] = State()
        self.local_positions[drone_ns] = PoseStamped()

        # Subscribe to drone state (MAVROS)
        rospy.Subscriber(
            f'/{drone_ns}/mavros/state',
            State,
            self._state_callback,
            callback_args=drone_ns
        )

        # Subscribe to local position (MAVROS)
        rospy.Subscriber(
            f'/{drone_ns}/mavros/local_position/pose',
            PoseStamped,
            self._local_position_callback,
            callback_args=drone_ns
        )

        # Subscribe to per-drone processed range topic (Simulator)
        rospy.Subscriber(
            f'/{drone_ns}/uwb/range',
            UwbRange,
            self._uwb_range_callback,
            callback_args=drone_ns
        )

        # Subscribe to per-drone raw signal topic (Simulator)
        rospy.Subscriber(
            f'/{drone_ns}/uwb/raw_signal',
            RawUWBSignal,
            self._raw_signal_callback,
            callback_args=drone_ns
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

    def print_drone_status(self):
        if not self.discovered_drones:
            rospy.loginfo_throttle(5, "[UWBMonitor] Waiting for drones to be discovered...")
            return

        rospy.loginfo("\n" + "=" * 80)
        rospy.loginfo(f"DRONE POSITION STATUS ({len(self.discovered_drones)} discovered)")
        rospy.loginfo("=" * 80)

        # Sort for consistent display
        sorted_drones = sorted(list(self.discovered_drones))

        for ns in sorted_drones:
            pos   = self.get_drone_position(ns)
            state = self.drone_states.get(ns)
            tick  = "✓" if state and state.connected else "✗"
            rospy.loginfo(
                f"{tick} {ns.upper():8} | Position: ({pos[0]:7.2f}, {pos[1]:7.2f}, {pos[2]:7.2f}) m"
            )

        rospy.loginfo("\nUWB RANGE MEASUREMENTS (Dynamic links)")
        rospy.loginfo("=" * 80)

        nearby = {}
        # Iterate over all possible unique pairs
        for i, src in enumerate(sorted_drones):
            for dst in sorted_drones[i+1:]:
                dist_3d = self.get_distance_between_drones(src, dst)
                dist_2d = self.get_distance_2d_between_drones(src, dst)

                # Ensure publishers exist for this pair
                key = f"{src}_{dst}"
                if key not in self.dist_pub:
                    self.dist_pub[key]   = rospy.Publisher(f'/uwb_sim/distance/{key}',   Float64, queue_size=5)
                    self.dist2d_pub[key] = rospy.Publisher(f'/uwb_sim/distance2d/{key}', Float64, queue_size=5)

                self.dist_pub[key].publish(Float64(data=dist_3d))
                self.dist2d_pub[key].publish(Float64(data=dist_2d))

                # Check for measurements in both directions (TWR)
                uwb_msg_src, age_src = self._get_latest_range(src, dst)
                uwb_msg_dst, age_dst = self._get_latest_range(dst, src)

                meas_str = "NO MEAS"
                if uwb_msg_src:
                    los = "LOS" if uwb_msg_src.los else "NLOS"
                    meas_str = f"meas={uwb_msg_src.distance_3d:.3f}m ({los}, age={age_src:.2f}s)"
                
                rospy.loginfo(f"  {src} ↔ {dst:8} | GT: {dist_3d:7.3f}m | UWB: {meas_str}")

                # Accumulate nearby info
                if dist_2d <= self.near_threshold:
                    entry = {'dist_2d': round(dist_2d, 4), 'dist_3d': round(dist_3d, 4)}
                    nearby.setdefault(src, {})[dst] = entry
                    nearby.setdefault(dst, {})[src] = entry

        if nearby:
            self.nearby_pub.publish(String(data=json.dumps(nearby)))

        rospy.loginfo("=" * 80 + "\n")

    def _get_latest_range(self, src, dst):
        key = f"{src}_{dst}"
        if key in self.latest_uwb_ranges:
            data = self.latest_uwb_ranges[key]
            return data['msg'], (rospy.Time.now() - data['timestamp']).to_sec()
        return None, None


def main():
    rospy.init_node('swarm_uwb_monitor', anonymous=True)
    monitor = DynamicUWBMonitor()
    
    rate = rospy.Rate(1)  # 1 Hz status display
    while not rospy.is_shutdown():
        monitor.print_drone_status()
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
