#!/usr/bin/env python3
"""
Neighbor discovery example for the dynamic UWB simulator.

This node:
1. Watches Gazebo model discovery via /gazebo/model_states
2. Subscribes to other drones' UWB topics dynamically
3. Tracks neighbors that are currently reaching this drone
"""

import rospy
import os
import sys
from collections import defaultdict
from gazebo_msgs.msg import ModelStates
from nexus_swarm_sim.msg import UwbRange

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
HELPER_DIR = os.path.join(REPO_ROOT, "scripts")
if HELPER_DIR not in sys.path:
    sys.path.insert(0, HELPER_DIR)

from vehicle_naming import model_name_to_public_id, model_name_to_ros_namespace, normalize_public_id


class NeighborDiscovery:
    def __init__(self, drone_id="nexus/1", drone_prefix="nexus", topic_suffix="range"):
        self.drone_id = normalize_public_id(drone_id, prefix=drone_prefix)
        self.drone_prefix = drone_prefix
        self.topic_suffix = topic_suffix

        self.neighbors = {}
        self.ping_history = defaultdict(list)
        self.max_history = 10
        self.max_range_m = rospy.get_param("~max_range_m", 50.0)
        self.min_quality = rospy.get_param("~min_quality", 0.5)

        self.known_subscribers = {}
        rospy.Subscriber("/gazebo/model_states", ModelStates, self._on_model_states)

        rospy.loginfo(
            "[Neighbor Discovery] %s: ready (prefix=%s, topic_suffix=%s)",
            self.drone_id,
            self.drone_prefix,
            self.topic_suffix,
        )

    def _on_model_states(self, msg):
        for model_name in msg.name:
            peer_id = model_name_to_public_id(model_name, self.drone_prefix)
            if peer_id == model_name:
                continue
            if peer_id == self.drone_id:
                continue
            if peer_id in self.known_subscribers:
                continue

            topic = f"{model_name_to_ros_namespace(model_name, self.drone_prefix)}/uwb/{self.topic_suffix}"
            self.known_subscribers[peer_id] = rospy.Subscriber(
                topic,
                UwbRange,
                self._on_ping_received,
                callback_args=peer_id,
            )
            rospy.loginfo(
                "[Neighbor Discovery] %s: listening to %s",
                self.drone_id,
                topic,
            )

    def _on_ping_received(self, msg, src_drone):
        if msg.dst_id != self.drone_id:
            return

        if msg.quality < self.min_quality:
            rospy.logwarn(
                "[Neighbor Discovery] %s: low quality ping from %s (quality=%.2f)",
                self.drone_id,
                src_drone,
                msg.quality,
            )
            return

        if msg.distance_3d > self.max_range_m:
            rospy.logwarn(
                "[Neighbor Discovery] %s: ping from %s exceeds max range (%.2fm)",
                self.drone_id,
                src_drone,
                msg.distance_3d,
            )
            return

        if src_drone not in self.neighbors:
            rospy.loginfo(
                "[Neighbor Discovery] %s: new neighbor discovered: %s",
                self.drone_id,
                src_drone,
            )

        self.neighbors[src_drone] = {
            "distance": msg.distance_3d,
            "rssi": msg.rssi,
            "quality": msg.quality,
            "los": msg.los,
            "last_ping": rospy.Time.now(),
            "payload": msg.payload,
        }

        self.ping_history[src_drone].append(
            {
                "distance": msg.distance_3d,
                "rssi": msg.rssi,
                "quality": msg.quality,
            }
        )
        if len(self.ping_history[src_drone]) > self.max_history:
            self.ping_history[src_drone].pop(0)

    def print_status(self):
        rospy.loginfo("\n%s", "=" * 60)
        rospy.loginfo("[Neighbor Discovery] %s STATUS", self.drone_id)
        rospy.loginfo("%s", "=" * 60)

        if not self.neighbors:
            rospy.loginfo("  No neighbors discovered yet")
        else:
            rospy.loginfo("  Active neighbors: %d", len(self.neighbors))
            for neighbor_id, state in sorted(self.neighbors.items()):
                payload_len = len(state["payload"])
                distances = [m["distance"] for m in self.ping_history[neighbor_id]]
                avg_dist = sum(distances) / len(distances)
                los_label = "LOS" if state["los"] else "NLOS"
                rospy.loginfo(
                    "    %s: dist=%.2fm (avg=%.2fm), rssi=%.1fdBm, quality=%.2f, %s, payload_size=%dB",
                    neighbor_id,
                    state["distance"],
                    avg_dist,
                    state["rssi"],
                    state["quality"],
                    los_label,
                    payload_len,
                )

        rospy.loginfo("%s\n", "=" * 60)


def main():
    rospy.init_node("neighbor_discovery", anonymous=False)
    drone_id = rospy.get_param("~drone_id", "nexus/1")
    drone_prefix = rospy.get_param("~drone_prefix", "nexus")
    topic_suffix = rospy.get_param("~topic_suffix", "range")

    discovery = NeighborDiscovery(drone_id, drone_prefix, topic_suffix)

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        discovery.print_status()
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Neighbor discovery node shutting down")
