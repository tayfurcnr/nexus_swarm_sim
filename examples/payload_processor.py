#!/usr/bin/env python3
"""
Example: reading enriched UWB payloads from /<drone_id>/uwb/range_with_payload.
"""

import struct

import rospy
from gazebo_msgs.msg import ModelStates
from nexus_swarm_sim.msg import UwbRange


class UWBPayloadCodec:
    PAYLOAD_SIZE = 33

    @staticmethod
    def decode(payload):
        if len(payload) != UWBPayloadCodec.PAYLOAD_SIZE:
            return None

        crc = 0
        for byte in payload[:-1]:
            crc ^= byte
        if (crc & 0xFF) != payload[-1]:
            return None

        idx = 0
        version = payload[idx]
        idx += 1
        drone_num = payload[idx]
        idx += 1
        position = struct.unpack_from("!fff", payload, idx)
        idx += 12
        velocity = struct.unpack_from("!fff", payload, idx)
        idx += 12
        heading = struct.unpack_from("!f", payload, idx)[0]
        idx += 4
        flags = struct.unpack_from("!H", payload, idx)[0]

        return {
            "version": version,
            "drone_num": drone_num,
            "position": position,
            "velocity": velocity,
            "heading": heading,
            "armed": bool(flags & 0x01),
            "flying": bool(flags & 0x02),
        }


class PayloadProcessor:
    def __init__(self, drone_prefix="nexus", topic_suffix="range_with_payload"):
        self.drone_prefix = drone_prefix
        self.topic_suffix = topic_suffix
        self.drone_status = {}
        self.known_subscribers = {}

        rospy.Subscriber("/gazebo/model_states", ModelStates, self._on_model_states)

    def _on_model_states(self, msg):
        for model_name in msg.name:
            if not model_name.startswith(self.drone_prefix):
                continue
            if model_name in self.known_subscribers:
                continue

            topic = f"/{model_name}/uwb/{self.topic_suffix}"
            self.known_subscribers[model_name] = rospy.Subscriber(
                topic,
                UwbRange,
                self._uwb_callback,
            )
            rospy.loginfo("[PayloadProcessor] Listening to %s", topic)

    def _uwb_callback(self, msg):
        decoded = UWBPayloadCodec.decode(msg.payload)
        if decoded is None:
            rospy.loginfo("[%s] No valid payload in message", msg.src_id)
            return

        self.drone_status[msg.src_id] = {
            "drone_num": decoded["drone_num"],
            "position": decoded["position"],
            "velocity": decoded["velocity"],
            "heading": decoded["heading"],
            "armed": decoded["armed"],
            "flying": decoded["flying"],
            "rssi": msg.rssi,
            "distance": msg.distance_3d,
        }

        rospy.loginfo(
            "[%s] dist=%.2fm pos=(%.2f, %.2f, %.2f) heading=%.2f armed=%s flying=%s",
            msg.src_id,
            msg.distance_3d,
            decoded["position"][0],
            decoded["position"][1],
            decoded["position"][2],
            decoded["heading"],
            decoded["armed"],
            decoded["flying"],
        )

    def print_all_status(self):
        rospy.loginfo("\n%s", "=" * 60)
        rospy.loginfo("DRONE TELEMETRY FROM UWB PAYLOAD")
        rospy.loginfo("%s", "=" * 60)
        for drone_id, status in sorted(self.drone_status.items()):
            rospy.loginfo(
                "%s: pos=(%.2f, %.2f, %.2f), vel=(%.2f, %.2f, %.2f), armed=%s, RSSI=%.1fdBm",
                drone_id,
                status["position"][0],
                status["position"][1],
                status["position"][2],
                status["velocity"][0],
                status["velocity"][1],
                status["velocity"][2],
                status["armed"],
                status["rssi"],
            )
        rospy.loginfo("%s", "=" * 60)


def main():
    rospy.init_node("uwb_payload_processor")
    drone_prefix = rospy.get_param("~drone_prefix", "nexus")
    topic_suffix = rospy.get_param("~topic_suffix", "range_with_payload")
    processor = PayloadProcessor(drone_prefix, topic_suffix)

    rate = rospy.Rate(0.2)
    while not rospy.is_shutdown():
        processor.print_all_status()
        rate.sleep()


if __name__ == "__main__":
    main()
