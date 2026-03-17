#!/usr/bin/env python3
"""
Payload injector example for the dynamic UWB simulator.

This node enriches a drone's outgoing UWB range messages by embedding
its current state into the payload field and republishes them to
/<drone_id>/uwb/range_with_payload.
"""

import struct

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from nexus_swarm_sim.msg import UwbRange


class UWBPayloadCodec:
    VERSION = 0x01
    PAYLOAD_SIZE = 33

    @staticmethod
    def encode(drone_num, position, velocity, heading, armed=False, flying=False):
        payload = bytearray(UWBPayloadCodec.PAYLOAD_SIZE)
        idx = 0

        payload[idx] = UWBPayloadCodec.VERSION
        idx += 1

        payload[idx] = drone_num & 0xFF
        idx += 1

        for val in position:
            struct.pack_into("!f", payload, idx, float(val))
            idx += 4

        for val in velocity:
            struct.pack_into("!f", payload, idx, float(val))
            idx += 4

        struct.pack_into("!f", payload, idx, float(heading))
        idx += 4

        flags = 0
        if armed:
            flags |= 0x01
        if flying:
            flags |= 0x02
        struct.pack_into("!H", payload, idx, flags)

        crc = 0
        for byte in payload[:-1]:
            crc ^= byte
        payload[-1] = crc & 0xFF

        return bytes(payload)

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


class PayloadInjector:
    def __init__(self, drone_id="nexus0"):
        self.drone_id = drone_id
        self.drone_num = self._parse_drone_num(drone_id)

        self.position = np.array([0.0, 0.0, 0.0])
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.heading = 0.0
        self.armed = False
        self.flying = False

        rospy.Subscriber(
            f"/{self.drone_id}/mavros/local_position/pose",
            PoseStamped,
            self._on_pose_update,
        )
        rospy.Subscriber(
            f"/{self.drone_id}/uwb/range",
            UwbRange,
            self._on_uwb_ping,
        )

        self.uwb_pub = rospy.Publisher(
            f"/{self.drone_id}/uwb/range_with_payload",
            UwbRange,
            queue_size=10,
        )

        rospy.loginfo(
            "[PayloadInjector] %s: republishing to %s",
            self.drone_id,
            f"/{self.drone_id}/uwb/range_with_payload",
        )

    @staticmethod
    def _parse_drone_num(drone_id):
        digits = "".join(ch for ch in drone_id if ch.isdigit())
        return int(digits) if digits else 0

    def _on_pose_update(self, msg):
        self.position = np.array(
            [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        )

        q = msg.pose.orientation
        self.heading = np.arctan2(
            2 * (q.w * q.z + q.x * q.y),
            1 - 2 * (q.y * q.y + q.z * q.z),
        )

    def _on_uwb_ping(self, msg):
        if msg.src_id != self.drone_id:
            return

        msg.payload = UWBPayloadCodec.encode(
            drone_num=self.drone_num,
            position=self.position,
            velocity=self.velocity,
            heading=self.heading,
            armed=self.armed,
            flying=self.flying,
        )
        self.uwb_pub.publish(msg)


def main():
    rospy.init_node("payload_injector", anonymous=False)
    drone_id = rospy.get_param("~drone_id", "nexus0")
    PayloadInjector(drone_id)
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Payload injector shutting down")
