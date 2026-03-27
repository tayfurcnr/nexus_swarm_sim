#!/usr/bin/env python3
"""
Example consumer for RawUWBSignal DS-TWR exchanges.

This node listens to all vehicle raw-signal topics, reconstructs exchanges by
exchange_seq, and reports complete or incomplete DS-TWR transactions involving
one monitored vehicle.
"""

from collections import defaultdict

import rospy
from gazebo_msgs.msg import ModelStates
from nexus_swarm_sim.msg import RawUWBSignal


FRAME_NAMES = {
    RawUWBSignal.FRAME_UNKNOWN: "UNKNOWN",
    RawUWBSignal.FRAME_BLINK: "BLINK",
    RawUWBSignal.FRAME_POLL: "POLL",
    RawUWBSignal.FRAME_RESP: "RESP",
    RawUWBSignal.FRAME_FINAL: "FINAL",
    RawUWBSignal.FRAME_DATA: "DATA",
}


class DstwrExchangeConsumer:
    def __init__(self, drone_id="nexus1", drone_prefix="nexus", exchange_timeout_s=0.75):
        self.drone_id = drone_id
        self.drone_prefix = drone_prefix
        self.exchange_timeout_s = exchange_timeout_s

        self.known_subscribers = {}
        self.exchange_state = {}
        self.completed_count = 0
        self.incomplete_count = 0

        rospy.Subscriber("/gazebo/model_states", ModelStates, self._on_model_states)
        rospy.Timer(rospy.Duration(0.2), self._expire_old_exchanges)

        rospy.loginfo(
            "[DSTWRConsumer] %s: ready (prefix=%s timeout=%.2fs)",
            self.drone_id,
            self.drone_prefix,
            self.exchange_timeout_s,
        )

    def _on_model_states(self, msg):
        for model_name in msg.name:
            if not model_name.startswith(self.drone_prefix):
                continue
            if model_name in self.known_subscribers:
                continue

            topic = f"/{model_name}/uwb/raw_signal"
            self.known_subscribers[model_name] = rospy.Subscriber(
                topic,
                RawUWBSignal,
                self._on_raw_signal,
                callback_args=model_name,
            )
            rospy.loginfo("[DSTWRConsumer] %s: listening to %s", self.drone_id, topic)

    def _on_raw_signal(self, msg, _topic_vehicle):
        if msg.src_id != self.drone_id and msg.dst_id != self.drone_id:
            return

        peer_id = msg.dst_id if msg.src_id == self.drone_id else msg.src_id
        exchange_key = self._exchange_key(peer_id, msg.exchange_seq)
        state = self.exchange_state.setdefault(
            exchange_key,
            {
                "peer_id": peer_id,
                "exchange_seq": msg.exchange_seq,
                "first_seen": msg.header.stamp,
                "last_seen": msg.header.stamp,
                "frames": {},
                "frame_seqs": [],
                "invalid_seen": False,
            },
        )

        state["last_seen"] = msg.header.stamp
        state["frames"][msg.frame_type] = {
            "src_id": msg.src_id,
            "dst_id": msg.dst_id,
            "frame_seq": msg.frame_seq,
            "stamp": msg.header.stamp,
            "valid": msg.valid,
            "status_code": msg.status_code,
            "payload_len": len(msg.frame_payload),
        }
        state["frame_seqs"].append(msg.frame_seq)
        state["invalid_seen"] = state["invalid_seen"] or (not msg.valid)

        if self._is_complete_exchange(state):
            self._log_complete_exchange(state)
            self.completed_count += 1
            self.exchange_state.pop(exchange_key, None)

    def _expire_old_exchanges(self, _event):
        now = rospy.Time.now()
        expired_keys = []

        for exchange_key, state in self.exchange_state.items():
            age = (now - state["last_seen"]).to_sec()
            if age < self.exchange_timeout_s:
                continue

            self._log_incomplete_exchange(state)
            self.incomplete_count += 1
            expired_keys.append(exchange_key)

        for exchange_key in expired_keys:
            self.exchange_state.pop(exchange_key, None)

    def _is_complete_exchange(self, state):
        frames = state["frames"]
        return (
            RawUWBSignal.FRAME_POLL in frames
            and RawUWBSignal.FRAME_RESP in frames
            and RawUWBSignal.FRAME_FINAL in frames
        )

    def _log_complete_exchange(self, state):
        poll = state["frames"][RawUWBSignal.FRAME_POLL]
        resp = state["frames"][RawUWBSignal.FRAME_RESP]
        final = state["frames"][RawUWBSignal.FRAME_FINAL]
        rospy.loginfo(
            "[DSTWRConsumer] COMPLETE peer=%s exchange_seq=%d poll_seq=%d resp_seq=%d final_seq=%d invalid=%s",
            state["peer_id"],
            state["exchange_seq"],
            poll["frame_seq"],
            resp["frame_seq"],
            final["frame_seq"],
            state["invalid_seen"],
        )

    def _log_incomplete_exchange(self, state):
        present = [
            FRAME_NAMES[frame_type]
            for frame_type in sorted(state["frames"].keys())
        ]
        rospy.logwarn(
            "[DSTWRConsumer] INCOMPLETE peer=%s exchange_seq=%d seen=%s invalid=%s",
            state["peer_id"],
            state["exchange_seq"],
            ",".join(present) if present else "none",
            state["invalid_seen"],
        )

    def _exchange_key(self, peer_id, exchange_seq):
        return f"{peer_id}:{exchange_seq}"

    def print_statistics(self):
        rospy.loginfo("%s", "=" * 70)
        rospy.loginfo(
            "[DSTWRConsumer] %s stats: complete=%d incomplete=%d open=%d",
            self.drone_id,
            self.completed_count,
            self.incomplete_count,
            len(self.exchange_state),
        )
        rospy.loginfo("%s", "=" * 70)


def main():
    rospy.init_node("raw_dstwr_exchange_consumer_example", anonymous=False)

    drone_id = rospy.get_param("~drone_id", "nexus1")
    drone_prefix = rospy.get_param("~drone_prefix", "nexus")
    exchange_timeout_s = rospy.get_param("~exchange_timeout_s", 0.75)

    consumer = DstwrExchangeConsumer(drone_id, drone_prefix, exchange_timeout_s)

    rate = rospy.Rate(0.5)
    while not rospy.is_shutdown():
        consumer.print_statistics()
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("DS-TWR exchange consumer shutting down")
