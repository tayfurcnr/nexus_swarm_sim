#!/usr/bin/env python3
"""
Example raw signal processor for the dynamic UWB simulator.
"""

from collections import defaultdict, deque

import numpy as np
import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Float32
from nexus_swarm_sim.msg import RawUWBSignal

SPEED_OF_LIGHT = 299792458.0


class RawSignalProcessor:
    def __init__(self, drone_id="nexus1", drone_prefix="nexus", enable_filtering=True, filter_window=5):
        self.drone_id = drone_id
        self.drone_prefix = drone_prefix
        self.enable_filtering = enable_filtering
        self.filter_window = filter_window

        self.distance_history = defaultdict(lambda: deque(maxlen=filter_window))
        self.snr_history = defaultdict(lambda: deque(maxlen=filter_window))
        self.measurement_count = 0
        self.min_snr_db = 5.0
        self.outlier_threshold = 2.0
        self.known_subscribers = {}

        rospy.Subscriber("/gazebo/model_states", ModelStates, self._on_model_states)

        self.distance_pub = rospy.Publisher(
            f"/{self.drone_id}/uwb/distance",
            Float32,
            queue_size=10,
        )
        self.measurement_pub = rospy.Publisher(
            f"/{self.drone_id}/uwb/processed_measurement",
            Vector3Stamped,
            queue_size=10,
        )

        rospy.loginfo(
            "[SignalProcessor] %s: ready (prefix=%s)",
            self.drone_id,
            self.drone_prefix,
        )

    def _on_model_states(self, msg):
        for model_name in msg.name:
            if not model_name.startswith(self.drone_prefix):
                continue
            if model_name == self.drone_id:
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
            rospy.loginfo("[SignalProcessor] %s: listening to %s", self.drone_id, topic)

    def _on_raw_signal(self, msg, src_drone):
        if msg.dst_id != self.drone_id:
            return

        if msg.snr_db < self.min_snr_db:
            rospy.logwarn(
                "[SignalProcessor] %s: low SNR from %s (%.1fdB)",
                self.drone_id,
                src_drone,
                msg.snr_db,
            )
            return

        distance_m = self._toa_to_distance(msg.toa_ns)
        quality = self._assess_quality(src_drone, distance_m, msg.snr_db)
        if quality < 0.3:
            return

        if self.enable_filtering:
            distance_m = self._apply_filter(src_drone, distance_m, msg.snr_db)

        self._publish_distance(src_drone, distance_m, msg.snr_db, quality)
        self.measurement_count += 1

    @staticmethod
    def _toa_to_distance(toa_ns):
        return (toa_ns * 1e-9 * SPEED_OF_LIGHT) / 2.0

    def _assess_quality(self, src_drone, distance_m, snr_db):
        quality = 1.0

        if snr_db < 5:
            quality *= 0.0
        elif snr_db < 15:
            quality *= (snr_db - 5) / 10.0
        else:
            quality *= min(1.0, snr_db / 30.0)

        if distance_m < 0.2 or distance_m > 300.0:
            quality *= 0.1

        if len(self.distance_history[src_drone]) > 2:
            prev_distances = list(self.distance_history[src_drone])
            mean_dist = np.mean(prev_distances)
            std_dist = np.std(prev_distances) + 0.1
            z_score = abs(distance_m - mean_dist) / std_dist
            if z_score > self.outlier_threshold:
                quality *= 1.0 / (1.0 + z_score)

        return np.clip(quality, 0.0, 1.0)

    def _apply_filter(self, src_drone, distance_m, snr_db):
        self.distance_history[src_drone].append(distance_m)
        self.snr_history[src_drone].append(snr_db)
        if len(self.distance_history[src_drone]) < 2:
            return distance_m

        distances = list(self.distance_history[src_drone])
        snrs = list(self.snr_history[src_drone])
        weights = np.exp(np.array(snrs) / 10.0)
        weights /= weights.sum()
        return np.average(distances, weights=weights)

    def _publish_distance(self, src_drone, distance_m, snr_db, quality):
        self.distance_pub.publish(Float32(data=distance_m))

        meas_msg = Vector3Stamped()
        meas_msg.header.stamp = rospy.Time.now()
        meas_msg.header.frame_id = f"from_{src_drone}_to_{self.drone_id}"
        meas_msg.vector.x = distance_m
        meas_msg.vector.y = snr_db
        meas_msg.vector.z = quality
        self.measurement_pub.publish(meas_msg)

    def print_statistics(self):
        rospy.loginfo("\n%s", "=" * 70)
        rospy.loginfo("[SignalProcessor] %s STATISTICS", self.drone_id)
        rospy.loginfo("%s", "=" * 70)
        rospy.loginfo("Total measurements processed: %d", self.measurement_count)

        for src_drone, distances_deque in sorted(self.distance_history.items()):
            distances = list(distances_deque)
            snrs = list(self.snr_history[src_drone])
            if not distances:
                continue
            rospy.loginfo("%s -> %s:", src_drone, self.drone_id)
            rospy.loginfo(
                "  Distance avg=%.3fm std=%.3fm min=%.3fm max=%.3fm",
                np.mean(distances),
                np.std(distances),
                np.min(distances),
                np.max(distances),
            )
            rospy.loginfo(
                "  SNR avg=%.1fdB std=%.1fdB min=%.1fdB max=%.1fdB",
                np.mean(snrs),
                np.std(snrs),
                np.min(snrs),
                np.max(snrs),
            )

    def seed_history(self, src_drone, distance_m, snr_db):
        self.distance_history[src_drone].append(distance_m)
        self.snr_history[src_drone].append(snr_db)


def main():
    rospy.init_node("signal_processor", anonymous=False)

    drone_id = rospy.get_param("~drone_id", "nexus1")
    drone_prefix = rospy.get_param("~drone_prefix", "nexus")
    enable_filtering = rospy.get_param("~enable_filtering", True)
    filter_window = rospy.get_param("~filter_window", 5)

    processor = RawSignalProcessor(drone_id, drone_prefix, enable_filtering, filter_window)

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        processor.print_statistics()
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Signal processor shutting down")
