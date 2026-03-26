#!/usr/bin/env python3

import sys

import rospy
from mavros_msgs.srv import StreamRate, StreamRateRequest


class StreamRateConfigurator:
    def __init__(self):
        self.service_name = rospy.get_param("~service_name", "mavros/set_stream_rate")
        self.stream_id = int(rospy.get_param("~stream_id", StreamRateRequest.STREAM_ALL))
        self.message_rate = int(rospy.get_param("~message_rate", 20))
        self.enable = bool(rospy.get_param("~enable", True))
        self.connection_timeout = float(rospy.get_param("~connection_timeout", 30.0))

    def run(self):
        rospy.loginfo("Waiting for stream-rate service: %s", self.service_name)
        try:
            rospy.wait_for_service(self.service_name, timeout=self.connection_timeout)
        except rospy.ROSException:
            rospy.logfatal("Timed out waiting for stream-rate service: %s", self.service_name)
            return 1
        set_stream_rate = rospy.ServiceProxy(self.service_name, StreamRate)

        req = StreamRateRequest()
        req.stream_id = self.stream_id
        req.message_rate = self.message_rate
        req.on_off = self.enable

        rospy.loginfo(
            "Configuring MAVLink stream rate: stream_id=%d message_rate=%d on_off=%s",
            req.stream_id,
            req.message_rate,
            req.on_off,
        )
        set_stream_rate(req)
        rospy.loginfo("MAVLink stream-rate configuration complete")
        return 0


def main():
    rospy.init_node("stream_rate_configurator", anonymous=False)
    return StreamRateConfigurator().run()


if __name__ == "__main__":
    sys.exit(main())
