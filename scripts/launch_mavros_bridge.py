#!/usr/bin/env python3

import os
import signal
import socket
import subprocess
import sys
import time
from urllib.parse import urlparse

import rospy


def wait_for_tcp(host, port, timeout, poll_interval):
    deadline = time.monotonic() + timeout
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        try:
            with socket.create_connection((host, port), timeout=1.0):
                return True
        except OSError:
            time.sleep(poll_interval)
    return False


def terminate_process_tree(process):
    if process is None or process.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(process.pid), signal.SIGTERM)
    except ProcessLookupError:
        return


def main():
    rospy.init_node("mavros_bridge_launcher", anonymous=False)

    fcu_url = str(rospy.get_param("~fcu_url", "tcp://127.0.0.1:5760")).strip()
    wait_url = str(rospy.get_param("~wait_url", fcu_url)).strip()
    gcs_url = str(rospy.get_param("~gcs_url", "")).strip()
    tgt_system = int(rospy.get_param("~tgt_system", 1))
    tgt_component = int(rospy.get_param("~tgt_component", 1))
    respawn_mavros = bool(rospy.get_param("~respawn_mavros", True))
    wait_timeout = float(rospy.get_param("~wait_timeout", 90.0))
    wait_poll_interval = float(rospy.get_param("~wait_poll_interval", 0.5))
    restart_delay = float(rospy.get_param("~restart_delay", 2.0))
    ros_namespace = str(rospy.get_namespace()).rstrip("/")

    parsed = urlparse(wait_url)
    if parsed.scheme != "tcp" or not parsed.hostname or not parsed.port:
        rospy.logfatal("Unsupported wait_url for readiness check: %s", wait_url)
        return 1

    rospy.loginfo(
        "Waiting for SITL TCP endpoint %s:%d before starting MAVROS",
        parsed.hostname,
        parsed.port,
    )
    if not wait_for_tcp(parsed.hostname, parsed.port, wait_timeout, wait_poll_interval):
        rospy.logfatal(
            "Timed out waiting for SITL TCP endpoint %s:%d",
            parsed.hostname,
            parsed.port,
        )
        return 1

    env = os.environ.copy()
    if ros_namespace:
        env["ROS_NAMESPACE"] = ros_namespace

    cmd = [
        "roslaunch",
        "mavros",
        "apm.launch",
        f"fcu_url:={fcu_url}",
        f"gcs_url:={gcs_url}",
        f"tgt_system:={tgt_system}",
        f"tgt_component:={tgt_component}",
        "respawn_mavros:=true" if respawn_mavros else "respawn_mavros:=false",
    ]

    process = None
    rospy.on_shutdown(lambda: terminate_process_tree(process))

    while not rospy.is_shutdown():
        rospy.loginfo("Starting MAVROS bridge: %s", " ".join(cmd))
        process = subprocess.Popen(cmd, env=env, preexec_fn=os.setsid)
        exit_code = process.wait()
        if rospy.is_shutdown():
            break
        if not respawn_mavros:
            return exit_code
        rospy.logwarn("MAVROS bridge exited with code %d, restarting in %.1fs", exit_code, restart_delay)
        time.sleep(restart_delay)

    return 0


if __name__ == "__main__":
    sys.exit(main())
