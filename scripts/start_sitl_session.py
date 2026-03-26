#!/usr/bin/env python3

import os
import subprocess
import sys

import rospy


def prepend_env_path(name, value):
    current = os.environ.get(name, "")
    if current:
        os.environ[name] = f"{value}:{current}"
    else:
        os.environ[name] = value


def main():
    rospy.init_node("sitl_session_launcher", anonymous=False)

    ardupilot_root = os.path.expanduser(rospy.get_param("~ardupilot_root", "~/ardupilot"))
    ardupilot_gazebo_root = os.path.expanduser(
        rospy.get_param("~ardupilot_gazebo_root", "~/ardupilot_gazebo")
    )
    vehicle = rospy.get_param("~vehicle", "ArduCopter")
    frame = rospy.get_param("~frame", "gazebo-iris")
    instance = int(rospy.get_param("~instance", 0))
    sysid = int(rospy.get_param("~sysid", 1))
    use_dir = rospy.get_param("~use_dir", "/tmp/ardupilot_sitl_single_vehicle")
    custom_location = rospy.get_param("~custom_location", "")
    add_param_file = rospy.get_param("~add_param_file", "")
    enable_mavproxy = bool(rospy.get_param("~enable_mavproxy", False))
    mavproxy_console = bool(rospy.get_param("~mavproxy_console", False))
    mavproxy_map = bool(rospy.get_param("~mavproxy_map", False))

    sim_vehicle = os.path.join(ardupilot_root, "Tools", "autotest", "sim_vehicle.py")
    if not os.path.isfile(sim_vehicle):
        rospy.logfatal("sim_vehicle.py not found at %s", sim_vehicle)
        return 1

    prepend_env_path("GAZEBO_MODEL_PATH", os.path.join(ardupilot_gazebo_root, "models"))
    prepend_env_path("GAZEBO_RESOURCE_PATH", os.path.join(ardupilot_gazebo_root, "worlds"))
    prepend_env_path("GAZEBO_PLUGIN_PATH", os.path.join(ardupilot_gazebo_root, "build"))
    prepend_env_path("PATH", os.path.join(ardupilot_root, "Tools", "autotest"))

    cmd = [
        sim_vehicle,
        "-v",
        vehicle,
        "-f",
        frame,
        "-I",
        str(instance),
        "--sysid",
        str(sysid),
        "--use-dir",
        use_dir,
        "--no-rebuild",
    ]

    if not enable_mavproxy:
        cmd.append("--no-mavproxy")
    else:
        if mavproxy_console:
            cmd.append("--console")
        if mavproxy_map:
            cmd.append("--map")

    if custom_location:
        cmd.extend(["--custom-location", custom_location])

    if add_param_file:
        cmd.extend(["--add-param-file", os.path.expanduser(str(add_param_file).strip())])

    rospy.loginfo("Starting SITL session: %s", " ".join(cmd))

    process = subprocess.Popen(cmd, cwd=ardupilot_root)
    return process.wait()


if __name__ == "__main__":
    sys.exit(main())
