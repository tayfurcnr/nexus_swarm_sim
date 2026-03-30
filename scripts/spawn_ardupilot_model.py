#!/usr/bin/env python3

import os
import sys

import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIR not in sys.path:
    sys.path.insert(0, SCRIPT_DIR)

from model_renderer import render_model


def main():
    rospy.init_node("ardupilot_model_spawner", anonymous=False)

    template_path = os.path.expanduser(
        rospy.get_param("~template_path", "~/ardupilot_gazebo/models/iris_with_ardupilot/model.sdf")
    )
    model_name = rospy.get_param("~model_name")
    robot_namespace = rospy.get_param("~robot_namespace", f"/{model_name}")
    reference_frame = rospy.get_param("~reference_frame", "world")
    to_ardupilot_port = int(rospy.get_param("~to_ardupilot_port"))
    from_ardupilot_port = int(rospy.get_param("~from_ardupilot_port"))
    output_path = rospy.get_param("~output_path", "")
    enable_gimbal = bool(rospy.get_param("~enable_gimbal", True))

    if not os.path.isfile(template_path):
        rospy.logfatal("ArduPilot model template not found: %s", template_path)
        return 1

    model_xml, gimbal_model_root = render_model(
        template_path,
        model_name,
        robot_namespace,
        to_ardupilot_port,
        from_ardupilot_port,
        enable_gimbal=enable_gimbal,
    )

    if gimbal_model_root:
        current_model_path = os.environ.get("GAZEBO_MODEL_PATH", "")
        os.environ["GAZEBO_MODEL_PATH"] = f"{gimbal_model_root}:{current_model_path}" if current_model_path else gimbal_model_root

    if output_path:
        os.makedirs(os.path.dirname(output_path), exist_ok=True)
        with open(output_path, "w", encoding="utf-8") as f:
            f.write(model_xml)

    pose = Pose()
    pose.position.x = float(rospy.get_param("~x", 0.0))
    pose.position.y = float(rospy.get_param("~y", 0.0))
    pose.position.z = float(rospy.get_param("~z", 0.2))
    pose.orientation.w = 1.0

    service_name = "/gazebo/spawn_sdf_model"
    rospy.loginfo("Waiting for %s", service_name)
    rospy.wait_for_service(service_name, timeout=30.0)
    spawn_model = rospy.ServiceProxy(service_name, SpawnModel)

    rospy.loginfo(
        "Spawning %s with fdm_in=%d fdm_out=%d at (%.2f, %.2f, %.2f)",
        model_name,
        from_ardupilot_port,
        to_ardupilot_port,
        pose.position.x,
        pose.position.y,
        pose.position.z,
    )
    # Provide a per-vehicle ROS namespace so Gazebo ROS plugins inside the model
    # don't all publish into the global /gimbal namespace in multi-vehicle runs.
    resp = spawn_model(model_name, model_xml, robot_namespace, pose, reference_frame)
    if not resp.success:
        rospy.logfatal("Failed to spawn %s: %s", model_name, resp.status_message)
        return 1

    rospy.loginfo("Spawned %s successfully", model_name)
    return 0


if __name__ == "__main__":
    sys.exit(main())
