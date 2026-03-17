#!/usr/bin/env python3

import os
import sys

import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose


def render_model(template_path, model_name, to_ardupilot_port, from_ardupilot_port):
    with open(template_path, "r", encoding="utf-8") as f:
        content = f.read()

    content = content.replace('<model name="iris_demo">', f'<model name="{model_name}">', 1)
    content = content.replace("iris_demo::iris::iris/imu_link::imu_sensor", f"{model_name}::iris::iris/imu_link::imu_sensor")
    content = content.replace("<fdm_port_in>9002</fdm_port_in>", f"<fdm_port_in>{from_ardupilot_port}</fdm_port_in>", 1)
    content = content.replace("<fdm_port_out>9003</fdm_port_out>", f"<fdm_port_out>{to_ardupilot_port}</fdm_port_out>", 1)
    return content


def main():
    rospy.init_node("ardupilot_model_spawner", anonymous=False)

    template_path = os.path.expanduser(
        rospy.get_param("~template_path", "~/ardupilot_gazebo/models/iris_with_ardupilot/model.sdf")
    )
    model_name = rospy.get_param("~model_name")
    reference_frame = rospy.get_param("~reference_frame", "world")
    to_ardupilot_port = int(rospy.get_param("~to_ardupilot_port"))
    from_ardupilot_port = int(rospy.get_param("~from_ardupilot_port"))
    output_path = rospy.get_param("~output_path", "")

    if not os.path.isfile(template_path):
        rospy.logfatal("ArduPilot model template not found: %s", template_path)
        return 1

    model_xml = render_model(template_path, model_name, to_ardupilot_port, from_ardupilot_port)

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
    resp = spawn_model(model_name, model_xml, "", pose, reference_frame)
    if not resp.success:
        rospy.logfatal("Failed to spawn %s: %s", model_name, resp.status_message)
        return 1

    rospy.loginfo("Spawned %s successfully", model_name)
    return 0


if __name__ == "__main__":
    sys.exit(main())
