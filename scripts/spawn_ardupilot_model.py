#!/usr/bin/env python3

import os
import re
import shutil
import sys

import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose


def _load_gimbal_model():
    candidates = [
        os.path.expanduser("~/.gazebo/models/gimbal_small_2d/model.sdf"),
        os.path.expanduser("~/Downloads/models/gimbal_small_2d/model.sdf"),
    ]
    for path in candidates:
        if os.path.isfile(path):
            with open(path, "r", encoding="utf-8") as f:
                return f.read()
    raise FileNotFoundError("gimbal_small_2d/model.sdf not found in known Gazebo model paths")


def _gimbal_model_source_path():
    candidates = [
        os.path.expanduser("~/.gazebo/models/gimbal_small_2d"),
        os.path.expanduser("~/Downloads/models/gimbal_small_2d"),
    ]
    for path in candidates:
        if os.path.isfile(os.path.join(path, "model.sdf")):
            return path
    raise FileNotFoundError("gimbal_small_2d model directory not found in known Gazebo model paths")


def _prepare_namespaced_gimbal_model(model_name):
    source_dir = _gimbal_model_source_path()
    target_root = f"/tmp/nexus_swarm_sim_models/{model_name}"
    target_dir = os.path.join(target_root, "gimbal_small_2d")
    os.makedirs(target_root, exist_ok=True)
    if os.path.isdir(target_dir):
        shutil.rmtree(target_dir)
    shutil.copytree(source_dir, target_dir)

    model_sdf_path = os.path.join(target_dir, "model.sdf")
    with open(model_sdf_path, "r", encoding="utf-8") as f:
        gimbal_model = f.read()

    gimbal_model = gimbal_model.replace(
        "<plugin name=\"camera_controller\" filename=\"libgazebo_ros_camera.so\">",
        "<plugin name=\"camera_controller\" filename=\"libgazebo_ros_camera.so\">\n"
        f"          <robotNamespace>/{model_name}</robotNamespace>",
        1,
    )
    gimbal_model = gimbal_model.replace(
        "<frameName>gimbal_camera_optical_frame</frameName>",
        f"<frameName>{model_name}/gimbal_camera_optical_frame</frameName>",
        1,
    )

    with open(model_sdf_path, "w", encoding="utf-8") as f:
        f.write(gimbal_model)

    return target_root, target_dir


def render_model(template_path, model_name, to_ardupilot_port, from_ardupilot_port):
    with open(template_path, "r", encoding="utf-8") as f:
        content = f.read()

    content = content.replace('<model name="iris_demo">', f'<model name="{model_name}">', 1)
    content = content.replace("iris_demo::iris::iris/imu_link::imu_sensor", f"{model_name}::iris::iris/imu_link::imu_sensor")
    content = content.replace("<fdm_port_in>9002</fdm_port_in>", f"<fdm_port_in>{from_ardupilot_port}</fdm_port_in>", 1)
    content = content.replace("<fdm_port_out>9003</fdm_port_out>", f"<fdm_port_out>{to_ardupilot_port}</fdm_port_out>", 1)
    gimbal_model_root, gimbal_model_dir = _prepare_namespaced_gimbal_model(model_name)
    content = content.replace(
        "<uri>model://gimbal_small_2d</uri>",
        f"<uri>file://{gimbal_model_dir}</uri>",
        1,
    )

    return content, gimbal_model_root


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

    model_xml, gimbal_model_root = render_model(template_path, model_name, to_ardupilot_port, from_ardupilot_port)

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
    resp = spawn_model(model_name, model_xml, f"/{model_name}", pose, reference_frame)
    if not resp.success:
        rospy.logfatal("Failed to spawn %s: %s", model_name, resp.status_message)
        return 1

    rospy.loginfo("Spawned %s successfully", model_name)
    return 0


if __name__ == "__main__":
    sys.exit(main())
